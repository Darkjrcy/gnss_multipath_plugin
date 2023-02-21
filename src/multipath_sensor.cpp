// Copyright 2018 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#include <string>
#include <algorithm>
#include <limits>
#include <memory>
#include <cmath>
#include <filesystem>

#include <boost/make_shared.hpp>
#include <boost/variant.hpp>

#include <gazebo/transport/transport.hh>
#include <gazebo/common/Exception.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/HingeJoint.hh>
#include <gazebo/sensors/SensorTypes.hh>
#include <gazebo/sensors/RaySensor.hh>

#include <gazebo_ros/conversions/sensor_msgs.hpp>
#include <gazebo_ros/node.hpp>
#include <gazebo_ros/utils.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/time.hpp>

#include <ignition/math/Rand.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Matrix3.hh>

#include <predict/predict.h>
#include <gnss_multipath_plugin/multipath_sensor.hpp>
#include "gnss_multipath_plugin/msg/gnss_multipath_fix.hpp"
#include "gnss_multipath_plugin/geodetic_conv.hpp"


namespace gazebo_plugins
{

class GazeboRosMultipathSensorPrivate
{
public:
  /// Node for ROS communication.
  gazebo_ros::Node::SharedPtr ros_node_;

  // Aliases
  using LaserScan = sensor_msgs::msg::LaserScan;
  using LaserScanPub = rclcpp::Publisher<LaserScan>::SharedPtr;
 
  /// Publisher of output
  /// \todo use std::variant one c++17 is supported in ROS2
  boost::variant<LaserScanPub> pub_;

  /// TF frame output is published in
  std::string frame_name_;
  
  /// Subscribe to gazebo's laserscan, calling the appropriate callback based on output type
  void SubscribeGazeboLaserScan();

  /// Publish a sensor_msgs/LaserScan message from a gazebo laser scan
  void PublishLaserScan(ConstLaserScanStampedPtr & _msg);

  /// Gazebo transport topic to subscribe to for laser scan
  std::string sensor_topic_;

  /// Minimum intensity value to publish for laser scan / pointcloud messages
  double min_intensity_{0.0};

  /// Gazebo node used to subscribe to laser scan
  gazebo::transport::NodePtr gazebo_node_;

  /// Gazebo subscribe to parent sensor's laser scan
  gazebo::transport::SubscriberPtr laser_scan_sub_;
  gazebo::sensors::RaySensorPtr parent_ray_sensor_;
  gazebo::physics::WorldPtr world_;
  gazebo::physics::EntityPtr parent_entity_;

  // Time information to get the satellite position
  struct tm *timeinfo_;

  
  bool disable_noise_;
  geodetic_converter::GeodeticConverter g_geodetic_converter;
  
  int num_sat_;
  // TlE parameters to store the satellite's orbital parameters
  const char **tle_lines_;

  // Satellite orbital parameters to predict satellite's position
  predict_orbital_elements_t **sat_orbit_elements_;

  // Satellite position information at a given time
  struct predict_position *sat_position_;

  void SetRayAngles(std::vector<double> _azimuth, std::vector<double> _elevation); 
  bool GetLeastSquaresEstimate(std::vector<double> _meas, std::vector<std::vector<double>> _sat_ecef, Eigen::Vector3d & _rec_ecef);
  void ParseSatelliteTLE();
  void GetSatellitesInfo(struct tm *_timeinfo, std::vector<double> _rec_lla,
      std::vector<std::vector<double>> &_sat_ecef, std::vector<double> &_true_range, std::vector<double> &_azimuth,
      std::vector<double> &_elevation);
  time_t MakeTimeUTC(const struct tm* timeinfo_utc);
  double origin_lat_, origin_lon_, origin_alt_;
  
  //Offset publisher to publish multipath offset
  rclcpp::Publisher<gnss_multipath_plugin::msg::GNSSMultipathFix>::SharedPtr gnss_multipath_fix_publisher_;
};

GazeboRosMultipathSensor::GazeboRosMultipathSensor()
: impl_(std::make_unique<GazeboRosMultipathSensorPrivate>())
{
}

GazeboRosMultipathSensor::~GazeboRosMultipathSensor()
{
  // Must release subscriber and then call fini on node to remove it from topic manager.
  impl_->laser_scan_sub_.reset();
  if (impl_->gazebo_node_) {
    impl_->gazebo_node_->Fini();
  }
  impl_->gazebo_node_.reset();
}

void GazeboRosMultipathSensor::Load(gazebo::sensors::SensorPtr _sensor, sdf::ElementPtr _sdf)
{
  // Initialize time data structure
  impl_->timeinfo_ = (struct tm*)calloc(1, sizeof(struct tm));
  // Create ros_node configured from sdf
  impl_->ros_node_ = gazebo_ros::Node::Get(_sdf);
  impl_->parent_ray_sensor_ =
    std::dynamic_pointer_cast<gazebo::sensors::RaySensor>(_sensor);
  std::string worldName = _sensor->WorldName();
  impl_->world_ = gazebo::physics::get_world(worldName);
  std::string parent_entity_name = impl_->parent_ray_sensor_->ParentName();
  impl_->parent_entity_ = impl_->world_->EntityByName("laser_0");
  RCLCPP_INFO(impl_->ros_node_->get_logger(), parent_entity_name);
  // Get QoS profiles
  const gazebo_ros::QoS & qos = impl_->ros_node_->get_qos();

  // Get QoS profile for the publisher
  rclcpp::QoS pub_qos = qos.get_publisher_qos("~/out", rclcpp::SensorDataQoS().reliable());

  // Get tf frame for output
  impl_->frame_name_ = gazebo_ros::SensorFrameID(*_sensor, *_sdf);

  // Get output type from sdf if provided
  if (!_sdf->HasElement("output_type")) {
    RCLCPP_WARN(
      impl_->ros_node_->get_logger(), "missing <output_type>, defaults to sensor_msgs/LaserScan");
      impl_->pub_ = impl_->ros_node_->create_publisher<sensor_msgs::msg::LaserScan>(
        "~/out", pub_qos);
  } else {
    std::string output_type_string = _sdf->Get<std::string>("output_type");
    if (output_type_string == "sensor_msgs/LaserScan") {
      impl_->pub_ = impl_->ros_node_->create_publisher<sensor_msgs::msg::LaserScan>(
        "~/out", pub_qos);
    } 
    else {
      RCLCPP_ERROR(
        impl_->ros_node_->get_logger(), "Invalid <output_type> [%s]", output_type_string.c_str());
      return;
    }
  }
  
  if (!_sdf->HasElement("min_intensity")) {
    RCLCPP_DEBUG(
      impl_->ros_node_->get_logger(), "missing <min_intensity>, defaults to %f",
      impl_->min_intensity_);
  } else {
    impl_->min_intensity_ = _sdf->Get<double>("min_intensity");
  }
  if (!_sdf->HasElement("disableNoise"))
  {
    RCLCPP_DEBUG(impl_->ros_node_->get_logger(), "Default to disable gaussian noise");
    impl_->disable_noise_ = true;
  }
  else
  {
    impl_->disable_noise_ = _sdf->Get<bool>("disableNoise");
    RCLCPP_INFO(impl_->ros_node_->get_logger(), "disable noise is %d\n", impl_->disable_noise_);

  }
  if(!_sdf->HasElement("origin_latitude"))
  {
    RCLCPP_DEBUG(impl_->ros_node_->get_logger(),  "Must provide origin latitutde");
  }
  else
  {
    std::string origin_lat = _sdf->Get<std::string>("origin_latitude");
    std::istringstream ss(origin_lat);
    double origin_lat_;
    ss >> origin_lat_;
    impl_->origin_lat_ = origin_lat_;
  }
  if(!_sdf->HasElement("origin_longitude"))
  {
    RCLCPP_DEBUG(impl_->ros_node_->get_logger(),  "Must provide origin longitude");
  }
  else
  {
    std::string origin_lon = _sdf->Get<std::string>("origin_longitude");
    std::istringstream ss(origin_lon);
    double origin_lon_;
    ss >> origin_lon_;
    impl_->origin_lon_ = origin_lon_;
  }

  if(!_sdf->HasElement("origin_altitude"))
  {
    RCLCPP_DEBUG(impl_->ros_node_->get_logger(),  "Must provide origin altitude");
  }
  else
  {
    std::string origin_alt = _sdf->Get<std::string>("origin_altitude");
    std::istringstream ss(origin_alt);
    double origin_alt_;
    ss >> origin_alt_;
    impl_->origin_alt_ = origin_alt_;
  }
  // Initializing the converter with the geodetic location of the origin.
  impl_->g_geodetic_converter.initialiseReference(impl_->origin_lat_, impl_->origin_lon_, impl_->origin_alt_);
  RCLCPP_INFO(impl_->ros_node_->get_logger(), "Origin Lat Lon Alt:%f %f %f",impl_->origin_lat_, impl_->origin_lon_, impl_->origin_alt_);

  RCLCPP_INFO(impl_->ros_node_->get_logger(), "Num sat:%d", impl_->num_sat_);
  RCLCPP_INFO(impl_->ros_node_->get_logger(), std::filesystem::current_path());
  impl_->ParseSatelliteTLE();
  // Create gazebo transport node and subscribe to sensor's laser scan
  impl_->gazebo_node_ = boost::make_shared<gazebo::transport::Node>();
  impl_->gazebo_node_->Init(_sensor->WorldName());

  // TODO(ironmig): use lazy publisher to only process laser data when output has a subscriber
  impl_->sensor_topic_ = _sensor->Topic();
  impl_->gnss_multipath_fix_publisher_ = impl_->ros_node_->create_publisher<gnss_multipath_plugin::msg::GNSSMultipathFix>("gnss_multipath_fix", 10); 
  impl_->SubscribeGazeboLaserScan();
}

void GazeboRosMultipathSensorPrivate::SubscribeGazeboLaserScan()
{
  if (pub_.type() == typeid(LaserScanPub)) {
    laser_scan_sub_ = gazebo_node_->Subscribe(
      sensor_topic_, &GazeboRosMultipathSensorPrivate::PublishLaserScan, this);
  } 
  else {
    RCLCPP_ERROR(ros_node_->get_logger(), "Publisher is an invalid type. This is an internal bug.");
  }
}

void GazeboRosMultipathSensorPrivate::PublishLaserScan(ConstLaserScanStampedPtr & _msg)
{
  //rclcpp::Time t1 = rclcpp::Clock{}.now();
  std::vector<double> sat_true_range_;
  std::vector<std::vector<double>> sat_ecef_;
  std::vector<double> elevation_;
  std::vector<double> azimuth_; 
  std::vector<double> ray_elevation_;
  std::vector<double> ray_azimuth_;
  std::vector<double> rec_lla_{0,0,0};
  // Reciever's pose in ENU world coordinates
  ignition::math::Vector3d true_rec_world_pos_ = parent_entity_->WorldPose().Pos();
  // Reciever's position in geodetic (LLA) coordinates
  g_geodetic_converter.enu2Geodetic(true_rec_world_pos_[0], true_rec_world_pos_[1],
                              true_rec_world_pos_[2], &rec_lla_[0], &rec_lla_[1], &rec_lla_[2]);
  
  // Constructing a time structure for getting satellites position
  timeinfo_->tm_year = 2021 - 1900;
  timeinfo_->tm_mon = 5 - 1;
  timeinfo_->tm_mday = 15;
  timeinfo_->tm_hour = 2;
  timeinfo_->tm_min = 35;
  timeinfo_->tm_sec = 0;
  timeinfo_->tm_isdst = 0;

  // GetSatellitesECEF(timeinfo_, sat_ecef_);
  // GetSatellitesTrueRange(rec_lla_, sat_true_range_);
  // GetSatellitesAzimuthElevationAngles(rec_lla_, azimuth_, elevation_);
  GetSatellitesInfo(timeinfo_,rec_lla_, sat_ecef_, sat_true_range_, azimuth_, elevation_ );
  num_sat_ = sat_true_range_.size();
  for ( int i = 0; i < num_sat_; i++ )
  {
    // Setting the azimuth and elevation angles for the satellite ray and the reflected ray for each satellite.
    ray_azimuth_.push_back(azimuth_[i]);
    ray_azimuth_.push_back(azimuth_[i]+ M_PI); // assume reflection is 180 deg azimuth from the direct ray
    ray_elevation_.push_back(elevation_[i]);
    ray_elevation_.push_back(elevation_[i]); // assume reflection has the same elevation as the direct ray
  }
  //Updating the ray angles for sateliite and reflected ray.
  SetRayAngles(ray_azimuth_, ray_elevation_);

  //Ray_ranges stores the ranges of both satellite and reflected rays.
  std::vector<double> ray_ranges_; 
  ray_ranges_.resize(_msg->scan().count());
  std::copy(_msg->scan().ranges().begin(),
            _msg->scan().ranges().end(),
            ray_ranges_.begin());
                        
  
  //Publishing the range offset and satellites blocked
  gnss_multipath_plugin::msg::GNSSMultipathFix gnss_multipath_fix_msg;
  gnss_multipath_fix_msg.header.stamp = rclcpp::Time(_msg->time().sec(), _msg->time().nsec());
  gnss_multipath_fix_msg.header.frame_id = frame_name_;
  gnss_multipath_fix_msg.navsatfix.header.stamp = rclcpp::Time(_msg->time().sec(), _msg->time().nsec());
  gnss_multipath_fix_msg.navsatfix.header.frame_id = frame_name_;
  gnss_multipath_fix_msg.enu_true.resize(3);
  gnss_multipath_fix_msg.enu_gnss_fix.resize(3);
  int num_sat_blocked_ = 0;
  std::vector<double> visible_sat_range_meas;
  std::vector<std::vector<double>> visible_sat_ecef;
  
  // Iterate for all the visible satellites (positive elevation angles)
  for (int i=0; i < num_sat_; i++)
  {
    // noise
    double noise_;
    if (!disable_noise_)
    {  
        noise_ = ignition::math::Rand::DblNormal(0.0, 1.0);
    }
    if (ray_elevation_[2*i] > (20*M_PI)/180)
    {
      // ray is obstructed
      if (ray_ranges_[2*i] < _msg->scan().range_max()) 
      {
        // check range of the mirror ray corresponding to the satellite ray 
        double mir_ray_range_ = ray_ranges_[2*i+1];

        // check if mirror ray also is obstructed, providing a reflection
        if (mir_ray_range_ < _msg->scan().range_max())
        {
          double range_offset =  mir_ray_range_ * ( 1 + sin(M_PI/2.0 - 2*ray_elevation_[2*i]));
          if (range_offset < 100)
          {
            visible_sat_range_meas.push_back(sat_true_range_[i] + range_offset +  noise_);
            visible_sat_ecef.push_back(sat_ecef_[i]);
          }
          else
          {
            // The multipath distance is large, signal intensity will be degraded significantly, so ignored.
            num_sat_blocked_++;
          }
        } 
        else 
        {
          // otherwise no line of sight to any satellites and no reflections, no reading
          num_sat_blocked_++;
        }
      } 
      else 
      {
        // unobstructed sat reading
        visible_sat_range_meas.push_back(sat_true_range_[i] + noise_);
        visible_sat_ecef.push_back(sat_ecef_[i]);
      }
    }
    else
    {
      // low elevation sat reading
      num_sat_blocked_++;
    }
  }
  if ((num_sat_ - num_sat_blocked_) > 4)
  {
    // Calculate the reciever's position using the satellite's predicted coordinates and the pseudo-ranges.
    Eigen::Vector3d rec_ecef(0,0,0);              
    GetLeastSquaresEstimate(visible_sat_range_meas,  visible_sat_ecef, rec_ecef);
    double enu_x,enu_y,enu_z, latitude=0, longitude=0, altitude=0;
    g_geodetic_converter.ecef2Geodetic(rec_ecef(0), rec_ecef(1),rec_ecef(2), &latitude,
                      &longitude, &altitude);
    
    g_geodetic_converter.geodetic2Enu(latitude,
                      longitude, altitude, &enu_x,&enu_y,&enu_z);
    // RCLCPP_INFO(ros_node_->get_logger(), "Lat Lon Alt:%f %f %f",latitude , longitude, altitude);
    // RCLCPP_INFO(ros_node_->get_logger(), "ENU: %f %f %f", enu_x,enu_y,enu_z);
    // RCLCPP_INFO(ros_node_->get_logger(), "True: %f %f %f",true_rec_world_pos_[0],true_rec_world_pos_[1],true_rec_world_pos_[2]);
    
    gnss_multipath_fix_msg.navsatfix.status.status = 1;
    
    gnss_multipath_fix_msg.navsatfix.latitude = latitude;
    gnss_multipath_fix_msg.navsatfix.longitude = longitude;
    gnss_multipath_fix_msg.navsatfix.altitude = altitude;

    gnss_multipath_fix_msg.enu_gnss_fix[0] = enu_x;
    gnss_multipath_fix_msg.enu_gnss_fix[1] = enu_y;
    gnss_multipath_fix_msg.enu_gnss_fix[2] = enu_z;
  }
  else 
  {
    // RCLCPP_INFO(ros_node_->get_logger(), "No Fix");
    gnss_multipath_fix_msg.navsatfix.status.status = 0;
    gnss_multipath_fix_msg.enu_gnss_fix[0] = NAN;
    gnss_multipath_fix_msg.enu_gnss_fix[1] = NAN;
    gnss_multipath_fix_msg.enu_gnss_fix[2] = NAN;
  }
  gnss_multipath_fix_msg.enu_true[0] = true_rec_world_pos_[0];
  gnss_multipath_fix_msg.enu_true[1] = true_rec_world_pos_[1];
  gnss_multipath_fix_msg.enu_true[2] = true_rec_world_pos_[2];
  gnss_multipath_fix_msg.visible_sat_count = num_sat_ - num_sat_blocked_;
  gnss_multipath_fix_publisher_->publish(gnss_multipath_fix_msg);

  // Convert Laser scan to ROS LaserScan
  auto ls = gazebo_ros::Convert<sensor_msgs::msg::LaserScan>(*_msg);
  // Set tf frame
  ls.header.frame_id = frame_name_;
  // Publish output
  boost::get<LaserScanPub>(pub_)->publish(ls);
  // rclcpp::Time t2 = rclcpp::Clock{}.now();
  // RCLCPP_INFO(ros_node_->get_logger(), "Exec time %f", t1.seconds()- t2.seconds());
}

void GazeboRosMultipathSensorPrivate::SetRayAngles(std::vector<double> _azimuth, std::vector<double> _elevation) 
{ 
  gazebo::physics::MultiRayShapePtr laser_shape_ = parent_ray_sensor_->LaserShape();
  ignition::math::Vector3d start, end, axis, end_scan;
  ignition::math::Quaterniond ray;
  for(unsigned int i=0; i < _elevation.size(); i++ ) 
  {
    double yaw_angle = _azimuth[i];
    double pitch_angle = _elevation[i];
    // since we're rotating a unit x vector, a pitch rotation will now be
    // around the negative y axis
    ray.Euler(ignition::math::Vector3d(0.0, -pitch_angle, yaw_angle));
    axis = parent_entity_->WorldPose().Rot().Inverse() * parent_ray_sensor_->Pose().Rot() * ray * ignition::math::Vector3d::UnitX;
    start = (axis * laser_shape_->GetMinRange()) + parent_ray_sensor_->Pose().Pos();
    end = (axis * laser_shape_->GetMaxRange()) + parent_ray_sensor_->Pose().Pos();
    laser_shape_->SetRay(i, start, end);
  }
}

bool GazeboRosMultipathSensorPrivate::GetLeastSquaresEstimate(std::vector<double> _meas,
                 std::vector<std::vector<double>> _sat_ecef, Eigen::Vector3d &_rec_ecef)
{
  const int nsat = _meas.size();
  Eigen::MatrixXd A, b;
  A.resize(nsat, 4);
  b.resize(nsat, 1);
  Eigen::Matrix<double, 4, 1> dx;
  Eigen::ColPivHouseholderQR<Eigen::MatrixXd> solver;
  double cdt = 0;
  int iter = 0;
  do
  {
    iter++;
    int i = 0;
    for (int i = 0 ; i < nsat; i++)
    {
      Eigen::Vector3d sat_pos(_sat_ecef[i][0], _sat_ecef[i][1], _sat_ecef[i][2]);
      double dist = (_rec_ecef - sat_pos).norm();
      A.block<1,3>(i,0) = (_rec_ecef - sat_pos).normalized();
      b(i) = _meas[i] - (dist + cdt);
      A(i,3) = -1;
      
    }
    solver.compute(A);
    dx = solver.solve(b);
    _rec_ecef += dx.topRows<3>();
  } while (dx.norm() > 1e-6 && iter < 10);
  //RCLCPP_INFO(ros_node_->get_logger(), "%f %f %f",_rec_ecef(0),_rec_ecef(1),_rec_ecef(2));
  // Q = (A.transpose()*A).inverse();
  // GDOP = math::sqrt(Q.trace());
  return iter < 10;
}


time_t GazeboRosMultipathSensorPrivate::MakeTimeUTC(const struct tm* timeinfo_utc)
{
	time_t curr_time = time(NULL);
	int timezone_diff = 0; //deviation of the current timezone from UTC in seconds

	//get UTC time, interpret resulting tm as a localtime
	struct tm timeinfo_gmt;
	gmtime_r(&curr_time, &timeinfo_gmt);
	time_t time_gmt = mktime(&timeinfo_gmt);

	//get localtime, interpret resulting tm as localtime
	struct tm timeinfo_local;
	localtime_r(&curr_time, &timeinfo_local);
	time_t time_local = mktime(&timeinfo_local);

	//find the time difference between the two interpretations
	timezone_diff += difftime(time_local, time_gmt);

	//hack for preventing mktime from assuming localtime: add timezone difference to the input struct.
	struct tm ret_timeinfo;
	ret_timeinfo.tm_sec = timeinfo_utc->tm_sec + timezone_diff;
	ret_timeinfo.tm_min = timeinfo_utc->tm_min;
	ret_timeinfo.tm_hour = timeinfo_utc->tm_hour;
	ret_timeinfo.tm_mday = timeinfo_utc->tm_mday;
	ret_timeinfo.tm_mon = timeinfo_utc->tm_mon;
	ret_timeinfo.tm_year = timeinfo_utc->tm_year;
	ret_timeinfo.tm_isdst = timeinfo_utc->tm_isdst;
	return mktime(&ret_timeinfo);
}

void GazeboRosMultipathSensorPrivate::ParseSatelliteTLE()
{
  num_sat_ = 8;
  RCLCPP_INFO(ros_node_->get_logger(), "Num sat:%d", num_sat_);
  // Initialization of data structures for satellite prediction 
  tle_lines_ = (const char**) calloc(2*num_sat_, sizeof(const char*));
  sat_orbit_elements_ = (predict_orbital_elements_t**) calloc(num_sat_, sizeof(predict_orbital_elements_t*));
  sat_position_ = (struct predict_position*) calloc(num_sat_, sizeof(struct predict_position));
  for ( int i = 0; i < 2*num_sat_; i++ )
  {
      tle_lines_[i] = (const char*) calloc(100, sizeof(const char));
  }
  for ( int i = 0; i < num_sat_; i++ )
  {
      sat_orbit_elements_[i] = (predict_orbital_elements_t*) calloc(1, sizeof( predict_orbital_elements_t));
  }
  //G01
  tle_lines_[0] = "1 37753U 11036A   23033.10118579 -.00000081  00000+0  00000+0 0  9997";
  tle_lines_[1] = "2 37753  56.6933  19.7748 0121088  52.9316 129.8426  2.00566862 84583";

  //G04
  tle_lines_[2] = "1 43873U 18109A   23032.57137684 -.00000044  00000+0  00000+0 0  9992";
  tle_lines_[3] = "2 43873  55.1375 140.8489 0023229 190.4825 204.0790  2.00558671 30373";

  //G07
  tle_lines_[4] = "1 32711U 08012A   23032.64592938  .00000056  00000+0  00000+0 0  9990";
  tle_lines_[5] = "2 32711  54.4610 199.0507 0168701 232.8355 125.5954  2.00572261109059";

  //G08
  tle_lines_[6] = "1 40730U 15033A   23033.72582779 -.00000076  00000+0  00000+0 0  9996";
  tle_lines_[7] = "2 40730  55.0217 317.3848 0082059  10.2474 349.9421  2.00566329 55333";

  //G09
  tle_lines_[8] = "1 40105U 14045A   23032.56334991 -.00000048  00000+0  00000+0 0  9995";
  tle_lines_[9] = "2 40105  54.7481 137.6538 0023243 114.6825 245.5467  2.00562646 61376";

  //G16
  tle_lines_[10] = "1 27663U 03005A   23033.62997381  .00000047  00000+0  00000+0 0  9997";
  tle_lines_[11] = "2 27663  55.3642 263.9131 0131905  42.8180 326.5456  2.00551168146626";

  //G21
  tle_lines_[12] = "1 27704U 03010A   23033.55723828 -.00000091  00000+0  00000+0 0  9994";
  tle_lines_[13] = "2 27704  55.0899  13.9998 0249685 312.4590 223.4527  2.00565951145429";

  //G27
  tle_lines_[14] = "1 39166U 13023A   23033.18918996 -.00000082  00000+0  00000+0 0  9995";
  tle_lines_[15] = "2 39166  55.5293 318.6885 0111059  38.9232 321.8895  2.00565200 71193";
  
  for ( int i = 0; i < num_sat_; i++ )
  {
      // Create orbit object
      sat_orbit_elements_[i] = predict_parse_tle(tle_lines_[2*i], tle_lines_[2*i+1]);
  }
}

void GazeboRosMultipathSensorPrivate::GetSatellitesInfo(struct tm *_timeinfo, std::vector<double> _rec_lla,
      std::vector<std::vector<double>> &_sat_ecef, std::vector<double> &_true_range, std::vector<double> &_azimuth,
      std::vector<double> &_elevation)
{
  // Predict the julian time using the current utc time
  predict_julian_date_t curr_time = predict_to_julian(MakeTimeUTC(_timeinfo));
  std::vector<double> ecef = {0,0,0};
  for ( int i = 0; i < num_sat_; i++ )
  {
    // Predict satellite's position based on the current time.
    predict_orbit(sat_orbit_elements_[i], &sat_position_[i], curr_time);
    g_geodetic_converter.geodetic2Ecef(sat_position_[i].latitude*180.0/M_PI, sat_position_[i].longitude*180.0/M_PI , 
                  sat_position_[i].altitude*1000 , &ecef[0], &ecef[1], &ecef[2]);
    predict_observer_t *obs = predict_create_observer("obs", _rec_lla[0]*M_PI/180.0, _rec_lla[1]*M_PI/180.0, _rec_lla[2]);
    struct predict_observation reciever_obs;
    predict_observe_orbit(obs, &sat_position_[i], &reciever_obs);
    _sat_ecef.push_back(ecef);
    _true_range.push_back(reciever_obs.range*1000); //From Km to m  
    _azimuth.push_back(reciever_obs.azimuth);
    _elevation.push_back(reciever_obs.elevation);
  }
}

// Register this plugin with the simulator
GZ_REGISTER_SENSOR_PLUGIN(GazeboRosMultipathSensor)
}  // namespace gazebo_plugins