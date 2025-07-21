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
// Modifications made by Jorge Estupiñan, 2025.
// - Updated GNSS logic to include multipath simulation using libpredict
// - Pass the plugin to Gazebo Harmonic with the Gazebo sim logic
// - Make the plugin universal so it can be used in any entity inside Gazebo sim
// - Use Lidars as the sensor as ray sensors are not inside Gazebo sim.
// - Use the actual date and time for the Satellite trajectory.

// Basic libraries:
#include <thread>
#include <memory>
#include <string>
#include <chrono>
#include <vector>
#include <cmath>
#include <ctime>
#include <cstring>
// ROS2 libraries:
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
// Include the header files:
#include "gnss_multipath_plugin/multipath_sensor.hpp"
// Transformation between reference frames for staellites:
#include "gnss_multipath_plugin/geodetic_conv.hpp"
// Custom messages:
#include "gnss_multipath_plugin/msg/gnss_multipath_fix.hpp"
// Gazebo Harmonic plugin integration:
#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/components/Pose.hh> 
#include <gz/transport/Node.hh>
#include <gz/msgs/pointcloud_packed.pb.h>
#include <gz/math/Quaternion.hh>
#include <gz/math/Vector3.hh>
#include <gz/math/Rand.hh>
// Library that predicts te satellites
#include <predict/predict.h>


// Start the plugin:
namespace gnss_multipath_plugin
{
  class GNSSMultipathPluginPrivate
  {
  public:
    // ROS2 NODES:
    // ROS Node used now to publish:
    rclcpp::Node::SharedPtr ros_node_;
    // Publisher of the Point Clouds coming form the gz Lidar-gpu:
    rclcpp::Publisher<gnss_multipath_plugin::msg::GNSSMultipathFix>::SharedPtr gnss_multipath_fix_publisher_;



    // GAZEBO NODES:
    // Define the lidar topic of Gazebo:
    std::string lidar_topic_;
    // Define the frame id where the Lidar is implemented:
    std::string frame_id_ = "lidar_link";
    // Start the Gazebo Node:
    gz::transport::Node gz_node_;
    // Save the Lidar information:
    struct LidarPointInfo{float range;float azimuth;float elevation;};
    std::vector<LidarPointInfo> lidar_data;

    


    // SATELLITE INFORMATION:
    // Origin latitude, longitude and altitude of the model:
    double origin_lat_, origin_lon_, origin_alt_;
    // Variable to add o eliminate noise on teh GNSS:
    bool disable_noise_;
    // Define a Goedetic converter to go from staellites position to enu:
    geodetic_converter::GeodeticConverter g_geodetic_converter;
    // Define the satellite grop characteristics;
    int num_sat_; // number of satellites
    float elevation_limit_; //Limit in the elvation to see if a satellite is not visible 
    std::vector<std::string> tle_lines_; // TlE parameters to store the satellite's orbital parameters
    predict_orbital_elements_t **sat_orbit_elements_; // Satellite orbital parameters to predict satellite's position
    struct predict_position *sat_position_; // Satellite position information at a given time
    // Obtains reciver actual position from Gazebo:
    gz::sim::Entity model_entity_{gz::sim::kNullEntity}; // entity hnadler for the model
    gz::sim::Model model_{gz::sim::kNullEntity}; // Model characteritics
    gz::math::Pose3d current_pose_; // Save the current pose of the entity:
    // Time information for the satellite position:
    struct tm *timeinfo_;



    // IMPORTANT PRIVATE FUNCTIONS:
    // Transform from gazebo message to ROS2 message and Use it to geenrate tha azimuth, range and elevation of the Lidar detections:
    void OnLidarPoints(const gz::msgs::PointCloudPacked &pc_msg);
    // Get the satellite information:
    void ParseSatelliteTLE();
    // Obtein the relative aziuth and elvation from the Lidar to the satellites in the world frame direction:
    void GetSatellitesInfo(struct tm *_timeinfo, std::vector<double> _rec_lla,
      std::vector<std::vector<double>> &_sat_ecef, std::vector<double> &_true_range, std::vector<double> &_azimuth,
      std::vector<double> &_elevation);
    // Change time to UTC:
    time_t MakeTimeUTC(const struct tm* timeinfo_utc);
    // Check whcih satellites are visible:
    std::tuple<bool, bool, double> IsSatelliteVisible(double sat_az, double sat_elev,double az_span, double el_span);
    // Get REciever position using Least Squares:
    bool GetLeastSquaresEstimate(std::vector<double> _meas,std::vector<std::vector<double>> _sat_ecef, Eigen::Vector3d &_rec_ecef);
    // Calculate the dilution of precission of teh GNSS (DOP):
    void CalculateDOP(std::vector<std::vector<double>> _sat_ecef, Eigen::Vector3d _rec_ecef, std::vector<double> &_dop);
  };


  // Constructure
  GNSSMultipathPlugin::GNSSMultipathPlugin():impl_(std::make_unique<GNSSMultipathPluginPrivate>())
  {
  }


  // Define the Configure section of the plugin:
  void GNSSMultipathPlugin::Configure(const gz::sim::Entity &_entity,const std::shared_ptr<const sdf::Element> &_sdf, gz::sim::EntityComponentManager &, gz::sim::EventManager &)
  {
    // Obtain the eniity where the plugin is applied:
    this->impl_->model_entity_ = _entity;
    this->impl_->model_ = gz::sim::Model(_entity);
    // Check if ROS2 is started:
    if (!rclcpp::ok()){
      return;
    }
    // Start the ROS2 Node:
    impl_->ros_node_ = std::make_shared<rclcpp::Node>("gnss_lidar_node");
    // Make the ROS2 Node to spin:
    std::thread([node = impl_->ros_node_]() {
            rclcpp::executors::SingleThreadedExecutor exec;
            exec.add_node(node);
            exec.spin();
        }).detach();



    // Obtain varibles from the SDF:
    // Obtian the frame id from the sdf:
    if (_sdf->HasElement("frame_id")){
      impl_->frame_id_ = _sdf->Get<std::string>("frame_id");
    }
    // Obtain the topic where gazebo publsihes the lidar information:
    if (_sdf->HasElement("topic")){
      impl_->lidar_topic_ = _sdf->Get<std::string>("topic");
    }
    else{
      impl_->lidar_topic_ = "/lidar/scan";  // default topic
    }
    // Obtein the origin latitude, longitude and altitude
    if (_sdf->HasElement("origin_latitude")){
      std::string origin_lat = _sdf->Get<std::string>("origin_latitude");
      std::istringstream ss(origin_lat);
      double origin_lat_;
      ss >> origin_lat_;
      impl_->origin_lat_ = origin_lat_;
    }
    if (_sdf->HasElement("origin_longitude")){
      std::string origin_lon = _sdf->Get<std::string>("origin_longitude");
      std::istringstream ss(origin_lon);
      double origin_lon_;
      ss >> origin_lon_;
      impl_->origin_lon_ = origin_lon_;
    }
    if (_sdf->HasElement("origin_altitude")){
      std::string origin_alt = _sdf->Get<std::string>("origin_altitude");
      std::istringstream ss(origin_alt);
      double origin_alt_;
      ss >> origin_alt_;
      impl_->origin_alt_ = origin_alt_;
    }
    // Check if the noise is enabeled:
    if (_sdf->HasElement("disableNoise")){
      impl_->disable_noise_ = _sdf->Get<bool>("disableNoise");
    }
    // Define the maximum elveation same as the lidar:
    if (_sdf->HasElement("ElevationLimit")){
      std::string elevation_lim = _sdf->Get<std::string>("ElevationLimit");
      std::istringstream ss(elevation_lim);
      double elevation_limit_;
      ss >> elevation_limit_;
      impl_->elevation_limit_ = elevation_limit_;
    }



    // Constructing a time structure for getting satellites position
    std::time_t now = std::time(nullptr);
    impl_->timeinfo_ = std::gmtime(&now);



    // Initialize the converter with the geodetic location of the origin.
    impl_->g_geodetic_converter.initialiseReference(impl_->origin_lat_, impl_->origin_lon_, impl_->origin_alt_);
    // Open the satellite information:
    impl_->ParseSatelliteTLE();
    // Start the publisher of the ROS2 PointClouds:
    impl_->gnss_multipath_fix_publisher_ = impl_->ros_node_->create_publisher<gnss_multipath_plugin::msg::GNSSMultipathFix>("gnss_multipath_fix", 10); 
    // Start the subscriber of the gz_node of gpu-lidar:
    impl_->gz_node_.Subscribe(impl_->lidar_topic_,std::function<void(const gz::msgs::PointCloudPacked&)>([this](const gz::msgs::PointCloudPacked &msg){
      this->impl_->OnLidarPoints(msg);
    }));
  }



  // Post update function to interact with models in Gazebo at runtime:
  void GNSSMultipathPlugin::PostUpdate(const gz::sim::UpdateInfo &_info,const gz::sim::EntityComponentManager &_ecm)
  {
    (void)_info;
    // CHeck if the model exists:
    if (impl_->model_entity_ == gz::sim::kNullEntity){
      return;
    }
    // Get the position of the entity:
    auto pose_comp = _ecm.Component<gz::sim::components::Pose>(impl_->model_entity_);
    // Check if pose exists:
    if (!pose_comp){
      return;
    }
    // Save the position in the current position:
    impl_->current_pose_ = pose_comp->Data();
    
  }

  

  // Callback function of the Gazebo Gpu-LIdar subscriber And Use it to generate the elevation and azimuth of the lidar detections:
  void GNSSMultipathPluginPrivate::OnLidarPoints(const gz::msgs::PointCloudPacked &pc_msg)
  {
    
    // OBTAIN THE INFORMATION TO GET THE ELEVATION AND AZIMUTH:
    // Clear previous data
    this->lidar_data.clear();

    // Define the offset from the msg:
    int x_offset = -1, y_offset = -1, z_offset = -1;
    for (int i = 0;i<pc_msg.field_size();++i){
      const auto &field = pc_msg.field(i);
      if (field.name() == "x") x_offset = field.offset();
      if (field.name() == "y") y_offset = field.offset();
      if (field.name() == "z") z_offset = field.offset();
    }

    // Return in case the x, y, z fileds are empty:
    if (x_offset < 0 || y_offset < 0 || z_offset < 0){
      RCLCPP_ERROR(this->ros_node_->get_logger(), "Missing x, y, or z fields in PointCloudPacked");
      return;
    }

    // Obtain the x,y,z position iterationg from the Point Clouds:
    const int point_step = pc_msg.point_step();
    const int num_points = pc_msg.width() * pc_msg.height();
    const auto &data = pc_msg.data();
    float horizontal_step = 2 * M_PI / static_cast<float>(pc_msg.width());
    float vertical_step = elevation_limit_ /static_cast<float>(pc_msg.height());
    // Traformation from the Lidar frame to the world frame:
    gz::math::Quaterniond orientation = this->current_pose_.Rot();
    // Varibles to speed the visible satellite identification:
    float max_elevation_collected = 0;
    float min_azimuth_collected = 2 * M_PI;
    float max_azimuth_collected = 0;
    int number_of_collected_rays = 0;
    for (int i = 0; i< num_points; ++i){
      // Get the specific ith point:
      const unsigned char *point_ptr = reinterpret_cast<const unsigned char *>(&data[i * point_step]);
      // Extract position data from the lidar link frame:
      float x = *reinterpret_cast<const float *>(point_ptr + x_offset);
      float y = *reinterpret_cast<const float *>(point_ptr + y_offset);
      float z = *reinterpret_cast<const float *>(point_ptr + z_offset);
      // Check if any value is Nan (The ray doesn't find an structure):
      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)){
        continue;
      }
      // Pass these coordinates to the world frame:
      gz::math::Vector3d position_lidar(x,y,z);
      gz::math::Vector3d position_world = orientation.RotateVector(position_lidar);
      float x1 = position_world.X();
      float y1 = position_world.Y();
      float z1 = position_world.Z();
      // Obtain the range value:
      float range = std::sqrt(x1*x1 + y1*y1 + z1*z1);
      // Check if any of the ranges are 0 (CRASH):
      if (range == 0.0f){
        continue;
      }
      // Determine the azimuth and elevation 
      float azimuth = std::atan2(y1,x1);
      float elevation = std::asin(z1/range);
      if (azimuth < 0){
        azimuth += 2 * M_PI;
      }
      // Define the limits of the Lidar readings:
      if (elevation > max_elevation_collected){
        max_elevation_collected = elevation;
      }
      if (azimuth > max_azimuth_collected){
        max_azimuth_collected = azimuth;
      }
      if (azimuth < min_azimuth_collected){
        min_azimuth_collected = azimuth;
      }
      number_of_collected_rays += 1;
      // Save the informaion in the vector:vis_num_sat_
      this->lidar_data.push_back({range,azimuth,elevation});
    }



    
    // OBTAIN THE SATELLITES ELVATION ANS AZIMUTH:
    // Genrate the vectors that contain the information:
    std::vector<double> sat_true_range_;
    std::vector<std::vector<double>> sat_ecef_;
    std::vector<double> elevation_;
    std::vector<double> azimuth_; 
    std::vector<double> rec_lla_{0,0,0};
    // Save the satellite position and distance:
    std::vector<double> visible_sat_range_meas;
    std::vector<std::vector<double>> visible_sat_ecef;
    // Obtain the position of the reciever: 
    gz::math::Vector3d pos = this->current_pose_.Pos();
    // Reciever's position in geodetic (LLA) coordinates
    this->g_geodetic_converter.enu2Geodetic(pos.X(), pos.Y(), pos.Z(),&rec_lla_[0], &rec_lla_[1], &rec_lla_[2]);
    // Collect positive elevation satellites ECEF, true ranges and angles 
    GetSatellitesInfo(timeinfo_,rec_lla_, sat_ecef_, sat_true_range_, azimuth_, elevation_ );
    // number fo visible satellites
    int starting_vis_sat_ = sat_true_range_.size(); //Overall
    int vis_num_sat_ =  0;//Using the multipath
    // Check which satellites are really visible:
    for (int i = 0;i < starting_vis_sat_;i++){
      double sat_az = std::fmod(azimuth_[i] + 2 * M_PI, 2 * M_PI);
      double sat_elev = elevation_[i];
      // Add the noise in case is not disabled
      double noise_ = 0.0;
      if (!disable_noise_)
      {  
          // White noise
          noise_ = gz::math::Rand::DblNormal(0.0, 1.0);
      }
      // See is the satellite are above the elevation limit:
      if (sat_elev > (max_elevation_collected + vertical_step / 2.0)){
        visible_sat_range_meas.push_back(sat_true_range_[i] + noise_);
        visible_sat_ecef.push_back(sat_ecef_[i]);
        vis_num_sat_ += 1;
      }
      else{
        // Determine if the asmiuth is inside the detection limits:
        if (sat_az <= max_azimuth_collected+horizontal_step/2 && sat_az >= min_azimuth_collected-horizontal_step/2){
          // Check if there are structures to reflect the gps rays:
          if (number_of_collected_rays < 2){
            continue;
          }
          // Analize if the satellite signal crash with a structure:
          auto [is_obstructed, is_reflected, offset] = this->IsSatelliteVisible(sat_az,sat_elev,horizontal_step,vertical_step);
          // Check if the satellite is not obstructed:
          if (!is_obstructed){
            visible_sat_range_meas.push_back(sat_true_range_[i] + noise_);
            visible_sat_ecef.push_back(sat_ecef_[i]);
            vis_num_sat_ += 1;
          }
          // Check is the satellite is mirroed:
          else if (is_reflected){
            visible_sat_range_meas.push_back(sat_true_range_[i] + offset + noise_);
            visible_sat_ecef.push_back(sat_ecef_[i]);
            vis_num_sat_ += 1;
          }
        }
        else{
          visible_sat_range_meas.push_back(sat_true_range_[i] + noise_);
          visible_sat_ecef.push_back(sat_ecef_[i]);
          vis_num_sat_ += 1;
        }
      }
    }




    //OBTAIN TEH POSITION OF THE RECIEVER:
    // Generate teh message that is going to be published in ROS2:
    gnss_multipath_plugin::msg::GNSSMultipathFix gnss_multipath_fix_msg;
    gnss_multipath_fix_msg.header.stamp = this->ros_node_->get_clock()->now();
    gnss_multipath_fix_msg.header.frame_id = this->frame_id_;
    gnss_multipath_fix_msg.navsatfix.header.stamp = this->ros_node_->get_clock()->now();
    gnss_multipath_fix_msg.navsatfix.header.frame_id = this->frame_id_;
    gnss_multipath_fix_msg.enu_true.resize(3);
    gnss_multipath_fix_msg.enu_gnss_fix.resize(3);
    gnss_multipath_fix_msg.dop.resize(5);
    // Find the position using REcursive Least Squares:
    if (vis_num_sat_ > 3){
      Eigen::Vector3d rec_ecef(0,0,0);  
      GetLeastSquaresEstimate(visible_sat_range_meas,  visible_sat_ecef, rec_ecef);
      // Calculate thte DOP (Dilution of Precision.):
      std::vector<double> dop;
      CalculateDOP(visible_sat_ecef, rec_ecef, dop);
      // Obtain the positions calcualted and real:
      double enu_x,enu_y,enu_z, latitude=0, longitude=0, altitude=0;
      g_geodetic_converter.ecef2Geodetic(rec_ecef(0), rec_ecef(1),rec_ecef(2), &latitude,&longitude, &altitude);
      g_geodetic_converter.geodetic2Enu(latitude,longitude, altitude, &enu_x,&enu_y,&enu_z);
      gnss_multipath_fix_msg.navsatfix.status.status = 1;
      gnss_multipath_fix_msg.navsatfix.latitude = latitude;
      gnss_multipath_fix_msg.navsatfix.longitude = longitude;
      gnss_multipath_fix_msg.navsatfix.altitude = altitude;
      gnss_multipath_fix_msg.enu_gnss_fix[0] = enu_x;
      gnss_multipath_fix_msg.enu_gnss_fix[1] = enu_y;
      gnss_multipath_fix_msg.enu_gnss_fix[2] = enu_z;

      //Copy DOP values to the message.
      std::copy(dop.begin(), dop.end(),gnss_multipath_fix_msg.dop.begin());
    }
    else{
      gnss_multipath_fix_msg.navsatfix.status.status = 0;
      gnss_multipath_fix_msg.enu_gnss_fix[0] = NAN;
      gnss_multipath_fix_msg.enu_gnss_fix[1] = NAN;
      gnss_multipath_fix_msg.enu_gnss_fix[2] = NAN;
    }
    // Save the real ENU location too:
    gnss_multipath_fix_msg.enu_true[0] = pos.X();
    gnss_multipath_fix_msg.enu_true[1] = pos.Y();
    gnss_multipath_fix_msg.enu_true[2] = pos.Z();
    gnss_multipath_fix_msg.visible_sat_count = vis_num_sat_;
    gnss_multipath_fix_publisher_->publish(gnss_multipath_fix_msg);




  }




  // Define the Satlleite net and trajecotires:
  void GNSSMultipathPluginPrivate::ParseSatelliteTLE()
  {
    num_sat_ = 50;
    RCLCPP_INFO(ros_node_->get_logger(), "Num sat: %d", num_sat_);

    // Initialize predict memory
    tle_lines_.clear();
    sat_orbit_elements_ = (predict_orbital_elements_t **)calloc(num_sat_, sizeof(predict_orbital_elements_t *));
    sat_position_ = (struct predict_position *)calloc(num_sat_, sizeof(struct predict_position));

    // Add all TLE line pairs
    tle_lines_ = {
      "1 24876U 97035A   23033.20764968 -.00000038  00000+0  00000+0 0  9994",
      "2 24876  55.5459 146.7067 0065794  52.6664 307.9046  2.00564086187267", // Satellite 1

      "1 26360U 00025A   23033.06431156 -.00000048  00000+0  00000+0 0  9992",
      "2 26360  54.2496  69.3979 0045611 194.8807 151.7827  2.00555919166578", // Satellite 2

      "1 27663U 03005A   23033.62997381  .00000047  00000+0  00000+0 0  9997",
      "2 27663  55.3642 263.9131 0131905  42.8180 326.5456  2.00551168146626", // Satellite 3

      "1 27704U 03010A   23033.55723828 -.00000091  00000+0  00000+0 0  9994",
      "2 27704  55.0899  13.9998 0249685 312.4590 223.4527  2.00565951145429", // Satellite 4

      "1 28190U 04009A   23034.15509573 -.00000085  00000+0  00000+0 0  9992",
      "2 28190  55.8516 324.9248 0088848 128.9769  57.1221  2.00572247138304", // Satellite 5

      "1 28474U 04045A   23033.60153542 -.00000090  00000+0  00000+0 0  9993",
      "2 28474  55.4150  14.2264 0202589 283.4591  74.0418  2.00562393133746", // Satellite 6

      "1 28874U 05038A   23033.61665581 -.00000084  00000+0  00000+0 0  9995",
      "2 28874  55.9094 322.3891 0139570 278.1790 267.2996  2.00560021127153", // Satellite 7

      "1 29486U 06042A   23033.78720829  .00000046  00000+0  00000+0 0  9993",
      "2 29486  54.6970 200.1224 0106835  27.4847 197.4334  2.00565983119862", // Satellite 8

      "1 29601U 06052A   23032.92873802  .00000048  00000+0  00000+0 0  9990",
      "2 29601  55.3786 262.8889 0086684  75.9810 285.0549  2.00564342118716", // Satellite 9

      "1 32260U 07047A   23033.23014611 -.00000051  00000+0  00000+0 0  9995",
      "2 32260  53.3839 131.0174 0145380  67.1246 294.4003  2.00563380112126", // Satellite 10

      "1 32384U 07062A   23033.85587446 -.00000084  00000+0  00000+0 0  9994",
      "2 32384  56.0351 323.1715 0020102 142.8969  77.6124  2.00574229110895", // Satellite 11

      "1 32711U 08012A   23032.64592938  .00000056  00000+0  00000+0 0  9990",
      "2 32711  54.4610 199.0507 0168701 232.8355 125.5954  2.00572261109059", // Satellite 12

      "1 35752U 09043A   23034.15624287 -.00000035  00000+0  00000+0 0  9992",
      "2 35752  55.2549  76.2529 0055618  63.8864 319.4178  2.00571659 98673", // Satellite 13

      "1 36585U 10022A   23033.21070384  .00000056  00000+0  00000+0 0  9997",
      "2 36585  54.6695 258.2376 0109114  58.2673 124.2846  2.00550759 92904", // Satellite 14

      "1 37753U 11036A   23033.10118579 -.00000081  00000+0  00000+0 0  9997",
      "2 37753  56.6933  19.7748 0121088  52.9316 129.8426  2.00566862 84583", // Satellite 15

      "1 38833U 12053A   23033.33039722  .00000038  00000+0  00000+0 0  9995",
      "2 38833  53.5190 193.9407 0135373  49.9651 311.1852  2.00564336 74752", // Satellite 16

      "1 39166U 13023A   23033.18918996 -.00000082  00000+0  00000+0 0  9995",
      "2 39166  55.5293 318.6885 0111059  38.9232 321.8895  2.00565200 71193", // Satellite 17

      "1 39533U 14008A   23033.19159522  .00000049  00000+0  00000+0 0  9992",
      "2 39533  53.6091 199.4781 0062019 209.5871 150.0417  2.00566418 65548", // Satellite 18

      "1 39741U 14026A   23033.96686822 -.00000082  00000+0  00000+0 0  9994",
      "2 39741  56.6508  19.2656 0030744 308.2184  51.5125  2.00572503 63854", // Satellite 19

      "1 40105U 14045A   23032.56334991 -.00000048  00000+0  00000+0 0  9995",
      "2 40105  54.7481 137.6538 0023243 114.6825 245.5467  2.00562646 61376", // Satellite 20

      "1 40294U 14068A   23032.43441650 -.00000049  00000+0  00000+0 0  9997",
      "2 40294  55.9964  78.9035 0042465  56.8096 303.6631  2.00557360 60507", // Satellite 21

      "1 40534U 15013A   23033.06804744  .00000058  00000+0  00000+0 0  9991",
      "2 40534  53.5692 255.1758 0077227  23.8247 336.6011  2.00567871 57134", // Satellite 22

      "1 40730U 15033A   23033.72582779 -.00000076  00000+0  00000+0 0  9996",
      "2 40730  55.0217 317.3848 0082059  10.2474 349.9421  2.00566329 55333", // Satellite 23

      "1 41019U 15062A   23033.28107185 -.00000041  00000+0  00000+0 0  9996",
      "2 41019  55.9834  78.7168 0085778 220.7636 138.6587  2.00562556 53141", // Satellite 24

      "1 41328U 16007A   23033.36788295 -.00000045  00000+0  00000+0 0  9999",
      "2 41328  54.9593 138.3485 0067345 231.5397 127.8428  2.00559919 51156", // Satellite 25

      "1 43873U 18109A   23032.57137684 -.00000044  00000+0  00000+0 0  9992",
      "2 43873  55.1375 140.8489 0023229 190.4825 204.0790  2.00558671 30373", // Satellite 26

      "1 44506U 19056A   23033.40051584 -.00000083  00000+0  00000+0 0  9998",
      "2 44506  55.7692  19.9151 0029851 186.7930 347.2623  2.00561526 25382", // Satellite 27

      "1 46826U 20078A   23032.06127720  .00000047  00000+0  00000+0 0  9997",
      "2 46826  54.4254 260.8119 0026417 188.1073  16.6882  2.00564630 16797", // Satellite 28

      "1 48859U 21054A   23033.87596315 -.00000082  00000+0  00000+0 0  9990",
      "2 48859  55.2812  21.7836 0009051 219.9340  39.0694  2.00560997 12067", // Satellite 29

      "1 55268U 23009A   23033.46134089  .00000045  00000+0  00000+0 0  9993",
      "2 55268  55.1007 196.8026 0008185  97.8399 262.2308  2.00570647   571", // Satellite 30

      "1 45854U 20041A   23034.26264002 -.00000033  00000+0  00000+0 0  9992",
      "2 45854  55.7045  77.1516 0029948 185.9714 192.1492  2.00567044 19363", // Satellite 31

      "1 25544U 98067A   23336.68982639  .00000705  00000+0  20032-4 0  9990", // ISS (ZARYA) Satellite 32
      "2 25544  51.6458 312.3924 0006825  45.6224  86.8994 15.50176866302416",

      "1 33591U 09005A   23336.78231046  .00000022  00000+0  00000+0 0  9991", // COSMOS 2441 Satellite 33
      "2 33591  65.8835  10.6343 0015421 212.0573 319.0167 13.71492753204816",

      "1 43013U 17073A   23336.68968102  .00001320  00000+0  17721-3 0  9996", // FENGYUN 3D Satellite 34
      "2 43013  98.7671  22.3744 0010496  60.3647 299.8618 14.22169261292645",

      "1 43014U 17073B   23336.68403268  .00001134  00000+0  16483-3 0  9994", // FENGYUN 3E Satellite 35
      "2 43014  98.7701  22.3847 0010669  58.2738 301.9546 14.22256701292678",

      "1 25338U 98030A   23336.58333612 -.00000047  00000+0  00000+0 0  9996", // NOAA 15 Satellite 36
      "2 25338  98.7363 345.6201 0010145  47.2243 312.9661 14.25842157379948",

      "1 28654U 05018A   23336.72564822  .00000091  00000+0  00000+0 0  9998", // GPS BIIR-12 Satellite 37
      "2 28654  55.0495 112.3124 0025531 156.0256 204.1698  2.00558564 69498",

      "1 29486U 06042A   23336.58487498  .00000245  00000+0  00000+0 0  9993", // GALILEO 1 Satellite 38
      "2 29486  56.7452 134.5862 0001234 156.2487  56.9785  1.70477253 84314",

      "1 43226U 18011A   23336.57623611  .00000042  00000+0  00000+0 0  9994", // O3B F4 - 7 39 Satellites Satellite 39
      "2 43226  0.0412 164.6235 0000235  95.2137 261.3258  1.00270012 21067",

      "1 37820U 11042A   23336.61869874 -.00000047  00000+0  00000+0 0  9993", // SES 3 - 8 40 Satellites Satellite 40
      "2 37820  0.0473 268.0842 0003035 193.1284 166.8629  1.00270002 44459",

      "1 37843U 11055A   23336.58663952 -.00000120  00000+0  00000+0 0  9999", // ECHOSTAR 17 - 9  Satellite 41
      "2 37843  0.0112  35.1122 0002091 221.0164  62.3124  1.00270334 44164",

      "1 28884U 05042A   23336.59899847  .00000076  00000+0  00000+0 0  9994", // FENGYUN 1D  Satellite 42
      "2 28884  98.7543 201.4762 0000123 165.0474 195.2548 14.25743222499999",

      "1 37214U 10063A   23336.58262118  .00000121  00000+0  00000+0 0  9996", // METOP-B  Satellite 43
      "2 37214  98.7446 211.3456 0002134  47.2564 313.5761 14.21403416257473",

      "1 43010U 17071A   23336.57936790  .00000092  00000+0  00000+0 0  9990", // SENTINEL-5P  Satellite 44
      "2 43010  97.6261 200.2365 0001523  45.2474 314.7512 14.19812367457891",

      "1 43613U 18091A   23336.57658938  .00000113  00000+0  00000+0 0  9994", // METEOSAT 11  Satellite 45
      "2 43613  98.7426 345.3684 0001235 156.2875 303.0985 14.21469201288847",

      "1 24208U 96056A   23336.59899365 -.00000056  00000+0  00000+0 0  9999", // RADARSAT-1  Satellite 46
      "2 24208  98.5875 345.2112 0000127 356.3156  10.1326 14.19843398247563",

      "1 41822U 16059A   23336.62343650 -.00000023  00000+0  00000+0 0  9990", // NOAA 20  Satellite 47
      "2 41822  98.7394 345.4693 0000154  52.1873 307.7654 14.25844198496873",

      "1 43012U 17072A   23336.56893665 -.00000034  00000+0  00000+0 0  9995", // JPSS-1  Satellite 48
      "2 43012  98.7395 214.4783 0000124  25.2564 334.8676 14.20483763245648",

      "1 27424U 02022A   23336.61712693 -.00000014  00000+0  00000+0 0  9998", // AQUA  Satellite 49
      "2 27424  98.2014 345.1122 0002214 356.2316   3.7499 14.56923289954738",

      "1 25544U 98067A   23336.57396889 -.00000123  00000+0  00000+0 0  9993", // ISS Satellite 50
      "2 25544  51.6407  42.0623 0001422 259.2353  98.2229 15.49373111404088",

    };

    if (tle_lines_.size() != static_cast<size_t>(num_sat_ * 2)) {
        RCLCPP_ERROR(ros_node_->get_logger(),
        "TLE line count mismatch: expected %d lines, got %zu",
        num_sat_ * 2, tle_lines_.size());
      return;
    }

    for (int i = 0; i < num_sat_; ++i) {
      sat_orbit_elements_[i] = predict_parse_tle(tle_lines_[2 * i].c_str(), tle_lines_[2 * i + 1].c_str());
    }
  }




  // Function to get the satellite information of the azimuth and elevation:
  void GNSSMultipathPluginPrivate::GetSatellitesInfo(struct tm *_timeinfo, std::vector<double> _rec_lla,
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
      if (reciever_obs.elevation > 0)
      {
        _sat_ecef.push_back(ecef);
        _true_range.push_back(reciever_obs.range*1000); //From Km to m  
        _azimuth.push_back(reciever_obs.azimuth);
        _elevation.push_back(reciever_obs.elevation);
      }
      
    }
  }


  // Change time format tot eh UTC format:
  time_t GNSSMultipathPluginPrivate::MakeTimeUTC(const struct tm* timeinfo_utc)
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

  


  // Dtect if a obstructed satellite has a mirroed multipath offset
  std::tuple<bool, bool, double> GNSSMultipathPluginPrivate::IsSatelliteVisible(double sat_az, double sat_elev,double az_span, double el_span)
  {
    auto angular_diff = [](double a1, double a2) -> double {
      double diff = std::fmod(a1 - a2 + 3 * M_PI, 2 * M_PI) - M_PI;
      return std::abs(diff);
    };

    bool obstructed = false;
    bool has_mirror = false;
    double offset = 0.0;
    double mirror_az = std::fmod(sat_az + M_PI, 2 * M_PI);

    for (const auto &ray : this->lidar_data)
    {
      // Check if the satellite crashes with a structure (matched by ray)
      if (std::abs(sat_elev - ray.elevation) <= el_span / 2.0){
        if (angular_diff(sat_az, ray.azimuth) <= az_span / 2.0)
        {
          obstructed = true;
        }

        // Check if the mirrored azimuth is present
        if (angular_diff(mirror_az, ray.azimuth) <= az_span / 2.0)
        {
          has_mirror = true;
          offset = ray.range * (1.0 + std::sin(M_PI / 2.0 - 2.0 * ray.elevation));
        }
      }
    }

    // Interpret result based on flags
    if (!obstructed)
      return {false, false, 0.0};  // No crash → visible LoS

    if (has_mirror)
      return {true, true, offset};  // Obstructed but reflected

    return {true, false, 0.0};  // Obstructed and no reflection
  }




  // Get the position of teh reciever using recursive Least Squares:
  bool GNSSMultipathPluginPrivate::GetLeastSquaresEstimate(std::vector<double> _meas,std::vector<std::vector<double>> _sat_ecef, Eigen::Vector3d &_rec_ecef)
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
    return iter < 10;
  }



  // Calculate teh Dilution of presiccion of the Reciever psoition form the GNSS
  void GNSSMultipathPluginPrivate::CalculateDOP(std::vector<std::vector<double>> _sat_ecef, Eigen::Vector3d _rec_ecef, std::vector<double> &_dop)
  {
    const int nsat = _sat_ecef.size();
  
    Eigen::MatrixXd A, Q, Q_local;
    A.resize(nsat, 4);
    for (int i = 0 ; i < nsat; i++)
    {
      Eigen::Vector3d sat_pos(_sat_ecef[i][0], _sat_ecef[i][1], _sat_ecef[i][2]);
      A.block<1,3>(i,0) = (_rec_ecef - sat_pos).normalized();
      A(i,3) = -1;
    }
    Q = (A.transpose()*A).inverse();
    //GDOP
    _dop.push_back(std::sqrt(Q.trace()));
    //PDOP
    _dop.push_back(std::sqrt((Q.block<3,3>(0,0)).trace()));
    //TDOP
    _dop.push_back(std::sqrt(Q(3,3)));
    double latitude = 0.0, longitude=0.0, altitude=0.0;
    g_geodetic_converter.ecef2Geodetic(_rec_ecef(0), _rec_ecef(1),_rec_ecef(2), &latitude,
                        &longitude, &altitude);
    double phi = latitude; 			
    double lambda = longitude;	

    double sl = sin(lambda);
    double cl = cos(lambda);
    double sp = sin(phi);
    double cp = cos(phi);
    
    Eigen::MatrixXd Rot_matrix(4, 4);
    Rot_matrix(0, 0) = -cl*sp;
    Rot_matrix(0, 1) = -sl*sp;
    Rot_matrix(0, 2) = cp;
    Rot_matrix(0, 3) = 0;

    Rot_matrix(1, 0) = -sl;
    Rot_matrix(1, 1) = cl;
    Rot_matrix(1, 2) = 0;
    Rot_matrix(1, 3) = 0;

    Rot_matrix(2, 0) = cl*cp;
    Rot_matrix(2, 1) = cp*sl;
    Rot_matrix(2, 2) = sp;
    Rot_matrix(2, 3) = 0;

    Rot_matrix(3, 0) = 0;
    Rot_matrix(3, 1) = 0;
    Rot_matrix(3, 2) = 0;
    Rot_matrix(3, 3) = 1;

    // calculate the local cofactor matrix
    Q_local = Rot_matrix*Q*Rot_matrix.transpose();
    // calculate 'HDOP' and 'VDOP' 
    // HDOP
    _dop.push_back(std::sqrt(Q_local(0,0)*Q_local(1,1)));
    // VDOP
    _dop.push_back(std::sqrt(Q_local(2,2)));
    //RCLCPP_INFO(ros_node_->get_logger(), "gdop:%f pdop:%f tdop:%f hdop:%f vdop:%f", _dop[0],_dop[1],_dop[2],_dop[3], _dop[4]);
  }

}


GZ_ADD_PLUGIN(
  gnss_multipath_plugin::GNSSMultipathPlugin,
  gz::sim::System,
  gz::sim::ISystemConfigure,
  gz::sim::ISystemPostUpdate)

GZ_ADD_PLUGIN_ALIAS(gnss_multipath_plugin::GNSSMultipathPlugin, "gnss_lidar_plugin")
