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

#include <multipath_sim/multipath_sensor.hpp>
#include "multipath_sim/msg/multipath_offset.hpp"  

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
  //physics::WorldPtr world_;
  //physics::EntityPtr parent_entity_;

  int num_sat_;
  bool disable_noise_;
  std::vector<double> sat_dir_elevation_;
  std::vector<double> sat_dir_azimuth_;
  void setRayAngles(std::vector<double> &ray_ranges); 
  
  rclcpp::Publisher<multipath_sim::msg::MultipathOffset>::SharedPtr offset_publisher_;
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
  // Create ros_node configured from sdf
  impl_->ros_node_ = gazebo_ros::Node::Get(_sdf);
  impl_->parent_ray_sensor_ =
    std::dynamic_pointer_cast<gazebo::sensors::RaySensor>(_sensor);
  std::string worldName = _sensor->WorldName();
  impl_->world_ = gazebo::physics::get_world(worldName);
  std::string parent_entity_name = impl_->parent_ray_sensor_->ParentName();
  impl_->parent_entity_ = impl_->world_->EntityByName(parent_entity_name);
  
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

  if (!_sdf->HasElement("satNum"))
  {
    RCLCPP_DEBUG(impl_->ros_node_->get_logger(), "missing <satNum>, defaults to 12");
    impl_->num_sat_ = 12;
  }
  else{
    impl_->num_sat_ = _sdf->Get<int>("satNum");
  }
  // Allocate enough memory for the reflection rays
  impl_->sat_dir_azimuth_.resize(impl_->num_sat_ * 2);
  impl_->sat_dir_elevation_.resize(impl_->num_sat_ * 2);

  if(!_sdf->HasElement("satAzimuth"))
  {
    RCLCPP_DEBUG(impl_->ros_node_->get_logger(),  "Must provide azimuth of satellites");
  }
  else
  {
    std::string sat_az = _sdf->Get<std::string>("satAzimuth");
    std::istringstream ss(sat_az);
    for (int i = 0; i < impl_->num_sat_; i++) {
      double sat_az_i;
      ss >> sat_az_i;
      // direct and reflection as an adjacent pair in the vector
      impl_->sat_dir_azimuth_[i*2] = sat_az_i;
      impl_->sat_dir_azimuth_[i*2+1] = sat_az_i + M_PI; // assume reflection is 180 deg azimuth from the direct ray
    }
  }

  if(!_sdf->HasElement("satElevation"))
  {
    RCLCPP_DEBUG(impl_->ros_node_->get_logger(),  "Must provide elevation of satellites");
  }
  else
  {
    std::string sat_el = _sdf->Get<std::string>("satElevation");
    std::istringstream ss(sat_el);
    for (int i = 0; i < impl_->num_sat_; i++) {
      double sat_el_i;
      ss >> sat_el_i;
      impl_->sat_dir_elevation_[i*2] = sat_el_i;
      impl_->sat_dir_elevation_[i*2+1] = sat_el_i; // assume reflection has the same elevation as the direct ray
    }
  }

  // Create gazebo transport node and subscribe to sensor's laser scan
  impl_->gazebo_node_ = boost::make_shared<gazebo::transport::Node>();
  impl_->gazebo_node_->Init(_sensor->WorldName());

  // TODO(ironmig): use lazy publisher to only process laser data when output has a subscriber
  impl_->sensor_topic_ = _sensor->Topic();
  impl_->offset_publisher_ = impl_->ros_node_->create_publisher<multipath_sim::msg::MultipathOffset>("offset", 10); 
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
  
  //Ray_ranges stores the ranges of both satellite and reflected rays.
  std::vector<double> ray_ranges; 
  ray_ranges.resize(_msg->scan().count());
  std::copy(_msg->scan().ranges().begin(),
            _msg->scan().ranges().end(),
            ray_ranges.begin());

  //Updating the ray angles for sateliite and reflected ray.
  setRayAngles(ray_ranges);
  //Publishing the range offset and satellite blocked
  multipath_sim::msg::MultipathOffset offset_msg;
  offset_msg.header.stamp = rclcpp::Time(_msg->time().sec(), _msg->time().nsec());
  offset_msg.header.frame_id = frame_name_;
  offset_msg.sats_blocked.resize(num_sat_);
  int num_sat_blocked = 0;
  std::vector<double> range_offset;
  for (int i=0; i < ray_ranges.size(); i=i+2)
  {
    // noise
    double noise;
    if (!disable_noise_)
    {  
        noise = ignition::math::Rand::DblNormal(0.0, 1.0);
    }
    if (sat_dir_elevation_[i] > 0.20)
    {
      // ray is obstructed
      if (ray_ranges[i] < _msg->scan().range_max()) 
      {
        // check range of the mirror ray corresponding to the satellite ray 
        double mir_ray_range = ray_ranges[i+1];

        // check if mirror ray also is obstructed, providing a reflection
        if (mir_ray_range < _msg->scan().range_max())
        {
          range_offset.push_back(mir_ray_range * ( 1 + sin(M_PI/2.0 - 2*sat_dir_elevation_[i])) +  noise);
        } else {
          // otherwise no line of sight to any satellites and no reflections, no reading
          num_sat_blocked++;
          range_offset.push_back(0);
          offset_msg.sats_blocked[i/2] = 1;
        }
        
      } else {
        // unobstructed sat reading
        range_offset.push_back(noise);
      }
    }
    else
    {
      // low elevation sat reading
      offset_msg.sats_blocked[i/2] = 1;
      range_offset.push_back(noise);
    }
  }
  
  offset_msg.range_offset.resize(range_offset.size());
  std::copy(range_offset.begin(),
            range_offset.end(),
            offset_msg.range_offset.begin());
  offset_publisher_->publish(offset_msg);

  // Convert Laser scan to ROS LaserScan
  auto ls = gazebo_ros::Convert<sensor_msgs::msg::LaserScan>(*_msg);
  // Set tf frame
  ls.header.frame_id = frame_name_;
  // Publish output
  boost::get<LaserScanPub>(pub_)->publish(ls);
}
void GazeboRosMultipathSensorPrivate::setRayAngles(std::vector<double> &ray_ranges) 
{ 
  gazebo::physics::MultiRayShapePtr LaserShape = parent_ray_sensor_->LaserShape();
  std::vector<double> &elevation = sat_dir_elevation_;
  std::vector<double> &azimuth = sat_dir_azimuth_;
  ignition::math::Vector3d start, end, axis, end_scan;
  ignition::math::Quaterniond ray;
  for(unsigned int l=0; l < elevation.size(); l++ ) 
  {
    double yawAngle = azimuth[l];
    double pitchAngle = elevation[l];
    // since we're rotating a unit x vector, a pitch rotation will now be
    // around the negative y axis
    ray.Euler(ignition::math::Vector3d(0.0, -pitchAngle, yawAngle));
    axis = parent_entity_->WorldPose().Rot().Inverse() * parent_ray_sensor_->Pose().Rot() * ray * ignition::math::Vector3d::UnitX;
    start = (axis * LaserShape->GetMinRange()) + parent_ray_sensor_->Pose().Pos();
    end = (axis * LaserShape->GetMaxRange()) + parent_ray_sensor_->Pose().Pos();
    end_scan = (axis * std::min(LaserShape->GetMaxRange(), ray_ranges[l])) + parent_ray_sensor_->Pose().Pos();
    LaserShape->SetRay(l, start, end);
  }
}

// Register this plugin with the simulator
GZ_REGISTER_SENSOR_PLUGIN(GazeboRosMultipathSensor)

}  // namespace gazebo_plugins