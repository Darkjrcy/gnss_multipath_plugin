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
#include <boost/make_shared.hpp>
#include <gazebo/common/common.hh>
#include <gazebo_ros/conversions/builtin_interfaces.hpp>
#include <gazebo/physics/Model.hh>
#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <gazebo/transport/transport.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/rendering/Visual.hh>
#include <gazebo/physics/World.hh>
#include <multipath_sim/sphere_model_plugin.hpp>
#include "multipath_sim/msg/multipath_offset.hpp"  
#include <memory>

namespace gazebo_plugins
{
/// Class to hold private data members (PIMPL pattern)
class GazeboRosTemplatePrivate
{
public:
  /// Connection to world update event. Callback is called while this is alive.
  gazebo::event::ConnectionPtr update_connection_;
  //gazebo::physics::LinkPtr link_;
  /// Node for ROS communication.
  gazebo_ros::Node::SharedPtr ros_node_;
  /// Gazebo node used to subscribe to laser scan
  gazebo::transport::NodePtr gazebo_node_;
  //gazebo::transport::PublisherPtr visPub;
  gazebo::rendering::VisualPtr model_;
  /// Protect variables accessed on callbacks.
  std::mutex lock_;
  /// Offset received on command.
  multipath_sim::msg::MultipathOffset offset_msg_;
  rclcpp::Subscription<multipath_sim::msg::MultipathOffset>::SharedPtr offset_subscription_; 
  void offset_callback(multipath_sim::msg::MultipathOffset::SharedPtr msg);
  double accum = 0;
  double offset_norm = 0.0;
};

GazeboRosTemplate::GazeboRosTemplate()
: impl_(std::make_unique<GazeboRosTemplatePrivate>())
{
}

GazeboRosTemplate::~GazeboRosTemplate()
{
  if (impl_->gazebo_node_) {
    impl_->gazebo_node_->Fini();
  }
  impl_->gazebo_node_.reset();
}

void GazeboRosTemplate::Load(gazebo::rendering::VisualPtr visual, sdf::ElementPtr sdf)
{
  // Create a GazeboRos node instead of a common ROS node.
  // Pass it SDF parameters so common options like namespace and remapping
  // can be handled.
  impl_->model_= visual;
  impl_->ros_node_ = gazebo_ros::Node::Get(sdf);
  // Get QoS profiles
  const gazebo_ros::QoS & qos = impl_->ros_node_->get_qos();
  impl_->offset_subscription_ = impl_->ros_node_->create_subscription<multipath_sim::msg::MultipathOffset>(
    "/multipath/offset", qos.get_subscription_qos("/multipath/offset", rclcpp::QoS(1)),
    std::bind(&GazeboRosTemplatePrivate::offset_callback, impl_.get(), std::placeholders::_1)); 
  impl_->update_connection_ = gazebo::event::Events::ConnectPreRender(boost::bind(&GazeboRosTemplate::OnUpdate, this));
     
}

void GazeboRosTemplatePrivate::offset_callback(const multipath_sim::msg::MultipathOffset::SharedPtr msg) 
{
  std::lock_guard<std::mutex> scoped_lock(lock_);
  offset_msg_ = *msg;
}
void GazeboRosTemplate::OnUpdate()
{
    double max_offset = *max_element(impl_->offset_msg_.range_offset.begin(), impl_->offset_msg_.range_offset.end());
    double scale = 1 + 5 * max_offset/100;

    ignition::math::Vector3d initial_scale(scale , scale, scale);
    impl_->model_->SetScale(initial_scale);

    static float r=0.0, increment=0.01;
    if (r > 1) {
    increment = -0.01;
    r = 1;
    }
    if (r < 0) {
    r = 0;
    increment = 0.01;
    }
    r += increment;
    ignition::math::Color c_ambient(0, 0.0, 0.0, 1);
    impl_->model_->SetAmbient(c_ambient);
    ignition::math::Color c_diffuse(0.0, 0.0, 0.0, 1);
    impl_->model_->SetDiffuse(c_diffuse);
    if (scale>4.5)
    {
      ignition::math::Color c_emmisive(r, 0.0, 0.0, 1);
      impl_->model_->SetEmissive(c_emmisive);
    }
    else
    {
      ignition::math::Color c_emmisive(0.0, r, 0.0, 1);
      impl_->model_->SetEmissive(c_emmisive);
    }
      
    impl_->model_->SetTransparency(0.8);
   

}

// Register this plugin with the simulator
GZ_REGISTER_VISUAL_PLUGIN(GazeboRosTemplate)
}  // namespace gazebo_plugins