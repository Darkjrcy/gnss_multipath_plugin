#pragma once

// Basic libraries
#include <memory>
#include <string>

// ROS2 libraries
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

// Gazebo Harmonic plugin integration
#include <gz/sim/System.hh>

namespace gnss_multipath_plugin
{
    class GNSSMultipathPluginPrivate;

    class GNSSMultipathPlugin: public gz::sim::System,public gz::sim::ISystemConfigure, public gz::sim::ISystemPostUpdate
    {
    public:
        // Constructor:
        GNSSMultipathPlugin();
        // Destructor
        ~GNSSMultipathPlugin() override = default;
        // Configure Function:
        void Configure(const gz::sim::Entity &_entity,const std::shared_ptr<const sdf::Element> &_sdf,gz::sim::EntityComponentManager &_ecm,gz::sim::EventManager &) override;
        void PostUpdate(const gz::sim::UpdateInfo &_info,const gz::sim::EntityComponentManager &_ecm) override;

    private:
        std::unique_ptr<GNSSMultipathPluginPrivate> impl_;
    };
}
