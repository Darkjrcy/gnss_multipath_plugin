// Include the gz libraries:
#include <gz/plugin/Register.hh>
#include <gz/msgs/imu.pb.h>
#include <gz/sim/Model.hh>
#include <gz/transport/Node.hh>
#include <gz/math/Quaternion.hh>
#include <gz/math/Vector3.hh>
#include <gz/math/Rand.hh>
#include <gz/msgs/pose.pb.h>
#include <gz/sim/components/LinearVelocity.hh>
#include <gz/msgs/clock.pb.h>
// Basic libraries:
#include <thread>
#include <memory>
#include <string>
#include <chrono>
#include <vector>
#include <cmath>
#include <ctime>
#include <cstring>
#include <mutex>
#include <Eigen/Dense>
// ROS2 libraties:
#include <rclcpp/rclcpp.hpp>
#include "gnss_multipath_plugin/msg/states_info.hpp"
#include <std_srvs/srv/trigger.hpp>
// Custom header:
#include "gnss_multipath_plugin/adsb_plugin.hpp"



// Start the namespace of the plugin:
namespace adsb_plugin
{
    class ADSBPluginPrivate{
        public:
            // Deifne the GZ odes to subscribe of Both the GNSS position and IMU data
            // GNSS position:
            std::string imu_topic_;
            // Start the GZ node of the imu:
            gz::transport::Node gz_node_;
            // Linear velocity of teh model to start teh estimation process:
            gz::math::Vector3d true_linear_vel_;
            std::mutex vel_mutex_;
            // MUtex to avoid bith GNSS and Lidar publsih at the same time;:
            std::mutex ekf_mutex_;


            // Gazebo Sim model:
            gz::sim::Entity model_entity_{gz::sim::kNullEntity};
            gz::sim::Model model_{gz::sim::kNullEntity};


            // Model characteristics:
            // Frame Id where the IMU is connected:
            std::string frame_id_ = "imu_link";

            // ADD THIS: Save the physical link
            gz::sim::Entity link_entity_{gz::sim::kNullEntity};


            // ROS2 
            // ROS2 node:
            rclcpp::Node::SharedPtr ros_node_;
            // Publsiher to publish the ADS-B estiation:
            rclcpp::Publisher<gnss_multipath_plugin::msg::StatesInfo>::SharedPtr adsb_pub_;
            // Topic where the adsb_est is published:
            std::string adsb_topic_;
            // Create a trigger service to restart the adsb estimation process:
            rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_adsb_srv_;
            
            
            // Boolean to start the estmiation:
            bool restart_est = false;
            // Boolena to see if the intial condiiton started:
            bool initial_cond_1_ = false;
            bool initial_cond_2_ = false;
            // Vairbel to see when there is an update from the GNSS:
            bool update_gnss_ = false;


            // Define the GNSS meassured position:
            double x_meas_;
            double y_meas_;
            double z_meas_;
            // Define the initial position:
            double x0_;
            double y0_;
            double z0_;
            double vx0_;
            double vy0_;
            double vz0_;
            // Save the course, fpa and roll from the IMU:
            double course_;
            double fpa_;
            double roll_;
            // Get the roll pitch and yaw angles from IMU:
            double pitch_;
            double yaw_;


            // Define the staets after the propgation:
            double x_next_;
            double y_next_;
            double z_next_;
            double vx_next_;
            double vy_next_;
            double vz_next_;

            // Variable to save the previous roll, pitch and yaw speeds:
            double p0_ = 0.0;
            double q0_ = 0.0;
            double r0_ = 0.0;
            // Variable to save actual rolll, pitch and yaw speeds:
            double p_next_;
            double q_next_;
            double r_next_;

            // Characteristics required for the GUAM simulation:
            // Vertical acceleration:
            double acc_up_;
            // Total acceleration:
            double acc_tot_;
            // FLight characteristics:
            double alpha_;
            double beta_;
            // ANgular accelertaion:
            double p_dot_;
            double q_dot_;
            double r_dot_;


            // Define the matrices used in the EKF:
            using Mat66 = Eigen::Matrix<double,6,6>;
            Mat66 P0;
            Mat66 Ak;
            Eigen::Matrix<double,3,6> Ck;
            Eigen::Matrix<double,6,3> Kk;
            Mat66 Q;
            Eigen::Matrix3d R;


            // Past time that the estimation or meassurement where done:
            double past_time_ = 0.0;
            double actual_time_ = 0.0;


            // FUnctions used during the ADS-B estimation:
            // Function that is launched every time an imu message come:
            void OnImuData(const gz::msgs::IMU &msg);
            // Function to save the GNSS position data:
            void OnGNSSData(const gz::msgs::Pose &msg);
            // Function to define that the esitmation is tarted:
            void trigger_callback_start(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                std::shared_ptr<std_srvs::srv::Trigger::Response> response);
            // Function to publish the adsb_info:
            void publish_adsb(double east, double north, double up, double v_east, double v_north, double v_up, 
                double course, double fpa, double roll, double p, double q, double r, double acc_up, 
                double acc_tot, double alpha, double beta, double p_dot, double q_dot, double r_dot);
    };



    // Constructor:
    ADSBPlugin::ADSBPlugin():impl_(std::make_unique<ADSBPluginPrivate>()){}



    // Define the configure secction of the plugin:
    void ADSBPlugin::Configure(const gz::sim::Entity &_entity,const std::shared_ptr<const sdf::Element> &_sdf, gz::sim::EntityComponentManager &_ecm, gz::sim::EventManager &){
        // Obtain the eniity where the plugin is applied:
        this->impl_->model_entity_ = _entity;
        this->impl_->model_ = gz::sim::Model(_entity);

        // Define the noise matrices:
        impl_->Q = (ADSBPluginPrivate::Mat66() <<
            4, 0, 0, 0, 0, 0,
            0, 4, 0, 0, 0, 0,
            0, 0, 4, 0, 0, 0,
            0, 0, 0, 100, 0, 0,
            0, 0, 0, 0, 100, 0,
            0, 0, 0, 0, 0, 100).finished();      

        impl_->R = (Eigen::Matrix3d() <<
            500, 0, 0,
            0, 500, 0,
            0, 0, 500).finished();

        // Start to define the sdf characteristics:
        // Obtain the topic where gazebo publsihes the lidar information:
        if (_sdf->HasElement("topic")){
            impl_->imu_topic_ = _sdf->Get<std::string>("topic");
        }
        else{
            impl_->imu_topic_ = "/imu/out";  // default topic
        }
        // Obtain the frame_id:
        if (_sdf->HasElement("frame_id")){
            impl_->frame_id_ = _sdf->Get<std::string>("frame_id");
        }
        // Obtain wehre the ADS-B topic is published:
        std::string adsb_topic = "adsb_info";
        if (_sdf->HasElement("cmdADSBTopic")){
            adsb_topic = _sdf->Get<std::string>("cmdADSBTopic");
        }
        // Obtain the topic of to publish the GNSS data:
        std::string cmd_GNSS_topic = "gnss_multipath_fix";
        if (_sdf->HasElement("cmdGNSSTopic")){
            cmd_GNSS_topic = _sdf->Get<std::string>("cmdGNSSTopic");
        }
        // Get the start_adsb_service:
        std::string adsb_start_srv_stopic = "adsb_start_srv";
        if (_sdf->HasElement("adsb_start_srv_stopic")){
            adsb_start_srv_stopic = _sdf->Get<std::string>("adsb_start_srv_stopic");
        }
        // Get the name fo the model:
        std::string robot_namespace = "airplane_1";
        if (_sdf->HasElement("statesFrame")){
            robot_namespace = _sdf->Get<std::string>("statesFrame");
        }

        // Subscribe to the IMU gz sensor data:
        impl_->gz_node_.Subscribe(impl_->imu_topic_ , std::function<void(const gz::msgs::IMU&)>([this](const gz::msgs::IMU &msg){
            this->impl_->OnImuData(msg);
        }));
        // SUbscribe to the GNSS position topic in gz  sim:
        impl_->gz_node_.Subscribe(cmd_GNSS_topic, std::function<void(const gz::msgs::Pose&)>([this](const gz::msgs::Pose &msg){
            this->impl_->OnGNSSData(msg);
        }));

        // Start the ROS2 publications and nodes:
        if (!rclcpp::ok()){
            return;
        }

        rclcpp::NodeOptions options;
        options.arguments({ "--ros-args", "-r", "__ns:=/" + robot_namespace });
        impl_->ros_node_ = std::make_shared<rclcpp::Node>("adsb_estimator_node", options);
        // Spin the ROS2 node:
        std::thread([node = impl_->ros_node_]() {
                rclcpp::executors::SingleThreadedExecutor exec;
                exec.add_node(node);
                exec.spin();
            }).detach();
        // Publisher the adsb_information:
        impl_->adsb_pub_ =  impl_->ros_node_->create_publisher<gnss_multipath_plugin::msg::StatesInfo>(adsb_topic, 10); 
        // Service to star the estimation:
        impl_->start_adsb_srv_ = impl_->ros_node_->create_service<std_srvs::srv::Trigger>(
            adsb_start_srv_stopic,
            std::bind(&ADSBPluginPrivate::trigger_callback_start, impl_.get(),
                      std::placeholders::_1, std::placeholders::_2)
        );

        // Add the lienar velcoity reader: 
        impl_->link_entity_ = impl_->model_.LinkByName(_ecm, "base_link");
        if (impl_->link_entity_ == gz::sim::kNullEntity) {
            impl_->link_entity_ = impl_->model_.LinkByName(_ecm, "lidar_link");
        }

        // Tell the physics engine to compute the World Velocity for this link
        if (impl_->link_entity_ != gz::sim::kNullEntity) {
            _ecm.CreateComponent(impl_->link_entity_, gz::sim::components::WorldLinearVelocity());
        } else {
            RCLCPP_ERROR(impl_->ros_node_->get_logger(), "Could not find base_link or lidar_link!");
        }
    }



    // Post update function of the plugin
    void ADSBPlugin::PostUpdate(const gz::sim::UpdateInfo &_info, const gz::sim::EntityComponentManager &_ecm)
    {
        (void)_info;
        // Ensure the model exists
        if (this->impl_->model_entity_ == gz::sim::kNullEntity) return;

        //Get the World Linear Velocity component
        auto linear_vel_comp = _ecm.Component<gz::sim::components::WorldLinearVelocity>(this->impl_->link_entity_);

        if (linear_vel_comp)
        {
            std::lock_guard<std::mutex> lock(this->impl_->vel_mutex_);
            // This gives you vx, vy, vz in the World Frame
            this->impl_->true_linear_vel_ = linear_vel_comp->Data();
            // ka ethe initial_conditio 2 satisfied:
            if (this->impl_->true_linear_vel_.Length() >= 0.0 && !this->impl_->initial_cond_2_ ){
                this->impl_->initial_cond_2_ = true;
                this->impl_->vx0_ = 0.5 * this->impl_->true_linear_vel_.X();
                this->impl_->vy0_ = 0.5 * this->impl_->true_linear_vel_.Y();
                this->impl_->vz0_ = 0.5  * this->impl_->true_linear_vel_.Z();
            }
        }
    }




    // Callback function for messages thath come form hte IMU sensor:
    void ADSBPluginPrivate::OnImuData(const gz::msgs::IMU &msg){
        std::lock_guard<std::mutex> lock(ekf_mutex_);
        // Start only if the restart is lareay false:
        if (!initial_cond_1_ && restart_est){
            return;
        }

        // Check if GNSS is being updated:
        if (update_gnss_){
            return;
        }

        // First get the orientation:
        gz::math::Quaterniond q(msg.orientation().w(), msg.orientation().x(),
            msg.orientation().y(), msg.orientation().z());
        // Pass it to roll, pitch and yaw:
        gz::math::Vector3d euler = q.Euler();
        roll_ = euler.X();
        pitch_ = euler.Y();
        yaw_ = euler.Z();
        // Define the course, fpa, and roll;
        course_ = M_PI/2 - yaw_;
        fpa_ = -pitch_;

        // Get the angular speeds comming from the IMU:
        p_next_ = msg.angular_velocity().x();
        q_next_ = -msg.angular_velocity().y();
        r_next_ = -msg.angular_velocity().z();

        // Get teh actual time:
        actual_time_ = msg.header().stamp().sec() + (msg.header().stamp().nsec() * 1e-9);

        // Define the acceleration:
        gz::math::Vector3d local_accel(msg.linear_acceleration().x(), msg.linear_acceleration().y(),
            msg.linear_acceleration().z());
        // Rotate it to the WOrld frame:
        gz::math::Vector3d world_accel = q.RotateVector(local_accel);

        // Get the acceleration up:
        acc_up_ = world_accel.Z();
        acc_tot_ = local_accel.Length();

        // Propagte the system using first order dynamics:
        const double dt = (actual_time_-past_time_);
        x_next_ = vx0_*dt+x0_;
        y_next_ = vy0_*dt+y0_;
        z_next_ = vz0_*dt+z0_;
        vx_next_ = world_accel.X()*dt+vx0_;
        vy_next_ = world_accel.Y()*dt+vy0_;
        vz_next_ = world_accel.Z()*dt+vz0_;

        // Prpagate teh covariance state matrix P:
        // Obtain the Jacobian amtrice:
        Ak << 1, 0, 0, dt, 0, 0,
            0, 1, 0, 0, dt, 0,
            0, 0, 1, 0, 0, dt,
            0, 0, 0, 1, 0, 0,
            0, 0, 0, 0, 1, 0,
            0, 0, 0, 0, 0, 1;

        Ck << 1, 0, 0, 0, 0, 0,
            0, 1, 0, 0, 0, 0,
            0, 0, 1, 0, 0, 0;
        // Propagate P0:
        P0 = Ak*P0*Ak.transpose() + Q*dt;

        // EStimate teh angular accleration:
        if (dt > 0.0) {
            p_dot_ = (p_next_ - p0_) / dt;
            q_dot_ = (q_next_ - q0_) / dt;
            r_dot_ = (r_next_ - r0_) / dt;
        } else {
            p_dot_ = 0.0;
            q_dot_ = 0.0;
            r_dot_ = 0.0;
        }

        // Estimte teh angle of attack and sideslip:
        gz::math::Vector3d global_vel(vx_next_, vy_next_, vz_next_);
        // Pass it to the body frame:
        gz::math::Vector3d body_vel = q.RotateVectorReverse(global_vel);
        // Extraxt the body velocities (Forward-Left-Up) body frame: 
        double u = body_vel.X();
        double v = body_vel.Y();
        double w = body_vel.Z();
        // Get teh flight characteristics:
        alpha_ = std::atan2(-w, u);
        beta_  = std::atan2(-v, std::sqrt(u * u + w * w));

        // Publish the results, this is going to be done alter:
        publish_adsb(x_next_, y_next_, z_next_, vx_next_, vy_next_, vz_next_, course_, fpa_, roll_, p_next_, q_next_, r_next_, 
            acc_up_, acc_tot_, alpha_, beta_, p_dot_, q_dot_, r_dot_);

        // Reset the initial conditions to make the system forloop:
        x0_ = x_next_;
        y0_ = y_next_;
        z0_ = z_next_;
        vx0_ = vx_next_;
        vy0_ = vy_next_;
        vz0_ = vz_next_;
        past_time_ = actual_time_;
        // Define the actual 
        q0_ = q_next_;
        p0_ = p_next_;
        r0_ = r_next_;


    }



    // Callback function for Pose messages
    void ADSBPluginPrivate::OnGNSSData(const gz::msgs::Pose &msg){
        std::lock_guard<std::mutex> lock(ekf_mutex_);
        // Tell teh ssytem there is a gnss signal coming:
        update_gnss_ = true;

        // Define the meassured position:
        x_meas_ = msg.position().x();
        y_meas_ = msg.position().y();
        z_meas_ = msg.position().z();
        // Make a vector: 
        Eigen::Vector3d act_meas(x_meas_, y_meas_, z_meas_);

        // If ir started define the intiial condiutions:
        if (restart_est){
            x0_ = x_meas_;
            y0_ = y_meas_;
            z0_ = z_meas_;
            if (!initial_cond_2_) {
                return;
            }
        }

        // Cehck if both the velocity and position are collected:
        if (!initial_cond_1_ && initial_cond_2_){
            // Restart the varibles usde in teh EKF:
            past_time_ = msg.header().stamp().sec() + (msg.header().stamp().nsec() * 1e-9);

            // Define the initial state covariance matrix with ampt that good psoition estimation but a good velocity estimation:
            P0 << 100, 0, 0, 0, 0, 0,
                0, 100, 0, 0, 0, 0,
                0, 0, 100, 0, 0, 0,
                0, 0, 0, 15, 0, 0,
                0, 0, 0, 0, 15, 0,
                0, 0, 0, 0, 0, 15;

            initial_cond_1_ = true;
            restart_est = false;
            return;
        }

        // In case this is an update using the last est state form the propagation:
        const double dt = (actual_time_-past_time_);
        Ak << 1, 0, 0, dt, 0, 0,
            0, 1, 0, 0, dt, 0,
            0, 0, 1, 0, 0, dt,
            0, 0, 0, 1, 0, 0,
            0, 0, 0, 0, 1, 0,
            0, 0, 0, 0, 0, 1;

        Ck << 1, 0, 0, 0, 0, 0,
            0, 1, 0, 0, 0, 0,
            0, 0, 1, 0, 0, 0;

        // Pass the estimated positions to a vector:
        Eigen::Matrix<double, 6, 1> x0_vect;
        x0_vect << x0_, y0_, z0_, vx0_, vy0_, vz0_;
        // Make the same for the update;
        Eigen::Matrix<double, 6, 1> xnext_vec;
        // Use the EKF witht he meassurement to update the estimations:
        Eigen::Vector3d z = act_meas;              
        Eigen::Vector3d innov = z - Ck * x0_vect;  
        Eigen::Matrix3d S = Ck * P0 * Ck.transpose() + R;
        Kk = P0 * Ck.transpose() * S.inverse();    
        xnext_vec = x0_vect + Kk * innov; 
        Eigen::MatrixXd I = Eigen::MatrixXd::Identity(6, 6);
        P0 = (I-Kk*Ck)*P0;

        // Update the estiamtions:
        x0_ = xnext_vec(0);
        y0_ = xnext_vec(1);
        z0_ = xnext_vec(2);
        vx0_ = xnext_vec(3);
        vy0_ = xnext_vec(4);
        vz0_ = xnext_vec(5);
        past_time_ = msg.header().stamp().sec() + (msg.header().stamp().nsec() * 1e-9);

        // Publish to the adsb:
        publish_adsb(x_next_, y_next_, z_next_, vx_next_, vy_next_, vz_next_, course_, fpa_, roll_, p_next_, q_next_, r_next_, 
            acc_up_, acc_tot_, alpha_, beta_, p_dot_, q_dot_, r_dot_);

        // After the GNSS gives the meassurement make it be negative:
        update_gnss_ = false;

    }



    // Start service to start the estimation:
    void ADSBPluginPrivate::trigger_callback_start(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        (void)request; 

        restart_est = true; 
        initial_cond_1_ = false; 
        initial_cond_2_ = false; 
        response->success = true;
        response->message = "ADS-B Started.";
    }



    // Function to publish the adsb data:
    void ADSBPluginPrivate::publish_adsb(double east, double north, double up, double v_east, double v_north, double v_up, 
        double course, double fpa, double roll, double p, double q, double r, double acc_up,
        double acc_tot, double alpha, double beta, double p_dot, double q_dot, double r_dot)
    {
        gnss_multipath_plugin::msg::StatesInfo msg;

        // Position (your states are ENU; the message expects N/E/U):
        msg.north = north;
        msg.east  = east;
        msg.up    = up;

        // Velocity (same ENU→N/E/U mapping):
        msg.v_north = v_north;
        msg.v_east  = v_east;
        msg.v_up    = v_up;

        // Publish the accelation up and the toal acceleration:
        msg.acc_up = acc_up;
        msg.acc_tot = acc_tot;

        // Send the total velocity:
        msg.v_tot = v_north*v_north + v_east*v_east + v_up*v_up;

        // Course: ground track from North (use velocity, not yaw)
        msg.course = course;
        msg.fpa = fpa;
        msg.roll = roll;

        // Publish FLight characteristiccs:
        msg.angle_attack = alpha;
        msg.sideslip = beta;

        // Body angualr speeds:
        msg.p = p;
        msg.q = q;
        msg.r = r;

        // Add the angular accelrations:
        msg.p_dot = p_dot;
        msg.q_dot = q_dot;
        msg.r_dot = r_dot;

        adsb_pub_->publish(msg);
    }


}  




GZ_ADD_PLUGIN(
  adsb_plugin::ADSBPlugin,
  gz::sim::System,
  gz::sim::ISystemConfigure,
  gz::sim::ISystemPostUpdate)

GZ_ADD_PLUGIN_ALIAS(adsb_plugin::ADSBPlugin, "adsb_plugin")