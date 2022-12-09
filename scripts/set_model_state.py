#!/usr/bin/env python3
import utm
import json
import os
import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.action import ActionClient
from gazebo_msgs.msg import EntityState
from gazebo_msgs.srv import SetEntityState
from geometry_msgs.msg import Pose


def quaternion_from_euler(ai, aj, ak):
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = np.cos(ai)
    si = np.sin(ai)
    cj = np.cos(aj)
    sj = np.sin(aj)
    ck = np.cos(ak)
    sk = np.sin(ak)
    cc = ci*ck
    cs = ci*sk
    sc = si*ck
    ss = si*sk

    q = np.empty((4, ))
    q[0] = cj*sc - sj*cs
    q[1] = cj*ss + sj*cc
    q[2] = cj*cs - sj*sc
    q[3] = cj*cc + sj*ss

    return q

def target_traj_gps(t, *args):
    car_data_gzb = args[0]
    index = int(t % car_data_gzb.shape[0])
    next_index = index + 1
    # Stacking the position, orientation and velocity of the model 
    pos = [car_data_gzb[index,0], car_data_gzb[index,1], 1]
    next_pos = [car_data_gzb[next_index,0], car_data_gzb[next_index,1], 1]
    att = np.array([0, 0, car_data_gzb[index, 2]])
    vel = (np.array(next_pos) - np.array(pos))
    return np.concatenate([pos, att, vel])

class TargetTrajNode(Node):
    """
    Move a target entity(gazebo model) along a set trajectory defined by traj_f
    traj_f should always take @t and @begin_pose as the first two arguments
    """
    def __init__(self, target_name=None, traj_f=None, begin_pose=np.zeros(6, dtype=np.float64), *traj_args) -> None:
        super().__init__(f"{target_name}_trajectory")
        self.client = self.create_client(SetEntityState, "/set_entity_state")
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('gazebo set_entity_state service is not availabel, waiting...')
        
        self.target_name = target_name
        self.traj_f = traj_f
        self.begin_pose = begin_pose
        self.traj_args = traj_args

        timer_period = 1
        self.timer = self.create_timer(timer_period, self.traj_callback)
        self.elapsed = 0
        self.time_last = self.get_clock().now()

        self.get_logger().info(f"setting state for {self.target_name}")
        self.state_msg = EntityState()
        self.state_msg.name = self.target_name
        # state_msg.reference_frame = "world" # default frame

        self.state_msg.pose.position.x = self.begin_pose[0]
        self.state_msg.pose.position.y = self.begin_pose[1]
        self.state_msg.pose.position.z = self.begin_pose[2]
    
        self.get_logger().info(f"set initial states for {self.target_name}")
        self.request = SetEntityState.Request()

    def traj_callback(self):
        self.elapsed += (self.get_clock().now()-self.time_last).nanoseconds / 1e9
        self.time_last = self.get_clock().now()

        _pose = self.traj_f(self.elapsed, *self.traj_args)
        self.state_msg.pose.position.x = _pose[0]
        self.state_msg.pose.position.y = _pose[1]
        self.state_msg.pose.position.z = _pose[2]

        _q = quaternion_from_euler(_pose[3], _pose[4], _pose[5])
        self.state_msg.pose.orientation.x = _q[0]
        self.state_msg.pose.orientation.y = _q[1]
        self.state_msg.pose.orientation.z = _q[2]
        self.state_msg.pose.orientation.w = _q[3]
        
        self.state_msg.twist.linear.x = _pose[6]
        self.state_msg.twist.linear.y = _pose[7]
        self.state_msg.twist.linear.z = _pose[8]
        self.request.state = self.state_msg
        future = self.client.call_async(self.request)
        
        if future.done():
            print(future.done())
            response = future.result()
            #self.get_logger().info("response: " + response)

def conv_gzb_to_latlon(x_coordinate, y_coordinate, lat_bound, lon_bound, origin_offset):
    origin_lon = lon_bound[0] + (lon_bound[1]-lon_bound[0])/2.0 
    origin_lat = lat_bound[0] + (lat_bound[1]-lat_bound[0])/2.0
    origin_x, origin_y, zone_number, zone_letter = utm.from_latlon(origin_lat, origin_lon)
    origin = np.array([origin_x, origin_y])
    x_coordinate += origin[0] - origin_offset[0]
    y_coordinate += origin[1] - origin_offset[1]
    lat, lon = utm.to_latlon(x_coordinate, y_coordinate, zone_number, zone_letter)
    return lat, lon

def conv_latlon_to_gzb(lat, lon, lat_bound, lon_bound, origin_offset):
    origin_lon = lon_bound[0] + (lon_bound[1]-lon_bound[0])/2.0 
    origin_lat = lat_bound[0] + (lat_bound[1]-lat_bound[0])/2.0
    origin_utm = utm.from_latlon(origin_lat, origin_lon)
    origin_utm = np.array([origin_utm[0], origin_utm[1]])
    x, y,_,_ = utm.from_latlon(lat, lon)
    x += -origin_utm[0] + origin_offset[0]
    y += -origin_utm[1] + origin_offset[1]
    return x, y

def main(args=None):
    rclpy.init(args=args)
    executor = rclpy.get_global_executor()
    #For HonKong Dataset 
    data_dir = os.path.abspath( os.path.join(os.path.dirname(__file__), os.pardir)) + "/data/" 
    with open(data_dir + "car_ground_truth.json", "r") as fstream:
        car_data = json.load(fstream)
    # Lat/Lon bounds for the segment of the map in gazebo     
    lat_bound = np.array([22.3066, 22.2903])
    lon_bound = np.array([114.171, 114.188])

    # Lat/Lon from the datset 
    lat =np.array(car_data['lat'])
    lon = np.array(car_data['lon'])
    
    # Applying correction in the offset to align the origin
    origin_offset = np.array([77, 15])
    
    # NED (ground truth) to ENU (gzb world)
    # Get the x y position in ENU (gzb world)
    gzb_x, gzb_y = conv_latlon_to_gzb(lat, lon, lat_bound, lon_bound, origin_offset)
    
    # Add heading and create a np array. 
    car_data_heading = np.deg2rad(90 - np.array(car_data["heading"]))
    car_data_gzb = np.vstack([gzb_x, gzb_y]).T
    car_data_gzb = np.vstack([car_data_gzb.T, car_data_heading]).T

    # Initial position of the vehicle
    x0_car = np.array([car_data_gzb[0,0], car_data_gzb[0,1], 1])

    traj = TargetTrajNode("laser_0", target_traj_gps, x0_car, car_data_gzb)
    executor.add_node(traj)

    # car_data_gzb = np.vstack([gzb_x-1, gzb_y-1]).T
    # car_data_gzb = np.vstack([car_data_gzb.T, car_data_heading]).T
    # traj = TargetTrajNode("drone_0", target_traj_gps, x0_car, car_data_gzb)
    # executor.add_node(traj)

    # car_data_gzb = np.vstack([gzb_x-2, gzb_y-2]).T
    # car_data_gzb = np.vstack([car_data_gzb.T, car_data_heading]).T
    # traj = TargetTrajNode("drone_1", target_traj_gps, x0_car, car_data_gzb)
    # executor.add_node(traj)
    # car_data_gzb = np.vstack([gzb_x+1, gzb_y+1]).T
    # car_data_gzb = np.vstack([car_data_gzb.T, car_data_heading]).T
    # traj = TargetTrajNode("drone_2", target_traj_gps, x0_car, car_data_gzb)
    # executor.add_node(traj)
    # car_data_gzb = np.vstack([gzb_x+2, gzb_y+2]).T
    # car_data_gzb = np.vstack([car_data_gzb.T, car_data_heading]).T
    # traj = TargetTrajNode("drone_3", target_traj_gps, x0_car, car_data_gzb)
    # executor.add_node(traj)
    # car_data_gzb = np.vstack([gzb_x+3, gzb_y+3]).T
    # car_data_gzb = np.vstack([car_data_gzb.T, car_data_heading]).T
    # traj = TargetTrajNode("drone_4", target_traj_gps, x0_car, car_data_gzb)
    # executor.add_node(traj)
    traj_sphere = TargetTrajNode("sphere", target_traj_gps, x0_car, car_data_gzb)
    executor.add_node(traj_sphere)
    
    
    executor.spin()
    traj.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
