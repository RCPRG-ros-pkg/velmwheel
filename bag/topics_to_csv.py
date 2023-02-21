#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped, Twist, PoseStamped
from sensor_msgs.msg import Imu, LaserScan, Image, JointState, PointCloud2
from sensor_msgs_py.point_cloud2 import read_points_numpy
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from velmwheel_msgs.msg import Wheels, EncodersStamped
from cv_bridge import CvBridge
from copy import copy
import cv2 
import numpy as np
import matplotlib.pyplot as plt
import csv

class TopicsToCsv(Node):

    def __init__(self):
        super(TopicsToCsv, self).__init__('topics_to_csv')

        self.subs = {}
        self.data = {}

        self.subs['clock'] = self.create_subscription(Clock, '/clock', self.msg_callback_clock, 1000,)
        self.data['clock'] = []

        self.subs['base_vel_setpoint'] = self.create_subscription(Twist, '/velmwheel/base/velocity_setpoint', self.msg_callback_base_vel_setpoint, 1000,)
        self.data['base_vel_setpoint'] = []

        self.subs['base_controls'] = self.create_subscription(Wheels, '/velmwheel/base/controls', self.msg_callback_base_controls, 1000,)
        self.data['base_controls'] = []

        self.subs['base_encoders'] = self.create_subscription(EncodersStamped, '/velmwheel/base/encoders', self.msg_callback_base_encoders, 1000,)
        self.data['base_encoders'] = []

        self.subs['base_joints'] = self.create_subscription(JointState, '/velmwheel/base/joint_states', self.msg_callback_base_joints, 1000,)
        self.data['base_joints'] = []
        self.joints = {}

        self.subs['base_vel'] = self.create_subscription(TwistStamped, '/velmwheel/base/velocity', self.msg_callback_base_vel, 1000,)
        self.data['base_vel'] = []

        self.subs['imu'] = self.create_subscription(Imu, '/velmwheel/imu/out', self.msg_callback_imu, 1000,)
        self.data['imu'] = []

        self.subs['odom_enc'] = self.create_subscription(Odometry, '/velmwheel/odom/encoders', self.msg_callback_odom_enc, 1000,)
        self.data['odom_enc'] = []

        self.subs['odom_laser'] = self.create_subscription(Odometry, '/velmwheel/odom/laser', self.msg_callback_odom_laser, 1000,)
        self.data['odom_laser'] = []

        self.subs['odom_filter'] = self.create_subscription(Odometry, '/velmwheel/odom/filtered', self.msg_callback_odom_filter, 1000,)
        self.data['odom_filter'] = []

        self.subs['sim_pose'] = self.create_subscription(PoseStamped, '/velmwheel/sim/pose', self.msg_callback_sim_pose, 1000,)
        self.data['sim_pose'] = []

        self.subs['sim_vel'] = self.create_subscription(TwistStamped, '/velmwheel/sim/velocity', self.msg_callback_sim_vel, 1000,)
        self.data['sim_vel'] = []

        self.subs['lidar_l'] = self.create_subscription(LaserScan, '/velmwheel/lidar_l/scan', self.msg_callback_lidar_l, 1000,)
        self.data['lidar_l'] = []

        self.subs['lidar_r'] = self.create_subscription(LaserScan, '/velmwheel/lidar_r/scan', self.msg_callback_lidar_r, 1000,)
        self.data['lidar_r'] = []

        self.subs['lidars'] = self.create_subscription(LaserScan, '/velmwheel/lidars/scan', self.msg_callback_lidars, 1000,)
        self.data['lidars'] = []

        self.br = CvBridge()

        self.subs['cam_f_color'] = self.create_subscription(Image, '/velmwheel/camera_f/color/image_raw', self.msg_callback_cam_f_col, 1000,)
        self.data['cam_f_color'] = []
        self.f_color = None

        self.subs['cam_f_depth'] = self.create_subscription(Image, '/velmwheel/camera_f/depth/image_rect_raw', self.msg_callback_cam_f_depth, 1000,)
        self.data['cam_f_depth'] = []

        self.subs['cam_f_points'] = self.create_subscription(PointCloud2, '/velmwheel/camera_f/depth/color/points', self.msg_callback_cam_f_points, 1000,)
        self.data['cam_f_points'] = []
        self.f_points = None

        self.subs['cam_b_color'] = self.create_subscription(Image, '/velmwheel/camera_b/color/image_raw', self.msg_callback_cam_b_col, 1000,)
        self.data['cam_b_color'] = []
        self.b_color = None

        self.subs['cam_b_depth'] = self.create_subscription(Image, '/velmwheel/camera_b/depth/image_rect_raw', self.msg_callback_cam_b_depth, 1000,)
        self.data['cam_b_depth'] = []

        self.subs['cam_b_points'] = self.create_subscription(PointCloud2, '/velmwheel/camera_b/depth/color/points', self.msg_callback_cam_b_points, 1000,)
        self.data['cam_b_points'] = []
        self.b_points = None

        print('Hi from topics_to_csv node.')

        # self.cam_f_color = None
        # self.cam_f_depth = None
        # self.cam_f_points = None
        # self.cam_b_color = None
        # self.cam_b_depth = None
        # self.cam_b_points = None
        self.field_names = ['time']
        self.period = 1/30
        self.rows_to_save = []

    def msg_callback_clock(self,msg):
        self.data['clock'].append({
            'sec':msg.clock.sec,
            'nanosec':msg.clock.nanosec,
            })

    def msg_callback_base_controls(self,msg):
        self.data['base_controls'].append({
            'sec':self.data['clock'][-1]['sec'],
            'nanosec':self.data['clock'][-1]['nanosec'],
            'wheel0':msg.values[0],
            'wheel1':msg.values[1],
            'wheel2':msg.values[2],
            'wheel3':msg.values[3],
            })

    def msg_callback_base_encoders(self,msg):
        self.data['base_encoders'].append({
            'sec':msg.header.stamp.sec,
            'nanosec':msg.header.stamp.nanosec,
            'enc0angle':msg.encoders[0].angle,
            'enc0vel':msg.encoders[0].velocity,
            'enc1angle':msg.encoders[1].angle,
            'enc1vel':msg.encoders[1].velocity,
            'enc2angle':msg.encoders[2].angle,
            'enc2vel':msg.encoders[2].velocity,
            'enc3angle':msg.encoders[3].angle,
            'enc3vel':msg.encoders[3].velocity,
            })

    def msg_callback_base_joints(self,msg):
        self.joints = {
            'sec':msg.header.stamp.sec,
            'nanosec':msg.header.stamp.nanosec,
            }
        for i,key in enumerate(msg.name):
            self.joints[f'{key}_pos'] = msg.position[i]
            self.joints[f'{key}_vel'] = msg.velocity[i]
        self.data['base_joints'].append(self.joints.copy())


    def msg_callback_base_vel(self,msg):
        self.data['base_vel'].append({
            'sec':msg.header.stamp.sec,
            'nanosec':msg.header.stamp.nanosec,
            'linvelX':msg.twist.linear.x,
            'linvelY':msg.twist.linear.y,
            'linvelZ':msg.twist.linear.z,
            'angvelX':msg.twist.angular.x,
            'angvelY':msg.twist.angular.y,
            'angvelZ':msg.twist.angular.z,
            })

    def msg_callback_base_vel_setpoint(self,msg):
        self.data['base_vel_setpoint'].append({
            'sec':self.data['clock'][-1]['sec'],
            'nanosec':self.data['clock'][-1]['nanosec'],
            'linvelX':msg.linear.x,
            'linvelY':msg.linear.y,
            'linvelZ':msg.linear.z,
            'angvelX':msg.angular.x,
            'angvelY':msg.angular.y,
            'angvelZ':msg.angular.z,
            })

    def msg_callback_imu(self,msg):
        self.data['imu'].append({
            'sec':msg.header.stamp.sec,
            'nanosec':msg.header.stamp.nanosec,
            'orientX':msg.orientation.x,
            'orientY':msg.orientation.y,
            'orientZ':msg.orientation.z,
            'orientW':msg.orientation.w,
            'angvelX':msg.angular_velocity.x,
            'angvelY':msg.angular_velocity.y,
            'angvelZ':msg.angular_velocity.z,
            'linaccX':msg.linear_acceleration.x,
            'linaccY':msg.linear_acceleration.y,
            'linaccZ':msg.linear_acceleration.z,
            })

    def msg_callback_odom_enc(self,msg):
        self.data['odom_enc'].append({
            'sec':msg.header.stamp.sec,
            'nanosec':msg.header.stamp.nanosec,
            'poseX':msg.pose.pose.position.x,
            'poseY':msg.pose.pose.position.y,
            'poseZ':msg.pose.pose.position.z,
            'orientX':msg.pose.pose.orientation.x,
            'orientY':msg.pose.pose.orientation.y,
            'orientZ':msg.pose.pose.orientation.z,
            'orientW':msg.pose.pose.orientation.w,
            'linvelX':msg.twist.twist.linear.x,
            'linvelY':msg.twist.twist.linear.y,
            'linvelZ':msg.twist.twist.linear.z,
            'angvelX':msg.twist.twist.angular.x,
            'angvelY':msg.twist.twist.angular.y,
            'angvelZ':msg.twist.twist.angular.z,
            })

    def msg_callback_odom_laser(self,msg):
        self.data['odom_laser'].append({
            'sec':msg.header.stamp.sec,
            'nanosec':msg.header.stamp.nanosec,
            'poseX':msg.pose.pose.position.x,
            'poseY':msg.pose.pose.position.y,
            'poseZ':msg.pose.pose.position.z,
            'orientX':msg.pose.pose.orientation.x,
            'orientY':msg.pose.pose.orientation.y,
            'orientZ':msg.pose.pose.orientation.z,
            'orientW':msg.pose.pose.orientation.w,
            'linvelX':msg.twist.twist.linear.x,
            'linvelY':msg.twist.twist.linear.y,
            'linvelZ':msg.twist.twist.linear.z,
            'angvelX':msg.twist.twist.angular.x,
            'angvelY':msg.twist.twist.angular.y,
            'angvelZ':msg.twist.twist.angular.z,
            })

    def msg_callback_odom_filter(self,msg):
        self.data['odom_filter'].append({
            'sec':msg.header.stamp.sec,
            'nanosec':msg.header.stamp.nanosec,
            'poseX':msg.pose.pose.position.x,
            'poseY':msg.pose.pose.position.y,
            'poseZ':msg.pose.pose.position.z,
            'orientX':msg.pose.pose.orientation.x,
            'orientY':msg.pose.pose.orientation.y,
            'orientZ':msg.pose.pose.orientation.z,
            'orientW':msg.pose.pose.orientation.w,
            'linvelX':msg.twist.twist.linear.x,
            'linvelY':msg.twist.twist.linear.y,
            'linvelZ':msg.twist.twist.linear.z,
            'angvelX':msg.twist.twist.angular.x,
            'angvelY':msg.twist.twist.angular.y,
            'angvelZ':msg.twist.twist.angular.z,
            })

    def msg_callback_sim_vel(self,msg):
        self.data['sim_vel'].append({
            'sec':msg.header.stamp.sec,
            'nanosec':msg.header.stamp.nanosec,
            'linvelX':msg.twist.linear.x,
            'linvelY':msg.twist.linear.y,
            'linvelZ':msg.twist.linear.z,
            'angvelX':msg.twist.angular.x,
            'angvelY':msg.twist.angular.y,
            'angvelZ':msg.twist.angular.z,
            })

    def msg_callback_sim_pose(self,msg):
        self.data['sim_pose'].append({
            'sec':msg.header.stamp.sec,
            'nanosec':msg.header.stamp.nanosec,
            'poseX':msg.pose.position.x,
            'poseY':msg.pose.position.y,
            'poseZ':msg.pose.position.z,
            'orientX':msg.pose.orientation.x,
            'orientY':msg.pose.orientation.y,
            'orientZ':msg.pose.orientation.z,
            'orientW':msg.pose.orientation.w,
            })

    def msg_callback_lidar_l(self,msg):
        self.data['lidar_l'].append({
            'sec':msg.header.stamp.sec,
            'nanosec':msg.header.stamp.nanosec,
            'ranges':msg.ranges,
            })

    def msg_callback_lidar_r(self,msg):
        self.data['lidar_r'].append({
            'sec':msg.header.stamp.sec,
            'nanosec':msg.header.stamp.nanosec,
            'ranges':msg.ranges,
            })

    def msg_callback_lidars(self,msg):
        self.data['lidars'].append({
            'sec':msg.header.stamp.sec,
            'nanosec':msg.header.stamp.nanosec,
            'ranges':msg.ranges,
            })

    def msg_callback_cam_f_col(self,msg):
        self.f_color = self.br.imgmsg_to_cv2(msg)
        self.f_color = np.concatenate((np.expand_dims(self.f_color[:,:,2], axis=2), np.expand_dims(self.f_color[:,:,1], axis=2), np.expand_dims(self.f_color[:,:,0], axis=2)), axis = 2)
        self.data['cam_f_color'].append({
            'sec':msg.header.stamp.sec,
            'nanosec':msg.header.stamp.nanosec,
            'image':self.f_color[:]
            })
        # print(self.f_color.shape)
        # cv2.imshow("camera", self.f_color)
        # cv2.waitKey(1)

    def msg_callback_cam_f_depth(self,msg):
        self.data['cam_f_depth'].append({
            'sec':msg.header.stamp.sec,
            'nanosec':msg.header.stamp.nanosec,
            'image':self.br.imgmsg_to_cv2(msg)
            })
        # plt.figure()
        # print(current_frame.shape)
        # plt.imshow(current_frame)
        # plt.show()

    def msg_callback_cam_f_points(self,msg):
        self.f_points = read_points_numpy(msg, field_names=['x', 'y', 'z'], reshape_organized_cloud = True)
        self.f_points = np.concatenate((np.expand_dims(self.f_points[:,:,0], axis=2), np.expand_dims(self.f_points[:,:,2], axis=2), np.expand_dims(-self.f_points[:,:,1], axis=2)), axis = 2)
        self.data['cam_f_points'].append({
            'sec':msg.header.stamp.sec,
            'nanosec':msg.header.stamp.nanosec,
            'points':self.f_points[:]
            })
        # fig = plt.figure()
        # ax = fig.add_subplot(projection='3d')
        # # ax.scatter(points[:,:,0], points[:,:,2], -points[:,:,1], s = 0.7,color = 'k')
        # ax.scatter(points[:,:,0], points[:,:,1], points[:,:,2], s = 0.7,color = 'k')
        # ax.set_xlabel('X Label')
        # ax.set_ylabel('Y Label')
        # ax.set_zlabel('Z Label')
        # plt.show()

    def msg_callback_cam_b_col(self,msg):
        self.b_color = self.br.imgmsg_to_cv2(msg)
        self.b_color = np.concatenate((np.expand_dims(self.b_color[:,:,2], axis=2), np.expand_dims(self.b_color[:,:,1], axis=2), np.expand_dims(self.b_color[:,:,0], axis=2)), axis = 2)
        self.data['cam_b_color'].append({
            'sec':msg.header.stamp.sec,
            'nanosec':msg.header.stamp.nanosec,
            'image':self.b_color[:]
            })

    def msg_callback_cam_b_depth(self,msg):
        self.data['cam_b_depth'].append({
            'sec':msg.header.stamp.sec,
            'nanosec':msg.header.stamp.nanosec,
            'image':self.br.imgmsg_to_cv2(msg)
            })

    def msg_callback_cam_b_points(self,msg):
        self.b_points = read_points_numpy(msg, field_names=['x', 'y', 'z'], reshape_organized_cloud = True)
        self.b_points = np.concatenate((np.expand_dims(self.b_points[:,:,0], axis=2), np.expand_dims(self.b_points[:,:,2], axis=2), np.expand_dims(-self.b_points[:,:,1], axis=2)), axis = 2)
        self.data['cam_b_points'].append({
            'sec':msg.header.stamp.sec,
            'nanosec':msg.header.stamp.nanosec,
            'points':self.b_points[:]
            })

    def gather_field_names(self):
        for msg in self.data.keys():
            for field in self.data[msg][0].keys():
                if field not in ['sec', 'nanosec']:
                    self.field_names.append(f'{msg}_{field}')
        print(f'Field names {self.field_names}')

    def save_data(self):
        current_time = self.data['clock'][0]['sec'] + self.data['clock'][0]['nanosec']/1000000000
        while current_time < self.data['clock'][-1]['sec'] + self.data['clock'][-1]['nanosec']/1000000000:
            self.rows_to_save.append({'time': current_time})
            current_time += self.period
        # print(rows_to_save)
        print(f'Rows in dataset: {len(self.rows_to_save)}')

        for msg in self.data.keys():
            if msg == 'clock':
                continue
            print(f'Processing topic: {msg}')
            for r in range(len(self.rows_to_save)):
                # print(f'Procesing row: {r}')
                for i in range(len(self.data[msg])):
                    if self.data[msg][i]['sec'] + self.data[msg][i]['nanosec']/1000000000 < self.rows_to_save[r]['time']:
                        continue
                    if self.data[msg][i]['sec'] + self.data[msg][i]['nanosec']/1000000000 - self.rows_to_save[r]['time'] < self.period:
                        for field in self.data[msg][0].keys():
                            if field not in ['sec', 'nanosec']:
                                if field in ['image', 'points']:
                                    self.rows_to_save[r][f'{msg}_{field}'] = i
                                else:
                                    self.rows_to_save[r][f'{msg}_{field}'] = self.data[msg][i][field]
                    else:
                        for field in self.data[msg][0].keys():
                            if field not in ['sec', 'nanosec']:
                                self.rows_to_save[r][f'{msg}_{field}'] = None
                    break
                if len(self.rows_to_save[r].keys()) < len(self.field_names):
                    for field in self.field_names:
                        if field not in self.rows_to_save[r].keys():
                            self.rows_to_save[r][field] = None

        for r in range(len(self.rows_to_save) - 1):
            for field in self.field_names:
                if self.rows_to_save[r+1][field] is None:
                    self.rows_to_save[r+1][field] = self.rows_to_save[r][field]

        while not all(self.rows_to_save[0].values()):
            done = True
            for field in self.field_names:
                if self.rows_to_save[0][field] is None:
                    self.rows_to_save = self.rows_to_save[1:]
                    done = False
                    break
            if done:
                break

        print(f'Rows in dataset after preprocessing: {len(self.rows_to_save)}')

        with open('dataset_' + str(len(self.rows_to_save)) + '_' + str(self.rows_to_save[-1]['time']), 'w', newline='') as file:
            writer = csv.DictWriter(file, fieldnames = self.field_names)
            writer.writeheader()
            writer.writerows(self.rows_to_save)
        file.close()

    def save_cam_f_color(self):

        f =  open('cam_f_color_' + str(len(self.rows_to_save)) + '_' + str(self.rows_to_save[-1]['time']) + '.npz', 'wb')
        for image in self.data['cam_f_color']:
            np.savez_compressed(f, image['image'])

        f.close()

    def save_cam_f_depth(self):

        f = open('cam_f_depth_' + str(len(self.rows_to_save)) + '_' + str(self.rows_to_save[-1]['time']) + '.npz', 'wb')
        for image in self.data['cam_f_depth']:
            np.savez_compressed(f, image['image'])

        f.close()

    def save_cam_f_points(self):

        f = open('cam_f_points_' + str(len(self.rows_to_save)) + '_' + str(self.rows_to_save[-1]['time']) + '.npz', 'wb')
        for image in self.data['cam_f_points']:
            np.savez_compressed(f, image['points'])

        f.close()

    def save_cam_b_color(self):

        f = open('cam_b_color_' + str(len(self.rows_to_save)) + '_' + str(self.rows_to_save[-1]['time']) + '.npz', 'wb')
        for image in self.data['cam_b_color']:
            np.savez_compressed(f, image['image'])

        f.close()

    def save_cam_b_depth(self):

        f = open('cam_b_depth_' + str(len(self.rows_to_save)) + '_' + str(self.rows_to_save[-1]['time']) + '.npz', 'wb')
        for image in self.data['cam_b_depth']:
            np.savez_compressed(f, image['image'])

        f.close()

    def save_cam_b_points(self):

        f = open('cam_b_points_' + str(len(self.rows_to_save)) + '_' + str(self.rows_to_save[-1]['time']) + '.npz', 'wb')
        for image in self.data['cam_b_points']:
            np.savez_compressed(f, image['points'])

        f.close()

def main(args=None):

    # Initialize rlc
    rclpy.init(args=args)
    # Create nodes
    node = TopicsToCsv()
    # Run nodes
    try:
        rclpy.spin(node)
    # except Exception as e:
    #     print(e)
    except:
        pass

    all_msgs = 0
    for topic in node.data.keys():
        print(f'{topic} messages: {len(node.data[topic])}')
        all_msgs += len(node.data[topic])
    print(f'There all {all_msgs} messages total.')

    node.gather_field_names()
    node.save_data()
    node.save_cam_f_color()
    node.save_cam_f_depth()
    node.save_cam_f_points()
    node.save_cam_b_color()
    node.save_cam_b_depth()
    node.save_cam_b_points()

    # Deinitialize rlc
    # rclpy.shutdown()

if __name__ == '__main__':
    main()
