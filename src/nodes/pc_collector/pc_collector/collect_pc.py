import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import read_points_numpy, dtype_from_fields
import numpy as np
import matplotlib.pyplot as plt

class PCCollector(Node):

    def __init__(self):
        super().__init__('pc_collector')

        # Create subscriber object
        # self.sub = self.create_subscription(PointCloud2, '/velmwheel/camera_f/depth/color/points', self.msg_callback, 10,)
        self.sub = self.create_subscription(PointCloud2, '/camera/depth/color/points', self.msg_callback, 10,)
        self.collected = None

    def msg_callback(self, msg):
        # print(msg.fields)
        old_width = msg.width
        msg.width = msg.height
        msg.height = old_width
        # print(msg.width)
        # print(msg.height)
        # print(msg.point_step)
        # print(msg.row_step)

        points = read_points_numpy(msg, field_names=['x', 'y', 'z'], reshape_organized_cloud = True)
        # print(points.shape)
        # points = points[240:,:]
        # points = points[::4,::4]
        # print(points.shape)
        # print('Test')

        # fig = plt.figure()
        # ax = fig.add_subplot(projection='3d')
        # ax.scatter(points[:,:,0], points[:,:,2], -points[:,:,1], s = 0.7,color = 'k')
        # ax.set_xlabel('X Label')
        # ax.set_ylabel('Y Label')
        # ax.set_zlabel('Z Label')
        # plt.show()
        # print('Test')

        if self.collected is None:
            self.collected = np.expand_dims(points, axis=0)
        else:
            self.collected = np.concatenate([self.collected, np.expand_dims(points, axis=0)], axis=0)

        print(f'Samples collected: {self.collected.shape[0]}')

def main(args=None):
    print('Hi from pc_collector.')

    # Initialize rlc
    rclpy.init(args=args)
    # Create nodes
    subscriber = PCCollector()
    # Run nodes
    try:
        rclpy.spin(subscriber)
    except:
        pass
    with open('samples.npz', 'wb') as f:
        np.savez_compressed(f, subscriber.collected)
    dataset = np.load('samples.npz')
    print(dataset['arr_0'].shape)
    # Deinitialize rlc
    rclpy.shutdown()


if __name__ == '__main__':
    main()
