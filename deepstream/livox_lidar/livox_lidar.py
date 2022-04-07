import rospy 
import std_msgs.msg
import numpy as np
import sensor_msgs.point_cloud2 as pcl2
from sensor_msgs.msg import PointCloud2
from ..io import DataCache

__all__ = ["LivoxLidar"]

class LivoxLidar(object):
    def __init__(self, return_data_cache: DataCache):
        rospy.init_node("livox_test", anonymous=True)
        self.sub = rospy.Subscriber(
            "/livox/lidar", PointCloud2, queue_size=10, callback=self.livoxCallBack
        )
        self.frame_count = 0
        self.data_cache = return_data_cache
        self.n_stack_frames = 1
        self.stack_pointcloud = []

    def update_stack_frames(self, n_stack_frames):
        self.n_stack_frames = n_stack_frames

    def stack_frame(self, pointcloud: np.ndarray):
        if self.n_stack_frames == 1:
            return pointcloud

        if len(self.stack_pointcloud) >= self.n_stack_frames:
            # only select the pcd in the end of stack 
            # to keep the timeliness of pcd 
            delta = len(self.stack_pointcloud) - self.n_stack_frames + 1
            self.stack_pointcloud = self.stack_pointcloud[delta:]
        self.stack_pointcloud.append(pointcloud)
        stack_result = np.vstack(self.stack_pointcloud)
        return stack_result 

    def livoxCallBack(self, msg):
        points_list = []
        # t_start = time.time()
        for point in pcl2.read_points(
            msg, skip_nans=True, field_names=("x", "y", "z", "intensity")):

            points_list.append(point)

        # t_end = time.time()
        # print("cost time: ", t_end - t_start)
        points_np = np.asarray(points_list)
        
        stack_points_np = self.stack_frame(points_np)
        # stack_points_np is related to the self.n_stack_frames
        #print(">> stack_points_np.shape : ", stack_points_np.shape) #  (22206, 4)
       
        self.data_cache.writeData(stack_points_np)
        # print("receive point clouds with size: ", points_np.shape)

