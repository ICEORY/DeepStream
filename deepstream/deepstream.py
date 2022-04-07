import time
from typing import Dict
from .io import DataCache
from .hik_cam import HIKCamera
from .livox_lidar import LivoxLidar
import threading
from PIL import Image
import os
from .range_projection import RangeProjection  # Python
import numpy as np


__all__ = ["DeepStream"]


class DeepStream(object):
    def __init__(self, range_per_panel: float = 40, n_panel: int = 6, save_path="./out", params_path="./params") -> None:
        # init params
        self.range_per_panel = range_per_panel
        self.n_panel = n_panel
        self.save_img_path = os.path.join(save_path, "image")
        self.save_pcd_path = os.path.join(save_path, "pcd")
        self.intrinsic, self.distort, self.extrinsic = self.read_params(params_path)

        # init hik camera device
        self.cam = HIKCamera()
        self.cam_thread_flag = False
        self.cam_data_cache = DataCache()
        self.cam_thread = None

        # init lidar
        self.lidar_data_cache = DataCache()
        self.lidar = LivoxLidar(self.lidar_data_cache)
        self.lidar_projector = RangeProjection(
            fov_up=12.55, fov_down=-12.55, fov_left=-40.85, fov_right=40.85, 
            proj_h=300, proj_w=500
        )

        self.lidar_range = {
            "x": [0, 240],
            "y": [-120, 120],
            "z": [-5, 5],
            "intensity": [0, 255]
        }
        # ------------------ Debug -----------------------
        self.n_calls = 0
        self.total_time = 0
        self.total_time1 = 0
 
    def read_params(self, root_path: str):
        intrinsic_path = os.path.join(root_path, "intrinsic.txt")
        #intrinsic_path = os.path.join(root_path, "intrinsic_new.txt")
        distort_path = os.path.join(root_path, "distort.txt")
        extrinsic_path = os.path.join(root_path, "extrinsic.txt")

        with open(intrinsic_path, "r") as f:
            for line in f.readlines():
                    if line == "\n":
                        break
                    intrinsic = np.array([float(x) for x in line.split()]).reshape(3,4)
                    print('>>>> intrinsic >>>', intrinsic.shape, intrinsic)

        with open(distort_path, "r") as f:
            for line in f.readlines():
                    if line == "\n":
                        break
                    distort = np.array([float(x) for x in line.split()]).reshape(5, 1) #(k_1,k_2,p_1,p_2,k_3)
   
        with open(extrinsic_path, "r") as f:
            for line in f.readlines():
                    if line == "\n":
                        break
                    extrinsic = np.array([float(x) for x in line.split()]).reshape(4,4)
                    print('>>>> extrinsic >>>', extrinsic.shape, extrinsic)

        return intrinsic, distort, extrinsic

    def update_range(self, range_per_panel: float) -> None:
        self.range_per_panel = range_per_panel

    def exit(self):
        self.close_camera()

    def save_lidar_pcd(self, timestamp=None, save_format="pcd"):
        #print("pcd save format : ", save_format)
        pcd_np = self.lidar_data_cache.data
        if pcd_np is not None:
            print("### ", pcd_np.shape)
            if timestamp is None:
                timestamp = time.time()
            if not os.path.isdir(self.save_pcd_path):
                os.makedirs(self.save_pcd_path)
            if save_format=="bin":
                # save as .bin format
                save_path = os.path.join(
                    self.save_pcd_path, "{}.bin".format(timestamp))
                #if not os.path.isdir(self.save_pcd_path):
                #    os.makedirs(self.save_pcd_path)
                pcd_np.tofile(save_path)
            elif save_format=="pcd":
                # save as .pcd format
                save_path = os.path.join(
                    self.save_pcd_path, "{}.pcd".format(timestamp))
                fo = open(save_path, "a+")
                pcd_num = pcd_np.shape[0]
                # ----------- Save Title -------------
                fo.write("# .PCD v.7 - Point Cloud Data file format\n")
                fo.write("VERSION .7\n")
                fo.write("FIELDS x y z intensity\n")
                fo.write("SIZE 4 4 4 4\n")
                fo.write("TYPE F F F F\n")
                fo.write("COUNT 1 1 1 1\n")
                fo.write("WIDTH {}\n".format(pcd_num))
                fo.write("HEIGHT 1\n")
                fo.write("VIEWPOINT 0 0 0 1 0 0 0\n")
                fo.write("POINTS {}\n".format(pcd_num))
                fo.write("DATA ascii\n")

                # ----------- Save Points -------------
                for i in range(pcd_np.shape[0]):
                        fo.write("%.6f %.6f %.6f %.6f\n"%(pcd_np[i][0], pcd_np[i][1], pcd_np[i][2], pcd_np[i][3]))
            else:
                        print("Cannot save such format point cloud file!!!")              
            msg = "Capture PCD success"
        else:
            msg = "No pointcloud captured"
        return msg

    def fusion_process(self, pointcloud: np.ndarray, pointcloud_color: np.ndarray,
                       range_per_panel: float, n_panels: int):

        pointcloud_color[:, 3:7] = 1
  	
        pcd_xyzrgba = np.hstack((pointcloud[:, :3], pointcloud_color))
        pcd_range = np.linalg.norm(pointcloud[:, :3], 2, axis=1) 
   
        result_list = []
        for i in range(n_panels):
            min_range = i * range_per_panel
            max_range = min_range + range_per_panel
            mask = (pcd_range > min_range) & (pcd_range < max_range)          

            seg_pcd = pcd_xyzrgba[mask]
            proj_pointcloud = self.lidar_projector.doProjection_small(seg_pcd) # iris
            #proj_pointcloud, _, _, _ = self.lidar_projector.doProjection(seg_pcd) # iris
            result_list.append(proj_pointcloud[:, :, 3:6])
   
        return result_list

    def color_pcd_get(self, pcd: np.ndarray, rgb_img: np.ndarray):
        '''
        This function enables projecting the point cloud on the RGB image according to the intrisic and extrisic matrix
        pcd:  n, 3
        rgb_img:  h, w, 3	
        return: color_pcd: n, 3
        '''
        
        # set intrinsic, distort, extrisic
        ## intrisic: 3x3
        ## extrisic: 3x4
        ## distortion_coef: 5x1

        img_h, img_w = rgb_img.shape[:2]
        proj_matrix = np.matmul(self.intrinsic, self.extrinsic) # (3, 4)(4, 4)->(3, 4)
        pcd_number = pcd.shape[0]
        pcd_hoord = np.concatenate([pcd[:, :3], np.ones((pcd_number, 1))], axis=1)
        
        # using intrinsic and extrinsic to project the pcd into pixel coord
        mapped_pcd = np.matmul(proj_matrix, pcd_hoord.T).T #(3, 4) (4, n)->(n, 3)
        # scale z to 1
        mapped_pcd = mapped_pcd[:, :2] / np.expand_dims(mapped_pcd[:, 2], axis=1) # n, 2
        keep_idx = (mapped_pcd[:, 0] > 0) * (mapped_pcd[:, 0] < img_w) * (mapped_pcd[:, 1] > 0)  * (mapped_pcd[:, 1] < img_h)
        #print("keep_idx >> ", keep_idx.sum()) # 20982
        mapped_pcd = np.fliplr(mapped_pcd)
        mapped_pcd = mapped_pcd[keep_idx].astype(np.int32)
        h_idx = mapped_pcd[:, 0]
        w_idx = mapped_pcd[:, 1]
        #print("h_idx >> ", h_idx.min(), h_idx.max()) # 468 1505
        #print("w_idx >> ", w_idx.min(), w_idx.max()) #  57 3071
        
        # retrive color points
        color_pcd = np.zeros((pcd.shape[0], 3))
        color_pcd[keep_idx, :] = rgb_img[h_idx, w_idx, :]/255.

        return color_pcd
    
    def update_lidar_stack_frames(self, n_frames: int):
        self.lidar.update_stack_frames(n_frames)
        
    # -------------- camera functions ------------------

    def enum_camera(self) -> list:
        return self.cam.enum_devices()

    def open_camera(self, device_idx: str):
        msg = self.cam.open_device(device_idx=device_idx)
        if self.cam.device_open_flag:
            # start thread
            self.cam_thread_flag = True
            self.cam_thread = threading.Thread(target=self._cam_work_thread)
            self.cam_thread.start()
        return msg

    def close_camera(self):
        self.cam_thread_flag = False
        # stop grab cam thread
        if self.cam_thread is not None:
            self.cam_thread.join()
            self.cam_thread = None

        # close device
        msg = self.cam.close_device()
        return msg

    def _cam_work_thread(self):
        while self.cam_thread_flag:
            cam_data = self.cam.get_data()
            # get camera data
            if cam_data is not None:
                self.cam_data_cache.writeData(cam_data)

    def get_cam_parameter(self) -> Dict:
        msg = self.cam.get_parameter()
        return {
            "frame_rate": 5, #self.cam.frame_rate,
            "exposure_time": self.cam.exposure_time,
            "gain": 20 #self.cam.gain
        }

    def set_cam_parameter(self, frame_rate: str, exposure_time: str, gain: str):
        return self.cam.set_parameter(frame_rate=frame_rate, exposure_time=exposure_time, gain=gain)

    def save_cam_jpeg(self, timestamp=None):
        img_np = self.cam_data_cache.data
        if img_np is not None:
            if timestamp is None:
                timestamp = time.time()
            save_path = os.path.join(
                self.save_img_path, "{}.jpeg".format(timestamp))
            if not os.path.isdir(self.save_img_path):
                os.makedirs(self.save_img_path)
            im = Image.fromarray(img_np)
            im.save(save_path)
            msg = "Capture image success"
        else:
            msg = "No image captured"
        return msg
