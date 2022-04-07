from PyQt5 import QtWidgets, uic, QtGui, QtCore
import sys
import os

import time
from PyQt5.QtCore import QThread, pyqtSignal
import numpy as np

import deepstream
import numpy as np
import pyqtgraph.opengl as gl

class CanvasThread(QThread):
    signal = pyqtSignal(np.ndarray)
    def __init__(self, data: deepstream.DataCache):
        super().__init__()
        self.data = data
        
    def run(self):
        while True:
            if self.data.update_state:
                self.signal.emit(self.data.readData())
                time.sleep(0.01)
            else:
                time.sleep(0.01)

class QtApplication(QtWidgets.QDialog):
    def __init__(self, ui_file=""):
        super(QtApplication, self).__init__()
        if not os.path.isfile(ui_file):
            raise FileNotFoundError("UI file not found: {}".format(ui_file))
        uic.loadUi(ui_file, self)
        
        # ----------- debug --------------------------
        self.total_time = 0
        self.n_call = 0
        self.time_color = 0
        self.pcd_sample_num = 21000 #21000
        self.pointcloud = np.ones((self.pcd_sample_num, 3))
        #self.colormap = plt.cm.jet
        #self.color_jet = np.ones((self.pcd_sample_num, 4))
        self.colormap = np.load('colormap_jet.npy')[:, 1:] #.astype(np.float32)

        
        # ----------- parameters ---------------------
        self.panel_width = 700 #500
        self.panel_height = 420 #300
        self.n_panels = 6 # iris
        self.cam_open_flag = False
        self.deep_stream = deepstream.DeepStream(
            range_per_panel=10, n_panel=self.n_panels, params_path="./params")

        # ----------- qt view cache -----------------     
        self.btn_cam_enum.clicked.connect(self._btn_cam_enum)
        self.btn_cam_open_close.clicked.connect(self._btn_open_cam)
        self.btn_cam_get_param.clicked.connect(self._btn_cam_get_param)
        self.btn_cam_set_param.clicked.connect(self._btn_cam_set_param)
        self.txt_cam_framerate.setValidator(QtGui.QDoubleValidator())
        self.txt_cam_gain.setValidator(QtGui.QDoubleValidator())
        self.txt_cam_exposure_time.setValidator(QtGui.QDoubleValidator())
        self.btn_cam_save_jpeg.clicked.connect(self._btn_cam_capture)
        self.hslider_pointsize.setMinimum(1)
        self.hslider_pointsize.setMaximum(10)
        self.hslider_pointsize.setSingleStep(1)
        self.hslider_pointsize.setValue(3)
        self.hslider_pointsize.valueChanged.connect(self._hslider_ps_change)
        self.txt_pointsize.setText(str(self.hslider_pointsize.value()))

        self.hslider_stack_frames.setMinimum(1)
        self.hslider_stack_frames.setMaximum(10)
        self.hslider_stack_frames.setSingleStep(1)
        self.hslider_stack_frames.setValue(1)
        self.hslider_stack_frames.valueChanged.connect(self._hslider_sframe_change)
        self.txt_stack_frames.setText(str(self.hslider_stack_frames.value()))

        self.combo_pcl_color_mode.addItems(["x", "y", "z", "intensity"])
        self.combo_pcl_color_mode.setCurrentIndex(3)
        self.radio_auto_color_align.setChecked(True)
        self.color_mode = self.combo_pcl_color_mode.currentText()       

        self.btn_set_color.clicked.connect(self._btn_set_color) 
        self.btn_lidar_capture.clicked.connect(self._btn_lidar_capture)
        self.btn_both_capture.clicked.connect(self._btn_capture_all)

        # -------------------------------------------------------------------
        # add point cloud canvas
        self.pcl_viewer = gl.GLViewWidget()
        self.hlayout_pcl.addWidget(self.pcl_viewer, 1)
        g = gl.GLGridItem()
        g.setSize(100, 100)
        g.setSpacing(5, 5)
        self.pcl_viewer.addItem(g)
        self.gl_pcl = None 


        # -------------------------------------------------------------------
        # add projection canvas
        self.proj_viewer = gl.GLViewWidget()
        self.view_fused.addWidget(self.proj_viewer, 1)
        g = gl.GLGridItem()
        g.setSize(100, 100)
        g.setSpacing(5, 5)
        self.proj_viewer.addItem(g)
        self.gl_proj = None 

        # -------------------------------------------------------------------
        # start canvas threading
        self.lidar_canvas_thread = None
        self.cam_canvas_thread = None

    def run(self):
        if self.lidar_canvas_thread is None:
            # update lidar canvas
            self.lidar_canvas_thread = CanvasThread(self.deep_stream.lidar_data_cache)
            self.lidar_canvas_thread.signal.connect(self._pointcloud_view_callback)
            self.lidar_canvas_thread.start()
            print("Start lidar thread !")
        else:
            self.txt_info.setPlainText("Treading has been started")
        if self.cam_canvas_thread is None:
            # update cam canvas
            self.cam_canvas_thread = CanvasThread(self.deep_stream.cam_data_cache)
            self.cam_canvas_thread.signal.connect(self._rgb_image_callback)
            self.cam_canvas_thread.start()
            print("Start camera thread !")
        else:
            self.txt_info.setPlainText("Camera thread has been started")
        self.show()

    def closeEvent(self, a0: QtGui.QCloseEvent) -> None:
        self.deep_stream.exit()
        return super().closeEvent(a0)

    ## ------------ LIDAR events ------------------
    def _hslider_ps_change(self):
        value = self.hslider_pointsize.value()
        self.txt_pointsize.setText(str(value))

    def _btn_lidar_capture(self):
        msg = self.deep_stream.save_lidar_pcd()
        self.txt_info.setPlainText(msg)
    
    ## ------------ ALGORITHM events --------------
    def _hslider_sframe_change(self):
        value = self.hslider_stack_frames.value()
        self.txt_stack_frames.setText(str(value))
        self.deep_stream.update_lidar_stack_frames(value)
    
    def _btn_capture_all(self):
        timestamp = time.time()
        msg_pcd = self.deep_stream.save_lidar_pcd(timestamp)
        msg_img = self.deep_stream.save_cam_jpeg(timestamp)
        self.txt_info.setPlainText("{}/{}".format(msg_pcd, msg_img))
    
    def _btn_set_color(self):
        self.color_mode = self.combo_pcl_color_mode.currentText()

    ## ------------ CAM events ---------------------

    def _btn_cam_enum(self):
        cam_list = self.deep_stream.enum_camera()
        if len(cam_list) != 0:
            self.cam_combo_enum.addItems(cam_list)
        else:
            self.txt_info.setPlainText("No device found")

    def _btn_open_cam(self):
        if self.cam_open_flag:
            # close camera
            msg = self.deep_stream.close_camera()
            self.btn_cam_open_close.setText("Open Device")
            self.cam_open_flag = False
        else:
            if self.cam_combo_enum.count() == 0:
                msg = "No device found"
            else:
                cam_idx = self.cam_combo_enum.currentIndex()
                msg = self.deep_stream.open_camera(str(cam_idx))
                self.btn_cam_open_close.setText("Close Device")
                self.cam_open_flag = True 
        self.txt_info.setPlainText(msg)    

    def _btn_cam_get_param(self):
        cam_param = self.deep_stream.get_cam_parameter()
        self.txt_cam_framerate.setText("{:0.4f}".format(cam_param["frame_rate"]))
        self.txt_cam_exposure_time.setText("{:0.4f}".format(cam_param["exposure_time"]))
        self.txt_cam_gain.setText("{:0.4f}".format(cam_param["gain"]))
    
    def _btn_cam_set_param(self):
        frame_rate = self.txt_cam_framerate.text()
        exposure_time = self.txt_cam_exposure_time.text()
        gain = self.txt_cam_gain.text()
        msg = self.deep_stream.set_cam_parameter(
            frame_rate=frame_rate, exposure_time=exposure_time, gain=gain
        )
        self.txt_info.setPlainText(msg)
        #self._btn_cam_get_param()
        self.txt_cam_framerate.setText("{}".format(frame_rate))
        self.txt_cam_exposure_time.setText("{}".format(exposure_time))
        self.txt_cam_gain.setText("{}".format(gain))

    def _btn_cam_capture(self):
        msg = self.deep_stream.save_cam_jpeg()
        self.txt_info.setPlainText(msg)

    ## ----------------------------------------------------------

        
    def _pointcloud_view_callback(self, pointcloud:np.ndarray):
        pointcloud = pointcloud[:self.pcd_sample_num, :]#.astype(np.float32)

        # check point cloud type
        if pointcloud.shape[1] != 4:
            raise ValueError("Invalid point cloud shape: {}".format(pointcloud.shape))
        ps_size = self.hslider_pointsize.value()

        # get point cloud color 
        #color_mode = self.combo_pcl_color_mode.currentText()
        if self.color_mode == "x":
            color_data = pointcloud[:, 0]
        elif self.color_mode == "y":
            color_data = pointcloud[:, 1]
        elif self.color_mode == "z":
            color_data = pointcloud[:, 2]
        else:
            color_data = pointcloud[:, 3]

        is_auto_color_align = self.radio_auto_color_align.isChecked()

        if is_auto_color_align:
            #min_value = color_data.min()
            #max_value = color_data.max()
            min_value = np.min(color_data)
            max_value = np.max(color_data)
        else:
            value_range = self.deep_stream.lidar_range[self.color_mode]
            min_value = value_range[0]
            max_value = value_range[1]

        norm_color = (color_data - min_value) / (max_value - min_value + 1e-6) * 255.
        #norm_color = np.subtract(color_data, min_value) / (max_value - min_value + 1e-6)

        color_jet = self.colormap[norm_color.astype(np.int8), : ]#[:, 1:]
        #color_jet = plt.cm.jet(norm_color)  # iris

        if self.gl_pcl is None:
            #self.pcd_size=ps_size*np.ones((self.pcd_sample_num)) * 0.01
            self.pcd_size= ps_size * 0.01
            self.gl_pcl = gl.GLScatterPlotItem(
                pos=pointcloud[:, :3], 
                size=ps_size*np.ones((pointcloud.shape[0])) * 0.01,
                #size=ps_size,
                color=color_jet, 
                pxMode=False)
            self.gl_pcl.setGLOptions('opaque')
            self.pcl_viewer.addItem(self.gl_pcl)
        else:
            #print("pcd callback !!")#, pointcloud[:, :3].min() )
            self.gl_pcl.setData(
                pos=pointcloud[:, :3], 
                size=ps_size*np.ones((pointcloud.shape[0])) * 0.01,
                #size=ps_size, # - 10 ms
                color=color_jet
                )
        
    def _rgb_image_callback(self, rgb_img: np.ndarray):
        # # check shape
        #print("img callback !")
        #print("rgb_img shape ", rgb_img.shape) # h, w, 3
        rgb_scene = self.rgb2qtscene(rgb_img=rgb_img, width=self.panel_width-2, height=self.panel_height-2)
        self.view_rgb.setScene(rgb_scene)

        # --------- render fusion pannel ---------
        pointcloud = self.deep_stream.lidar_data_cache.readDataOnly()
        pointcloud = pointcloud[:self.pcd_sample_num, :]#.astype(np.float32)
        #print("pointcloud ", pointcloud.shape)

        color_pcd = self.deep_stream.color_pcd_get(pointcloud, rgb_img) 

        ps_size = self.hslider_pointsize.value()
        #print("ps_size || hslider_stack_frames :", ps_size, self.hslider_stack_frames.value())
        #color_jet = np.random.randint(low=0, high=255, size=(self.pcd_sample_num, 3))

        if self.gl_proj is None:
            #self.pcd_size=ps_size*np.ones((self.pcd_sample_num)) * 0.01
            self.pcd_size= ps_size * 0.01
            self.gl_proj = gl.GLScatterPlotItem(
                pos=pointcloud[:, :3], 
                size=ps_size*np.ones((pointcloud.shape[0])) * 0.01,
                #size=ps_size,
                color=color_pcd, 
                pxMode=False)
            self.gl_proj.setGLOptions('opaque')
            self.proj_viewer.addItem(self.gl_proj)
        else:
            self.gl_proj.setData(
                pos=pointcloud[:, :3], 
                size=ps_size*np.ones((pointcloud.shape[0])) * 0.01,
                #size=ps_size, # - 10 ms
                color=color_pcd
                )

        
    @staticmethod
    def rgb2qtscene(rgb_img: np.ndarray, width:int, height:int):
        if rgb_img.ndim != 3:
            raise ValueError("Invalid rgb image shape: {}".format(rgb_img.ndim))

        # convert rgb image to qt format
        qt_img = QtGui.QImage(rgb_img, rgb_img.shape[1], rgb_img.shape[0], QtGui.QImage.Format_RGB888)
        pix_img = QtGui.QPixmap.fromImage(qt_img)
        scaled_img = pix_img.scaled(width, height, QtCore.Qt.KeepAspectRatio)
        pix_item = QtWidgets.QGraphicsPixmapItem(scaled_img)
        rgb_scene = QtWidgets.QGraphicsScene()
        rgb_scene.addItem(pix_item)
        return rgb_scene

app = QtWidgets.QApplication(sys.argv)
window = QtApplication("./ui/app_linux.ui")
window.run()
sys.exit(app.exec_())
