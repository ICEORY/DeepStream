
import sys
import numpy as np
# append dll path
## 嵌入式平台 aarch64架构
#sys.path.append("/opt/MVS/Samples/aarch64/Python/MvImport") 
## 64位linux系统
sys.path.append("/opt/MVS/Samples/64/Python/MvImport") #
 
from MvCameraControl_class import *

__all__ = ["HIKCamera"]

# 将返回的错误码转换为十六进制显示

def ToHexStr(num):
    chaDic = {10: 'a', 11: 'b', 12: 'c', 13: 'd', 14: 'e', 15: 'f'}
    hexStr = ""
    if num < 0:
        num = num + 2**32
    while num >= 16:
        digit = num % 16
        hexStr = chaDic.get(digit, str(digit)) + hexStr
        num //= 16
    hexStr = chaDic.get(num, str(num)) + hexStr
    return hexStr


def color_numpy(data, nWidth, nHeight):
    data_ = np.frombuffer(data, count=int(
        nWidth*nHeight*3), dtype=np.uint8, offset=0)
    data_r = data_[0:nWidth*nHeight*3:3]
    data_g = data_[1:nWidth*nHeight*3:3]
    data_b = data_[2:nWidth*nHeight*3:3]

    data_r_arr = data_r.reshape(nHeight, nWidth)
    data_g_arr = data_g.reshape(nHeight, nWidth)
    data_b_arr = data_b.reshape(nHeight, nWidth)
    numArray = np.zeros([nHeight, nWidth, 3], "uint8")

    numArray[:, :, 0] = data_r_arr
    numArray[:, :, 1] = data_g_arr
    numArray[:, :, 2] = data_b_arr
    return numArray


class HIKCamera(object):
    def __init__(self) -> None:
        self.deviceList = None
        self.device_open_flag = False
        self.obj_cam = None
        self.open_device_idx = 0
        self.frame_rate = 0
        self.gain = 0
        self.exposure_time = 0

        self.st_frame_info = None
        self.img_buff = None
        self.buf_cache = None
        self.n_payload_size = None

    # enum devices

    def enum_devices(self):
        self.deviceList = MV_CC_DEVICE_INFO_LIST()
        tlayerType = MV_GIGE_DEVICE | MV_USB_DEVICE
        ret = MvCamera.MV_CC_EnumDevices(tlayerType, self.deviceList)
        if ret != 0:
            print(">>> Enum devices fail! ret={}".format(ToHexStr(ret)))

        if self.deviceList.nDeviceNum == 0:
            print(">>> Find no device")

        print("Find {} devices!".format(self.deviceList.nDeviceNum))

        devList = []
        for i in range(0, self.deviceList.nDeviceNum):
            mvcc_dev_info = cast(self.deviceList.pDeviceInfo[i], POINTER(
                MV_CC_DEVICE_INFO)).contents
            if mvcc_dev_info.nTLayerType == MV_USB_DEVICE:
                print("u3v device: [{}]".format(i))
                chUserDefinedName = ""
                for per in mvcc_dev_info.SpecialInfo.stUsb3VInfo.chUserDefinedName:
                    if per == 0:
                        break
                    chUserDefinedName = chUserDefinedName + chr(per)
                print("device model name: {}".format(chUserDefinedName))

                strSerialNumber = ""
                for per in mvcc_dev_info.SpecialInfo.stUsb3VInfo.chSerialNumber:
                    if per == 0:
                        break
                    strSerialNumber = strSerialNumber + chr(per)
                print("user serial number: {}".format(strSerialNumber))
                devList.append(
                    "["+str(i)+"]USB: " + chUserDefinedName + "(" + str(strSerialNumber) + ")")
        return devList

    def open_device(self, device_idx) -> str:
        if self.deviceList is None:
            return "No device found"

        if not self.device_open_flag:
            # ch:选择设备并创建句柄 | en:Select device and create handle
            stDeviceList = cast(self.deviceList.pDeviceInfo[int(
                device_idx)], POINTER(MV_CC_DEVICE_INFO)).contents
            self.obj_cam = MvCamera()
            ret = self.obj_cam.MV_CC_CreateHandle(stDeviceList)
            if ret != 0:
                self.obj_cam.MV_CC_DestroyHandle()
                return "Create handle fail! ret={}".format(ToHexStr(ret))

            ret = self.obj_cam.MV_CC_OpenDevice(MV_ACCESS_Exclusive, 0)
            if ret != 0:
                return "Open device fail! ret={}".format(ToHexStr(ret))
                
            self.open_device_idx = device_idx
            self.device_open_flag = True

            stBool = c_bool(False)
            ret = self.obj_cam.MV_CC_GetBoolValue(
                "AcquisitionFrameRateEnable", stBool)
            if ret != 0:
                return "Get acquisition frame rate enable fail! ret[0x{}]".format(ToHexStr(ret))

            # ch:设置触发模式为off | en:Set trigger mode as off
            ret = self.obj_cam.MV_CC_SetEnumValue(
                "TriggerMode", MV_TRIGGER_MODE_OFF)
            if ret != 0:
                return "Set trigger mode fail! ret[0x{}]".format(ret)

            # get payload size
            stParam = MVCC_INTVALUE()
            memset(byref(stParam), 0, sizeof(MVCC_INTVALUE))
            ret = self.obj_cam.MV_CC_GetIntValue("PayloadSize", stParam)
            if ret != 0:
                return "Get payload size fail! ret=[0x{}]".format(ToHexStr(ret))

            nPayloadSize = stParam.nCurValue

            # start grabbing
            ret = self.obj_cam.MV_CC_StartGrabbing()
            if ret != 0:
                return "Start grabbing fail! ret=[0x{}]".format(ToHexStr(ret))

            self.buf_cache = (c_ubyte * nPayloadSize)()
            self.n_payload_size = nPayloadSize

            return "Open device success"
        else:
            return "Device has been opened"

    def get_data(self):
        if self.st_frame_info is None:
            self.st_frame_info = MV_FRAME_OUT_INFO_EX()
            memset(byref(self.st_frame_info), 0, sizeof(self.st_frame_info))
        
        ret = self.obj_cam.MV_CC_GetOneFrameTimeout(
            byref(self.buf_cache), self.n_payload_size, self.st_frame_info, 1000)

        if ret == 0:
            # print("get one frame: Width[{}], Height[{}], nFrameNum[{}]".format(
            #     self.st_frame_info.nWidth, self.st_frame_info.nHeight, self.st_frame_info.nFrameNum))
            nRGBSize = self.st_frame_info.nWidth * self.st_frame_info.nHeight * 3
            
        else:
            print("no data, nret = "+ToHexStr(ret))
            return

        # 转换像素结构体赋值
        stConvertParam = MV_CC_PIXEL_CONVERT_PARAM()
        memset(byref(stConvertParam), 0, sizeof(stConvertParam))
        stConvertParam.nWidth = self.st_frame_info.nWidth
        stConvertParam.nHeight = self.st_frame_info.nHeight
        stConvertParam.pSrcData = self.buf_cache
        stConvertParam.nSrcDataLen = self.st_frame_info.nFrameLen
        stConvertParam.enSrcPixelType = self.st_frame_info.enPixelType
        stConvertParam.enDstPixelType = PixelType_Gvsp_RGB8_Packed
        stConvertParam.pDstBuffer = (c_ubyte * nRGBSize)()
        stConvertParam.nDstBufferSize = nRGBSize

        ret = self.obj_cam.MV_CC_ConvertPixelType(stConvertParam)
        if ret != 0:
            print("convert pixel fail! ret=[0x{}]".format(ret))

        if self.img_buff is None:
            self.img_buff = (c_ubyte * stConvertParam.nDstLen)()
        memmove(byref(self.img_buff), stConvertParam.pDstBuffer, stConvertParam.nDstLen)
        numArray = color_numpy(
            self.img_buff, self.st_frame_info.nWidth, self.st_frame_info.nHeight)

        return numArray

    def close_device(self) -> str:
        if self.device_open_flag:
            # free buffer
            self.img_buff = None 
            self.buf_cache = None 

            # stop grabbing
            ret = self.obj_cam.MV_CC_StopGrabbing()
            if ret != 0:
                return "Stop grabbing fail! ret=[0x{}]".format(ToHexStr(ret))

            # close device
            ret = self.obj_cam.MV_CC_CloseDevice()
            if ret != 0:
                return "Close device fail! ret=[0x{}]".format(ToHexStr(ret))

            # destroy handel
            self.obj_cam.MV_CC_DestroyHandle()
            self.device_open_flag = False
            return "Close device success"
        else:
            return "No device opened"

    def get_parameter(self) -> str:
        if self.device_open_flag:
            stFloatParam_FrameRate = MVCC_FLOATVALUE()
            memset(byref(stFloatParam_FrameRate), 0, sizeof(MVCC_FLOATVALUE))
            stFloatParam_exposureTime = MVCC_FLOATVALUE()
            memset(byref(stFloatParam_exposureTime),
                   0, sizeof(MVCC_FLOATVALUE))
            stFloatParam_gain = MVCC_FLOATVALUE()
            memset(byref(stFloatParam_gain), 0, sizeof(MVCC_FLOATVALUE))
            ret = self.obj_cam.MV_CC_GetFloatValue(
                "AcquisitionFrameRate", stFloatParam_FrameRate)
            if ret != 0:
                return "Get acquistion frame rate fail! ret=[0x{}]".format(ToHexStr(ret))
            self.frame_rate = stFloatParam_FrameRate.fCurValue
            
            ret = self.obj_cam.MV_CC_GetFloatValue(
                "ExposureTime", stFloatParam_exposureTime)
            if ret != 0:
                return "Get exposure time fail! ret=[0x{}]".format(ToHexStr(ret))
            self.exposure_time = stFloatParam_exposureTime.fCurValue
            
            ret = self.obj_cam.MV_CC_GetFloatValue("Gain", stFloatParam_gain)
            if ret != 0:
                return "Get gain fail! ret=[0x{}]".format(ToHexStr(ret))
            self.gain = stFloatParam_gain.fCurValue
            return "Get camera parameters success"
        else:
            return "No device opened"
            
    def set_parameter(self, frame_rate: str, exposure_time: str, gain: str) -> str:
        """get camera parameters: frame rate, exposure time, gain

        :param frame_rate: frame rate to acquist image
        :type frame_rate: str
        :param exposure_time: exposure time
        :type exposure_time: str
        :param gain: gain
        :type gain: str
        """
        if frame_rate == "" or exposure_time == "" or gain == "":
            return "Please type in the text box!"

        if self.device_open_flag:
            ret = self.obj_cam.MV_CC_SetFloatValue(
                "ExposureTime", float(exposure_time))
            if ret != 0:
                return "Set exposure time fail! ret=[0x{}]".format(ToHexStr(ret))

            ret = self.obj_cam.MV_CC_SetFloatValue("Gain", float(gain))
            if ret != 0:
                return "Set gain fail! ret=[0x{}]".format(ToHexStr(ret))

            ret = self.obj_cam.MV_CC_SetFloatValue(
                "AcquisitionFrameRate", float(frame_rate))
            if ret != 0:
                return "Set acquistion frame rate fail! ret={}".format(ToHexStr(ret))

            self.exposure_time = exposure_time
            self.frame_rate = frame_rate
            self.gain = gain
            return "Set parameter success"
        else:
            return "No device opened"
