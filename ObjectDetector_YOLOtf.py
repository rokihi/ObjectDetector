#!/usr/bin/env python
# -*- coding: utf-8 -*-
# -*- Python -*-

"""
 @file ObjectDetector_YOLOtf.py
 @brief YOLO_tensorflow
 @date $Date$


"""
import sys
import time
sys.path.append(".")

# Import RTM module
import RTC
import OpenRTM_aist

import TrajectoryPlanner_idl
import ManipulatorCommonInterface_MiddleLevel_idl
import RGBDCamera

# Import Service implementation class
# <rtc-template block="service_impl">
from TrajectoryPlanner_idl_example import *

# </rtc-template>

# Import Service stub modules
# <rtc-template block="consumer_import">
import Manipulation, Manipulation__POA
import JARA_ARM, JARA_ARM__POA


# </rtc-template>


# This module's spesification
# <rtc-template block="module_spec">
objectdetector_yolotf_spec = ["implementation_id", "ObjectDetector_YOLOtf", 
		 "type_name",         "ObjectDetector_YOLOtf", 
		 "description",       "YOLO_tensorflow", 
		 "version",           "1.1.2", 
		 "vendor",            "ota", 
		 "category",          "ObjectRecognition", 
		 "activity_type",     "STATIC", 
		 "max_instance",      "1", 
		 "language",          "Python", 
		 "lang_type",         "SCRIPT",
		 "conf.default.scale_x", "0.001",
		 "conf.default.scale_y", "0.001",
		 "conf.default.scale_z", "1.0",
		 "conf.default.camera_offset_x", "-0.056",
		 "conf.default.camera_offset_y", "-0.047",
		 "conf.default.camera_offset_z", "0.072",

		 "conf.__widget__.scale_x", "text",
		 "conf.__widget__.scale_y", "text",
		 "conf.__widget__.scale_z", "text",
		 "conf.__widget__.camera_offset_x", "text",
		 "conf.__widget__.camera_offset_y", "text",
		 "conf.__widget__.camera_offset_z", "text",

         "conf.__type__.scale_x", "double",
         "conf.__type__.scale_y", "double",
         "conf.__type__.scale_z", "double",
         "conf.__type__.camera_offset_x", "double",
         "conf.__type__.camera_offset_y", "double",
         "conf.__type__.camera_offset_z", "double",

		 ""]
# </rtc-template>

##
# @class ObjectDetector_YOLOtf
# @brief YOLO_tensorflow
# 
# 
class ObjectDetector_YOLOtf(OpenRTM_aist.DataFlowComponentBase):
	
	##
	# @brief constructor
	# @param manager Maneger Object
	# 
	def __init__(self, manager):
		OpenRTM_aist.DataFlowComponentBase.__init__(self, manager)

		image_arg = [None] * ((len(RTC._d_CameraImage) - 4) / 2)
		self._d_image = RTC.CameraImage(RTC.Time(0,0), 0,0,0,"",0.0,"")
		"""
		"""
		self._imageIn = OpenRTM_aist.InPort("image", self._d_image)
		RGBDimage_arg = [None] * ((len(RGBDCamera._d_TimedRGBDCameraImage) - 4) / 2)
		self._d_RGBDimage = RGBDCamera.TimedRGBDCameraImage(*RGBDimage_arg)
		"""
		"""
		self._RGBDimageIn = OpenRTM_aist.InPort("RGBDimage", self._d_RGBDimage)
		result_arg = [None] * ((len(RTC._d_TimedStringSeq) - 4) / 2)
		self._d_result = RTC.TimedStringSeq(RTC.Time(0,0),[])
		"""
		"""
		self._resultStringOut = OpenRTM_aist.OutPort("resultString", self._d_result)

		"""
		"""
		self._ObjectDetectionPort = OpenRTM_aist.CorbaPort("ObjectDetection")
		"""
		"""
		self._manipulatorCommonInterfaceMiddlePort = OpenRTM_aist.CorbaPort("manipulatorCommonInterfaceMiddle")

		"""
		"""
		self._detectObjProvider = ObjectDetectionService_i()
		

		"""
		"""
		self._ManipMiddle = OpenRTM_aist.CorbaConsumer(interfaceType=JARA_ARM.ManipulatorCommonInterface_Middle)

		# initialize of configuration-data.
		# <rtc-template block="init_conf_param">
		"""
		
		 - Name:  scale_x
		 - DefaultValue: 0.001
		"""
		self._scale_x = [0.001]
		"""
		
		 - Name:  scale_y
		 - DefaultValue: 0.001
		"""
		self._scale_y = [0.001]
		"""
		
		 - Name:  scale_z
		 - DefaultValue: 1.0
		"""
		self._scale_z = [1.0]
		"""
		
		 - Name:  camera_offset_x
		 - DefaultValue: 72
		"""
		self._camera_offset_x = [-0.056]
		"""
		
		 - Name:  camera_offset_y
		 - DefaultValue: -47
		"""
		self._camera_offset_y = [-0.047]
		"""
		
		 - Name:  camera_offset_z
		 - DefaultValue: 56
		"""
		self._camera_offset_z = [0.072]
		
		# </rtc-template>


		 
	##
	#
	# The initialize action (on CREATED->ALIVE transition)
	# formaer rtc_init_entry() 
	# 
	# @return RTC::ReturnCode_t
	# 
	#
	def onInitialize(self):
		# Bind variables and configuration variable
		self.bindParameter("scale_x", self._scale_x, "0.001")
		self.bindParameter("scale_y", self._scale_y, "0.001")
		self.bindParameter("scale_z", self._scale_z, "1.0")
		self.bindParameter("camera_offset_x", self._camera_offset_x, "0.072")
		self.bindParameter("camera_offset_y", self._camera_offset_y, "-0.047")
		self.bindParameter("camera_offset_z", self._camera_offset_z, "0.056")
		
		# Set InPort buffers
		self.addInPort("image",self._imageIn)
		self.addInPort("RGBDimage",self._RGBDimageIn)
		
		# Set OutPort buffers
		self.addOutPort("resultString",self._resultStringOut)
		
		# Set service provider to Ports
		self._ObjectDetectionPort.registerProvider("ObjectDetectionService", "Manipulation::ObjectDetectionService", self._detectObjProvider)
		
		# Set service consumers to Ports
		self._manipulatorCommonInterfaceMiddlePort.registerConsumer("JARA_ARM_ManipulatorCommonInterface_Middle", "JARA_ARM::ManipulatorCommonInterface_Middle", self._ManipMiddle)
		
		# Set CORBA Service Ports
		self.addPort(self._ObjectDetectionPort)
		self.addPort(self._manipulatorCommonInterfaceMiddlePort)
		
		return RTC.RTC_OK
	
	#	##
	#	# 
	#	# The finalize action (on ALIVE->END transition)
	#	# formaer rtc_exiting_entry()
	#	# 
	#	# @return RTC::ReturnCode_t
	#
	#	# 
	#def onFinalize(self):
	#
	#	return RTC.RTC_OK
	
	#	##
	#	#
	#	# The startup action when ExecutionContext startup
	#	# former rtc_starting_entry()
	#	# 
	#	# @param ec_id target ExecutionContext Id
	#	#
	#	# @return RTC::ReturnCode_t
	#	#
	#	#
	#def onStartup(self, ec_id):
	#
	#	return RTC.RTC_OK
	
	#	##
	#	#
	#	# The shutdown action when ExecutionContext stop
	#	# former rtc_stopping_entry()
	#	#
	#	# @param ec_id target ExecutionContext Id
	#	#
	#	# @return RTC::ReturnCode_t
	#	#
	#	#
	#def onShutdown(self, ec_id):
	#
	#	return RTC.RTC_OK
	
		##
		#
		# The activated action (Active state entry action)
		# former rtc_active_entry()
		#
		# @param ec_id target ExecutionContext Id
		# 
		# @return RTC::ReturnCode_t
		#
		#
	def onActivated(self, ec_id):
		
		self._detectObjProvider.setComp(self)
	
		return RTC.RTC_OK
	
		##
		#
		# The deactivated action (Active state exit action)
		# former rtc_active_exit()
		#
		# @param ec_id target ExecutionContext Id
		#
		# @return RTC::ReturnCode_t
		#
		#
	def onDeactivated(self, ec_id):
	
		return RTC.RTC_OK
	
		##
		#
		# The execution action that is invoked periodically
		# former rtc_active_do()
		#
		# @param ec_id target ExecutionContext Id
		#
		# @return RTC::ReturnCode_t
		#
		#
	def onExecute(self, ec_id):
		
		frame = self._ManipMiddle._ptr().getFeedbackPosCartesian()[1].carPos
		self._detectObjProvider.setBaseFrame(frame)
		
		if self._imageIn.isNew():
			self.image_type='RTCCameraImage'
			self._d_image = self._imageIn.read()
			self._d_result.data=[]
 			
			#print self._d_image.height, self._d_image.width, self._d_image.bpp
 
			#cvimage = numpy.fromstring( self._d_image.pixels, dtype=numpy.uint8 ).reshape( self._d_image.height, self._d_image.width, -1 )
 			#cv2.imshow('cameraimage',cvimage)
 			#key = cv2.waitKey(1)

		if self._RGBDimageIn.isNew():
			self.image_type='RGBDCameraImage'
			self._d_RGBDimage = self._RGBDimageIn.read()
			self._d_result.data=[]
			
		return RTC.RTC_OK
	
	#	##
	#	#
	#	# The aborting action when main logic error occurred.
	#	# former rtc_aborting_entry()
	#	#
	#	# @param ec_id target ExecutionContext Id
	#	#
	#	# @return RTC::ReturnCode_t
	#	#
	#	#
	#def onAborting(self, ec_id):
	#
	#	return RTC.RTC_OK
	
	#	##
	#	#
	#	# The error action in ERROR state
	#	# former rtc_error_do()
	#	#
	#	# @param ec_id target ExecutionContext Id
	#	#
	#	# @return RTC::ReturnCode_t
	#	#
	#	#
	#def onError(self, ec_id):
	#
	#	return RTC.RTC_OK
	
	#	##
	#	#
	#	# The reset action that is invoked resetting
	#	# This is same but different the former rtc_init_entry()
	#	#
	#	# @param ec_id target ExecutionContext Id
	#	#
	#	# @return RTC::ReturnCode_t
	#	#
	#	#
	#def onReset(self, ec_id):
	#
	#	return RTC.RTC_OK
	
	#	##
	#	#
	#	# The state update action that is invoked after onExecute() action
	#	# no corresponding operation exists in OpenRTm-aist-0.2.0
	#	#
	#	# @param ec_id target ExecutionContext Id
	#	#
	#	# @return RTC::ReturnCode_t
	#	#

	#	#
	#def onStateUpdate(self, ec_id):
	#
	#	return RTC.RTC_OK
	
	#	##
	#	#
	#	# The action that is invoked when execution context's rate is changed
	#	# no corresponding operation exists in OpenRTm-aist-0.2.0
	#	#
	#	# @param ec_id target ExecutionContext Id
	#	#
	#	# @return RTC::ReturnCode_t
	#	#
	#	#
	#def onRateChanged(self, ec_id):
	#
	#	return RTC.RTC_OK
	



def ObjectDetector_YOLOtfInit(manager):
    profile = OpenRTM_aist.Properties(defaults_str=objectdetector_yolotf_spec)
    manager.registerFactory(profile,
                            ObjectDetector_YOLOtf,
                            OpenRTM_aist.Delete)

def MyModuleInit(manager):
    ObjectDetector_YOLOtfInit(manager)

    # Create a component
    comp = manager.createComponent("ObjectDetector_YOLOtf")

def main():
	mgr = OpenRTM_aist.Manager.init(sys.argv)
	mgr.setModuleInitProc(MyModuleInit)
	mgr.activateManager()
	mgr.runManager()

if __name__ == "__main__":
	main()

