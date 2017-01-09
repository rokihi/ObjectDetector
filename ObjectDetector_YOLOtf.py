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
import RGBDCamera

# Import Service implementation class
# <rtc-template block="service_impl">
from TrajectoryPlanner_idl_example import *

# </rtc-template>

# Import Service stub modules
# <rtc-template block="consumer_import">
# </rtc-template>


# This module's spesification
# <rtc-template block="module_spec">
objectdetector_yolotf_spec = ["implementation_id", "ObjectDetector_YOLOtf", 
		 "type_name",         "ObjectDetector_YOLOtf", 
		 "description",       "YOLO_tensorflow", 
		 "version",           "1.1.0", 
		 "vendor",            "ota", 
		 "category",          "ObjectRecognition", 
		 "activity_type",     "STATIC", 
		 "max_instance",      "1", 
		 "language",          "Python", 
		 "lang_type",         "SCRIPT",
		 "conf.default.scale_x", "1.0",
		 "conf.default.scale_y", "1.0",
		 "conf.default.scale_z", "1.0",

		 "conf.__widget__.scale_x", "text",
		 "conf.__widget__.scale_y", "text",
		 "conf.__widget__.scale_z", "text",

         "conf.__type__.scale_x", "double",
         "conf.__type__.scale_y", "double",
         "conf.__type__.scale_z", "double",

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
		self._d_image = RTC.CameraImage(*image_arg)
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
		self._detectObjProvider = ObjectDetectionService_i()
		


		# initialize of configuration-data.
		# <rtc-template block="init_conf_param">
		"""
		
		 - Name:  scale_x
		 - DefaultValue: 1.0
		"""
		self._scale_x = [1.0]
		"""
		
		 - Name:  scale_y
		 - DefaultValue: 1.0
		"""
		self._scale_y = [1.0]
		"""
		
		 - Name:  scale_z
		 - DefaultValue: 1.0
		"""
		self._scale_z = [1.0]
		
		# </rtc-template>

		self.image_type = 'RGBDCameraImage'
		 
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
		self.bindParameter("scale_x", self._scale_x, "1.0")
		self.bindParameter("scale_y", self._scale_y, "1.0")
		self.bindParameter("scale_z", self._scale_z, "1.0")
		
		# Set InPort buffers
		self.addInPort("image",self._imageIn)
		self.addInPort("RGBDimage",self._RGBDimageIn)
		
		# Set OutPort buffers
		self.addOutPort("resultString",self._resultStringOut)
		
		# Set service provider to Ports
		self._ObjectDetectionPort.registerProvider("ObjectDetectionService", "Manipulation::ObjectDetectionService", self._detectObjProvider)
		
		# Set service consumers to Ports
		
		# Set CORBA Service Ports
		self.addPort(self._ObjectDetectionPort)
		
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
		
		self._d_result.data=[]
		self._detectObjProvider.setComp(self)
#		print('CreateWindow')
# 		cv2.namedWindow("ReceiveImage", cv.CV_WINDOW_AUTOSIZE)

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
		
# 		print('DestoryWindow ')
# 		cv2.destroyAllWindows()
	
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
		if self._imageIn.isNew():
			self.image_type='RTCCameraImage'
			self._d_image = self._imageIn.read()
			self._d_result.data=[]
 			self.datatype
			print self._d_image.height, self._d_image.width, self._d_image.bpp
 
			#cvimage = numpy.fromstring( self._d_image.pixels, dtype=numpy.uint8 ).reshape( self._d_image.height, self._d_image.width, -1 )
# 			#cv2.imshow('cameraimage',cvimage)
# 			#key = cv2.waitKey(1)
#  		
			#yolo.detect_from_cvmat(cvimage)
					
# 			for i in range(len(yolo.result)):
# 				x = int(yolo.result[i][1])
# 				y = int(yolo.result[i][2])
# 				w = int(yolo.result[i][3])
# 				h = int(yolo.result[i][4])
# 				print '	ID : ' + yolo.result[i][0] + ' , [x,y,w,h]=[' + str(x) + ',' + str(y) + ',' + str(w) + ',' + str(h)+'], Confidence = ' + str(yolo.result[i][5])
#   				
#   				for j in range (0, 4):
# 				  	self._d_result.data.append(str(int(yolo.result[i][j])))
				
# 				print self._d_result.data
		if self._RGBDimageIn.isNew():
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

