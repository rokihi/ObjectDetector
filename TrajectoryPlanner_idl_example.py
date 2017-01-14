#!/usr/bin/env python
# -*- coding: utf-8 -*-
# -*- Python -*-

"""
 @file TrajectoryPlanner_idl_examplefile.py
 @brief Python example implementations generated from TrajectoryPlanner.idl
 @date $Date$


"""

import omniORB
from omniORB import CORBA, PortableServer
import Manipulation, Manipulation__POA
import RTC

import sys
import time
sys.path.append(".")
# Import RTM module
# import RTC
# import OpenRTM_aist
import ExtendedDataTypes_idl
# from ExtendedDataTypes_idl import Pose3D, Orientation3D

import numpy as np
import cv2

import ObjectDetector_YOLOtf 
import YOLO_small_tf 
yolo = YOLO_small_tf.YOLO_TF()

class ObjectDetectionService_i (Manipulation__POA.ObjectDetectionService):
    """
    @class ObjectDetectionService_i
    Example class implementing IDL interface Manipulation.ObjectDetectionService
    """
    def setComp(self, comp):
        
        self.RTComp = comp
        # self.RTComp.test()
        
        # yolo= YOLO_small_tf.YOLO_TF()

    def __init__(self):
        """
        @brief standard constructor
        Initialise member variables here
        """
        self.geometry = (0, 0, 0, 0, 0, 0, 0, 0, 0)
        
        self.objectID = ''
        self.pose = (0, 0, 0, 0, 0, 0)
        self.objInfo = Manipulation.ObjectInfo(self.objectID, self.pose)
        self.frame = [[]]
        
        yolo.disp_console = True
        yolo.imshow = True
        yolo.tofile_img = "RTC_result_img.jpg"
        yolo.tofile_txt = "RTC_result_txt.txt"
        yolo.filewrite_img = True
        yolo.filewrite_txt = True 
        
        pass
    
    def getDepth(self, x, y, cameraimg_width, depthimg_width):
        min_distance = 0.2
        max_distance = 2.0
        
        ratio = depthimg_width * 1.0 / cameraimg_width
        
        position = int(x * ratio) * depthimg_width + int(y * ratio)  # target position in depth image
        # print position
        avg_scope = 15
        
        depth_around = np.array(self.RTComp._d_RGBDimage.data.depthImage.raw_data[position - avg_scope - depthimg_width:position + avg_scope + 1 - depthimg_width] + 
                        self.RTComp._d_RGBDimage.data.depthImage.raw_data[position - avg_scope : position + avg_scope + 1] + 
                        self.RTComp._d_RGBDimage.data.depthImage.raw_data[position - avg_scope + depthimg_width:position + avg_scope + 1 + depthimg_width])
        if depth_around.sum() == 0:
            depth = 0
        else:
            depth = depth_around.sum() / np.nonzero(depth_around)[0].size  # average without 0 
            
        # print depth_around
        
        if depth < min_distance:
            depth = min_distance
        elif depth > max_distance:
            depth = max_distance
        
        return depth  # * 1000 # [mm]
    
    
    def transCameraToRobot(self, pos_in_img, imgw, imgh):
        # mat34 = RTComp._manipMiddle._ptr().getFeedbackPosCartesian()[1].carPos  # [0] is RETURN_ID, [1] is CarPosWithElbow
        
        mat34 = self.frame
        print np.round(mat34, 1)
        mat34 = np.array(mat34)
        mat44 = np.concatenate((mat34, [[0, 0, 0, 1]]), axis=0)
        # print np.linalg.det(mat44)
        inv44 = np.linalg.inv(mat44)
        
        pos_in_img = [pos_in_img[0] - imgw * 1.0 / 2, pos_in_img[1] - imgh * 1.0 / 2, pos_in_img[2]]  # pos from center 
        print self.RTComp._scale_y[0]
        
        scale_x = 0.001
        scale_y = 0.001
        pos_from_camera = np.array([[pos_in_img[0] * scale_x,
                                 pos_in_img[1] * scale_y,
                                 pos_in_img[2] * 1.0, 1]]).T
        # pos_from_arm = np.array([[1, 0, 0.5, 1]]).T
        ofs_x=-self.RTComp._camera_offset_x[0]
        ofs_y=-self.RTComp._camera_offset_y[0]
        ofs_z=-self.RTComp._camera_offset_z[0]
        print ofs_x,ofs_y,ofs_z
        ofs_x=-0.056
        ofs_y=-0.047
        ofs_z=0.072
        trans_cam_arm = np.array([[0, 1, 0, -ofs_x],
                                  [-1, 0, 0, -ofs_y],
                                  [0, 0, 1, -ofs_z],
                                  [0, 0, 0, 1]])
        pos_from_arm = np.dot(trans_cam_arm, pos_from_camera)

        pos_from_robo = np.dot(mat44, pos_from_arm)
        
        print "pos_in_img   :" + str(np.array(pos_in_img))
        print "pos_from_cam :" + str(pos_from_camera.T)
        print "pos_from_arm :" + str(pos_from_arm.T)
        print "pos_from_robo:" + str(pos_from_robo.T)
        print 
        
        return pos_from_robo
    
    # ReturnValue detectObject(in ObjectIdentifier objectID, out ObjectInfo objInfo)
    def detectObject(self, objectID):
        # raise CORBA.NO_IMPLEMENT(0, CORBA.COMPLETED_NO)
        # *** Implement me
        # Must return: objInfo
        # print 'objectID:' + objectID.name
        self.objInfo.objectID = objectID.name
        self.objInfo = Manipulation.ObjectInfo(Manipulation.ObjectIdentifier(objectID.name),RTC.Pose3D(RTC.Point3D(0.0,0.0,0.0), RTC.Orientation3D(0.0,0.0,0.0)))
        print self.objInfo
        
        if self.RTComp.image_type == 'RTCCameraImage':
            print 'format: ' + self.RTComp._d_image.format
            cvimage = np.fromstring(self.RTComp._d_image.pixels, dtype=np.uint8).reshape(self.RTComp._d_image.height, self.RTComp._d_image.width, -1)
        
        elif self.RTComp.image_type == 'RGBDCameraImage':
            imgw = self.RTComp._d_RGBDimage.data.cameraImage.image.width
            imgh = self.RTComp._d_RGBDimage.data.cameraImage.image.height
            imgformat = self.RTComp._d_RGBDimage.data.cameraImage.image.format
            dimgw = self.RTComp._d_RGBDimage.data.depthImage.width
            dimgh = self.RTComp._d_RGBDimage.data.depthImage.height   

            print 'Color Format: ' + str(imgformat)
            
            if str(imgformat) == 'CF_RGB':
                cvimage = np.fromstring(self.RTComp._d_RGBDimage.data.cameraImage.image.raw_data, dtype=np.uint8).reshape(imgh, imgw, -1)
            else:
                print "Error: only CF_RGB is available."
            
        yolo.detect_from_cvmat(cvimage)


        for i in range(len(yolo.result)):
            w = yolo.result[i][3]
            h = yolo.result[i][4]
            x = yolo.result[i][1]
            y = yolo.result[i][2]
            z = 0
            
            if self.RTComp.image_type == 'RGBDCameraImage':
                z = self.getDepth(x, y, imgw, dimgw)
            
            target_pos = self.transCameraToRobot([x, y, z], imgw, imgh)
             
            x = target_pos[0, 0]
            y = target_pos[1, 0]
            z = target_pos[2, 0]
            
            print '    ID' + str(i) + ': ' + yolo.result[i][0] + ', [x,y,z,w,h (mm)]=[' + str(int(x*1000)) + ',' + str(int(y*1000)) + ',' + str(int(z*1000)) + \
                 ',' + str(int(w)) + ',' + str(int(h)) + '], Confidence = ' + str(('%03.3f' % yolo.result[i][5]))
            
            self.RTComp._d_result.data.append(str(yolo.result[i][0]))            
            for j in range (1, 4):
                self.RTComp._d_result.data.append(str(int(yolo.result[i][j])))
            
            if yolo.result[i][0] == objectID.name:
                
                # objInfo.pose = Pose3D(Point3D(x, y, z), Orientation3D(0, 0, 0))

                self.objInfo.pose.position.x = x
                self.objInfo.pose.position.y = y
                self.objInfo.pose.position.z = z
                #self.objInfo.pose.orientation.r = 0.0
                
                print 'Picking Object: ' + str(self.objInfo)
                
                break
            
        result = Manipulation.ReturnValue(Manipulation.OK,"Detected")
        return (result, self.objInfo)

    # ReturnValue setBaseFrame(in Matrix34 frame)
    def setBaseFrame(self, frame):
        # raise CORBA.NO_IMPLEMENT(0, CORBA.COMPLETED_NO)
        # *** Implement me
        # Must return: result
        self.frame = frame
        # print np.round(self.frame, 1)
        result = (0, "set Base Frame")
        return result


# class ObjectHandleStrategyService_i (Manipulation__POA.ObjectHandleStrategyService):
#     """
#     @class ObjectHandleStrategyService_i
#     Example class implementing IDL interface Manipulation.ObjectHandleStrategyService
#     """
# 
#     def __init__(self):
#         """
#         @brief standard constructor
#         Initialise member variables here
#         """
#         pass
# 
#     # ReturnValue getApproachOrientation(in ObjectInfo objInfo, out EndEffectorPose eePos)
#     def getApproachOrientation(self, objInfo):
#         raise CORBA.NO_IMPLEMENT(0, CORBA.COMPLETED_NO)
#         # *** Implement me
#         # Must return: result, eePos
# 
# 
# 
# class KinematicSolverService_i (Manipulation__POA.KinematicSolverService):
#     """
#     @class KinematicSolverService_i
#     Example class implementing IDL interface Manipulation.KinematicSolverService
#     """
# 
#     def __init__(self):
#         """
#         @brief standard constructor
#         Initialise member variables here
#         """
#         pass
# 
#     # ReturnValue solveKinematics(in EndEffectorPose targetPose, in JointAngleSeq startJointAngles, out JointAngleSeq targetJointAngles)
#     def solveKinematics(self, targetPose, startJointAngles):
#         raise CORBA.NO_IMPLEMENT(0, CORBA.COMPLETED_NO)
#         # *** Implement me
#         # Must return: result, targetJointAngles
# 
# 
# 
# class CollisionDetectionService_i (Manipulation__POA.CollisionDetectionService):
#     """
#     @class CollisionDetectionService_i
#     Example class implementing IDL interface Manipulation.CollisionDetectionService
#     """
# 
#     def __init__(self):
#         """
#         @brief standard constructor
#         Initialise member variables here
#         """
#         pass
# 
#     # ReturnValue isCollide(in RobotIdentifier robotID, in JointAngleSeq jointAngles, out CollisionPairSeq collisions)
#     def isCollide(self, robotID, jointAngles):
#         raise CORBA.NO_IMPLEMENT(0, CORBA.COMPLETED_NO)
#         # *** Implement me
#         # Must return: result, collisions
# 
# 
# 
# class ManipulationPlannerService_i (Manipulation__POA.ManipulationPlannerService):
#     """
#     @class ManipulationPlannerService_i
#     Example class implementing IDL interface Manipulation.ManipulationPlannerService
#     """
# 
#     def __init__(self):
#         """
#         @brief standard constructor
#         Initialise member variables here
#         """
#         pass
# 
#     # ReturnValue planManipulation(in RobotJointInfo jointsInfo, in JointAngleSeq startJointAngles, in JointAngleSeq goalJointAngles, out ManipulationPlan manipPlan)
#     def planManipulation(self, jointsInfo, startJointAngles, goalJointAngles):
#         raise CORBA.NO_IMPLEMENT(0, CORBA.COMPLETED_NO)
#         # *** Implement me
#         # Must return: result, manipPlan
# 
# 
# 
# class ModelServerService_i (Manipulation__POA.ModelServerService):
#     """
#     @class ModelServerService_i
#     Example class implementing IDL interface Manipulation.ModelServerService
#     """
# 
#     def __init__(self):
#         """
#         @brief standard constructor
#         Initialise member variables here
#         """
#         pass
# 
#     # ReturnValue getModelInfo(in RobotIdentifier robotID, out RobotJointInfo jointsInfo)
#     def getModelInfo(self, robotID):
#         raise CORBA.NO_IMPLEMENT(0, CORBA.COMPLETED_NO)
#         # *** Implement me
#         # Must return: result, jointsInfo
# 
#     # ReturnValue getMeshInfo(in RobotIdentifier robotID, out MeshInfo mesh)
#     def getMeshInfo(self, robotID):
#         raise CORBA.NO_IMPLEMENT(0, CORBA.COMPLETED_NO)
#         # *** Implement me
#         # Must return: result, mesh
# 
# 
# 
# class MotionGeneratorService_i (Manipulation__POA.MotionGeneratorService):
#     """
#     @class MotionGeneratorService_i
#     Example class implementing IDL interface Manipulation.MotionGeneratorService
#     """
# 
#     def __init__(self):
#         """
#         @brief standard constructor
#         Initialise member variables here
#         """
#         pass
# 
#     # ReturnValue followManipPlan(in ManipulationPlan manipPlan)
#     def followManipPlan(self, manipPlan):
#         raise CORBA.NO_IMPLEMENT(0, CORBA.COMPLETED_NO)
#         # *** Implement me
#         # Must return: result
# 
#     # ReturnValue getCurrentRobotJointAngles(out JointAngleSeq jointAngles)
#     def getCurrentRobotJointAngles(self):
#         raise CORBA.NO_IMPLEMENT(0, CORBA.COMPLETED_NO)
#         # *** Implement me
#         # Must return: result, jointAngles


if __name__ == "__main__":
    import sys
    
    # Initialise the ORB
    orb = CORBA.ORB_init(sys.argv)
    
    # As an example, we activate an object in the Root POA
    poa = orb.resolve_initial_references("RootPOA")

    # Create an instance of a servant class
    servant = ObjectDetectionService_i()

    # Activate it in the Root POA
    poa.activate_object(servant)

    # Get the object reference to the object
    objref = servant._this()
    
    # Print a stringified IOR for it
    print orb.object_to_string(objref)

    # Activate the Root POA's manager
    poa._get_the_POAManager().activate()

    # Run the ORB, blocking this thread
    orb.run()

