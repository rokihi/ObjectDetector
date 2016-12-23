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

import sys
import time
sys.path.append(".")
# Import RTM module
#import RTC
#import OpenRTM_aist

import ExtendedDataTypes_idl
#from ExtendedDataTypes_idl import Pose3D, Orientation3D

import numpy
import cv2

import ObjectDetector_YOLOtf 
import YOLO_small_tf 
yolo= YOLO_small_tf.YOLO_TF()

class ObjectDetectionService_i (Manipulation__POA.ObjectDetectionService):
    """
    @class ObjectDetectionService_i
    Example class implementing IDL interface Manipulation.ObjectDetectionService
    """
    
    def setComp(self, comp):
        
        self.RTComp = comp
        self.RTComp.test()
        
        #yolo= YOLO_small_tf.YOLO_TF()
    
        yolo.disp_console = True
        yolo.imshow = True
        yolo.tofile_img = "RTC_result_img.jpg"
        yolo.tofile_txt = "RTC_result_txt.txt"
        yolo.filewrite_img = True
        yolo.filewrite_txt = True 
        
        
    def __init__(self):
        """
        @brief standard constructor
        Initialise member variables here
        """
        
        self.geometry=(0,0,0, 0,0,0, 0,0,0)
        
        self.objInfo=('',0,0,0,0,0,0)
        
        pass

    # void detectObject(in ObjectIdentifier objectID, out ObjectInfo objInfo)
    def detectObject(self, objectID):
        #raise CORBA.NO_IMPLEMENT(0, CORBA.COMPLETED_NO)
        # *** Implement me
        # Must return: objInfo
        print 'objectID:'+ objectID.name
        print self.objInfo
        
        cvimage = numpy.fromstring( self.RTComp._d_image.pixels, dtype=numpy.uint8 ).reshape( self.RTComp._d_image.height, self.RTComp._d_image.width, -1 )
        yolo.detect_from_cvmat(cvimage)
        
        
        for i in range(len(yolo.result)):
            x = int(yolo.result[i][1])
            y = int(yolo.result[i][2])
            z = 0 #depthdata 
            w = int(yolo.result[i][3])
            h = int(yolo.result[i][4])
            
            print '    ID : ' + yolo.result[i][0] + ' , [x,y,w,h]=[' + str(x) + ',' + str(y) + ',' + str(w) + ',' + str(h)+'], Confidence = ' + str(yolo.result[i][5])
            
            self.RTComp._d_result.data.append(str(yolo.result[i][0]))
            
            
            for j in range (1, 4):

                self.RTComp._d_result.data.append(str(int(yolo.result[i][j])))
            
            if yolo.result[i][0]==objectID.name:
                
                #objInfo.pose = Pose3D(Point3D(x, y, z), Orientation3D(0, 0, 0))
                
                self.objInfo.pose =(x,y,z,0,0,0)
                print 'Picking Object : ' + self.objInfo
        
        return self.objInfo

    # void setGeometry(in RTC::Geometry3D geometry)
    def setGeometry(self, geometry):
        #raise CORBA.NO_IMPLEMENT(0, CORBA.COMPLETED_NO)
        # *** Implement me
        # Must return: None
        self.geometry = geometry 

# 
# class KinematicsSolverService_i (Manipulation__POA.KinematicsSolverService):
#     """
#     @class KinematicsSolverService_i
#     Example class implementing IDL interface Manipulation.KinematicsSolverService
#     """
# 
#     def __init__(self):
#         """
#         @brief standard constructor
#         Initialise member variables here
#         """
#         pass
# 
#     # void solveInverseKinematics(in ObjectInfo objInfo, out RobotJointInfo goalRobotJointInfo)
#     def solveInverseKinematics(self, objInfo):
#         raise CORBA.NO_IMPLEMENT(0, CORBA.COMPLETED_NO)
#         # *** Implement me
#         # Must return: goalRobotJointInfo
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
#     # boolean isCollide(in RobotIdentifier manipInfo, in RobotJointInfo jointSeq, out CollisionInfo collision)
#     def isCollide(self, manipInfo, jointSeq):
#         raise CORBA.NO_IMPLEMENT(0, CORBA.COMPLETED_NO)
#         # *** Implement me
#         # Must return: result, collision
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
#     # void planManipulation(in RobotIdentifier robotID, in RobotJointInfo startRobotJointInfo, in RobotJointInfo goalRobotJointInfo, out ManipulationPlan manipPlan)
#     def planManipulation(self, robotID, startRobotJointInfo, goalRobotJointInfo):
#         raise CORBA.NO_IMPLEMENT(0, CORBA.COMPLETED_NO)
#         # *** Implement me
#         # Must return: manipPlan
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
#     # void getModelInfo(in RobotIdentifier robotID, out RobotJointInfo robotInfo)
#     def getModelInfo(self, robotID):
#         raise CORBA.NO_IMPLEMENT(0, CORBA.COMPLETED_NO)
#         # *** Implement me
#         # Must return: robotInfo
# 
#     # void getMeshInfo(in RobotIdentifier robotID, out MeshInfo mesh)
#     def getMeshInfo(self, robotID):
#         raise CORBA.NO_IMPLEMENT(0, CORBA.COMPLETED_NO)
#         # *** Implement me
#         # Must return: mesh
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
#     # void followManipPlan(in ManipulationPlan manipPlan)
#     def followManipPlan(self, manipPlan):
#         raise CORBA.NO_IMPLEMENT(0, CORBA.COMPLETED_NO)
#         # *** Implement me
#         # Must return: None
# 
#     # void getCurrentRobotJointInfo(in RobotIdentifier robotID, out RobotJointInfo robotJoint)
#     def getCurrentRobotJointInfo(self, robotID):
#         raise CORBA.NO_IMPLEMENT(0, CORBA.COMPLETED_NO)
#         # *** Implement me
#         # Must return: robotJoint


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

