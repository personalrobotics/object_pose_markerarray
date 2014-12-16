#!/usr/bin/env python
PACKAGE = 'object_pose_apriltags'
#import roslib; roslib.load_manifest(PACKAGE)
import numpy, rospy, rospkg, copy, yaml
import rospkg
from visualization_msgs.msg import Marker, MarkerArray
import percy.pose, percy.detector, time, math
from tf.transformations import quaternion_matrix,quaternion_from_matrix

TAG_AT_CENTRE = 24
OBJECT_TO_ENTER = 'plate'

def get_tag_obj_transforms(input_markers):

    output_markers = MarkerArray()
    tag_detections = {}

    for marker in input_markers.markers:
        tag_id = marker.id
        position_data = marker.pose.position
        orientation_data = marker.pose.orientation

        tag_pose = numpy.matrix(quaternion_matrix([orientation_data.x,
                                                   orientation_data.y,
                                                   orientation_data.z,
                                                   orientation_data.w]))
        tag_pose[0,3] = position_data.x
        tag_pose[1,3] = position_data.y
        tag_pose[2,3] = position_data.z

        tag_detections[tag_id] = tag_pose
    

    #Setup the data structure to dump into config file
    # config_file_data = {}
    # config_file_data['objects'] = {}
    # config_file_data['objects'][OBJECT_TO_ENTER] = {}
    # config_file_data['objects'][OBJECT_TO_ENTER]['tags'] = {}
    if TAG_AT_CENTRE in tag_detections:
        transform = tag_detections[TAG_AT_CENTRE]
        tag_transforms = {}
        for tag in tag_detections:
            tag_transforms[tag] = numpy.linalg.inv(tag_detections[tag])*transform

        print (tag_transforms)

def get_transform_mat(position,orientation):

    tag_pose = numpy.matrix(quaternion_matrix([orientation[0],orientation[1],orientation[2],orientation[3]]))
    tag_pose[0,3] = position[0]
    tag_pose[1,3] = position[1]
    tag_pose[2,3] = position[2]

    return tag_pose
