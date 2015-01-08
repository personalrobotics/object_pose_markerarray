#!/usr/bin/env python
PACKAGE = 'object_pose_markerarray'
#import roslib; roslib.load_manifest(PACKAGE)
import numpy, rospy, rospkg, copy, yaml, json
from visualization_msgs.msg import Marker, MarkerArray
import percy.pose, time, math
from tf.transformations import quaternion_matrix,quaternion_from_matrix

rospack = rospkg.RosPack()
object_list = ['plate','glass','bowl']
TAG_TO_CONSIDER = 27

# Read settings from config file - check object_pose_apriltags/config folder to see the files it has
def get_fixed_tag_positions():

    obj_to_tags = {}
    tag_to_obj = {}

    for obj in object_list:
        filepath = rospack.get_path(PACKAGE)+'/config/'+obj+'.config.json'
        print filepath
        object_tag_data = json.load(open(filepath))
        obj_to_tags[obj] = {}
        
        for tag_str in object_tag_data:
            tag=int(tag_str[3:])
            tag_to_obj[tag] = obj
            obj_to_tags[obj][tag] = numpy.matrix(object_tag_data[tag_str])
        

    return obj_to_tags,tag_to_obj



#Given the observed markers, it relates them to the stored tags and publishes
def publish_object_poses(input_markers):

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
        tag_detections[tag_id] = [tag_pose]

    obj_tag_poses = {}


    for tag in tag_detections:
        if tag not in tag_to_obj: #Not a noted marker
            continue

        #Get the object for that tag
        tag_obj = tag_to_obj[tag]

        # Multiply apriltags value with pre-stored value
        tag_to_cam = tag_detections[tag]*obj_to_tags[tag_obj][tag]
        if tag_obj not in obj_tag_poses:
            #Only append
            obj_tag_poses[tag_obj] = {}

        obj_tag_poses[tag_obj][tag] = tag_to_cam 

    print(obj_tag_poses)
    for obj in obj_tag_poses:

        
        for tag in obj_tag_poses[obj]: 
            obj_pose_transform = obj_tag_poses[obj][tag]
            output_marker = Marker()
            output_marker.header.stamp = rospy.Time.now()
            output_marker.header.frame_id = input_markers.markers[0].header.frame_id
            output_marker.ns = obj
            output_marker.id = tag
            output_marker.type = Marker.CUBE
            output_marker.action = output_marker.ADD
            output_marker.pose.position.x = obj_pose_transform[0,3]
            output_marker.pose.position.y = obj_pose_transform[1,3]
            output_marker.pose.position.z = obj_pose_transform[2,3]
            output_marker.scale.x = 0.01
            output_marker.scale.y = 0.01
            output_marker.scale.z = 0.01
            output_marker.color.r = 1.0
            output_marker.color.a = 1.0
            output_quat = quaternion_from_matrix(obj_pose_transform)
            output_marker.pose.orientation.x = output_quat[0]
            output_marker.pose.orientation.y = output_quat[1]
            output_marker.pose.orientation.z = output_quat[2]
            output_marker.pose.orientation.w = output_quat[3]
            output_marker.mesh_use_embedded_materials = False
            output_markers.markers.append(output_marker)
            
        
    obj_pose_pub.publish(output_markers)  

    

obj_pose_transform_list = {}
rospy.init_node('object_poses', anonymous=True)
obj_to_tags,tag_to_obj = get_fixed_tag_positions()

#Will publish to this marker array
obj_pose_pub = rospy.Publisher('object_poses_array', MarkerArray)

#THIS CURRENTLY SUBSCRIBES TO THE OUTPUT OF THE TRACKER - YOU CAN MAKE IT DIRECTLY SUBSCRIBE TO APRILTAGS IF YOU WANT
sub_topic = rospy.get_param('~marker_array','/apriltags/marker_array')
marker_sub = rospy.Subscriber(sub_topic,MarkerArray,publish_object_poses)
rospy.spin()


