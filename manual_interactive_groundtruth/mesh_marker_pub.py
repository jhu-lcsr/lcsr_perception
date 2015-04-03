#!/usr/bin/env python

from __future__ import print_function

import rospy
from visualization_msgs.msg import Marker

import csv
import os
import copy
import rospy

import threading

from lcsr_barrett.wam_teleop import *

from std_msgs.msg import Header
from visualization_msgs.msg import *
from geometry_msgs.msg import *

from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *

import tf_conversions.posemath as pm

class ObjectCloud():
    def __init__(self):

        frame_id = rospy.get_param('~frame_id', '/world')
        n_objects = rospy.get_param('~n_objects', 1)
        object_class = rospy.get_param('~object_class', 'object')
        mesh_resource = rospy.get_param('~mesh_uri')
        input_file = rospy.get_param('~input_file','')
        output_file = rospy.get_param('~output_file')

        # read input file
        initial_poses = []
        if os.path.exists(input_file):
            rospy.loginfo("reading poses from file: "+input_file)
            with open(input_file, 'rb') as csvfile:
                posereader = csv.reader(csvfile, delimiter=' ')
                for x,y,z,qx,qy,qz,qw in posereader:
                    initial_poses.append(Pose(Point(float(x),float(y),float(z)),Quaternion(float(qx),float(qy),float(qz),float(qw))))

        self.objects = []
        for i in range(n_objects):
            initially_set = True
            if i >= len(initial_poses):
                initial_poses.append(Pose())
                initial_poses[-1].orientation.w = 1.0
                initially_set = False

            self.objects.append(ObjectMarker(frame_id, object_class, i, initial_poses[i]))
            if initially_set:
                self.objects[-1].moved = True

        self.marker_pub = rospy.Publisher('%s_objects' % object_class, MarkerArray)

        r = rospy.Rate(30.0)
        while not rospy.is_shutdown():
            msg = MarkerArray()
            for obj in self.objects:
                if obj.moved:
                    rospy.logdebug("update %s" % obj.name)
                    obj.update_marker_pose()
                    obj.publish_transform()

                    m = Marker()
                    m.lifetime = rospy.Duration(1.0)
                    m.ns = object_class
                    m.id = obj.object_id
                    m.header.frame_id = frame_id
                    m.header.stamp = rospy.Time.now()
                    m.type = Marker.MESH_RESOURCE
                    m.mesh_resource = mesh_resource 
                    m.scale.x = m.scale.y = m.scale.z = 1.0
                    m.color.r = 1.0
                    m.color.g = m.color.b = 0.2
                    m.color.a = 0.5

                    m.pose = obj.pose
                    msg.markers.append(m)

            self.marker_pub.publish(msg)

            r.sleep()

        rospy.loginfo("writing poses to file: "+output_file)
        with open(output_file, 'wb') as f:
            posewriter = csv.writer(f, delimiter=' ')
            for obj in self.objects:
                if obj.moved:
                    posewriter.writerow([
                        obj.pose.position.x,
                        obj.pose.position.y,
                        obj.pose.position.z,
                        obj.pose.orientation.x,
                        obj.pose.orientation.y,
                        obj.pose.orientation.z,
                        obj.pose.orientation.w])

class ObjectMarker():
    def __init__(self, frame_id, object_class, object_id, initial_pose):

        # Get WAMTeleop params
        self.frame_id = frame_id
        self.object_id = object_id

        # create marker server
        self.name ="%s_%d" % (object_class, object_id) 
        self.server = InteractiveMarkerServer(self.name)

        # tf
        self.transform = TransformStamped(
            Header(0,rospy.Time.now(),self.name),
            self.frame_id,
            Transform(Vector3(),Quaternion(0,0,0,1)))
        self.transform.transform.translation = initial_pose.position
        self.transform.transform.rotation = initial_pose.orientation
        self.broadcaster = tf.TransformBroadcaster()
        self.listener = tf.TransformListener()

        self.moved = False
        self.pose = initial_pose
        self.pose.orientation.w = 1.0

        # create marker
        self.int_marker = InteractiveMarker()
        self.int_marker.header.frame_id = self.frame_id
        self.int_marker.name = self.name
        self.int_marker.description = self.name
        self.int_marker.scale = rospy.get_param('~marker_scale', 0.1);
        self.int_marker.pose = initial_pose

        # add visual marker for the center
        m = Marker()
        m.type = Marker.SPHERE
        m.scale.x = m.scale.y = m.scale.z = self.int_marker.scale / 4.0
        m.color.r = m.color.g = m.color.b = 0.5
        m.color.a = 0.5

        imc = InteractiveMarkerControl()
        imc.always_visible = True
        imc.orientation_mode = InteractiveMarkerControl.VIEW_FACING
        imc.interaction_mode = InteractiveMarkerControl.MOVE_PLANE
        imc.independent_marker_orientation = True
        imc.markers.append(m)
        self.int_marker.controls.append(imc)

        imc = InteractiveMarkerControl()
        imc.name = "rotate_x"
        imc.always_visible = True
        imc.orientation = Quaternion(1,0,0,1)
        imc.orientation_mode = InteractiveMarkerControl.INHERIT
        imc.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        self.int_marker.controls.append(imc)

        imc = InteractiveMarkerControl()
        imc.name = "translate_x"
        imc.always_visible = True
        imc.orientation = Quaternion(1,0,0,1)
        imc.orientation_mode = InteractiveMarkerControl.INHERIT
        imc.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        self.int_marker.controls.append(imc)

        imc = InteractiveMarkerControl()
        imc.name = "rotate_y"
        imc.always_visible = True
        imc.orientation = Quaternion(0,1,0,1)
        imc.orientation_mode = InteractiveMarkerControl.INHERIT
        imc.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        self.int_marker.controls.append(imc)

        imc = InteractiveMarkerControl()
        imc.name = "translate_y"
        imc.always_visible = True
        imc.orientation = Quaternion(0,1,0,1)
        imc.orientation_mode = InteractiveMarkerControl.INHERIT
        imc.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        self.int_marker.controls.append(imc)

        imc = InteractiveMarkerControl()
        imc.name = "rotate_z"
        imc.always_visible = True
        imc.orientation = Quaternion(0,0,1,1)
        imc.orientation_mode = InteractiveMarkerControl.INHERIT
        imc.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        self.int_marker.controls.append(imc)

        imc = InteractiveMarkerControl()
        imc.name = "translate_z"
        imc.always_visible = True
        imc.orientation = Quaternion(0,0,1,1)
        imc.orientation_mode = InteractiveMarkerControl.INHERIT
        imc.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        self.int_marker.controls.append(imc)
        self.server.insert(self.int_marker, self.marker_cb)

        #self.menu_handler = MenuHandler()
        #resync_entry = self.menu_handler.insert( "Resync", callback=self.menu_resync_cb)
        #grasp_entry = self.menu_handler.insert( "Grasp", callback=self.menu_grasp_cb)
        #release_entry = self.menu_handler.insert( "Release", callback=self.menu_release_cb)

        #self.menu_handler.setCheckState(resync_entry, MenuHandler.UNCHECKED)
        #self.menu_handler.setCheckState(grasp_entry, MenuHandler.UNCHECKED)
        #self.menu_handler.setCheckState(release_entry, MenuHandler.UNCHECKED)
        #self.menu_handler.apply(self.server, self.int_marker.name)

        # apply the changes
        self.server.applyChanges()

        # Create python thread for sending command
        #self.thread = threading.Thread(target=self.update_thread)
        #self.thread.start()

    def menu_grasp_cb(self, msg):
        handle = msg.menu_entry_id
        checked = self.menu_handler.getCheckState(handle) == MenuHandler.CHECKED
        if not checked:
            self.menu_handler.setCheckState(handle, MenuHandler.CHECKED)
            rospy.loginfo('grasp')
            self.finger_ref = 0.0
        else:
            self.menu_handler.setCheckState(handle, MenuHandler.UNCHECKED)
            self.finger_ref = 0.5

        self.transform.header.stamp = rospy.Time.now()

        self.menu_handler.reApply(self.server)
        self.server.applyChanges()

    def menu_release_cb(self, msg):
        handle = msg.menu_entry_id
        checked = self.menu_handler.getCheckState(handle) == MenuHandler.CHECKED
        if not checked:
            self.menu_handler.setCheckState(handle, MenuHandler.CHECKED)
            rospy.loginfo('release')
            self.finger_ref = 1.0
        else:
            self.menu_handler.setCheckState(handle, MenuHandler.UNCHECKED)
            self.finger_ref = 0.5

        self.transform.header.stamp = rospy.Time.now()

        self.menu_handler.reApply(self.server)
        self.server.applyChanges()

    def menu_resync_cb(self, msg):
        handle = msg.menu_entry_id
        checked = self.menu_handler.getCheckState(handle) == MenuHandler.CHECKED
        if not checked:
            self.menu_handler.setCheckState(handle, MenuHandler.CHECKED)
            rospy.loginfo('resync')
            self.resync_pose = True
        else:
            self.menu_handler.setCheckState(handle, MenuHandler.UNCHECKED)
            self.resync_pose = False

        self.transform.header.stamp = rospy.Time.now()

        self.menu_handler.reApply(self.server)
        self.server.applyChanges()

    def marker_cb(self, msg):

        rospy.logdebug("%s updated %s"  % (self.name, str(msg.header.stamp)))

        self.moved = True
        ts = copy.copy(self.transform)
        ts.header.stamp = rospy.Time.now()
        ts.transform.translation = msg.pose.position
        ts.transform.rotation = msg.pose.orientation

        self.pose = msg.pose
        self.transform = ts

        self.server.setPose( self.name, msg.pose )
        self.server.applyChanges()

    def publish_transform(self):

        t = copy.copy(self.transform)
        p = t.transform.translation
        q = t.transform.rotation

        self.broadcaster.sendTransform(
            translation = (p.x, p.y, p.z),
            rotation = (q.x, q.y, q.z, q.w),
            time = rospy.Time.now(),
            child = self.name,
            parent = self.frame_id)

    def update_marker_pose(self):
        self.server.setPose(
            self.name,
            Pose(self.transform.transform.translation, self.transform.transform.rotation))
        self.server.applyChanges()

    def update_thread(self):
        """"""

        r = rospy.Rate(30.0)
        while not rospy.is_shutdown():

            # update poses
            self.update_marker_pose()
            self.publish_transform()

            r.sleep()

def main():
    rospy.init_node('mesh_marker_pub')

    cloud = ObjectCloud()

if __name__ == '__main__':
    main()
