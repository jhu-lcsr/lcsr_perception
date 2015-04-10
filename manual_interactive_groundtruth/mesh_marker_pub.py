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

        # get ROS paramters
        frame_id = rospy.get_param('~frame_id', '/world')
        n_objects = rospy.get_param('~n_objects', 1)
        object_class = rospy.get_param('~object_class', 'object')
        mesh_resource = rospy.get_param('~mesh_uri')
        input_file = rospy.get_param('~input_file','')
        output_file = rospy.get_param('~output_file')

        # read input file of initial poses
        initial_poses = []
        if os.path.exists(input_file):
            rospy.loginfo("reading poses from file: "+input_file)
            with open(input_file, 'rb') as csvfile:
                posereader = csv.reader(csvfile, delimiter=' ')
                for x,y,z,qx,qy,qz,qw in posereader:
                    initial_poses.append(Pose(Point(float(x),float(y),float(z)),Quaternion(float(qx),float(qy),float(qz),float(qw))))

        # create marker server
        self.name ="%s_groundtruth" % (object_class)
        self.server = InteractiveMarkerServer(self.name, q_size=1)

        # create tf hooks
        self.broadcaster = tf.TransformBroadcaster()
        self.listener = tf.TransformListener()

        # create interactive markers for each object
        self.objects = []
        for i in range(n_objects):
            initially_set = True
            if i >= len(initial_poses):
                initial_poses.append(Pose())
                initial_poses[-1].orientation.w = 1.0
                initially_set = False

            self.objects.append(ObjectMarker(initially_set, frame_id, object_class, mesh_resource, i, initial_poses[i], self.server, self.listener, self.broadcaster))
            if initially_set:
                self.objects[-1].enabled = True

        # initialize the server
        self.server.applyChanges()

        # main loop
        r = rospy.Rate(30.0)
        while not rospy.is_shutdown():
            for obj in self.objects:
                if obj.enabled:
                    rospy.logdebug("publish tf for %s" % obj.name)
                    obj.publish_transform()

            # update the marker server
            self.server.applyChanges()

            r.sleep()

        # save the final poses of all modified objects
        rospy.loginfo("writing poses to file: "+output_file)
        with open(output_file, 'wb') as f:
            posewriter = csv.writer(f, delimiter=' ')
            for obj in self.objects:
                if obj.enabled:
                    posewriter.writerow([
                        obj.pose.position.x,
                        obj.pose.position.y,
                        obj.pose.position.z,
                        obj.pose.orientation.x,
                        obj.pose.orientation.y,
                        obj.pose.orientation.z,
                        obj.pose.orientation.w])

class ObjectMarker():
    def __init__(self, enabled, frame_id, object_class, mesh_resource,  object_id, initial_pose, server, listener, broadcaster):

        # Get params
        self.frame_id = frame_id
        self.object_id = object_id
        self.object_class = object_class
        self.mesh_resource = mesh_resource

        self.initial_pose = copy.copy(initial_pose)

        self.enabled = enabled

        # create marker server
        self.name ="%s_%d" % (object_class, object_id)
        self.server = server

        # initialize pose
        self.pose = initial_pose

        # store tf structures
        self.transform = None
        self.broadcaster = broadcaster
        self.listener = listener

        # create marker
        self.int_marker = InteractiveMarker()
        self.int_marker.header.frame_id = self.frame_id
        self.int_marker.name = self.name
        self.int_marker.description = self.name
        self.int_marker.scale = rospy.get_param('~marker_scale', 0.1);
        self.int_marker.pose = self.pose

        # insert the interactive marker
        self.server.insert(self.int_marker, self.marker_cb)

        # create a menu control to show or hide the controls
        self.menu_handler = MenuHandler()
        show_controls_entry = self.menu_handler.insert("Show Controls", callback=self.show_controls_cb)
        reset_pose_entry = self.menu_handler.insert("Reset Pose", callback=self.reset_pose_cb)
        enable_entry = self.menu_handler.insert("Enable", callback=self.enable_cb)

        self.menu_handler.setCheckState(show_controls_entry, MenuHandler.UNCHECKED)
        self.menu_handler.setCheckState(enable_entry, MenuHandler.CHECKED if self.enabled else MenuHandler.UNCHECKED)
        self.menu_handler.apply(self.server, self.int_marker.name)

        # initialize controls
        self.init_controls()
        self.show_controls(False)
        self.set_enabled(self.enabled)

        # apply the changes
        self.server.applyChanges()

    def show_controls_cb(self, msg):
        handle = msg.menu_entry_id
        checked = self.menu_handler.getCheckState(handle) == MenuHandler.CHECKED
        if not checked:
            self.menu_handler.setCheckState(handle, MenuHandler.CHECKED)
            rospy.loginfo('Showing controls for %s' % self.name)
            self.show_controls(True)
        else:
            self.menu_handler.setCheckState(handle, MenuHandler.UNCHECKED)
            rospy.loginfo('Hiding controls for %s' % self.name)
            self.show_controls(False)

        self.menu_handler.reApply(self.server)

    def reset_pose_cb(self, msg):
        handle = msg.menu_entry_id
        checked = self.menu_handler.getCheckState(handle) == MenuHandler.CHECKED

        self.update_pose(copy.copy(self.initial_pose))

    def init_controls(self):
        self.int_marker.controls = []

        # only add controls if they're enabled
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

        # add visual marker for the center
        m = Marker()
        m.lifetime = rospy.Duration(1.0)
        m.ns = self.object_class
        m.id = self.object_id
        m.type = Marker.MESH_RESOURCE
        m.mesh_resource = self.mesh_resource
        m.mesh_use_embedded_materials = False
        m.scale.x = m.scale.y = m.scale.z = 1.0
        m.color.r = 1.0
        m.color.g = m.color.b = 0.2
        m.color.a = 0.3

        # this works for orientation, but not position
        #m.pose = copy.copy(self.pose)

        m.pose = Pose()
        m.pose.orientation.w = 1.0

        imc = InteractiveMarkerControl()
        imc.name="mesh"
        imc.always_visible = True
        imc.orientation_mode = InteractiveMarkerControl.VIEW_FACING
        imc.interaction_mode = InteractiveMarkerControl.MOVE_PLANE
        imc.independent_marker_orientation = True
        imc.markers.append(m)
        self.int_marker.controls.append(imc)

    def show_controls(self, enable):
        """enable or disable controls"""

        if enable:
            self.int_marker.controls.extend(self.disabled_controls)
            self.disabled_controls = []
        else:
            controls = copy.copy(self.int_marker.controls)
            self.int_marker.controls = []
            self.disabled_controls = []
            for con in controls:
                if con.name == 'mesh':
                    self.int_marker.controls.append(con)
                else:
                    self.disabled_controls.append(con)

    def enable_cb(self, msg):
        handle = msg.menu_entry_id
        checked = self.menu_handler.getCheckState(handle) == MenuHandler.CHECKED
        if not checked:
            self.menu_handler.setCheckState(handle, MenuHandler.CHECKED)
            rospy.loginfo('Enabling %s' % self.name)
            self.set_enabled(True)
        else:
            self.menu_handler.setCheckState(handle, MenuHandler.UNCHECKED)
            rospy.loginfo('Disabling %s' % self.name)
            self.set_enabled(False)

        self.menu_handler.reApply(self.server)

    def set_enabled(self, enable):
        # set the moved flag
        self.enabled = enable
        # set the color
        m = None
        for imc in self.int_marker.controls:
            if imc.name == 'mesh':
                m = imc.markers[0]

        if not m:
            return

        if enable:
            m.color.g = 1.0
            m.color.r = m.color.b = 0.2
        else:
            m.color.r = 1.0
            m.color.g = m.color.b = 0.2


    def marker_cb(self, msg):
        """Update the marker state on interactive marker feedback"""

        rospy.logdebug("%s updated %s"  % (self.name, str(msg.header.stamp)))

        # set this object as enabled
        self.update_pose(msg.pose)

    def update_pose(self, pose):
        """update the marker pose"""

        # store the pose / update the interactive marker
        self.pose = pose
        self.server.setPose(self.name, pose)
        rospy.logdebug(self.pose)

        # update the tf transform
        ts = TransformStamped()
        ts.header.frame_id = self.frame_id
        ts.header.stamp = rospy.Time.now()
        ts.transform.translation = pose.position
        ts.transform.rotation = pose.orientation
        self.transform = ts

    def publish_transform(self):
        """Publish the transform over tf"""

        if not self.transform:
            return

        # convenience vars
        t = self.transform
        p = t.transform.translation
        q = t.transform.rotation

        # broadast the transform
        self.broadcaster.sendTransform(
            translation = (p.x, p.y, p.z),
            rotation = (q.x, q.y, q.z, q.w),
            time = rospy.Time.now(),
            child = self.name,
            parent = self.frame_id)

def main():
    rospy.init_node('mesh_marker_pub')

    cloud = ObjectCloud()

if __name__ == '__main__':
    main()
