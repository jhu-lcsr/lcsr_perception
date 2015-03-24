#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker

def main():
    rospy.init_node('mesh_marker_pub')

    pub = rospy.Publisher('groundtruth_marker', Marker)

    m = Marker()
    m.header.frame_id = 'groundtruth'
    m.pose.orientation.w = 1.0
    s = m.scale
    s.x = s.y = s.z = 1.0
    c = m.color
    c.r = 1.0
    c.a = 0.6
    
    m.type = Marker.MESH_RESOURCE
    m.mesh_resource = rospy.get_param('~mesh_uri')

    r = rospy.Rate(10)

    while not rospy.is_shutdown():
        m.header.stamp = rospy.Time.now()
        pub.publish(m)

        r.sleep()

if __name__ == '__main__':
    main()
