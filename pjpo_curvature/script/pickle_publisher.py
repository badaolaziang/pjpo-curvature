#!/usr/bin/env python
import pickle
import sys
import rospy
from os import path
from pjpo_curvature.msg import CenterLine, Obstacles, DynamicObstacles
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped


def main():
    rospy.init_node('pickle_publisher')

    pickle_path = path.join(path.dirname(__file__), 'reference.pickle')
    if sys.argv[0].endswith('.pickle'):
        pickle_path = sys.argv[0]

    with open(pickle_path) as f:
        reference = pickle.load(f)

        center_pub = rospy.Publisher('/center_line', CenterLine, queue_size=1, latch=True)
        center_pub.publish(reference['center'])

        if 'static' in reference:
            obstacles_pub = rospy.Publisher('/obstacles', Obstacles, queue_size=1, latch=True)
            obstacles_pub.publish(reference['static'])

        if 'dynamic' in reference:
            obstacles_pub = rospy.Publisher('/dynamic_obstacles', DynamicObstacles, queue_size=1, latch=True)
            obstacles_pub.publish(reference['dynamic'])

        if 'path' in sys.argv:
            ref_pub = rospy.Publisher('/center_line_path', Path, queue_size=1, latch=True)
            msg = Path()
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = 'map'
            for pt in reference['center'].points:
                pose = PoseStamped()
                pose.header = msg.header
                pose.pose.position.x = pt.x
                pose.pose.position.y = pt.y
                msg.poses.append(pose)

            ref_pub.publish(msg)

    rospy.spin()


if __name__ == '__main__':
    main()
