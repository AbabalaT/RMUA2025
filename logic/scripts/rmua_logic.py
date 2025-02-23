#! /usr/bin/env python3
import rospy
from quadrotor_msgs.msg import GoalSet
from geometry_msgs.msg import PoseStamped

current_step = 0

def scheduler_callback(event):
    global current_step
    if current_step == 0:
        current_step = 1

# def rviz_point_callback(msg):
#     global goal_msg
#     goal_msg.drone_id = 0
#     goal_msg.goal = [msg.point.x, msg.point.y, msg.point.z]
#     goal_pos_pub.publish(goal_msg)

if __name__ == '__main__':
    rospy.init_node("logic_control_node")
    goal_pos_pub = rospy.Publisher("/goal_with_id", GoalSet, queue_size=10)
    scheduler_timer = rospy.Timer(rospy.Duration(0.1), scheduler_callback)
    # rviz_point_sub = rospy.Subscriber("/clicked_point", PoseStamped, rviz_point_callback)
    rospy.spin()