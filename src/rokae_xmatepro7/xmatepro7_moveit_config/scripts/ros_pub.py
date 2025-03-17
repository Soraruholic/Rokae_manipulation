import rospy
from Robotic.RosNode import RosPublisher

if __name__ == '__main__':
    try:
        joint_state_publisher = RosPublisher()
        joint_state_publisher.publish_joint_states()
    except rospy.ROSInterruptException:
        pass
    