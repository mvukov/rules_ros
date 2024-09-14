import example_msgs.msg
import rospy


def callback(msg):
    rospy.loginfo(f'Python listener received: {msg.message}')


def main():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('chatter', example_msgs.msg.Example, callback)
    rospy.spin()


if __name__ == '__main__':
    main()
