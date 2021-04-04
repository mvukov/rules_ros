import example_msgs.msg
import rospy


def callback(msg):
    print(msg)


def main():
    rospy.init_node("listener", anonymous=True)
    sub = rospy.Subscriber("chatter", example_msgs.msg.Example, callback)
    rospy.spin()


if __name__ == '__main__':
    main()
