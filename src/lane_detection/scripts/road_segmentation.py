import rospy
from std_msgs.msg import String
from keras.models import Sequential


def road_segmentation():
    rospy.init_node('road_segmentation')
    rate = rospy.Rate(10)

    pub = rospy.Publisher('chatter', String, queue_size=10)

    model = Sequential()

    while not rospy.is_shutdown():
        rate.sleep()
        pub.publish("test")


if __name__ == '__main__':
    try:
        road_segmentation()
    except rospy.ROSInterruptException:
        pass
