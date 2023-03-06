#!/usr/bin/env python3
# license removed for brevity
import rospy
import tf
from std_msgs.msg import Float64MultiArray

# def callback_sensor(msg):
#     send_tf("yumi_base_link", "sensor", msg.data[0],msg.data[1],msg.data[2],msg.data[3],msg.data[4],msg.data[5])

# def callback_surface(msg):
#     send_tf("yumi_base_link", "tf", msg.data[0],msg.data[1],msg.data[2],msg.data[3],msg.data[4],msg.data[5])


def callback_frame_tf(msg):
    send_tf("base", "tf", msg.data[0], msg.data[1],
            msg.data[2], msg.data[3], msg.data[4], msg.data[5])


def send_tf(parent, child, x, y, z, roll, pitch, yaw):
    # print("Sending transform")
    br.sendTransform((x, y, z),
                     tf.transformations.quaternion_from_euler(
                         roll, pitch, yaw),
                     rospy.Time.now(),
                     child,
                     parent)


rospy.init_node('talker', anonymous=True)
br = tf.TransformBroadcaster()
# rospy.Subscriber("/tf_sensor", Float64MultiArray, callback_sensor)
# rospy.Subscriber("/tf_surface", Float64MultiArray, callback_surface)
rospy.Subscriber("/frame_tf", Float64MultiArray, callback_frame_tf)

rospy.spin()
# if __name__ == '__main__':

#     try:
#         talker()
#     except rospy.ROSInterruptException:
#         pass
