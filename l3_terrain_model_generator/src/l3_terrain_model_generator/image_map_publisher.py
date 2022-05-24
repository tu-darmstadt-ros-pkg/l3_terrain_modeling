#!/usr/bin/env python
# simple script to publish a image from a file.
import rospy
import rospkg
import time
import cv2
import sensor_msgs.msg

def callback(self):
    """ Convert a image to a ROS compatible message
        (sensor_msgs.Image).
    """
    global publisher, imagePath, published

    if not published:
        img = cv2.imread(imagePath, cv2.IMREAD_UNCHANGED)

        rosimage = sensor_msgs.msg.Image()
        if img.dtype.itemsize == 2:
            if len(img.shape) == 3:
                if img.shape[2] == 3:
                    rosimage.encoding = 'bgr16'
                if img.shape[2] == 4:
                    rosimage.encoding = 'bgra16'
            else:
                rosimage.encoding = 'mono16'
        if img.dtype.itemsize == 1:
            if len(img.shape) == 3:
                if img.shape[2] == 3:
                    rosimage.encoding = 'bgr8'
                if img.shape[2] == 4:
                    rosimage.encoding = 'bgra8'
            else:
                rosimage.encoding = 'mono8'

        rosimage.width = img.shape[1]
        rosimage.height = img.shape[0]
        rosimage.step = img.strides[0]
        rosimage.data = img.tostring()
        rosimage.header.stamp = rospy.Time.now()
        rosimage.header.frame_id = 'world'

        publisher.publish(rosimage)
        published = True


#Main function initializes node and subscribers and starts the ROS loop
def main_program():
    global publisher, imagePath, published
    published = False
    rospack = rospkg.RosPack()
    rospy.init_node('l3_terrain_image_publisher')
    imagePath = rospy.get_param('~image_path')
    publisher = rospy.Publisher('raw_image_map', sensor_msgs.msg.Image, queue_size=10, latch=True)
    rospy.Timer(rospy.Duration(2), callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        main_program()
    except rospy.ROSInterruptException: pass
