##################
## this block is libraries installed with target if libraries exist default pip remove it
# import sys

# # Specify the target directory where the packages are installed
# target_directory = '/mnt/d/ocr/libs'

# # Add the target directory to sys.path
# sys.path.append(target_directory)
# ###################

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String

from cv_bridge import CvBridge

import numpy as np

import cv2
import threading

watch_key = None



def create_contour(image, cost):
    min_left, max_right, max_height, min_height = 65536, 0, 0, 65536

    for row in range(image.shape[0]):
        for column in range(image.shape[1]):
            pixel = image[row, column]

            if pixel < 10:
                if column < min_left:
                    min_left = column
                if column > max_right:
                    max_right = column

                if row < min_height:
                    min_height = row
                if row > max_height:
                    max_height = row

    contour = {
        'lu': (min_left - cost, min_height - cost),
        'ru': (max_right + cost, min_height - cost),
        'rb': (max_right + cost, max_height + cost),
        'lb': (min_left - cost, max_height + cost)
    }

    return contour



# if keyboard is triggered for example o is typed. it will take arranged image from camera_callback and will make preprocesing of image
# and conversion of Image msg
# need to variable watch_key -- says o is triggered
# camera callback sees watch_key true and  
def keyboard_callback(key_typed : String):
    global watch_key
    key = key_typed.data
    
    if ( key == '7' ):
        watch_key = '7'
        print("key" + key)
        

    elif ( key == '9'):
        watch_key = '9'
        print("key" + key)
    
    elif ( key == '5' ):
        watch_key = '5'
        print("key" + key)

    else:
        watch_key = None
        # print("key" + watch_key)

def create_barrel_contour(rgb_image):
    print("create barrel contour camera...")
    # Convert camera data to RGB image
    
    # Apply Gaussian blur for HSV conversion
    blurred_image = cv2.GaussianBlur(rgb_image, (5, 5), 0, 0)

    # Convert RGB to HSV
    hsv_image = cv2.cvtColor(blurred_image, cv2.COLOR_RGB2HSV)

    # Create mask for red color
    lower_red = np.array([110, 50, 50])
    upper_red = np.array([130, 255, 255])
    mask = cv2.inRange(hsv_image, lower_red, upper_red)
    mask = 255 - mask
    contour = create_contour(mask, 15)
    
    return contour

def camera_callback(camera : Image, modified_image_pub ):
    
    global watch_key

    if(watch_key == '9'): # ocr
        print("9 is clicked")
        bridge = CvBridge()
        # Convert the image message to OpenCV format
        cv_image = bridge.imgmsg_to_cv2(camera, desired_encoding='rgb8')

        # Convert RGB image to BGR
        bgr_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)

        # Convert BGR image back to RGB
        rgb_image = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2RGB)


        kernel_size = 1
        grayscale_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2GRAY)
        kernel = grayscale_image[0:kernel_size, 0:kernel_size].copy()

        # Perform convolution
        result = cv2.filter2D(grayscale_image, -1, kernel)
        contour = create_contour(result, 15)

        points = np.array([contour['lu'], contour['ru'], contour['rb'], contour['lb']])

        # Convert grayscale image to color
        # color_image = cv2.cvtColor(grayscale_image, cv2.COLOR_GRAY2BGR)

        cv2.polylines(rgb_image, [points], True, (0, 255, 0), 2)
        # cv2.imshow("Result", rgb_image)
        # cv2.waitKey(0)
        # Convert the image back to sensor_msgs/Image format
        arranged_image_msg = bridge.cv2_to_imgmsg(rgb_image, encoding='rgb8')
        # camera.data = arranged_image_msg
        modified_image_pub.publish(arranged_image_msg)
        return
    
    elif(watch_key == '7'): # qr
        print("7 is clicked")
        bridge = CvBridge()
        # Convert the image message to OpenCV format
        cv_image = bridge.imgmsg_to_cv2(camera, desired_encoding='rgb8')

        # Convert RGB image to BGR
        bgr_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)

        # Convert BGR image back to RGB
        rgb_image = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2RGB)


        kernel_size = 1
        grayscale_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2GRAY)
        kernel = grayscale_image[0:kernel_size, 0:kernel_size].copy()

        # Perform convolution
        result = cv2.filter2D(grayscale_image, -1, kernel)
        contour = create_contour(result, 15)

        points = np.array([contour['lu'], contour['ru'], contour['rb'], contour['lb']])

        # Convert grayscale image to color
        # color_image = cv2.cvtColor(grayscale_image, cv2.COLOR_GRAY2BGR)

        cv2.polylines(rgb_image, [points], True, (0, 255, 0), 2)
        # cv2.imshow("Result", rgb_image)
        # cv2.waitKey(0)
        # Convert the image back to sensor_msgs/Image format
        arranged_image_msg = bridge.cv2_to_imgmsg(rgb_image, encoding='rgb8')
        # camera.data = arranged_image_msg
        modified_image_pub.publish(arranged_image_msg)
        return

    elif(watch_key == '5'): # qr
        print("5 is clicked")
        bridge = CvBridge()
        # Convert the image message to OpenCV format
        cv_image = bridge.imgmsg_to_cv2(camera, desired_encoding='rgb8')

        # Convert RGB image to BGR
        bgr_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)

        # Convert BGR image back to RGB
        rgb_image = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2RGB)


        kernel_size = 1
        grayscale_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2GRAY)
        
        contour = create_barrel_contour(rgb_image)

        points = np.array([contour['lu'], contour['ru'], contour['rb'], contour['lb']])

        # Convert grayscale image to color
        # color_image = cv2.cvtColor(grayscale_image, cv2.COLOR_GRAY2BGR)

        cv2.polylines(rgb_image, [points], True, (0, 255, 0), 2)
        # cv2.imshow("Result", rgb_image)
        # cv2.waitKey(0)
        # Convert the image back to sensor_msgs/Image format
        arranged_image_msg = bridge.cv2_to_imgmsg(rgb_image, encoding='rgb8')
        # camera.data = arranged_image_msg
        modified_image_pub.publish(arranged_image_msg)
        return

    # print("publish")
    modified_image_pub.publish(camera)

if __name__ == '__main__':
    rospy.init_node('object_detection')
    camera_sub = rospy.Subscriber('/rtg/camera/rgb/image_raw', Image, camera_callback)
    rospy.spin()