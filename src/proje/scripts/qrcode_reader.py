#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
# from pyzbar import pyzbar
import zbarlight
from PIL import Image as PILImage

def decode_qr_code(image):
    # Convert the image to grayscale
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    cv2.imwrite("image.png", image)
    # Change brightness/contrast
    high_contrast = cv2.convertScaleAbs(gray, alpha=2, beta=10)
   
    # # Make binary image
    # high_contrast[high_contrast > 40] = 255
    # high_contrast[high_contrast <= 40] = 0

    cv2.imshow("image", high_contrast)

    # Convert the OpenCV image to PIL format
    pil_image = PILImage.fromarray(high_contrast)
    # rospy.loginfo(pil_image.size)
    pil_image.save('sugoma.png')

    # Decode QR codes in the image
    qr_codes = zbarlight.scan_codes(['qrcode'], pil_image)

    # Iterate over detected QR codes
    if qr_codes is not None:
        rospy.loginfo('number of qr codes: {}'.format(len(qr_codes)))
        for qr_code in qr_codes:
            # Extract the QR code's data
            qr_data = qr_code.decode('utf-8')
            rospy.loginfo('QR Code detected: {}'.format(qr_data))



    # # Detect QR codes in the image
    # barcodes = pyzbar.decode(contrast)
    # rospy.loginfo('number of qr codes: {}'.format(len(barcodes)))

    # # Iterate over detected QR codes
    # for barcode in barcodes:
    #     # Extract the QR code's data
    #     qr_data = barcode.data.decode('utf-8')
    #     rospy.loginfo('QR Code detected: {}'.format(qr_data))
        

def image_callback(msg):
    try:
        # Convert the raw image data to OpenCV format
        cv_image = CvBridge().imgmsg_to_cv2(msg, "bgr8")

        # Decode QR codes in the image
        decode_qr_code(cv_image)
        
        # Do something with the image (e.g., process, display)
        # cv2.imshow("Raw Image", cv_image)
        cv2.waitKey(1)  # Refresh the image window
        
    except Exception as e:
        rospy.logerr(e)

def main():
    rospy.init_node('image_subscriber', anonymous=True)
    
    # Set up the subscriber to the raw image topic
    image_topic = "/rtg/camera/rgb/image_raw"  # Replace with the actual image topic
    rospy.Subscriber(image_topic, Image, image_callback)
    
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
