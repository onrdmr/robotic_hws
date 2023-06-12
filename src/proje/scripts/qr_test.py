#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
# from pyzbar import pyzbar
import zbarlight
from PIL import Image as PILImage
from pyzbar import pyzbar
import numpy as np
# import pytesseract


def sharpen_image(image):
    sharpened = image

    for i in range(4):
        # Apply Gaussian blur
        blurred = cv2.GaussianBlur(sharpened, (0, 0), 3)
        # cv2.imshow('blurred', blurred)

        # Perform image blending
        sharpened = cv2.addWeighted(sharpened, 1.5, blurred, -0.5, 0)

    # for i in range(3):
    #     # Create a sharpening kernel
    #     kernel = np.array([[-1, -1, -1],
    #                     [-1,  9, -1],
    #                     [-1, -1, -1]])

    #     # Apply the sharpening kernel to the image
    #     sharpened = cv2.filter2D(sharpened, -1, kernel)

    return sharpened


def decode_qr_code(image):
    # Convert the image to grayscale
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    sharpened = sharpen_image(gray)

    # Change brightness/contrast
    contrasted = cv2.convertScaleAbs(sharpened, alpha=2.5, beta=-20)

    result = contrasted

    # # Make binary image
    # result[result > 200] = 255
    # result[result <= 200] = 0

    # kernel_size = 1
    # kernel = high_contrast[0:kernel_size, 0:kernel_size].copy()

    # # Perform convolution
    # result = cv2.filter2D(high_contrast, 0, kernel)

    # # Create a sharpening kernel
    # kernel = np.array([[-1, -1, -1],
    #                    [-1,  9, -1],
    #                    [-1, -1, -1]])
    # for i in range(100):
    #     # Apply the sharpening kernel to the image
    #     sharpened_image = cv2.filter2D(sharpened_image, -1, kernel)

    # detector = cv2.QRCodeDetector()
    # value, points, straight_qrcode = detector.detectAndDecode(result)
    # print(value)

    # Convert the OpenCV image to PIL format
    # pil_image = PILImage.fromarray(result)
    # pil_image.save('anan.png')
    # # rospy.loginfo(pil_image.size)

    # # Decode QR codes in the image
    # qr_codes = zbarlight.scan_codes(['qrcode'], pil_image)

    # # Iterate over detected QR codes
    # if qr_codes is not None:
    #     # rospy.loginfo('number of qr codes: {}'.format(len(qr_codes)))
    #     for qr_code in qr_codes:
    #         # Extract the QR code's data
    #         qr_data = qr_code.decode('utf-8')
    #         print(qr_data)
    #         # rospy.loginfo('QR Code detected: {}'.format(qr_data))

    # Detect QR codes in the image
    barcodes = pyzbar.decode(result)
    # rospy.loginfo('number of qr codes: {}'.format(len(barcodes)))

    # Iterate over detected QR codes
    for barcode in barcodes:
        # Extract the QR code's data
        qr_data = barcode.data.decode('utf-8')
        print(qr_data)
        # rospy.loginfo('QR Code detected: {}'.format(qr_data))

    cv2.imshow("gray", gray)
    cv2.imshow("contrasted", contrasted)
    cv2.imshow("sharpened", sharpened)
    cv2.imwrite("result.png", result)
    # cv2.imshow("result", result)

# def main():
#     img = cv2.imread("image.png", cv2.IMREAD_COLOR)
#     decode_qr_code(img)

# if __name__ == '__main__':
#     main()


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


def main():
    # image_path = "qr-code.jpg"
    image_path = "image.png"
    image = cv2.imread(image_path)

    if image is None:
        print("Failed to load image!")
        return

    kernel_size = 1
    grayscale_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    kernel = grayscale_image[0:kernel_size, 0:kernel_size].copy()

    # Perform convolution
    result = cv2.filter2D(grayscale_image, -1, kernel)
    contour = create_contour(result, 30)

    cropped_image = image[contour['lu'][1]: contour['lb']
                          [1], contour['lu'][0]: contour['ru'][0]]
    decode_qr_code(cropped_image)

    # points = np.array([contour['lu'], contour['ru'], contour['rb'], contour['lb']])

    # # Convert grayscale image to color
    # color_image = cv2.cvtColor(grayscale_image, cv2.COLOR_GRAY2BGR)

    # # Draw polylines on the color image
    # cv2.polylines(grayscale_image, [points], True, (0, 255, 0), 2)

    # roi_rect = cv2.boundingRect(points)
    # cropped_image = grayscale_image[roi_rect[1]:roi_rect[1] + roi_rect[3], roi_rect[0]:roi_rect[0] + roi_rect[2]]
    # cropped_image = cv2.bitwise_not(cropped_image)

    # tessdata_path = "/usr/share/tesseract-ocr/4.00/tessdata/"
    # pytesseract.pytesseract.tesseract_cmd = "/usr/bin/tesseract"
    # tessdata_config = f"--tessdata-dir {tessdata_path}"
    # text = pytesseract.image_to_string(cropped_image, config=tessdata_config)

    # print("Recognized Text:\n", text)

    # cv2.imshow("Cropped Image", cropped_image)
    # cv2.imwrite("/home/onur/robotic_hws/src/proje/src/Cropped_Image.jpg", cropped_image)

    # cv2.imshow("Grayscale Image", grayscale_image)
    # cv2.imshow("Result", result)
    # cv2.imshow("cropped image", cropped_image)

    cv2.waitKey(0)

    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
