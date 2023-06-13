import cv2
import pyzbar 

# Read the RGB image
image = cv2.imread('/home/onur/robotic_hws/src/proje/qr2.jpg')

def autofocus(image):
    # Convert image to grayscale for better QR code detection
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # Create a QR code scanner object
    scanner = cv2.QRCodeDetector()

    # Perform autofocus by applying Gaussian blur with increasing kernel sizes
    for kernel_size in range(1, 10, 2):
        blurred = cv2.GaussianBlur(gray, (kernel_size, kernel_size), 0)
        qr_codes = scanner.detectAndDecodeMulti(blurred)

        # If any QR codes are detected, return the autofocused image
        if qr_codes is not None:
            return blurred

    # If no QR codes are detected, return the original image
    return image


autofocus(image)