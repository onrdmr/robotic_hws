import sys
import cv2

target_directory = '/mnt/d/qreader'

# Add the target directory to sys.path
sys.path.append(target_directory)

from qreader import QReader


# Create a QReader instance
qreader = QReader()

# Get the image that contains the QR code (QReader expects an uint8 numpy array)
image = cv2.cvtColor(cv2.imread("/home/onur/robotic_hws/src/proje/barcode_example1.jpg"), cv2.COLOR_BGR2RGB)

# Use the detect_and_decode function to get the decoded QR data
decoded_text = qreader.detect_and_decode(image=image)

print(decoded_text)