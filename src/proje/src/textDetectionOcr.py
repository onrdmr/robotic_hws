import sys

# Specify the target directory where the packages are installed
target_directory = '/mnt/d/ocr/libs'

# Add the target directory to sys.path
sys.path.append(target_directory)

import easyocr
import cv2
import matplotlib.pyplot as plt

image_path = '/home/onur/robotic_hws/src/proje/src/Cropped_Image.jpg'

img = cv2.imread(image_path)


reader = easyocr.Reader(['en'], gpu=False)


text = reader.readtext(img)

print(text)