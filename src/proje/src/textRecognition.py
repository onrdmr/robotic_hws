import pytesseract
import cv2
from PIL import Image, ImageOps

image = cv2.imread('/home/onur/robotic_hws/src/proje/src/i_Cropped_Image.jpg')
# image = cv2.bit(image)

image = 255 - image

top = 50  # Top border size in pixels
bottom = 50  # Bottom border size in pixels
left = 50  # Left border size in pixels
right = 50  # Right border size in pixels

border_color = (255, 255, 255)

# Add a border to the image
expanded_image = cv2.copyMakeBorder(image, top, bottom, left, right, cv2.BORDER_CONSTANT, value=border_color)

# Display the expanded image
cv2.imshow("Expanded Image", expanded_image)
cv2.imwrite("/home/onur/robotic_hws/src/proje/src/expandedCropped.png", expanded_image)
cv2.waitKey(0)
# image.save('/home/onur/robotic_hws/src/proje/src/i_Cropped_Image.jpg')

text = pytesseract.image_to_string(expanded_image)
print("Recognized Text : ")
print(text)