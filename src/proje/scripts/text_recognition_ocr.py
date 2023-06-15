# ### FAST METHOD ### #
from google.cloud import vision

import os
import cv2

os.environ['GOOGLE_APPLICATION_CREDENTIALS'] = '/home/onur/robotic_hws/src/proje/scripts/vibrant-tiger-325510-ab7308715d5d.json'


from google.cloud import vision



def ocr_recognition(rgb_image):
    
    print("doint once ocr recognition")
    
    client = vision.ImageAnnotatorClient()

    _, image_bytes = cv2.imencode('.jpg', rgb_image)

    # Convert the bytes to content
    content = image_bytes.tobytes()

    image = vision.Image(content=content)

    response = client.text_detection(image=image)
    texts = response.text_annotations

    for text in texts:
        for i in range (10):
            print(f'\n"{text.description}"')

        # vertices = ([f'({vertex.x},{vertex.y})'
        #             for vertex in text.bounding_poly.vertices])

        # print('bounds: {}'.format(','.join(vertices)))


# image_path = r'/home/onur/robotic_hws/src/proje/src/text_deneme.png'
# detect_barcode(image_path)



# ### SLOW METHOD ### # 
# import sys

# # Specify the target directory where the packages are installed
# target_directory = '/mnt/d/ocr/libs'

# # Add the target directory to sys.path
# sys.path.append(target_directory)

# import easyocr
# import cv2
# import matplotlib.pyplot as plt

# import torch

# def ocr_recoginition(img):
#     print("before import")

#     gpu_available = torch.cuda.is_available()

#     if gpu_available:
#         print("GPU is available")
#     else:
#         print("GPU is not available")


#     print("text detection for ocr")
#     # image_path = '/home/onur/robotic_hws/src/proje/poison.jpg'

#     # img = cv2.imread(image_path)


#     reader = easyocr.Reader(['en'], gpu=True)


#     text = reader.readtext(img)

#     print(text)