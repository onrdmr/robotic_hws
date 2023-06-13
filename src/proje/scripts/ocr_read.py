import cv2
import numpy as np
# import pytesseract

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
    image_path = "/home/onur/robotic_hws/src/proje/corrosive.jpg"
    image = cv2.imread(image_path)

    if image is None:
        print("Failed to load image!")
        return

    kernel_size = 1
    grayscale_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    kernel = grayscale_image[0:kernel_size, 0:kernel_size].copy()

    # Perform convolution
    result = cv2.filter2D(grayscale_image, -1, kernel)
    contour = create_contour(result, 15)

    points = np.array([contour['lu'], contour['ru'], contour['rb'], contour['lb']])

    # Convert grayscale image to color
    color_image = cv2.cvtColor(grayscale_image, cv2.COLOR_GRAY2BGR)

    # Draw polylines on the color image
    cv2.polylines(image, [points], True, (0, 255, 0), 2)

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

    cv2.imshow("Grayscale Image", grayscale_image)
    cv2.imshow("Result", image)

    cv2.waitKey(0)

    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
