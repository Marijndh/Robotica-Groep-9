import cv2 as cv
import numpy as np

from ColorFiltering import Color
from create_grid_on_frame import create_grid_on_image

width_img = 1080
height_img = 720
location_bullseye_cells = []

# Functie om te controleren of een contour ellipsvormig is
def is_ellipse(contour, min_aspect_ratio=0.5):
    if len(contour) < 5:
        return False, (0, 0)
    ellipse = cv.fitEllipse(contour)
    (x, y), (MA, ma), angle = ellipse
    aspect_ratio = min(MA, ma) / max(MA, ma)
    return aspect_ratio >= min_aspect_ratio, (int(x), int(y))


def find_bulls_eyes(inv_contours):
    result = []
    for contour in inv_contours:
        is_ellipse_flag, center = is_ellipse(contour)
        if is_ellipse_flag:
            print(center, "center and flag: ", is_ellipse_flag)
            ellipse = cv.fitEllipse(contour)

            mask = create_mask(center)

            # Bereken het gemiddelde HSV in de cirkel
            hsv_img = cv.cvtColor(img, cv.COLOR_BGR2HSV)
            mean = cv.mean(hsv_img, mask=mask)
            # print(mean)
            color_bulls_eye(center, ellipse, mean, result)
    return result


def color_bulls_eye(center, ellipse, mean, result):
    bulls_eye = Color('bulls_eye', np.array([20, 0, 0]), np.array([25, 255, 255]))
    if bulls_eye.is_color(mean[0], mean[1], mean[2]):
        cv.circle(img, center, 5, (0, 0, 0), -1)
        cv.ellipse(img, ellipse, (0, 255, 0), 2)
        result.append(center)
        print('Bulls eye found at: ' + center.__str__())


# def check_bulls_eye_in_grid(bulls_eyes, grid_centers):
#     bulls_eye_cells = []
#     for center in bulls_eyes:
#         for cell in grid_centers:
#             if abs(center[0] - cell[0]) <= cell_size_width // 2 and abs(center[1] - cell[1]) <= cell_size_height // 2:
#                 bulls_eye_cells.append(cell)
#     return bulls_eye_cells

def create_mask(center):
    # Maak een masker voor de cirkel
    mask = np.zeros(gray_img.shape, dtype=np.uint8)
    cv.circle(mask, center, 5, (255, 255, 255), -1)  # Aangenomen straal van 10 pixels
    return mask


# Lees de afbeelding in
img = cv.imread('Images/bulls-eye.jpg')
img = cv.resize(img, (width_img, height_img))

# Converteer de afbeelding naar grijswaarden
gray_img = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

# Pas een Gaussiaanse vervaging toe
blurred_img = cv.GaussianBlur(gray_img, (9, 9), 2)

# Voer een drempelbewerking uit
ret, thresh_img = cv.threshold(blurred_img, 127, 255, cv.THRESH_BINARY_INV)

# Detecteer contouren in de inverse afbeelding
contours, _ = cv.findContours(thresh_img, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

bulls_eyes = find_bulls_eyes(contours)

# print('Bull_eyes locations:' + bulls_eyes.__str__())

# Toon de resultaten
cv.imshow('Origineel met Ellipsen', img)
# round omdat je int nodig hebt bij create_grid_on_image
cv.imshow("grid", create_grid_on_image(img, round(width_img / 2), round(height_img / 2)))
cv.waitKey(0)
cv.destroyAllWindows()
