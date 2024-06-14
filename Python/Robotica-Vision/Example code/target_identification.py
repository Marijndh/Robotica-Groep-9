import cv2 as cv
import numpy as np


def get_point_of_target(coordinates, centroid):
    points = np.array(coordinates)

    # Bepaal de afstanden tussen elk punt en het gemiddelde (centroïde) punt
    distances_to_centroid = np.linalg.norm(points - centroid, axis=1)

    # Zoek de index van het punt dat het verst van het gemiddelde punt ligt, dit is de 'punt'
    point_index = np.argmax(distances_to_centroid)
    point = points[point_index]

    # Nu hebben we het punt, laten we het verwijderen uit de lijst van punten om de resterende punten te analyseren
    remaining_points = np.delete(points, point_index, axis=0)

    # Bepaal de richting van de lijn van het centroid naar de punt van de tang
    direction = point - centroid

    # Draai de richting 90 graden om de loodrechte lijn te krijgen
    perpendicular_direction = np.array([-direction[1], direction[0]])

    # Categoriseer punten op basis van hun positie ten opzichte van de twee lijnen
    handle1 = None
    handle2 = None

    for pt in remaining_points:
        vector = pt - centroid
        dot_product_main = np.dot(vector, direction)
        dot_product_perp = np.dot(vector, perpendicular_direction)

        if dot_product_main > 0:
            if dot_product_perp > 0:
                if handle1 is None or np.linalg.norm(pt - centroid) < np.linalg.norm(handle1 - centroid):
                    handle1 = pt
            else:
                if handle2 is None or np.linalg.norm(pt - centroid) < np.linalg.norm(handle2 - centroid):
                    handle2 = pt
        else:
            if dot_product_perp > 0:
                if handle1 is None or np.linalg.norm(pt - centroid) < np.linalg.norm(handle1 - centroid):
                    handle1 = pt
            else:
                if handle2 is None or np.linalg.norm(pt - centroid) < np.linalg.norm(handle2 - centroid):
                    handle2 = pt

    # Teruggeven van de gecategoriseerde punten
    categorized_points = {
        'handvat1': handle1,
        'handvat2': handle2,
        'punt': point,
    }

    return categorized_points

img = cv.imread('../Images/rechte-tang4.jpeg')
img = cv.resize(img,(1080,720))

hsv_img = cv.cvtColor(img, cv.COLOR_BGR2HSV)

yellow_lower = np.array([20, 100, 100])
yellow_upper = np.array([30, 255, 255])

yellow_mask = cv.inRange(hsv_img, yellow_lower, yellow_upper)

# Voer morfologische operaties uit om de maskers te verbeteren
kernel = np.ones((5, 5), np.uint8)
yellow_mask = cv.morphologyEx(yellow_mask, cv.MORPH_CLOSE, kernel)

contours_yellow, _ = cv.findContours(yellow_mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

cv.drawContours(img, contours_yellow, -1, (0, 255, 255), 2)

for contour in contours_yellow:
    area = cv.contourArea(contour)
    if area > 500:
        hull = cv.convexHull(contour)

        M = cv.moments(contour)
        if M["m00"] != 0:
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
        else:
            cX, cY = 0, 0
        centroid = [cX, cY]

        extreme_left = tuple(hull[hull[:, :, 0].argmin()][0])
        extreme_right = tuple(hull[hull[:, :, 0].argmax()][0])
        extreme_top = tuple(hull[hull[:, :, 1].argmin()][0])
        extreme_bottom = tuple(hull[hull[:, :, 1].argmax()][0])

        points = [extreme_left, extreme_right, extreme_top, extreme_bottom]

        x, y, w, h = cv.boundingRect(contour)
        filtered = get_point_of_target(points, centroid)

        punt = filtered["punt"]
        handvat1 = filtered["handvat1"]
        handvat2 = filtered["handvat2"]
        get_gripper_degrees(punt, centroid)

        cv.circle(img, handvat1, 5, (255, 0, 0), -1)
        cv.circle(img, handvat2, 5, (255, 0, 0), -1)
        cv.circle(img, punt, 5, (0, 255, 0), -1)
        cv.circle(img, centroid, 5, (0, 0, 0), -1)

# Toon de resultaten
cv.imshow('Tangvormig Object', img)
cv.waitKey(0)
cv.destroyAllWindows()
