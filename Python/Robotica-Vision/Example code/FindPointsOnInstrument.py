import cv2 as cv
import numpy as np


def categorize_coordinates(coordinates):
    # Converteer de lijst van coördinaten naar een NumPy-array voor eenvoudiger gebruik
    points = np.array(coordinates)

    # Bepaal het gemiddelde van alle punten, dit zal helpen bij het bepalen van de 'achterkant'
    centroid = np.mean(points, axis=0)

    # Bepaal de afstanden tussen elk punt en het gemiddelde (centroïde) punt
    distances_to_centroid = np.linalg.norm(points - centroid, axis=1)

    # Zoek de index van het punt dat het verst van het gemiddelde punt ligt, dit is de 'punt'
    point_index = np.argmax(distances_to_centroid)
    point = points[point_index]

    # Nu hebben we het punt, laten we het verwijderen uit de lijst van punten om de resterende punten te analyseren
    remaining_points = np.delete(points, point_index, axis=0)

    # Bepaal de afstanden tussen elk punt en het punt 'punt'
    distances_to_point = np.linalg.norm(remaining_points - point, axis=1)

    # Zoek de twee punten die het dichtst bij elkaar liggen, dit zijn de 'handvatten'
    handle_indices = np.argsort(distances_to_point)[:2]
    handles = remaining_points[handle_indices]

    # Nu hebben we de handvatten, laten we ze verwijderen uit de lijst van overgebleven punten om de achterkant te bepalen
    remaining_points = np.delete(remaining_points, handle_indices, axis=0)

    # Het overgebleven punt is de 'achterkant'
    back = remaining_points[0]

    # Teruggeven van de gecategoriseerde punten
    categorized_points = {
        'handvat1': handles[0],
        'handvat2': handles[1],
        'punt': point,
        'achterkant': back
    }

    return categorized_points

# Lees de afbeelding in
img = cv.imread('../Images/rechte-tang.jpeg')
img = cv.resize(img,(720,1080))
# Converteer de afbeelding naar HSV-kleurruimte
hsv_img = cv.cvtColor(img, cv.COLOR_BGR2HSV)

# Definieer kleurintervallen voor geel, rood en blauw in HSV
yellow_lower = np.array([20, 100, 100])
yellow_upper = np.array([30, 255, 255])

# Maak een masker voor geel
yellow_mask = cv.inRange(hsv_img, yellow_lower, yellow_upper)

# Voer morfologische operaties uit om de masker te verbeteren
kernel = np.ones((5, 5), np.uint8)
yellow_mask = cv.morphologyEx(yellow_mask, cv.MORPH_CLOSE, kernel)

# Vind contouren in het gele masker
contours_yellow, _ = cv.findContours(yellow_mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

# Zoek de contour met het grootste oppervlak (het breedste deel van de tang)
largest_contour = max(contours_yellow, key=cv.contourArea)

# Bepaal het convex hull van de grootste contour
hull = cv.convexHull(largest_contour)

# Bepaal het uiterste punt van het handvat en het uiterste punt van de tang
extreme_left = tuple(hull[hull[:, :, 0].argmin()][0])
extreme_right = tuple(hull[hull[:, :, 0].argmax()][0])
extreme_top = tuple(hull[hull[:, :, 1].argmin()][0])
extreme_bottom = tuple(hull[hull[:, :, 1].argmax()][0])

filtered = categorize_coordinates([extreme_left,extreme_right,extreme_top,extreme_bottom])

# Teken een punt op het uiterste punt van het handvat en het uiterste punt van de tang
cv.circle(img, filtered["handvat1"], 5, (255, 0, 0), -1)
cv.circle(img, filtered["handvat2"], 5, (255, 0, 0), -1)
cv.circle(img, filtered["punt"], 5, (0, 255, 0), -1)
cv.circle(img, filtered["achterkant"], 5, (0, 0, 255), -1)

# Toon de resultaten
cv.imshow('Tangvormig Object', img)
cv.waitKey(0)
cv.destroyAllWindows()
