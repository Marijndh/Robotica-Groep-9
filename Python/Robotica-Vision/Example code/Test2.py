import cv2 as cv
import numpy as np

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


# Teken een punt op het uiterste punt van het handvat en het uiterste punt van de tang
cv.circle(img, extreme_left, 5, (0, 0, 0), -1)
cv.circle(img, extreme_right, 5, (0, 0, 0), -1)
cv.circle(img, extreme_top, 5, (0, 0, 0), -1)
cv.circle(img, extreme_bottom, 5, (0, 0, 0), -1)

# Toon de resultaten
cv.imshow('Tangvormig Object', img)
cv.waitKey(0)
cv.destroyAllWindows()
