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

red_lower1 = np.array([0, 100, 100])
red_upper1 = np.array([10, 255, 255])
red_lower2 = np.array([160, 100, 100])
red_upper2 = np.array([180, 255, 255])

blue_lower = np.array([90, 100, 100])
blue_upper = np.array([120, 255, 255])

# Maak maskers voor elke kleur
yellow_mask = cv.inRange(hsv_img, yellow_lower, yellow_upper)
red_mask1 = cv.inRange(hsv_img, red_lower1, red_upper1)
red_mask2 = cv.inRange(hsv_img, red_lower2, red_upper2)
red_mask = cv.bitwise_or(red_mask1, red_mask2)
blue_mask = cv.inRange(hsv_img, blue_lower, blue_upper)

# Voer morfologische operaties uit om de maskers te verbeteren
kernel = np.ones((5, 5), np.uint8)
yellow_mask = cv.morphologyEx(yellow_mask, cv.MORPH_CLOSE, kernel)
red_mask = cv.morphologyEx(red_mask, cv.MORPH_CLOSE, kernel)
blue_mask = cv.morphologyEx(blue_mask, cv.MORPH_CLOSE, kernel)

# Vind contouren in elk masker
contours_yellow, _ = cv.findContours(yellow_mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
contours_red, _ = cv.findContours(red_mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
contours_blue, _ = cv.findContours(blue_mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

# Teken contouren op de originele afbeelding
cv.drawContours(img, contours_yellow, -1, (0, 255, 255), 2)  # Geel
cv.drawContours(img, contours_red, -1, (0, 0, 255), 2)      # Rood
cv.drawContours(img, contours_blue, -1, (255, 0, 0), 2)     # Blauw

# Plaats een zwarte stip op het zwaartepunt van het gele vlak
for contour in contours_yellow:
    # Bereken het zwaartepunt van het gele vlak
    M = cv.moments(contour)
    if M["m00"] != 0:
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])
        # Teken een zwarte stip op het zwaartepunt van het gele vlak
        cv.circle(img, (cX, cY), 5, (0, 0, 0), -1)
        # Bereken de begrenzende doos van het gele vlak
        x, y, w, h = cv.boundingRect(contour)
        # Bereken het zwaartepunt van de begrenzende doos
        box_cX = x + w // 2
        box_cY = y + h // 2
        # Pas de x-coördinaat van het middenpunt van de begrenzende doos aan
        box_cX = cX
        # Teken een zwarte stip op het aangepaste middenpunt van de begrenzende doos
        cv.circle(img, (box_cX, box_cY), 5, (0, 0, 0), -1)
        # Teken de begrenzende doos op de originele afbeelding
        cv.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 2)

# Toon de resultaten
cv.imshow('Tangvormig Object', img)
cv.waitKey(0)
cv.destroyAllWindows()
