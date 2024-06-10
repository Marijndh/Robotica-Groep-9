import cv2 as cv

from Instrument import Instrument

class Frame:
    def __init__(self, img, contours=None, width=0, height=0, hierarchy=None):
        self.img = img
        self.contours = contours if contours is not None else []
        self.width = width
        self.height = height
        self.hierarchy = hierarchy

    def draw_instruments(self, frame, instruments):
        for i in instruments:
            objects = [i.body] + i.children
            cv.drawContours(frame, objects, -1, (0, 255, 0), 3)
            cv.circle(frame, i.centroid, 5, (0, 0, 0), -1)
            if i.color is not None:
                print(str(i.index) + ": " + i.color)

    def get_instruments(self, contours):
        instruments = []
        for cnr in range(len(contours)):
            cnt = contours[cnr]
            area = cv.contourArea(cnt)
            # perimeter = cv.arcLength(cnt, True)
            # factor = 4 * math.pi * area / perimeter ** 2
            if 5000.0 < area < 1080 * 720 * 0.8:  # instrumenten mogen niet groter dan 80% van de afbeelding zijn
                instruments.append(Instrument(cnr, cnt))
        return instruments

    def find_children(self, instruments, contours, hierarchy):
        for i in instruments:
            for j in range(len(hierarchy)):
                if hierarchy[j][3] == i.index and cv.contourArea(contours[j]) > 1000:
                    child = contours[j]
                    i.add_child(child)

    def get_targets(self):
        # Placeholder implementation
        # TODO: Implement this method
        return []