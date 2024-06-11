import cv2 as cv

from instrument import Instrument
from geometry_utils import GeometryUtils


class Frame:
    def __init__(self, path, width=0, height=0):
        self.img = cv.imread(path)
        self.img = cv.resize(self.img, (1080, 720))
        self.gray_image = cv.cvtColor(self.img, cv.COLOR_BGR2GRAY)
        self.gray_image = cv.medianBlur(self.gray_image, 5)
        self.hsv_image = cv.cvtColor(self.img, cv.COLOR_BGR2HSV)
        self.thresh = cv.threshold(self.gray_image, 127, 255, cv.THRESH_BINARY)[1]
        self.contours, self.hierarchy = cv.findContours(self.thresh, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
        self.hierarchy = self.hierarchy[0]
        self.width = width
        self.height = height
        self.instruments = []
        self.targets = []

    def draw_contours(self):
        for contour in self.contours:
            cv.drawContours(self.img, [contour], -1, (0, 255, 0), 2)

    def draw_instruments(self):
        for instrument in self.instruments:
            objects = [instrument.body] + instrument.children
            cv.drawContours(frame, objects, -1, (0, 255, 0), 3)
            cv.circle(frame, i.centroid, 5, (0, 0, 0), -1)
            if i.color is not None:
                print(str(i.index) + ": " + i.color)

    def find_instruments(self):
        contours = self.contours
        for cnr in range(len(contours)):
            cnt = contours[cnr]
            area = cv.contourArea(cnt)
            # perimeter = cv.arcLength(cnt, True)
            # factor = 4 * math.pi * area / perimeter ** 2
            if 5000.0 < area < 1080 * 720 * 0.8:  # instrumenten mogen niet groter dan 80% van de afbeelding zijn
                self.instruments.append(Instrument(cnt, cnr))

    def find_children(self):
        for instrument in instruments:
            for j in range(len(self.hierarchy)):
                if self.hierarchy[j][3] == instrument.index and cv.contourArea(self.contours[j]) > 1000:
                    child = self.contours[j]
                    instrument.add_child(child)

    def get_instrument_colors(self):
        for instrument in self.instruments:
            mask = np.zeros(self.hsv_image.shape[:2], np.uint8)
            contours = [instrument.body] + instrument.children
            cv.drawContours(mask, contours, -1, 255, -1)
            mean = cv.mean(hsv, mask=mask)
            instrument.set_hsv(mean)
            for color_name in primary_colors:
                if color_name in colors:
                    color = colors[color_name]
                    if color.is_color(mean[0], mean[1], mean[2]):
                        instrument.set_color(color_name)
                        break

    def get_targets(self):
        for contour in self.contours:
            result, center = GeometryUtils.is_ellipse(contour)
            if result:
                targets.append((center, contour))

    def show(self):
        cv.imshow('Result', self.img)
        cv.waitKey(0)
        cv.destroyAllWindows()
