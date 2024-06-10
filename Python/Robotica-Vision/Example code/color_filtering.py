import cv2 as cv
import numpy as np


class Color:
    def __init__(self, name, lower_bound, upper_bound):
        self.name = name
        self.lower_bound = lower_bound
        self.upper_bound = upper_bound

    def get_mask(self, hsv_frame):
        return cv.inRange(hsv_frame, self.lower_bound, self.upper_bound)

    def is_color(self, h, s, v):
        upper = self.upper_bound
        lower = self.lower_bound
        if upper[0] > h > lower[0] and upper[1] > s > lower[1] and upper[2] > v > lower[2]:
            return True
        else:
            return False


# Predefined colors with their HSV ranges
colors = {
    "red": Color("red", np.array([0, 200, 0]), np.array([20, 255, 255])),
    "green": Color("green", np.array([40, 0, 0]), np.array([70, 255, 255])),
    "blue": Color("blue", np.array([90, 50, 50]), np.array([140, 255, 255])),
    "pink": Color("pink", np.array([165, 0, 0]), np.array([180, 255, 255])),
}

# List of primary and intermediate colors
primary_colors = [
    'red', 'green', 'blue', 'pink'
]


# Function to display each color mask in a separate window
def display_individual_colors():
    # Load and process frame
    frame = cv.imread('../Images/blauwe-tang.jpeg')
    frame = cv.resize(frame, (640, 640))
    hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
    for color_name in primary_colors:
        if color_name in colors:
            color = colors[color_name]
            mask = color.get_mask(hsv)
            res = cv.bitwise_and(frame, frame, mask=mask)
            cv.imshow(color.name, res)
            cv.waitKey(0)
            cv.destroyAllWindows()
        else:
            print(f"Color {color_name} is not recognized")


# Function to display all colors in one image with outlines and labels
def display_all_colors():
    # Load and process frame
    frame = cv.imread('../Images/blauwe-tang.jpeg')
    frame = cv.resize(frame, (640, 640))
    hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
    combined_mask = None

    for color_name in primary_colors:
        if color_name in colors:
            color = colors[color_name]
            mask = color.get_mask(hsv)

            if combined_mask is None:
                combined_mask = mask
            else:
                combined_mask = combined_mask | mask

            contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL,
                                          cv.CHAIN_APPROX_SIMPLE)
            result = []
            for contour in contours:
                if cv.contourArea(contour) > 1000:
                    result.append(contour)
                    M = cv.moments(contour)
                    if M["m00"] != 0:
                        cX = int(M["m10"] / M["m00"])
                        cY = int(M["m01"] / M["m00"])
                    else:
                        cX, cY = 0, 0
                    cv.putText(frame, color.name, (cX - 20, cY),
                               cv.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 2,
                               cv.LINE_AA)
            cv.drawContours(frame, result, -1, (0, 0, 0), 2)
            print(color_name + ':' + str(len(result)))

    if combined_mask is not None:
        combined_res = cv.bitwise_and(frame, frame, mask=combined_mask)
        combined_res = cv.addWeighted(combined_res, 0.7, result_frame, 0.3, 0)
        cv.imshow('All Colors with Outlines', combined_res)
        cv.waitKey(0)
        cv.destroyAllWindows()

# Display individual colors
# display_individual_colors()

# Display all colors in one image
#display_all_colors()

# cv.destroyAllWindows()
