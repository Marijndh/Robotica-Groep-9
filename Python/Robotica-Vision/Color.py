import cv2 as cv
import numpy as np


class Color:
    def __init__(self, name, lower_bound, upper_bound, lower_bound2=None, upper_bound2=None):
        self.name = name
        self.lower_bound = lower_bound
        self.upper_bound = upper_bound
        self.lower_bound2 = lower_bound2
        self.upper_bound2 = upper_bound2

    def get_mask(self, hsv_frame):
        mask = cv.inRange(hsv_frame, self.lower_bound, self.upper_bound)
        if self.lower_bound2 is not None and self.upper_bound2 is not None:
            mask2 = cv.inRange(hsv_frame, self.lower_bound2, self.upper_bound2)
            mask = mask | mask2
        return mask


# Predefined colors with their HSV ranges
colors = {
    "red": Color("red", np.array([0, 150, 150]), np.array([6, 255, 255]), np.array([171, 150, 150]),
                 np.array([179, 255, 255])),
    "orange": Color("orange", np.array([7, 150, 150]), np.array([22, 255, 255])),
    "yellow": Color("yellow", np.array([23, 150, 150]), np.array([43, 255, 255])),
    "green": Color("green", np.array([44, 150, 150]), np.array([73, 255, 255])),
    "cyan": Color("cyan", np.array([74, 150, 150]), np.array([95, 255, 255])),
    "blue": Color("blue", np.array([95, 150, 150]), np.array([125, 255, 255])),
    "purple": Color("purple", np.array([126, 150, 150]), np.array([142, 255, 255])),
    "magenta": Color("magenta", np.array([143, 150, 150]), np.array([170, 255, 255])),
}

# List of primary and intermediate colors
primary_colors = [
    'red', 'green', 'blue', 'yellow', 'orange', 'purple',
    'cyan', 'magenta',
]

# Load and process frame
frame = cv.imread('color_circle.png')
frame = cv.resize(frame, (640, 640))
hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)


# Function to display each color mask in a separate window
def display_individual_colors():
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
    combined_mask = None
    result_frame = frame.copy()

    for color_name in primary_colors:
        if color_name in colors:
            color = colors[color_name]
            mask = color.get_mask(hsv)

            if combined_mask is None:
                combined_mask = mask
            else:
                combined_mask = combined_mask | mask

            contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
            for contour in contours:
                cv.drawContours(result_frame, [contour], -1, (0, 0, 0), 2)
                M = cv.moments(contour)
                if M["m00"] != 0:
                    cX = int(M["m10"] / M["m00"])
                    cY = int(M["m01"] / M["m00"])
                else:
                    cX, cY = 0, 0
                cv.putText(result_frame, color.name, (cX - 20, cY), cv.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 2,
                           cv.LINE_AA)

    if combined_mask is not None:
        combined_res = cv.bitwise_and(frame, frame, mask=combined_mask)
        combined_res = cv.addWeighted(combined_res, 0.7, result_frame, 0.3, 0)
        cv.imshow('All Colors with Outlines', combined_res)
        cv.waitKey(0)
        cv.destroyAllWindows()


# Display individual colors
#display_individual_colors()

# Display all colors in one image
display_all_colors()

cv.destroyAllWindows()
