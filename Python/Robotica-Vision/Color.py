class Color:
    def __init__(self, name, lower_bound, upper_bound):
        self.name = name
        self.lower_bound = lower_bound
        self.upper_bound = upper_bound

    def get_mask(self):
        # Placeholder implementation
        return None

    def is_color(self, h, s, v):
        # Placeholder implementation
        return self.lower_bound <= (h, s, v) <= self.upper_bound


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



# TODO: better sollution for the color being global yes or no, the code needs to be object oriented
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