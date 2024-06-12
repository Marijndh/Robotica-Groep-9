from color import Color


class ColorManager:
    def __init__(self):
        self.colors = {
            "red": Color("red", [0, 50, 0], [39, 255, 255]),
            "green": Color("green", [40, 50, 0], [70, 255, 255]),
            "blue": Color("blue", [90, 50, 50], [140, 255, 255]),
            "pink": Color("pink", [150, 0, 0], [180, 255, 255]),
        }
        self.primary_colors = ['red', 'green', 'blue', 'pink', 'silver']

    def get_primary_colors(self):
        return self.primary_colors

    def get_colors(self):
        return self.colors
