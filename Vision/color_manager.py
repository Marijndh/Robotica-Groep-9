from color import Color


# Predefining color values to determine colour of objects easier
class ColorManager:
    def __init__(self):
        self.colors = {
            "red": [Color("red", [0, 40, 0], [25, 256, 256]), Color("red", [165, 40, 50], [180, 256, 200])],
            "green": [Color("green", [35, 50, 0], [85, 256, 256])],
            "blue": [Color("blue", [90, 30, 50], [160, 256, 256])],
            "pink": [Color("pink", [150, 0, 200], [180, 85, 256])],
        }
        self.primary_colors = self.colors.keys()

    def get_primary_colors(self):
        return self.primary_colors

    def get_colors(self):
        return self.colors
