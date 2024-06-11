from color import Color


class ColorManager:
    def __init__(self):
        self.colors = {
            "red": Color("red", [0, 200, 0], [20, 255, 255]),
            "green": Color("green", [40, 0, 0], [70, 255, 255]),
            "blue": Color("blue", [90, 50, 50], [140, 255, 255]),
            "pink": Color("pink", [165, 0, 0], [180, 255, 255]),
        }
        self.primary_colors = ['red', 'green', 'blue', 'pink', 'silver']

    def get_color(self, name):
        return self.colors.get(name)

    def get_all_colors(self):
        return self.colors.values()

    def get_primary_colors(self):
        return [self.colors[color] for color in self.primary_colors]
