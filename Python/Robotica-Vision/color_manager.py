from color import Color


# Color maneger verantwoordelijk voor het hsv bereik van de kleuren en de primaire kleuren die deze representeert
# deze kunnen dan worden opgehaald door de get_colors en get_primary_colors functies
class ColorManager:
    def __init__(self):
        self.colors = {

            "red": Color("red", [0, 50, 0], [39, 256, 256]),
            "green": Color("green", [40, 30, 0], [70, 256, 256]),
            "blue": Color("blue", [80, 30, 50], [140, 256, 256]),
            "pink": Color("pink", [150, 0, 0], [180, 256, 256]),
        }
        self.primary_colors = ['red', 'green', 'blue', 'pink', 'silver']

    def get_primary_colors(self):
        return self.primary_colors

    def get_colors(self):
        return self.colors
