from instrument import Instrument
from frame import Frame
from color import Color
from target import Target


def main():
    frame = Frame('Images/twee_scharen.png', 1080, 720)
    frame.find_instruments()
    frame.find_children()
    frame.get_instrument_colors()
    frame.draw_instruments()
    frame.print_instruments()
    frame.show()


if __name__ == '__main__':
    main()
