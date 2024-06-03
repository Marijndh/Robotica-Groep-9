import cv2 as cv
import numpy as np

from FindInstruments import frame

# TODO: precisie kan nog worden aangepast
# Parameters
table_radius = 50  # cm
grid_resolution = 5  # cm per cel
num_cells = table_radius * 2 // grid_resolution


# het maken van een grid
def create_grid_on_image(frame, num_cells, grid_resolution):
    height, width, _ = frame.shape
    cell_size = int(height / num_cells)
    for i in range(0, num_cells):
        cv.line(frame, (0, i * cell_size), (width, i * cell_size), (0, 0, 0), 1)
        cv.line(frame, (i * cell_size, 0), (i * cell_size, height), (0, 0, 0), 1)
    return frame


frame = create_grid_on_image(frame, num_cells, grid_resolution)

cv.imshow('Grid', frame)
cv.waitKey(0)
cv.destroyAllWindows()
