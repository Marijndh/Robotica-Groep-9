# create_grid_on_frame.py
import cv2 as cv


# het maken van een grid
def create_grid_on_image(frame, num_cells_width, num_cells_height):
    height, width, _ = frame.shape
    cell_size_width = int(width / num_cells_width)
    cell_size_height = int(height / num_cells_height)
    for i in range(0, num_cells_width):
        cv.line(frame, (i * cell_size_width, 0), (i * cell_size_width, height), (0, 0, 0), 1)
    for i in range(0, num_cells_height):
        cv.line(frame, (0, i * cell_size_height), (width, i * cell_size_height), (0, 0, 0), 1)

    return frame
