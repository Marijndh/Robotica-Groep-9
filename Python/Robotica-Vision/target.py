class Target:
    def __init__(self, hitpoint, body):
        self.hitpoint = hitpoint
        self.body = body

    # possible code ?
    # def color_bulls_eye(center, ellipse, mean, img):
    # bulls_eye = Color('bulls_eye', np.array([20, 0, 0]), np.array([25, 255, 255]))
    # eyes = []
    # if bulls_eye.is_color(mean[0], mean[1], mean[2]):
    #     cv.circle(img, center, 5, (0, 0, 0), -1)
    #     cv.ellipse(img, ellipse, (0, 255, 0), 2)
    #     eyes.append(center)
    # return eyes
