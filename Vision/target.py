class Target:
    def __init__(self, centroid, body):
        self.centroid = centroid
        self.body = body

    def __str__(self):
        return "Target: " + self.centroid.__str__()
