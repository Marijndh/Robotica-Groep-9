class Target:
    def __init__(self, centroid):
        self.centroid = centroid

    def __str__(self):
        return "Target: " + self.centroid.__str__()
