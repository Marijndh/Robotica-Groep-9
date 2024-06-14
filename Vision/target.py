class Target:
    def __init__(self, hitpoint, body):
        self.hitpoint = hitpoint
        self.body = body

    def __str__(self):
        return "Target: " + self.hitpoint.__str__()
