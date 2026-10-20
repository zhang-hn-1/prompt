class TerminalFeedback:
    """Experimental forward terminal feedback; retain entry steering request."""
    def __init__(self):
        self.held_auxiliary = None
    def update(self, distance, speed, auxiliary):
        if self.held_auxiliary is None:
            if distance > .5:
                return None
            self.held_auxiliary = float(auxiliary)
        main = 0. if distance <= .2 else min(.5, max(0., 3. * (distance - .15) - 2. * speed))
        return (main, self.held_auxiliary)
