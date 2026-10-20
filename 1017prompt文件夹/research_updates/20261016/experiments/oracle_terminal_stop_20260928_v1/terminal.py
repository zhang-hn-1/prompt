class TerminalBrake:
    """Local experimental terminal brake; no global controller replacement."""
    def __init__(self):
        self.held_auxiliary = None
    def update(self, distance, speed, auxiliary):
        if self.held_auxiliary is None and distance <= .1 + 2.03 * speed:
            self.held_auxiliary = float(auxiliary)
        if self.held_auxiliary is None:
            return None
        return (0., self.held_auxiliary)
