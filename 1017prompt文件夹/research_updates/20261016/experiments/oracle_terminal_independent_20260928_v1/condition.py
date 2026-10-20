class ApproachCondition:
    def __init__(self, mode):
        self.mode=mode;self.active=False;self.done=False;self.count=0;self.low=0;self.aux=0.
    def update(self, distance, speed, auxiliary):
        if self.done:return None
        threshold=.7 if self.mode=='coast' else .5
        if not self.active:
            if distance>threshold:return None
            self.active=True;self.aux=float(auxiliary)
        if self.mode=='coast':
            if self.count>=5:self.done=True;return None
            self.count+=1
        else:
            self.low=self.low+1 if speed<.05 else 0
            if self.low>=5:self.done=True;return None
        return (0.,self.aux)
