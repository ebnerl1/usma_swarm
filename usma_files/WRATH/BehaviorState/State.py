EmptyCmd = lambda stack: None
PopStateCmd = lambda stack: stack.pop()
PushStateCmd = lambda state: (lambda stack: stack.append(state)) 

class State(object):

    # Must return a Cmd from above
    def update(self, behavior):
        pass

class IncrementState(object):

    def __init__(self, max):
        self.max = max
    
    def update(self, behavior):
        behavior.num += 1
        if (behavior.num >= self.max):
            return PopStateCmd
        return EmptyCmd

class DecrementState(object):

    def __init__(self, min):
        self.min = min
    
    def update(self, behavior):
        behavior.num -= 1
        if (behavior.num <= self.min):
            return PopStateCmd
        return EmptyCmd

class SwitchState(object):

    def __init__(self):
        self.increment = True
        self.stepsRemaining = 10
        self.max = 0
    
    def update(self, behavior):
        if self.stepsRemaining == 0:
            return PopStateCmd
        
        self.stepsRemaining -= 1

        if (self.increment):
            self.increment = False
            self.max += 1
            return PushStateCmd(IncrementState(self.max))
        self.increment = True
        return PushStateCmd(DecrementState(0))




