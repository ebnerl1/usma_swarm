from collections import deque
from State import *

class Behavior(object):

    def __init__(self):
        self.num = 0

behavior = Behavior()

print isinstance(behavior, Behavior)

stack = deque()

firstState = SwitchState()

stack.append(firstState)

while True:
    print behavior.num
    if (len(stack) == 0):
        break
    cmd = stack[-1].update(behavior)
    cmd(stack)
