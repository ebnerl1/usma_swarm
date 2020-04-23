from PIL import Image
import math
import ap_lib.gps_utils as gps

class HeatmapGenerator():

    # Size is pixel coords: (width, height)
    def __init__(self, size, bounds, radMaxVal, sampleSize = 10):
        self.brightnessValues = [[0.0 for i in range(size[0])] for ii in range(size[1])]
        self.gradients = [
            #      r    g    b    a
            (0.0, (0,   0,   0,   0)),
            (0.1, (0,   0,   0,   0)),
            (0.4, (0,   0,   255, 255)),
            (0.6, (0,   255, 0,   255)),
            (0.8, (255, 255, 0,   255)),
            (1.0, (255, 0,   0,   255))
        ]
        self.size = size
        self.radMaxVal = radMaxVal
        self.sampleSize = sampleSize
        self.bounds = bounds


    def evalGradient(self, brightness):
        for i in range(len(self.gradients)):
            value, color = self.gradients[i]
            if (value >= brightness):
                minValue, minColor = self.gradients[i-1]
                scaledBrightness = (brightness - minValue) / (value - minValue)
                minVal = 1 - scaledBrightness
                maxVal = scaledBrightness
                return (int(minColor[0] * minVal + color[0] * maxVal),
                        int(minColor[1] * minVal + color[1] * maxVal),
                        int(minColor[2] * minVal + color[2] * maxVal),
                        int(minColor[3] * minVal + color[3] * maxVal))
    

    def processRad(self, location, radCount):
        # Normalize radCount between min and max
        normRad = radCount * (1 / self.radMaxVal)
        if (normRad > 1.0): # in case of a mistaken radMaxVal
            normRad = 1.0

        # Convert Coord to Pixel Space
        px = int((location[0] - self.bounds[0]) * self.size[0] / (self.bounds[2] - self.bounds[0])) 
        py = int((location[1] - self.bounds[1]) * self.size[1] / (self.bounds[3] - self.bounds[1]))

        # For all squares that are within possible max distance (x, y):
        for i in range(-self.sampleSize, self.sampleSize):
            for ii in range(-self.sampleSize, self.sampleSize):
                x = px + i
                y = py + ii
                dist = int(math.sqrt((x - px) * (x - px) + (y - py) * (y - py)))
                if (dist >= self.sampleSize):
                    continue

                if (x > 0 and x < self.size[0] and y > 0 and y < self.size[1]):
                    # Get propagation value at coord
                    propValue = 1.0 - (1.0 / float(self.sampleSize)) * float(dist)

                    # Set coord brightness to prop * rad + (1 - prop) * cur
                    self.brightnessValues[x][y] = propValue * normRad + (1 - propValue) * self.brightnessValues[x][y]
    

    def saveImage(self, name):
        brightnessImage = Image.new('RGBA', self.size)
        print "new image"

        pixels = brightnessImage.load()
        for i in range(self.size[0]):
            for ii in range(self.size[1]):
                color = self.evalGradient(self.brightnessValues[self.size[1] - 1 - ii][i])
                pixels[i, ii] = color
        brightnessImage.save(name, "PNG")
