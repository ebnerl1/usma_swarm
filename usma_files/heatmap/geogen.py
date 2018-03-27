import csv
import time

#################### VARIABLES ####################

infile = "sample_data.csv" # Input file as a .csv
outfile = "sample_gj.geojson" # Output file as a .geojson
radcap = 100 # Highest amount of radiation to scale color gradient
sqrrad = 0.0005 # Area covered by one drone as a "square radius"
interval = 5 # Time in seconds in between scans

############################################################

# Decides color based on ratio of gradient to radcap, RGB
def colorSelect(rads):
  const = (rads/radcap) * 255
  color = (int(const), int(255 - const), 0)
  return '#%02x%02x%02x' % color

# [0] and [1] are long/lat, [2] is rads, [3] is color
def parseIn():
  data = []
  with open(infile, 'r') as inf:
    r = csv.reader(inf)
    for row in r:
      data.append([row[0],row[1],row[2],colorSelect(int(row[2]))])
    return data

# Iterates through entire parsed file to overwrite .geojson
def writeGJ():
  data = parseIn()
  with open(outfile, 'w') as outf:
    outf.write('{\n')
    outf.write('  "type": "FeatureCollection",\n')
    outf.write('  "features": [\n')

    for i in range(len(data)):

      entry = data[i]

      outf.write('    {\n')
      outf.write('      "type": "Feature",\n')
      outf.write('      "properties": {\n')
      outf.write('        "stroke": "#999999",\n')
      outf.write('        "stroke-width": 0.5,\n')
      outf.write('        "stroke-opacity": 0.8,\n')
      outf.write('        "fill": "{0}",\n'.format(entry[3]))
      outf.write('        "fill-opacity": 0.6\n')
      outf.write('      },\n')
      outf.write('      "geometry": {\n')
      outf.write('        "type": "Polygon",\n')
      outf.write('        "coordinates": [\n')
      outf.write('          [\n')
      outf.write('            [{1},{0}],\n'.format(float(entry[0]) - sqrrad, float(entry[1]) - sqrrad))
      outf.write('            [{1},{0}],\n'.format(float(entry[0]) + sqrrad, float(entry[1]) - sqrrad))
      outf.write('            [{1},{0}],\n'.format(float(entry[0]) + sqrrad, float(entry[1]) + sqrrad))
      outf.write('            [{1},{0}],\n'.format(float(entry[0]) - sqrrad, float(entry[1]) + sqrrad))
      outf.write('            [{1},{0}]\n'.format(float(entry[0]) - sqrrad, float(entry[1]) - sqrrad))
      outf.write('          ]\n')
      outf.write('        ]\n')
      outf.write('      }\n')

      if i+1 < len(data):
        outf.write('    },\n')
      else:
        outf.write('    }\n')

    outf.write('  ]\n')
    outf.write('}\n')

# Timer
def main():
  while True:
    writeGJ()
    print("JSON Updated: %.2f Seconds Elapsed" % time.perf_counter())
    time.sleep(interval)

if __name__ == '__main__':
    main()
