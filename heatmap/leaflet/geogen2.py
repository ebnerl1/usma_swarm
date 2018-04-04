import csv
import time

######################## VARIABLES ########################

infile = "sample_data2.csv" # Input file as a .csv
outfile = "sample_gj2.js" # Output file as a .geojson
radcap = 0 # Highest amount of radiation to scale color gradient
interval = 5 # Time in seconds in between scans

###########################################################

# [0] and [1] are long/lat, [2] is rads, [3] is alt
def parseIn():
  global radcap
  data = []
  with open(infile, 'r') as inf:

    r = csv.reader(inf)

    for row in r:

      lat = float(row[0][1:-1])
      lon = float(row[1][2:-1])
      counts = float(row[2][2:-1])
      alt = float(row[3][2:-1])

      if counts > radcap:
        radcap = counts

      data.append([lat, lon, counts, alt])

    for i in range(len(data)):
      data[i][2] = altConvert(data[i][2],data[i][3])

    return data

# Converts sample counts to actual counts
def altConvert(sample, alt):
  global radcap
  conv = sample * ((alt*0.3048)**2)
  return (float(conv / radcap))

# Iterates through entire parsed file to overwrite .geojson
def writeGJ():
  data = parseIn()
  with open(outfile, 'w') as outf:
    outf.write('var points = [\n')

    for i in range(len(data)):
      entry = data[i]
      outf.write('[{0},{1},"{2}"]'.format(entry[0],entry[1],entry[2]))
      if i+1 < len(data):
        outf.write(',\n')
      else:
        outf.write('\n')

    outf.write('];')

# Timer
def main():
  while True:
    writeGJ()
    print("File Updated: %.2f Seconds Elapsed" % time.perf_counter())
    time.sleep(interval)

if __name__ == '__main__':
    main()
