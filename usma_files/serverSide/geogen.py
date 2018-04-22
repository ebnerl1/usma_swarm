import csv
import time
from datetime import datetime
import json
import urllib
import procname

procname.setprocname("serverSide")

######################## VARIABLES ########################

infile = "raw_data.csv" # Input file as a .csv
outfile = "parsed_data.js" # Output file as a .geojson
archive = "archive_" + str(datetime.now()) + ".csv"
radcap = 0 # Highest amount of radiation to scale color gradient
interval = 5 # Time in seconds in between scans

###########################################################

mapalt = 0

def elevation(lat, lng):
    apikey = "AIzaSyDvuEAYeb9xoSun0PHXVkM7oxl_sRZD2H4"
    url = "https://maps.googleapis.com/maps/api/elevation/json"
    request = urllib.urlopen(url+"?locations="+str(lat)+","+str(lng)+"&key="+apikey)
    try:
        results = json.load(request).get('results')
        if 0 < len(results):
            elevation = results[0].get('elevation')
            # ELEVATION
            return elevation
        else:
            print ('HTTP GET Request failed.')
    except ValueError:
        print ('JSON decode failed: '+str(request))

# [0] and [1] are long/lat, [2] is rads, [3] is alt
def parseIn():
  global radcap
  data = []
  with open(infile, 'r') as inf:

    r = csv.reader(inf)

    for row in r:

      lat = float(row[0])
      lon = float(row[1])
      counts = float(row[2])
      alt = float(row[3])
      droneID = int(row[4])

      if counts > radcap:
        radcap = counts

      data.append([lat, lon, counts, alt, droneID])

    for i in range(len(data)):
      data[i][2] = altConvert(data[i][0], data[i][1], data[i][2], data[i][3])

    return data

# Converts sample counts to actual counts
def altConvert(lat, lon, sample, alt):
  global radcap
  global mapalt
  mapalt = elevation(lat, lon)
  heightAboveGround = (alt - mapalt) * 0.3048
  conv = sample * ((heightAboveGround)**2)
  return (float(conv / radcap))

#def writeArchive(data):
#  with open(archive, 'w') as outf:
#    outfwriter = csv.writer(outf)
#    outfwriter.writerow(

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
  start_time = time.time()
  while True:
    writeGJ()
    print("Parsed Data File Updated")
    time.sleep(interval)

if __name__ == '__main__':
    main()
