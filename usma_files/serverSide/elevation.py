import rasterio
import numpy as np
import urllib
import json

elevationFile = 'srtm_14_04.tif'

#43.873783,-112.728411,27

def elevationOffline(lat, lng):
    global elevationFile
    coords = [(lat,lng)]
    elevation = 0
    with rasterio.open(elevationFile) as src:
        for val in src.sample(coords):
            elevation = val
        return elevation[0]

def elevationOnline(lat, lng):
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

print(elevationOffline(-112.728411, 43.873783))
print(elevationOnline(43.873783, -112.728411))
