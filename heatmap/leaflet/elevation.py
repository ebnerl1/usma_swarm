import json
import urllib

def elevation(lat, lng):
    apikey = "AIzaSyDvuEAYeb9xoSun0PHXVkM7oxl_sRZD2H4"
    url = "https://maps.googleapis.com/maps/api/elevation/json"
    request = urllib.urlopen(url+"?locations="+str(lat)+","+str(lng)+"&key="+apikey)
    try:
        results = json.load(request).get('results')
        if 0 < len(results):
            elevation = results[0].get('elevation')
            # ELEVATION
            print(elevation)
            return elevation
        else:
            print ('HTTP GET Request failed.')
    except ValueError, e:
        print ('JSON decode failed: '+str(request))

elevation(41.3588842,-74.0319426)
