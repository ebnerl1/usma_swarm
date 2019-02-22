import sys

file = open("quad_blue.wp", "r")

start = 0

for line in file:
	if start == 0:
		start = 1
	else:
		fields = line.split("\t")
		id = fields[0]
		lat = fields[8]
		lon = fields[9]
		if id == '5':
			print "id is " + str(id) + " lat is " + str(lat) + " long is " + str(lon)

file.close()
