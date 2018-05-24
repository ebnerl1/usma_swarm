import csv



with open("archive_2018-04-26 14_10_38.csv", 'r') as outf:
  heatmapdata = []
  maxi = 0  
  fread = csv.reader(outf)
  for row in fread:
    heatmapdata.append([float(row[3]),float(row[4]),float(row[8])])
  for row in heatmapdata:
    if maxi < row[2]:
      maxi = row[2]
  for i in range(len(heatmapdata)):
    heatmapdata[i][2] = heatmapdata[i][2] / maxi  
  with open("savedparse.js", 'w') as outfile:
    outfile.write("points = [\n")
    for i in range(len(heatmapdata)):
      outfile.write('[{0},{1},"{2}"]'.format(heatmapdata[i][0],heatmapdata[i][1],heatmapdata[i][2]))
      if i + 1 < len(heatmapdata):
        outfile.write(',\n')
      else:
        outfile.write('\n')
        outfile.write('];')

