import csv
Hello = 3
myFile = open('Simulations.csv','a')
with myFile:
    writer = csv.writer(myFile)                    
    writer.writerow(['self._counter*0.1','self._total_score',Hello])
