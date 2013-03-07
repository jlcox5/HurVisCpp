#!/usr/bin/env python3.1

# Jonathan Cox
# Clemson University
# 26APR10

# Takes the hurricane data file and gives a file in the following format:
# Month(1-12) Day(#) Lat(Real) Long(Real) Dir(Deg) Speed(KPH) Wind(KPH) 
#(Passed type for now as wind and speed should be enough information)Type(Int)

# The key for Type is as follows:
# 0 - Tropical Depression
# 1 - Tropical Storm
# 2 - Category 1
# 3 - Category 2
# 4 - Category 3
# 5 - Category 4
# 6 - Category 5

import sys
import fileinput

def parseHurDat(fileName):
   toWrite = ''
   out = open("formatted.txt", "w")
   
   with open(str(fileName)) as hurDat:
      for line in hurDat:
         if line[0] != '*' and line[0:5] != "Storm" and line[0:5] != "Month" and len(line) > 10:
            # Split the line on Spaces
            line = line.split()

            # Convert Month:
            if line[0] == "January":
               toWrite = "1 "
            elif line[0] == "February":
               toWrite = "2 "
            elif line[0] == "March":
               toWrite = "3 "
            elif line[0] == "April":
               toWrite = "4 "
            elif line[0] == "May":
               toWrite = "5 "
            elif line[0] == "June":
               toWrite = "6 "
            elif line[0] == "July":
               toWrite = "7 "
            elif line[0] == "August":
               toWrite = "8 "
            elif line[0] == "September":
               toWrite = "9 "
            elif line[0] == "October":
               toWrite = "10 "
            elif line[0] == "November":
               toWrite = "11 "
            elif line[0] == "December":
               toWrite = "12 "
            else:
               toWrite = "-1"

            # Add Day:
            toWrite = toWrite + str(line[1]) + " "
            

            # Add Lat:
            lat = line[4][0:len(line[4])-1]
            if line[4][len(line[4])-1] == "S":
               toWrite = toWrite + "-"
            toWrite = toWrite + str(lat) + " "

            # Add Long:
            lon = line[5][0:len(line[5])-1]
            if line[5][len(line[5])-1] == "W":
               toWrite = toWrite + "-"
            toWrite = toWrite + str(lon) + " "

            # Add Dir:
            if line[6] == "--":
               toWrite = toWrite + "0" + " "
            else:
               toWrite = toWrite + str(line[6]) + " "

            # Add Speed KPH
            if line[10] == "--":
               toWrite = toWrite + "0" + " "
            else:
               toWrite = toWrite + str(line[10]) + " "

            # Add Wind KPH
            if line[14] == "--":
               toWrite = toWrite + "0" + " "
            else:
               toWrite = toWrite + str(line[14]) + " "

            toWrite = toWrite + "\n"
            if lat != "0.0" and lon != "0.0":
               out.write(toWrite)

         elif line[0] == '*':
            toWrite = "*\n"
            out.write(toWrite)
         # Output line to file

if __name__ == '__main__':
   print("Starting parse")
   parseHurDat(sys.argv[1])
   print("Finished!")
