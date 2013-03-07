#!/usr/bin/env python3.1

# Jonathan Cox
# Clemson University
# 
# This code will parse a file containing a predicted path for a hurricane
# and output a list of interpolated positions, bearings, and speeds

import math

class path(object):

  def __init__(self, pathFile):
    self.posList = []
    self.interpList = []    # List of positions, bearings and speeds 
                            #    from a point to succeeding point
    self.finalList = []     # List of positions, resulting bearings, and speeds
                            #    from a point to succeding point
    self.difList = []       # List of positions, bearing and speed differences
    self.initBearing = 0.0
    self.initSpeed = 0.0
    self.parseFile(pathFile)

    # Set time step
    self.ts = 3.0
    
    # Linear interpolate along all segments
    for i in range(0, len(self.posList)-1):
      p0 = self.posList[i]
      p1 = self.posList[i+1]

      h = 0
      if i == 0 : h = 9.0
      if i == 1 : h = 12.0
      if i == 2 : h = 12.0
      if i == 3 : h = 12.0
      if i == 4 : h = 24.0

      self.interpolateSegment(p0, p1, h)

    # Go and find the resulting bearings from one point to succeeding point
    self.findFinalInfo()

    # Find differences between resulting bearings and travel bearings
    self.findDifferences()

    #self.verifyGeoFunct()
      

  def parseFile(self, fileName):
    count = 0
    posInfo = []

    with open(str(fileName)) as pathData:
      for line in pathData:
        line = line.split()
        posInfo = [float(line[0]), float(line[1])]
        self.posList.append(posInfo)
        if count == 0:
          self.initBearing = float(line[2])
          self.initSpeed = float(line[3])
        count = count + 1

  def interpolateSegment(self, p0, p1, h):
    curTime = 0.0
    dist = self.haversine(p0, p1)
    speed = float(dist/h)        # Speed to travel along length of segment
    segments = int(h/self.ts)    # Number of segments between these two points
    disPerSeg = float(dist/segments)

    curPos = p0
    tBear = self.findBearing(curPos, p1)
    
    self.interpList.append([curPos[0], curPos[1], tBear, speed])
    for i in range(0, int(segments)-1):
      curPos = self.locatePosition(curPos[0], curPos[1], disPerSeg, tBear)
      #tBear = self.findBearing(curPos, p1)
      tBear = self.findBearing(curPos, p1)
      self.interpList.append([curPos[0], curPos[1], tBear, speed])

    if h == 24.0:
      self.interpList.append([p1[0], p1[1], tBear, speed])

  # Finds resulting bearings for each point (bearing at after traveling from p0 to p1) and
  # stores them in a list along with the lat and lon, and the previous speed
  def findFinalInfo(self):
    p = [self.interpList[0][0], self.interpList[0][1], self.initBearing, self.initSpeed]
    self.finalList.append(p)

    for i in range(1, len(self.interpList)):
       resBear = self.findBearing(self.interpList[i-1], self.interpList[i])
       #resBear = math.fmod(resBear + 180.0, 360.0)
       p = [self.interpList[i][0], self.interpList[i][1], resBear, self.interpList[i-1][3]]
       self.finalList.append(p)

  def findDifferences(self):
    for i in range(0, len(self.interpList)):
      lat = self.interpList[i][0]
      lon = self.interpList[i][1]
      bDif = self.findDif(self.interpList[i][2],  self.finalList[i][2])
      sDif = self.interpList[i][3] - self.finalList[i][3]

      if abs(bDif) < 0.00001 : bDif = 0.0
      self.difList.append([lat, lon, bDif, sDif])

  def verifyGeoFunct(self):
    bearingList = []
    distanceList = []
    testPos = []
    curPos = self.interpList[0]

    for i in range(0, len(self.interpList)-1):
      bearingList.append(self.findBearing(self.interpList[i], self.interpList[i+1]))
      distanceList.append(self.haversine(self.interpList[i], self.interpList[i+1]))

    print("Interpolated Size: ", len(self.interpList))
    print("Bearing List Size: ", len(bearingList))
    print("distanceList Size: ", len(distanceList))
    print("difList Size: ", len(self.difList))

    print("Computed values")
    for i in range(0, len(bearingList)):
      print("   Bearing: ", bearingList[i], "     Distance List: ", distanceList[i])
       
    print()

    # Rebuild path using distance and bearing information generated
    #testPos.append(self.interpList[0])
    #for i in range(0, len(bearingList)):
    #  newPos = self.locatePosition(curPos[0], curPos[1], distanceList[i], bearingList[i])
    #  testPos.append(newPos)
    #  curPos = newPos

    # Rebuild path using difference information
    testPos.append(self.interpList[0])
    curBearing = self.initBearing
    curSpeed = self.initSpeed

    # difList includes bearing difference of last point in path, which we do not want to include
    for i in range(0, len(self.difList)-1):
      print("   Bearing: ", curBearing, "     Speed: ", curSpeed, "     Traveling: ", curSpeed*3.0)
      curBearing = curBearing + self.difList[i][2]
      curSpeed = curSpeed + self.difList[i][3]
      newPos = self.locatePosition(curPos[0], curPos[1], curSpeed*3.0, curBearing)
      testPos.append(newPos)
      curPos = newPos

    #self.printInterpPath()
    print()

    print("Test Pos size: ", len(testPos))
    print("Rebuilt: ")
    for i in range (0, len(testPos)):
      print("   ", testPos[i])

  ### Functions below are utility functions ###

  def haversine(self, p0, p1):
    earthRad = 6378.1

    Lat1 = math.radians(p0[0])
    Lat2 = math.radians(p1[0])
    Lon1 = math.radians(p0[1])
    Lon2 = math.radians(p1[1])

    dLat = math.radians(p1[0] - p0[0])
    dLon = math.radians(p1[1] - p0[1])

    t = math.sin(dLon/2.0)
    t = t*t
    a = math.sin(dLat/2.0)*math.sin(dLat/2.0) + math.cos(Lat1)*math.cos(Lat2)*t

    b = 2.0*math.atan2(math.sqrt(a), math.sqrt(1.0-a))
    c = earthRad * b
    return c

  def findBearing(self, p1, p2):
    dLon = p2[1] - p1[1]
    dLon = math.radians(dLon)
    lat1 = math.radians(p1[0])
    lat2 = math.radians(p2[0])

    y = math.sin(dLon)*math.cos(lat2)
    x1 = math.cos(lat1)*math.sin(lat2)
    x2 = math.sin(lat1)*math.cos(lat2)*math.cos(dLon)
    x = x1 - x2

    toReturn = math.degrees(math.atan2(y, x))
            
    if toReturn < 0:
      toReturn = toReturn + 360.0

    return toReturn

  def locatePosition(self, lat, lon, dist, bear):
    toReturn = []
    earthRad = 6371.0
    
    if bear == 0:
      bear = 360.0

    rLon = math.radians(lon)
    rLat = math.radians(lat)
    rBear = math.radians(bear)
    d = dist/earthRad

    newLat = math.asin(math.sin(rLat)*math.cos(d) + math.cos(rLat)*math.sin(d)*math.cos(rBear))
    newLon = rLon + math.atan2(math.sin(rBear)*math.sin(d)*math.cos(rLat), math.cos(d)-math.sin(rLat)*math.sin(newLat))

    toReturn = [math.degrees(newLat), math.degrees(newLon)]
    return toReturn

  def findDif(self, a, b):
    r = a - b;

    if r > 180.0:
      r = r - 360.0
    elif r < -180.0:
      r = r + 360

    return r

  ### Print Functions ###

  def printPath(self):
    print("Initial Bearing: ", self.initBearing, "     Initial Speed: ", self.initSpeed)
    print("Position List: " )
    for i in range(0, len(self.posList)):
      print("   ", self.posList[i][0], self.posList[i][1])

  def printInterpPath(self):
    for i in range(0, len(self.interpList)):
      print(self.interpList[i][0], " ", self.interpList[i][1], " ", self.interpList[i][2], " ", self.interpList[i][3])

  def printDifferences(self):
    for i in range(0, len(self.difList)):
      print(self.difList[i][0], " ", self.difList[i][1], " ", self.difList[i][2], " ", self.difList[i][3])

if __name__ == '__main__':
   hurricane = path("hurGustav.pre")
   #hurricane.printPath()
   #print()
   #hurricane.printInterpPath()
   #print()
   hurricane.printDifferences()


