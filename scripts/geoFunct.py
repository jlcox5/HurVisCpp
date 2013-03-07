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

