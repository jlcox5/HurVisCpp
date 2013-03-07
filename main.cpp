/* Jonathan Cox
   CPSC 805
*/

#include <iostream>
#include <string>
#include <fstream>
#include <stdlib.h>

#include <GL/glu.h>

#include "posPoint.h"
#include "hurVis.h"
#include "geoFunct.h"
#include "advisory.h"
#include "path.h"
#include "P6Image.h"
#include "screenshot.h"
#include "Matrix.h"
#include "simulation.h"
#include "gridStruct.h"
#include "window.h"

#include <QApplication>
#include <QGLFormat>


gridStruct * dataGrid;
path * testPath;
path * testPath2;
path * curPath;

// Global Variables and Such
//int wWidth, wHeight;

simulation * sim;

// Distance of projected path
double projPathDist = 0.0;

// Map texture
GLuint textures[1];
P6Image * imageMap;

// Display mode variables
int displayDataPoints = -1;
int displayDataPaths = -1;
int displayProjectedPath = 1;
int displayGenPaths = 1;
int displayAvgPath = -1;
int heatMap = -1;
int interactive = -1;
int hitButton = -1;
int showCurPath = -1;
int chooseForward = -1;
int stall = 1;
int preOnly = -1;
int colorClosest = -1;
int averageOnAll = 1;

int modify = -1;
int totalPaths = 0;
double omega = 0.0;
double userSig = 298.0;   //  Starts at error cone radius at 69 hours
int curAdvisory = 0;

double speedRatio = 0.00;
double bearRatio = 0.00;

// Poor use of global variables....
double curDistance;

// Interpolate along a cubic curve to add data points
// f is the final point, s is the start point
void addPoints(posPoint * s, posPoint * f, int pid){
   Vector2d p, p0, p1;
   Vector2d v, v0, v1;
   Vector2d dp, dv;
   Vector2d a, b, c, d;
   Vector2d * screenCoord;
   posPoint * prev;
   double fDeg, fSpeed;
   Vector2d * findBear1, * findBear2, * binPush;

   float dt, Bearing, Speed;

   int N = 10;       // number of points to interpolate between known points
   float ds = 1.0 / N;
   float seg;
   float lat0, lon0, lat1, lon1, ratio0, ratio1;
   double dydt, dyds0, dyds1;
   double dxdt, dxds0, dxds1;
   double bear0, dist0, bear1, dist1, earthRad;
   int m, da;

   posPoint * np;

   std::vector<Vector2d*>::iterator dataIt;
   std::vector<path*>::iterator dataPathIt;

   dt = 6;             // number of hours between samples
   m = s->getMonth();
   da = s->getDay();
   earthRad = 6371.0;

   prev = s;
   findBear1 = new Vector2d;
   findBear2 = new Vector2d;

   // Values for s = 0
   bear0 = DegToRad(s->getDeg());     // Bearing in radians
   dist0 = s->getSpeed();       // Speed in km/h
   lat0 = DegToRad(s->getLat());   // Latitude in radians
   lon0 = DegToRad(s->getLon());   // Longitude in radians
   ratio0 = dist0/earthRad;     // r/R (a radian measure)
   dxdt = ratio0*sin(bear0);    // radians longtitude/h
   dxds0 = dxdt * dt;        // radians per unit of s
   dydt = ratio0*cos(bear0);    // radians latitude/h
   dyds0 = dydt * dt;        // radians per unit of s

   p0 = Vector2d(lon0, lat0);   // position, radians long, lat
   v0 = Vector2d(dxds0, dyds0);    // velocity, radians per unit of s

   // Values for s = 1
   bear1 = DegToRad(f->getDeg());     // Bearing in radians
   dist1 = f->getSpeed();       // Speed in km/h
   lat1 = DegToRad(f->getLat());   // Latitude in radians
   lon1 = DegToRad(f->getLon());   // Longitude in radians
   ratio1 = dist1/earthRad;     // r/R (a radian measure)
   dxdt = ratio1*sin(bear1);    // radians longtitude/h
   dxds1 = dxdt * dt;        // radians per unit of s
   dydt = ratio1*cos(bear1);    // radians latitude/h
   dyds1 = dydt * dt;        // radians per unit of s

   p1 = Vector2d(lon1, lat1);   // position, radians long, lat
   v1 = Vector2d(dxds1, dyds1);    // velocity, radians per unit of s

   // Solve for the cubic coefficients
   dp = p1 - p0;          // delta p between points (radians long and lat)
   dv = v1 - v0;          // delta v between points (radians per unit of s)

   a = p0;
   b = v0;
   c = 3 * dp - 2*v0 - v1;
   d = -2 * (dp - v0) + dv;

   // Now that coeffecients are known, interpolate points on the curve


   cout << "start point: " << s->getLat() << "," << s->getLon() << " " << s->getSpeed() << " " << s->getDeg() << "\n";
   cout << "coef: " << a << " , " << b << " , " << c << " , " << d << "\n";
   for(int i = 1; i < N; i++){
     seg = i * ds;    // compute parameter s at this subpoint on the curve
     //cout << "   Seg: " << seg << "\n";

     // Find longitude and latitude in degrees
     p = RadToDeg(a + (b + (c + d * seg) * seg) * seg);
     // Find the longitudinal velocity at this point in radians/hr
     v = (v0 + (2 * c + 3 * d * seg) * seg) / dt;

     Speed = earthRad * v.norm();          // speed in km/h
     Bearing = RadToDeg(atan2(v.x, v.y));     // bearing in degrees
     if(Bearing < 0){
        Bearing = 360 + Bearing;
     }

     np = new posPoint(m, da, p.y, p.x, Bearing, (int)Speed, f->getWind(), pid);
     cout << "    new point: " << np->getLon() << "," << np->getLat() << "\n";
     cout << "       with speed: " << Speed << "    bearing: " << Bearing << "\n";\

     findBear1->x = prev->getLon();
     findBear1->y = prev->getLat();
     findBear2->x = p.x;
     findBear2->y = p.y;
     fDeg = findBearing_2(findBear1, findBear2);
     fSpeed = haversine(findBear1->x, findBear2->x, findBear1->y, findBear2->y)/(dt*ds);
     binPush = new Vector2d(fDeg, fSpeed);
     sim->bearBin[(int)prev->getDeg()].push_back(binPush);
     if((int)prev->getDeg() == 0){
        sim->bearBin[360].push_back(binPush);
     }
     else if((int)prev->getDeg() == 360){
        sim->bearBin[0].push_back(binPush);
     }
     prev = np;

     sim->positionList.push_back(np);
     testPath2->positionList.push_back(np);

     // Push screen coordinates into path list so the path can be drawn
     screenCoord = new Vector2d(p.y, p.x);
     dataIt = testPath2->posList.end();
     testPath2->posList.insert(dataIt, screenCoord);
   }

   // Find value for the last generated point and final position
   findBear1->x = prev->getLon();
   findBear1->y = prev->getLat();
   findBear2->x = f->getLon();
   findBear2->y = f->getLat();
   fDeg = findBearing_2(findBear1, findBear2);
   fSpeed = haversine(findBear1->x, findBear2->x, findBear1->y, findBear2->y)/(dt*ds);
   binPush = new Vector2d(fDeg, fSpeed);
   sim->bearBin[(int)prev->getDeg()].push_back(binPush);
   if((int)prev->getDeg() == 0){
      sim->bearBin[360].push_back(binPush);
   }
   else if((int)prev->getDeg() == 360){
      sim->bearBin[0].push_back(binPush);
   }
   prev = np;
   cout << "finish point: " << f->getLat() << "," << f->getLon() << " " << f->getSpeed() << " " << f->getDeg() << "\n";

}

void parseHurDat(char * fileName){

   fstream in(fileName, fstream::in);
   string input, toF;
   char * fromFile;
   int i, count, foundFirst, clear;
   size_t startNum, endNum;
   int tMonth, tDay, tSpe, tWin;
   double tLat, tLon, tDeg;
   double fDeg, fSpeed;
   Vector2d tPos, tempPos;
   posPoint * toInsert, * prev;
   std::vector<Vector2d*>::iterator dataIt;
   std::vector<path*>::iterator dataPathIt;
   std::vector<posPoint*>::iterator _p;
   Vector2d * setPos, * findBear1, * findBear2, * binPush;

   // Build path list out of available data

   tLat = -1.0;
   tLon = -1.0;
   prev = NULL;
   foundFirst = 0;

   // Check file open
   if(in == NULL){
      cout << "Unable to open file: " << fileName << "\n";
      exit(1);
   }

   fromFile = (char*)malloc(sizeof(char)*512);

   in.getline(fromFile, 512);

   findBear1 = new Vector2d;
   findBear2 = new Vector2d;

   while(!in.eof()){
      tPos.set(0.0, 0.0);
      input.assign(fromFile, 512);
      //cout << "Line: " << input << "\n";
      startNum = input.find_first_of("*");
      if(startNum == string::npos){
         count = 0;
         startNum = input.find_first_of("-1234567890.");
         i = 0;
         while((startNum != string::npos) && (i == 0)){
            input = input.substr(startNum);
            endNum = input.find_first_of(" ");
            toF = input.substr(0, endNum);
            if(count == 0){ tMonth = atoi(toF.c_str()); }
            else if(count == 1){ tDay = atoi(toF.c_str()); }
            else if(count == 2){ tLat = atof(toF.c_str()); }
            else if(count == 3){ tLon = atof(toF.c_str()); }
            else if(count == 4){ tDeg = atof(toF.c_str()); }
            else if(count == 5){ tSpe = atoi(toF.c_str()); }
            else if(count == 6){ tWin = atoi(toF.c_str()); }
            if(endNum == string::npos){ i = -1; }
            else{ input = input.substr(endNum); }
            startNum = input.find_first_of("-1234567890.");
            count++;
            //cout << "toF: " << toF << "  input: " << input << "\n";
         }
         if(count != 7){
            cout << "Invalid point file: " << count << "\n";
            exit(1);
         }
         else{
            // Determine if we need to set the distance from the previous point to this point
            if( foundFirst != 0){
               //cout << "tLon: " << tLon << " tLat: " << tLat << "\n";
               tPos.set(tLat-prev->getLon(), tLon-prev->getLat());
               dataIt = testPath->posList.end();
               tempPos = translateToScreen(tLon, tLat);
               //cout << " path lat: " << tLat << " lon: " << tLon << "\n";
               setPos = new Vector2d;
               setPos->x = tempPos.x;
               setPos->y = tempPos.y;
               testPath->posList.insert(dataIt, setPos);
               dataIt = testPath2->posList.end();
               testPath2->posList.insert(dataIt, setPos);
               if( tPos.x != tPos.x || tPos.y != tPos.y){
                  cout << "bad tPos: " << tPos << " with tLon: " << tLon <<
                     " prev->Lon: " << prev->getLon() << "\n";
               }

               // First point of each hurricane in data file has no speed or bearing
               // Need to calculate this to avoid having bad data points
               if(foundFirst == 1){
                  findBear1->x = prev->getLon();
                  findBear1->y = prev->getLat();
                  findBear2->x = tLon;
                  findBear2->y = tLat;
                  fDeg = findBearing_2(findBear1, findBear2);
                  fSpeed = haversine(prev->getLon(), tLon, prev->getLat(), tLat)/6;
                  prev->setSpeed( (int)fSpeed );
                  prev->setDeg(fDeg);

                  binPush = new Vector2d(fDeg, fSpeed);
                  sim->bearBin[(int)fDeg].push_back(binPush);
                  if((int)fDeg == 0){
                     sim->bearBin[360].push_back(binPush);
                  }
                  else if((int)fDeg == 360){
                     sim->bearBin[0].push_back(binPush);
                  }
                  foundFirst = 2;
               }
               else{
                  //sim->bearBin[(int)prev->getDeg()].push_back(fDeg);
               }
               prev->setPosDif(tPos);
            }
            else{ 
               foundFirst = 1;
               if( prev != NULL ){
                  dataPathIt = sim->dataPathList.end();
                  sim->dataPathList.insert(dataPathIt, testPath);
                  dataPathIt = sim->dataPathAugList.end();
                  sim->dataPathAugList.insert(dataPathIt, testPath2);
                  tPos.set(0.0, 0.0);
                  prev->setPosDif(tPos);
               };
               testPath = new path(tLat, tLon, tDeg, tSpe);
               testPath2 = new path(tLat, tLon, tDeg, tSpe);
            }
            toInsert = new posPoint(tMonth, tDay, tLat, tLon, tDeg, tSpe, tWin, sim->dataPathAugList.size());

            if(toInsert->getPoint().x == 0.0 and toInsert->getPoint().y == 0.0){
               cout << "bad point!!! lat: " << tLat << " lon: " << tLon << " deg: " << tDeg << "\n";
            }

            // If this is the first point, go ahead and push it onto the position list
            if(foundFirst == 1){
               sim->positionList.push_back(toInsert);
               testPath->positionList.push_back(toInsert);
               testPath2->positionList.push_back(toInsert);
            }
            // Otherwise, if the speed is not 0.0, push it onto the list
            else if(toInsert->getSpeed() != 0.0){
               // Interpolate to find additional data points
               //addPoints(prev, toInsert, sim->dataPathAugList.size());
               sim->positionList.push_back(toInsert);
               testPath->positionList.push_back(toInsert);
               testPath2->positionList.push_back(toInsert);
            }
            prev = toInsert;
         }
      }
      else{
         foundFirst = 0;
         tPos.set(0.0, 0.0);
      }
      for( clear = 0; clear < 512; clear++){
         fromFile[clear] = '\0';
      }
      in.getline(fromFile, 512);
   }
   dataPathIt = sim->dataPathList.end();
   sim->dataPathList.insert(dataPathIt, testPath);
   dataPathIt = sim->dataPathAugList.end();
   sim->dataPathAugList.insert(dataPathIt, testPath2);
   sim->printBearBin();
}


//-----------------------Main--------------------------------------------------

int main( int argc, char *argv[] ){

   // Make sure we have a file to open
   if(argc < 2){
      cout << "Correct usage: ./main <inital method - int>\n";
      return 1;
   }

   int im = atoi(argv[1]);
   // Build simulation
   sim = new simulation(1, -100.0, -75.0, 17.0, 31.0, (180.0/60.0), 1.0, 1.6, 0.953, 15.8, -25.0, 33, -100.0, 100.0, im);
   if(sim->getDispNum() == 1){
      sim->setW(1024);
      sim->setH(790);
   }
   else{
      sim->setW(614);
      sim->setH(474);
   }
   sim->setPixPerKM(sim->getH()/haversine(sim->getMaxLon(), sim->getMaxLon(), sim->getMaxLat(), sim->getMaxLat()-sim->getTotLat()));
   cout << "pixPerKM: " << sim->getPixPerKM() << "\n";

   buildAdvisory();
   buildAdvisory();
   sim->buildExp();

   dataGrid = new gridStruct(17, 26, 60, 17, 33, -75, -100, "resHistCur.txt", "resPreCur.txt"); 

   //sim->buildStatPaths();

   // QT Stuff
   QApplication app(argc, argv);

   if(!QGLFormat::hasOpenGL()){
      qWarning("This system has no OpenGL support.  Exiting.");
      return -1;
   }

   QGLFormat glf;
   glf.setDoubleBuffer(TRUE);
   glf.setRgba(TRUE);
   glf.setDepth(TRUE);
   QGLFormat::setDefaultFormat(glf);

   // Main window
   window * win = new window();
   win->resize((int)(sim->getW()*(max(1, sim->getDispNum()/2))), 
               (int)(sim->getH()*(max(1, sim->getDispNum()/3))));

   win->setWindowTitle(QApplication::translate("hurricane", "Hurricane Visualization"));
   win->show();

   int result = app.exec();
   delete win;
   return result;
}

