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

