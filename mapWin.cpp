#include <QImage>
#include <QTimer>
#include <QEvent>
#include <QString>
#include <QFileDialog>
#include <QMouseEvent>

#include <QPixmap>
#include <QPoint>
#include <QPainter>

#include <GL/glu.h>
#include <fstream>
#include <stdlib.h>
#include <climits>

#include "mapWin.h"
#include "posPoint.h"
#include "advisory.h"
#include "path.h"
#include "P6Image.h"
#include "simulation.h"
#include "Matrix.h"


extern simulation * sim;

// Map texture
extern GLuint textures[1];
extern P6Image * imageMap;

// Display mode variables
extern int displayDataPoints;
extern int displayDataPaths;
extern int displayProjectedPath;
extern int displayGenPaths;
extern int displayAvgPath;
extern int heatMap;
extern int interactive;
extern int hitButton;
extern int showCurPath;
extern int chooseForward;
extern int stall;
extern int preOnly;
extern int colorClosest;
extern int averageOnAll;

extern int totalPaths;
extern double omega;
extern double userSig;   //  Starts at error cone radius at 69 hours
extern int curAdvisory;

extern path * testPath;
extern path * curPath;

extern double speedRatio;
extern double bearRatio;

// For random number generation
extern unsigned int counter;

mapWin::mapWin(QWidget * parent): QGLWidget(parent){
   timer = new QTimer(this);
   connect(timer, SIGNAL(timeout()), SLOT(update()));
   timer->setInterval(10);
   timer->setSingleShot(FALSE);
   timer->start();

   cout << "DB: " << format().doubleBuffer() << "   rgba: " << format().rgba() 
        << "   depth: " << format().depth() << endl;


   butMin.set(sim->getW()-125, sim->getH()-75);
   butMax.set(sim->getW()-25, sim->getH()-25);

   butMinBord.set(sim->getW()-127, sim->getH()-77);
   butMaxBord.set(sim->getW()-23, sim->getH()-23);

   butTextPos.set(sim->getW()-102, sim->getH()-45);
   buttonPressed = 0;
   completed = 0;

   step = 0;
}

mapWin::~mapWin(){
   makeCurrent();
}

void mapWin::initializeGL(){
   glClearColor(0.0, 0.0, 0.0, 1.0);
   gluOrtho2D(0.0, sim->getW(), sim->getH(), 0.0);
   glClear(GL_COLOR_BUFFER_BIT);
   glEnable(GL_TEXTURE_2D);
   glPolygonMode(GL_FRONT, GL_FILL);
   glPolygonMode(GL_BACK, GL_FILL);
   glDisable(GL_DEPTH_TEST);

   glGenTextures(1, &textures[0]);
   setAutoFillBackground(false); 
}

void mapWin::resizeGL(int w, int h){
   glViewport(0, 0, w, h);

   glMatrixMode(GL_PROJECTION);
   glLoadIdentity();
   gluOrtho2D(0.0, w, h, 0.0);

   glMatrixMode(GL_MODELVIEW);
   glLoadIdentity();
   imageOpen();
   setFocus();
}

void mapWin::paintGL(){
   std::vector<Vector2d*>::iterator pos1;
   std::vector<Vector2d*>::iterator pos2;
   std::vector<posPoint*>::iterator posListIt;
   std::vector<path*>::iterator pathIt;
   std::vector<advisory*>::iterator advIt;
   Vector2d draw1, draw2;
   float lineWidth;

   lineWidth = 0.5;

   glClear(GL_COLOR_BUFFER_BIT);
   if (displayProjectedPath == 1) {
      sim->drawForecastPath();
   }

   if(interactive == -1){
      glColor4f(1.0f, 0.0f, 0.0f, 1.0f);
      // Draw normal display with generated paths
      if(heatMap == -1){
         glEnable(GL_TEXTURE_2D);
         // Draw rectangle for background image
         glPolygonMode(GL_FRONT, GL_FILL);
         glPolygonMode(GL_BACK, GL_FILL);
         //glBindTexture(GL_TEXTURE_2D, textures[0]);
         glBegin(GL_POLYGON);
            glTexCoord2f(0.0, 0.0);
            glVertex3f(0.0, 0.0, 0.0);
            glTexCoord2f(1.0, 0.0);
            glVertex3f(sim->getW(), 0.0, 0.0);
            glTexCoord2f(1.0, 0.771484375);
            glVertex3f(sim->getW(), sim->getH(), 0.0);
            glTexCoord2f(0.0, 0.771484375);
            glVertex3f(0.0, sim->getH(), 0.0);
         glEnd();

         glDisable(GL_TEXTURE_2D);
         glEnable(GL_BLEND);
         glEnable(GL_LINE_SMOOTH);
         //glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
         glBlendFunc(GL_ONE, GL_ONE_MINUS_SRC_ALPHA);

         // Display generated paths
         /*if(sim->getCurrentAdv() < sim->advCombList.size()/2){
            if (displayGenPaths == 1) {
               if(colorClosest > 0){
                  sim->drawGenPathsClosest();
               }
               sim->drawGenPaths();
            }
         }*/

         //step = sim->drawGenPathsTrail(step);
         sim->drawGenPaths();

         glDisable(GL_LINE_SMOOTH);
         glDisable(GL_BLEND);
         glEnable(GL_TEXTURE_2D);
         glPolygonMode(GL_FRONT, GL_FILL);
         glPolygonMode(GL_BACK, GL_FILL);
      }
      // Draw heat map
      else{
         glDisable(GL_TEXTURE_2D);
         // Draw rectangle for background image
         glPolygonMode(GL_FRONT, GL_FILL);
         glPolygonMode(GL_BACK, GL_FILL);
         glEnable(GL_LINE_SMOOTH);
         glEnable(GL_BLEND);
         glColor3f(0.0f, 0.0f, 0.0f);
         glBegin(GL_POLYGON);
            glVertex3f(0.0, 0.0, 0.0);
            glVertex3f(sim->getW(), 0.0, 0.0);
            glVertex3f(sim->getW(), sim->getH(), 0.0);
            glVertex3f(0.0, sim->getH(), 0.0);
         glEnd();
         // Display generated paths
         if (displayGenPaths == 1) {
            sim->drawHeatMap();
         }
      }
   }

   // In interactive mode

   glDisable(GL_TEXTURE_2D);
   glEnable(GL_BLEND);
   glEnable(GL_LINE_SMOOTH);
   glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

   // Display the projected path
   if(sim->getCurrentAdv() >= (int)(sim->advCombList.size()/2)){
      if (displayProjectedPath == 1) {
         sim->drawForecastPath();
      }
   }

   if (displayAvgPath == 1){
      sim->drawAvgPath();
      sim->drawStdDevPath();
   }

   // Display all data points
   if (displayDataPoints == 1) {
      sim->drawDataPoints();
   }

   // Display all previous paths
   if (displayDataPaths == 1) {
      sim->drawDataPaths();
   }

   if(showCurPath > 0){
      drawCurPath();
   }

   sim->drawChips();
   //sim->drawTargetArea();
   sim->drawSectors(this);
   sim->drawSlider();
   drawNext();

   glDisable(GL_BLEND);
   glDisable(GL_LINE_SMOOTH);

   sim->drawChipText(this);
}

void mapWin::drawNext(){
  QFont f;


  // Button
  if(buttonPressed == 0){
     glColor4f(0.5, 0.5, 0.5, 0.9);
  }
  else{
     glColor4f(0.25, 0.25, 0.25, 0.9);
  }
  glBegin(GL_POLYGON);
     glVertex3f(butMin.x, butMin.y, 0);
     glVertex3f(butMax.x, butMin.y, 0);
     glVertex3f(butMax.x, butMax.y, 0);
     glVertex3f(butMin.x, butMax.y, 0);
  glEnd();

  // Border
  if(buttonPressed == 0){
     glColor4f(0.0, 0.0, 0.0, 0.9);
  }
  else{
     glColor4f(0.75, 0.75, 0.75, 0.9);
  }
  glLineWidth(3);
  glBegin(GL_LINE_LOOP);
     glVertex3f(butMinBord.x, butMinBord.y, 0);
     glVertex3f(butMaxBord.x, butMinBord.y, 0);
     glVertex3f(butMaxBord.x, butMaxBord.y, 0);
     glVertex3f(butMinBord.x, butMaxBord.y, 0);
  glEnd();

  // Text
  glDisable(GL_BLEND);
  glDisable(GL_LINE_SMOOTH);
  f = QFont();
  f.setBold(true);
  f.setPointSize(15);
  if(buttonPressed == 0){
     glColor4f(0.0, 0.0, 0.0, 1.0);
  }
  else{
     glColor4f(0.75, 0.75, 0.75, 1.0);
  }
  renderText(butTextPos.x, butTextPos.y, 0, "Next", f, 2000);
  //glEnable(GL_BLEND); 
  //glEnable(GL_LINE_SMOOTH);
}

void mapWin::update(){
   unsigned int curAdv = sim->getCurrentAdv();

   int r;
   double chosen = 0.0;

   if(sim->getCurrentAdv() < (int)(sim->advCombList.size()/2)){
         if(step == 0){
           for(int i = 0; i < 100; i++){
             totalPaths++;

             // Determine whether to do a historical path or points in the cone 
             srand ( counter );
             counter = (counter*21)%UINT_MAX;

             r = rand() % 10000000+1;
             chosen = (double)r/10000000.0;

             //if(chosen <= 0.32){
               testPath = new path(sim->adv->getLat(), sim->adv->getLon(), sim->adv->getDeg(), sim->adv->getSpeed(), curAdv);
               sim->adv->pathList.push_back(testPath);
               /*if(interactive == -1){
                  if(averageOnAll > 0){
                     sim->findAvgLine();
                  }
                  else{
                     sim->findAvgLineNew();
                  }
               }*/
             //}
             //else{
               //if(sim->adv->eyePointList.size() < 5){
               //  hurEyePoints * hep;
               //  hep = new hurEyePoints(sim->adv->pre);
               //  sim->adv->eyePointList.push_back(hep);
               //}
             //}
           }
           //curAdv++;
         }
         if(step == 24){
           for(unsigned int i = 0; i < sim->adv->pathList.size(); i++){
             delete(sim->adv->pathList[i]);
           }
           sim->adv->pathList.clear();
           step = -1;
         }
         step++;
   }

   sim->adv->findInConePercent();
   updateGL();
}

void mapWin::mouseMoveEvent(QMouseEvent * e){
   Vector2d dif, pos;
   e->accept();

   dif.set(e->x()-initPress.x, e->y()-initPress.y);
   pos.set(e->x(), e->y());
   sim->moveChip(dif);
   sim->moveSlider(pos);

   initPress.set(e->x(), e->y());

   updateGL();
}

void mapWin::mousePressEvent(QMouseEvent * e){
   Vector2d p;

   e->accept();

   p.set(e->x(), e->y());
   sim->selectChip(p);
   sim->checkSliderPressed(p);

   initPress.set(e->x(), e->y());

   checkButtonPress(e->x(), e->y());

   updateGL();
}

void mapWin::mouseReleaseEvent(QMouseEvent * e){
   e->accept();

   sim->releaseChip();
   //sim->printSectors();

   checkButtonRelease(e->x(), e->y());

   updateGL();
}

void mapWin::keyPressEvent(QKeyEvent * k){
   k->accept();
   std::vector<advisory*>::iterator advIt;

   cout << flush;

   if ( k->text().toStdString()== "1") {
      displayDataPoints = displayDataPoints * -1;
      updateGL();
   }

   if (k->text().toStdString() == "2") {
      displayDataPaths = displayDataPaths * -1;
      updateGL();
   }

   if (k->text().toStdString() == "3") {
      displayProjectedPath = displayProjectedPath * -1;
      updateGL();
   }

   if (k->text().toStdString() == "4") {
      displayGenPaths = displayGenPaths * -1;
      updateGL();
   }
   if (k->text().toStdString() == "5") {
      displayAvgPath = displayAvgPath * -1;
      updateGL();
   }
   if (k->text().toStdString() == "8"){
      showCurPath = showCurPath * -1;
      updateGL();
   }

   if (k->text().toStdString() == "i"){
      for(advIt = sim->advList.begin(); advIt != sim->advList.end(); advIt++){
               (*advIt)->pathList.clear();
      }
      if(speedRatio > 0){
         speedRatio -= 0.01;
         cout << "New Speed Ratio: " << speedRatio << endl;
      }
   }
   if (k->text().toStdString() == "I"){
      for(advIt = sim->advList.begin(); advIt != sim->advList.end(); advIt++){
               (*advIt)->pathList.clear();
      }
      if(speedRatio < 1.0){
         speedRatio += 0.01;
         cout << "New Speed Ratio: " << speedRatio << endl;
      }
   }
   if (k->text().toStdString() == "o"){
      for(advIt = sim->advList.begin(); advIt != sim->advList.end(); advIt++){
               (*advIt)->pathList.clear();
      }
      if(bearRatio > 0.0){
         bearRatio -= 0.01;
         cout << "New Bearing Ratio: " << bearRatio << endl;
      }
   }
   if (k->text().toStdString() == "O"){
      for(advIt = sim->advList.begin(); advIt != sim->advList.end(); advIt++){
               (*advIt)->pathList.clear();
      }
      if(bearRatio < 1.0){
         bearRatio += 0.01;
         cout << "New Bearing Ratio: " << bearRatio << endl;
      }
   }

   if (k->text().toStdString() == "h" || k->text().toStdString() == "H") {
      heatMap = heatMap * -1;
      updateGL();
   }

   if (k->text().toStdString() == "[" && sim->getPathOpacity() > 0.00){
      for(advIt = sim->advList.begin(); advIt != sim->advList.end(); advIt++){
         (*advIt)->pathList.clear();
      }
      sim->setPathOpacity(sim->getPathOpacity() - 0.01);
      cout << "New opacity: " << sim->getPathOpacity() << "\n";
   }
   if (k->text().toStdString() == "]"){
      for(advIt = sim->advList.begin(); advIt != sim->advList.end(); advIt++){
               (*advIt)->pathList.clear();
      }
      sim->setPathOpacity(sim->getPathOpacity() + 0.01);
      cout << "New opacity: " << sim->getPathOpacity() << "\n";
   }

   if (k->text().toStdString() == "," && sim->getAlpha()  >= 0.1){
      for(advIt = sim->advList.begin(); advIt != sim->advList.end(); advIt++){
               (*advIt)->pathList.clear();
      }
      sim->setAlpha(sim->getAlpha() - 0.1);
      cout << "New alpha: " << sim->getAlpha() << "\n";
   }
   if (k->text().toStdString() == "." && sim->getAlpha() < 1.0){
      for(advIt = sim->advList.begin(); advIt != sim->advList.end(); advIt++){
               (*advIt)->pathList.clear();
      }
      sim->setAlpha(sim->getAlpha() + 0.1);
      cout << "New alpha: " << sim->getAlpha() << "\n";
   }

   if (k->text().toStdString() == "-"){
      for(advIt = sim->advList.begin(); advIt != sim->advList.end(); advIt++){
               (*advIt)->pathList.clear();
      }

      userSig += 1.0;
      cout << "New userSig: " << userSig << "\n";
   }
   if (k->text().toStdString() == "="){
      for(advIt = sim->advList.begin(); advIt != sim->advList.end(); advIt++){
               (*advIt)->pathList.clear();
      }
      if(userSig > 2.0){
         userSig -= 1.0;
      }
      cout << "New userSig: " << userSig << "\n";
   }

   if (k->text().toStdString() == "s" || k->text().toStdString() == "S"){
      stall = stall * -1;
   }
   if (k->text().toStdString() == "c" || k->text().toStdString() == "C"){
      sim->incCurAdv(1);
      cout << "Current advisory: " << sim->getCurrentAdv() << endl;
   }
   if (k->text().toStdString() == "v" || k->text().toStdString() == "V"){
      for(advIt = sim->advList.begin(); advIt != sim->advList.end(); advIt++){
         (*advIt)->pathList.clear();
      }
      preOnly = preOnly * -1;
      if(preOnly > 0){
         cout << "Only using predicted data." << endl;
      }
      else{
         cout << "Using predicted and historical data." << endl;
      }
   }
   if (k->text().toStdString() == "b" || k->text().toStdString() == "B"){
      colorClosest = colorClosest * -1;
   }
   if (k->text().toStdString() == "x" || k->text().toStdString() == "X"){
      averageOnAll = averageOnAll * -1;
      if(averageOnAll > 0){
         cout << "Taking average of all paths." << endl;
      }
      else{
         cout << "Taking average of closest 68% paths." << endl;
      }
   }

   else if ((k->text().toStdString() == "q") || (k->text().toStdString() == "Q") || (k->key() == Qt::Key_Escape)) {
      exit(0);
   }
   updateGL();
}

void mapWin::imageBind(GLubyte * toBind, int texNum, int height, int width) {
   cout << "Binding image...." << endl;
   glBindTexture(GL_TEXTURE_2D, textures[texNum]);
   glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR );
   glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR );
   glTexImage2D(GL_TEXTURE_2D, 0, 3, height, width, 0, GL_RGB,
         GL_UNSIGNED_BYTE, toBind);
   glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_DECAL);
}

void mapWin::imageOpen(){
   cout << "Opening image...." << endl;
   FILE * in;
   in = fopen("map_bw.ppm", "rb");
   // Check file open
   if (in == NULL) {
      printf("Unable to open file map_bw.ppm\n");
      exit(1);
   }
   imageMap = new P6Image(in);
   imageBind(imageMap->getImageData(), 0, imageMap->getHeight(), imageMap->getWidth());

   updateGL();
}

// Draw the current path being used
void mapWin::drawCurPath(){
   std::vector<posPoint*>::iterator _p1;
   std::vector<posPoint*>::iterator _p2;
   Vector2d d1, d2;

   glColor4f(0.75, 0.0, 0.75, 1.0);
   glLineWidth(1.0);

   if(curPath != NULL && showCurPath > 0){
      _p2 = curPath->positionList.begin()+1;
      for(_p1 = curPath->positionList.begin(); _p2 != curPath->positionList.end(); _p1++){
         d1 = translateToScreen((*_p1)->getLon(), (*_p1)->getLat());
         d2 = translateToScreen((*_p2)->getLon(), (*_p2)->getLat());
         glBegin(GL_LINES);
           glVertex3f(d1.x, d1.y, 0.0);
           glVertex3f(d2.x, d2.y, 0.0);
         glEnd();
         _p2++;
      }
   }
}

void mapWin::checkButtonPress(double x, double y){
   if(x < butMaxBord.x && x > butMinBord.x){
      if(y < butMaxBord.y && y > butMinBord.y){
         buttonPressed = 1;
         return;
      }
   }
   buttonPressed = 0;
   return;
}

void mapWin::checkButtonRelease(double x, double y){
   if(x < butMaxBord.x && x > butMinBord.x){
      if(y < butMaxBord.y && y > butMinBord.y){
         if(sim->chipsPlaced() == 1){
            //printResults();
            sim->incCurAdv(1);
            completed++;
         }
      }
   }
   cout << "completed: " << completed << "     advList: " << sim->advList.size() << endl;
   if(completed == (int)(sim->advList.size())){
     printResults();
   }
   buttonPressed = 0;
}


// File format is as follows:
/* #EXP_START
    Hurricane Number
     0 if paths | 1 if error cone
     # of advisory (which hurricane)
     sector , value
     ...
   #EXP_STOP
*/
void mapWin::printResults(){
   fstream file;
   std::vector<advisory*>::iterator _a;
   unsigned int i, j;
   unsigned int numSectors = 8;

   file.open("results.txt", fstream::out | fstream::ate | fstream::app);

   file << "#EXP_START" << endl;

   sim->setCurAdv(0);
   for(i = 0; i < sim->advList.size(); i++){
      file << "  " << i << endl;
      if(sim->getCurrentAdv() < (int)(sim->advList.size()/2)){
         file << "    0  // PATHS" << endl;
      }
      else{
         file << "    1 // ERROR CONE" << endl;
      }

      file << "    " << sim->getCurrentAdv() << endl;

      for(j = 0; j < numSectors; j++){
         file << "    " << j << " , " << sim->adv->sectorValue(j) << endl;
      }
      sim->incCurAdv(1);
   }

   file << "#EXP_STOP" << endl;
   file << endl;

   file.close();
   exit(0);
}
