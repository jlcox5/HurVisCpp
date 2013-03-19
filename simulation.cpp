/*
 * simulation.cpp
 *
 *  Created on: Jan 16, 2011
 *      Author: Jonathan Cox
 *      Clemson University
 */

#ifdef __APPLE__
#  include <GLUT/glut.h>
#else
#  include <GL/glut.h>
#  include <GL/glu.h>
#endif

#include <algorithm>
#include <limits.h>

#include "simulation.h"

extern int colorClosest;
extern unsigned int counter;

simulation::simulation(int htd, double l1, double l2, double l3, double l4, double h, double a, double s, double sda,
            double tl1, double tl2, double ml1, double ml2, double dt, int im){

   int i;
   std::vector<Vector2d*> toPush;

   hurToDisplay = htd;
   lonL = l1;
   lonR = l2;
   latB = l3;
   latT = l4;

   hours = h;
   alpha = a;
   pathOpacity = 1.0;

   stdDevAlpha = sda;

   totalLat = tl1;
   totalLon = tl2;
   maxLat = ml1;
   maxLon = ml2;

   for(i=0; i < 361; i++){
      bearBin.push_back(toPush);
   }
   curHur = 0;
   initMethod = im;
}

// Build the statistical paths based on the normal distribution of bearing and speed
// differences
void simulation::buildStatPaths(){
   int i = 0;

   for(i=0; i < advList.size(); i++){
      //advList[i]->buildStatPaths();
   }
}

// Find the average line of the generated paths for each advisory
void simulation::findAvgLine(){
   //int i;

   //for(i=0; i < hurToDisplay; i++){
   if(getCurrentAdv() < advCombList.size()/2){
      adv->findAvgLine();
   }
   //}
}

// Find average over 68% of paths
void simulation::findAvgLineNew(){
   if(getCurrentAdv() < advCombList.size()/2){
      adv->findNewAvg();
   }
}

// Draw the generated paths for each advisory
void simulation::drawGenPaths(){
   //int i;

   /*for(i=0; i < hurToDisplay; i++){
      if(hurToDisplay == 6){
         glViewport( (i%3)*width, (int)(floor(i/3)+1)%2*height, width, height);
      }*/
   //adv->drawGenPaths();
   adv->drawGenPathsRainDrop();
   //adv->drawGenPathsEyePoint();
   adv->drawErrorConeRadius();
   //}
}

int simulation::drawGenPathsTrail(int step){
   step = adv->drawGenPathsTrail(step);
   return step;
}

void simulation::drawGenPathsClosest(){
   adv->drawGenPathsClosest();
}

// Draw the heat map for each advisory
void simulation::drawHeatMap(){
   //int i;

   /*for(i=0; i < hurToDisplay; i++){
      if(hurToDisplay == 6){
         glViewport( (i%3)*width, (int)(floor(i/3)+1)%2*height, width, height);
      }*/
   adv->drawHeatMap();
   //}
}

// Draw the average of the generated paths for each advisory
void simulation::drawAvgPath(){
   //int i;

   /*for(i=0; i < hurToDisplay; i++){
      if(hurToDisplay == 6){
         glViewport( (i%3)*width, (int)(floor(i/3)+1)%2*height, width, height);
      }*/
   if(getCurrentAdv() < advCombList.size()/2){
      adv->drawAvgPath();
      //adv->drawNewAvg();
   }
   //advList[curHur]->drawNewAvg();
   //}
}

// Draw the forecasted path and the error cone for each advisory
void simulation::drawForecastPath(){
   //int i;

   /*for(i=0; i < hurToDisplay; i++){
      if(hurToDisplay == 6){
         glViewport( (i%3)*width, (int)(floor(i/3)+1)%2*height, width, height);
      }*/
      adv->drawForecastPath();
      //adv->drawErrorSmooth();
   //}
}

// Draw the standard deviation for each advisory
void simulation::drawStdDevPath(){
   //int i;

   /*for(i=0; i < hurToDisplay; i++){
      if(hurToDisplay == 6){
         glViewport( (i%3)*width, (int)(floor(i/3)+1)%2*height, width, height);
      }*/
   if(getCurrentAdv() < advCombList.size()/2){
      adv->drawStdDevPath();
   }
      //advList[curHur]->drawStdDevPathRev();
   //}
}

// Draw the chips for each advisory
void simulation::drawChips(){
   adv->drawChipBox();
   adv->drawChips();
}

// Draw the chips for each advisory
void simulation::drawChipText(QGLWidget * g){
   adv->drawChipText(g);
}

void simulation::drawTargetArea(QGLWidget * g){
   adv->drawTargetArea(g);
}

void simulation::drawSectors(QGLWidget * g){
   adv->drawSectors(g);
}

void simulation::drawSlider(){
   adv->drawSlider();
}

// Draw all of the data points from historical data
void simulation::drawDataPoints(){
   std::vector<posPoint*>::iterator posListIt;
   Vector2d draw1, draw2;
   int i;

   glColor4f(1.0, 0.0, 0.0, 0.3);
   glLineWidth(0.75);
   glPointSize(3.0);

   for(i=0; i < hurToDisplay; i++){
      if(hurToDisplay == 6){
         glViewport( (i%3)*width, (int)(floor(i/3)+1)%2*height, width, height);
      }
      for (posListIt = positionList.begin(); posListIt != positionList.end(); posListIt++) {
         draw1.x = (*posListIt)->getLon();
         draw1.y = (*posListIt)->getLat();

         draw2 = locateDestination(draw1.x, draw1.y, 15.0, 15.0, (int)(*posListIt)->getDeg());

         draw1 = translateToScreen(draw1.x, draw1.y);
         draw2 = translateToScreen(draw2.x, draw2.y);

         glBegin(GL_POINTS);
            glVertex3f(draw1.x, draw1.y, 0.0);
         glEnd();
         glBegin(GL_LINES);
            glVertex3f(draw1.x, draw1.y, 0.0);
            glVertex3f(draw2.x, draw2.y, 0.0);
         glEnd();
      }
   }
}

// Draw all of the paths from historical data
void simulation::drawDataPaths(){
   std::vector<path*>::iterator pathIt;
   std::vector<Vector2d*>::iterator pos1;
   std::vector<Vector2d*>::iterator pos2;
   Vector2d draw1, draw2;
   float lineWidth;
   int i;

   // Draw all data tracks
   for(i=0; i < hurToDisplay; i++){
      if(hurToDisplay == 6){
         glViewport( (i%3)*width, (int)(floor(i/3)+1)%2*height, width, height);
      }
      for (pathIt = dataPathList.begin(); pathIt != dataPathList.end(); pathIt++) {
         pos2 = (*pathIt)->posList.begin() + 1;
         lineWidth = 1.0;
         glColor4f((*pathIt)->getInt().x, (*pathIt)->getInt().y,
               (*pathIt)->getInt().z, 0.25);
         for (pos1 = (*pathIt)->posList.begin(); pos2
               != (*pathIt)->posList.end(); pos1++) {
            glColor4f((*pathIt)->getInt().x, (*pathIt)->getInt().y,
                  (*pathIt)->getInt().z, 0.25);
            glLineWidth(lineWidth * 1.0);
            draw1.x = (*pos1)->x;
            draw1.y = (*pos1)->y;
            draw2.x = (*pos2)->x;
            draw2.y = (*pos2)->y;

            glBegin(GL_LINES);
            glVertex3f(draw1.x, draw1.y, 0.0);
            glVertex3f(draw2.x, draw2.y, 0.0);
            glEnd();
            pos2++;
         }
      }
   }
}

// Handling chips
void simulation::selectChip(Vector2d p){
   adv->selectChip(p);
}

void simulation::releaseChip(){
   adv->releaseChip();
}

void simulation::moveChip(Vector2d v){
   adv->moveChip(v);
}

int simulation::chipsPlaced(){
   return adv->chipsPlaced();
}

void simulation::checkSliderPressed(Vector2d p){
   adv->checkSliderPressed(p);
}

void simulation::printSectors(){
   adv->printSectors();
}

// Prints the contents of the bearing bin structure
void simulation::printBearBin(){
   std::vector<Vector2d*>::iterator _d;
   int i, z, total;
   int maxCount, minCount, zeroCount;
   minCount = 1000000;
   maxCount = -1;
   zeroCount = 0;
   total = 0;

   cout << "Printing information on bearing bins:\n";
   for(i = 0; i < (int)bearBin.size(); i++){
      cout << "  Bin " << i << ": " << bearBin[i].size() << " elements\n";
      z = 0;
      total += bearBin[i].size();

      // Check for minimum count
      if( (int)bearBin[i].size() < minCount ){
         minCount = bearBin[i].size();
      }

      // Check for maximum count
      if( (int)bearBin[i].size() > maxCount ){
         maxCount = bearBin[i].size();
      }
      // Check for no elements
      if( bearBin[i].size() == 0 ){
         zeroCount++;
      }

      // Loop through elements
      for(_d = bearBin[i].begin(); _d != bearBin[i].end(); _d++){
         if(z == 0){
            cout << "     ";
         }
         if(z == 10){
            cout << *(*_d) << "\n     ";
            z = 0;
         }
         else if(_d == bearBin[i].end()-1){
            cout << *(*_d) << "\n";
         }
         else{
            cout << *(*_d) << ", ";
         }
         z++;
      }
   }
   cout << "Total elements: " << total << "\n";
   cout << "   Max bin count: " << maxCount << "\n   Min bin count: " << minCount << "\n";
   cout << "   Bins with no elements: " << zeroCount << "\n";
}

void simulation::buildExp(){
   int i;

   //int index[] = {5, 3, 1, 2, 0, 4};
   int index[] = {0, 1, 2, 3, 4, 5};

   for(i=0; i < (advList.size())/2; i++){
      advErrorList.push_back(index[i]); 
      advPathList.push_back(index[i]);
   }

   for(i=0; i < advErrorList.size(); i++){
     if(initMethod == 0){
       advCombList.push_back(advPathList[i]);
       advCombList.push_back(advPathList.size()+advErrorList[i]);
     }
     else{
       advCombList.push_back(advPathList.size()+advErrorList[i]);
       advCombList.push_back(advPathList[i]);
     }
   }

   //cout << "Combined list: ";
   for(i=0; i < advCombList.size(); i++){
      cout << "  " << advCombList[i];
   }
   cout << endl;

   adv = advList[abs(advCombList[curHur])];
}
