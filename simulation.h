/*
 * simulation.h
 *
 *  Created on: Jan 16, 2011
 *      Author: jon
 */

#ifndef SIMULATION_H_
#define SIMULATION_H_

#include <QGLWidget>
#include "advisory.h"
#include "geoFunct.h"

class simulation{
   protected:
      // Window information
      int width, height;

      // Number of hurricanes to display
      int hurToDisplay;    // Number of hurricanes to display
      int curHur;          // Current active advisory
      int initMethod;      // Which method to show first

      // Map info
      double lonL, lonR;   // Longitude
      double latB, latT;   // Latitude

      // Map Data
      double totalLat, totalLon;   // Total degrees of latitude and longitude covered by the map
      double maxLat, maxLon;       // Max latitude and longitude for the map
      double pixPerKM;             // Number of pixels per kilometer


      // Simulation Values
      double hours;                // Hours per segment of each path
      double alpha;           // User defined variables for combining the velocity of the 
                                   // projected path and the data point found
      double pathOpacity;

      // Statistical Information Values
      double stdDevAlpha;          // Used for std deviation of error cone

   public:
      std::vector<advisory*> advList;       // List of advisories running

      //  Need to define this
      std::vector<posPoint*> positionList;
      std::vector<path*> dataPathList;
      std::vector<path*> dataPathAugList;

      std::vector<int> advErrorList;
      std::vector<int> advPathList;
      std::vector<int> advCombList;
      int expStart;

      // Variables used for the bearing bin idea
      // X = bearing     Y = speed
      std::vector< std::vector<Vector2d*> > bearBin;
      advisory * adv;

      simulation(int htd, double l1, double l2, double l3, double l4, double h, double a, double s, double sda,
            double tl1, double tl2, double ml1, double ml2, double dt, int im);

      // Get
      int getDispNum(){ return hurToDisplay; }
      double getW(){ return width; }
      double getH(){ return height; }
      double getAlpha(){ return alpha; }
      double getPixPerKM(){ return pixPerKM; }
      double getMaxLat(){ return maxLat; }
      double getMaxLon(){ return maxLon; }
      double getTotLat(){ return totalLat; }
      double getTotLon(){ return totalLon; }
      double getHours(){ return hours; }
      double getSDA(){ return stdDevAlpha; }
      double getPathOpacity(){ return pathOpacity; }

      // Set
      void setW(int w){ width = w; }
      void setH(int h){ height = h; }
      void setAlpha(double a){ alpha = a; }
      void setPixPerKM(double s){ pixPerKM = s; }
      void setMaxLat(double m){ maxLat = m; }
      void setMaxLon(double m){ maxLon = m; }
      void setTotLat(double m){ totalLat = m; }
      void setTotLon(double m){ totalLon = m; }
      void setHours(double h){ hours = h; }
      void setSDA(double s){ stdDevAlpha = s; }
      void setPathOpacity(double s){ pathOpacity = s; }
      void incCurAdv(int i){ curHur = (curHur+i)%12; adv = advList[abs(advCombList[curHur])]; getCurAdv(); }
      void setCurAdv(int i){ curHur = i%12; adv = advList[abs(advCombList[curHur])]; }
      int getCurAdv(){ return curHur; }

      // Statistical functions
      void findAvgLine();
      void findAvgLineNew();
      void buildStatPaths();

      // Draw Functions
      void drawGenPaths();
      void drawHeatMap();
      void drawAvgPath();
      void drawForecastPath();
      void drawStdDevPath();
      void drawDataPoints();
      void drawDataPaths();
      int drawGenPathsTrail(int);
      void drawGenPathsClosest();
      void drawChips();
      void drawChipText(QGLWidget *);
      void drawTargetArea(QGLWidget *);
      void drawSectors(QGLWidget *);
      void drawSlider();

      // Handling the chips
      void selectChip(Vector2d p);
      void releaseChip();
      void moveChip(Vector2d v);
      int chipsPlaced();
      void checkSliderPressed(Vector2d p);

      // Print functions
      void printBearBin();
      void printSectors();

      void buildExp();
      int getCurrentAdv(){ return advCombList[curHur]; }
};

#endif /* SIMULATION_H_ */
