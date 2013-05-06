/*
 * advsiory.h
 *
 *  Created on: Aug 20, 2010
 *  Jonathan Cox
 *  Clemson University
 */

#ifndef ADVSIORY_H_
#define ADVSIORY_H_

#include <QGLWidget>

#include <iostream>
#include <vector>
#include "path.h"
#include "geoFunct.h"
#include "predictedPath.h"
#include "chip.h"
#include "sector.h"
#include "hurEyePoints.h"
#include "slider.h"

bool sortPath(path * p1, path * p2);

class advisory{
   private:
      // Position variables
      double lat;
      double lon;
      double deg;
      double speed;
      double projPathDist;
      void buildProjPath(std::vector<Vector2d*> projPathPos, double projPathDir);
      void buildProjPathFull();
      void buildErrorCone();
      int presLat, presLon;
      int numIn68;
      int onRight, onLeft;

      // Used for user interaction
      std::vector<chip*> chips;
      std::vector<Vector2d*> sectorEnds;
      std::vector<Vector2d*> targetArea;
      std::vector<double> thetaList;
      std::vector<double> chipThetaList;
      std::vector<double> chipDistList;
      std::vector<std::vector<int>*> sectorFull;
      std::vector<sector*> sectorList;
      Slider slider;
      double rad;

      double inConePercent;
      double usePredictedPercent;
      double baseConePercent;

      double percBubDrawn;

   public:
      // List of projected paths
      std::vector<path*> pathList;
      std::vector<path*> pathListRev;
      std::vector<hurEyePoints*> eyePointList;

      // The predicted path 
      predictedPath * pre;

      std::vector<path*> closestToPre;
      std::vector<path*> closestToPreStdDev;
      std::vector<Vector2d*> newAvg;

      // The projected path of a specific hurricane at a given time step
      std::vector<Vector4d*> projPath;     // (lat, lon, bearing change, speed change)
      std::vector<Vector2d*> projPathFull;
      std::vector<double> distWeightCone;

      // Error cone - Error radius based on 2010 data
      // ecSmooth contains the data used to generate the pdf functions for speed and bearing
      // change 4d vector gives (lat, lon, bearing, speed) ---- lat and lon might be
      std::vector<Vector4d*> ecSmooth1;
      std::vector<Vector4d*> ecSmooth2;

      // trueCone holds the data used to draw the representation of the NHC error cone
      // as displayed in the user study
      std::vector<Vector2d*> trueCone1;
      std::vector<Vector2d*> trueCone2;

      // Constructor
      advisory(std::vector<Vector2d*> projPathPos, double curLat, double curLon, double curDeg, double curSpeed, string predFile);

      // Get functions
      double getLat(){ return lat; }
      double getLon(){ return lon; }
      double getDeg(){ return deg; }
      double getSpeed(){ return speed; }
      double getProjPathDist(){ return projPathDist; }

      // Draw functions
      void drawGenPaths();
      void drawGenPathsRainDrop();
      void drawGenPathsEyePoint();
      int drawGenPathsTrail(int);
      void drawHeatMap();
      void drawForecastPath();
      void drawErrorSmooth();
      void drawErrorConeRadius();
      void drawChips();
      void drawChipText(QGLWidget * g);
      void drawChipBox();
      void drawSlider();

      // Current point on path information
      int getPresLat(){ return presLat; }
      int getPresLon(){ return presLon; }
      void setPresLat(int i){ presLat = i; }
      void setPresLon(int i){ presLon = i; }

      // Functions for finding median and std deviation lines based on median
      void sortPath(int seg);

      // Build paths using predicted path normal distribution

      // Interpolations for error cone
      void buildECSmooth(std::vector<Vector2d> &, std::vector<Vector2d> &);
      void interpECSmooth(Vector2d p0, Vector2d p1, double h, int side);

      // Handling the chips
      void selectChip(Vector2d p);
      void releaseChip();
      void moveChip(Vector2d v);
      int chipsPlaced();
      void checkSliderPressed(Vector2d p);
      void moveSlider(Vector2d v);

      // Build the sectors for user interaction
      void buildTargetArea();
      void buildSectors();
      void drawTargetArea(QGLWidget *);
      void drawSectors(QGLWidget *);
      void printSectors();

      // Determines lock positions of chips once inside a sector
      void detChipLockPos();

      // Returns the value at the specified sector
      int sectorValue(int i);

      // Build and draw the NHC Error Cone
      void buildTrueCone();
      void buildTrueConeSide(int);
      void drawTrueCone();

      void findInConePercent();
      void updatePredictedPercent();
      double getInConePercent(){ return inConePercent; }
      double getPredictedPercent(){ return usePredictedPercent; }
      double getBasePercent(){ return baseConePercent; }
      
};

#endif /* ADVSIORY_H_ */
