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
      void buildErrorConeFull();
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
      std::vector<Vector4d*> projPath;
      std::vector<Vector2d*> projPathFull;
      std::vector<double> distWeightCone;

      // Error cone - Error radius based on 2010 data
      std::vector<Vector2d> stdDev1;
      std::vector<Vector2d> errorCone1;
      std::vector<Vector2d> stdDev2;
      std::vector<Vector2d> errorCone2;
      std::vector<Vector2d> top;
      std::vector<Vector4d*> ecSmooth1;
      std::vector<Vector4d*> ecSmooth2;

      // The velocities of each segment of the projected path
      std::vector<Vector2d*> segmentVelocity;

      // The average path of all the projected paths in pixels
      std::vector<Vector2d*> avgPath;
      std::vector<Vector2d*> avgPath2;
      std::vector<Vector2d*> avgPath3;

      std::vector<Vector2d*> avgSegVel;
      std::vector<double> avgSegDist;
      std::vector<double> avgSegBearing;

      // The average path of all the projected paths in lat/lon coordinates
      std::vector<Vector2d*>  avgPathLatLon;
      std::vector<Vector2d*>  avgPathLatLon2;
      std::vector<double> avgBearing;
      std::vector<double> projBearing;
      std::vector<double> speedAvg;
      std::vector<Vector2d> velAvg;

      std::vector<double> distRatio;
      std::vector<Vector2d> predVel;

      // The distance of the standard deviation of the projected path from the
      // average path
      std::vector<double>  stdDevDist;
      std::vector<double>  stdDevDistRev;

      // Constructor
      advisory(std::vector<Vector2d*> projPathPos, double curLat, double curLon, double curDeg, double curSpeed, string predFile);

      // Statistical functions
      void findAvgLine();
      void stdDeviationLine();

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
      void drawGenPathsClosest();
      void drawHeatMap();
      void drawAvgPath();
      void drawForecastPath();
      void drawStdDevPath();
      void drawStdDevPathRev();
      void drawErrorCone();
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

      // New average and cone methods
      void findNewAvg();    // Sorts paths, picks 68% and draws average as well as error cone
      void drawNewAvg();
      void newStdDeviationLine();

      // Functions for finding median and std deviation lines based on median
      void sortPath(int seg);

      // Build paths using predicted path normal distribution

      // Interpolations for error cone
      void buildECSmooth();
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

      // True cone data and build function
      std::vector<Vector2d*> trueCone1;
      std::vector<Vector2d*> trueCone2;

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
