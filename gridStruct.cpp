/* Jonathan Cox
   Clemson University
   Grid Structure
*/

#include <fstream>
#include <iostream>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "gridStruct.h"


using namespace std;

bool printBS = false;

gridStruct::gridStruct(int dLat, int dLon, int dpBin, int miLa, int maLa, int miLo, int maLo, string filename): degLat(dLat), degLon(dLon), degPerBin(dpBin), minLat(miLa), maxLat(maLa), minLon(abs(miLo)), maxLon(abs(maLo)){
   int i, j, k;
   int binSize = (int)ceil(360/dpBin);
   std::vector<std::vector<bin*> *> * lat;
   std::vector<bin*> * lon;
   bin * bi;

   extraLines = 3;

   cout << "binSize: " << binSize << "\n";

   //for(i = 0; i < degLat; i++){
   /*for(i = 0; i < 1; i++){
      lat = new std::vector<std::vector<bin*>*>();
      grid.push_back(lat);
      cout << "Making lat: " << i  << "\n";
      for(j = 0; j < degLon; j++){
         cout << "    Making lon: " << j << "\n";
         lon = new std::vector<bin*>();
         lat->push_back(lon);
         for(k = 0; k < binSize; k++){
            cout << "Making bin: " << minLat+i << ", " << -1*maxLon+j << "\n";
            bi = new bin(minLat+i, -1*maxLon+j);
            lon->push_back(bi); 
         }
         cout << "    Lon size: " << lon->size() << "\n"; 
      }
      cout << "Lat size: " << lat->size() << "\n";
   }*/

   for(i = 0; i < degLat; i++){
      lat = new std::vector<std::vector<bin*>*>();
      grid.push_back(lat);
      for(j = 0; j < degLon; j++){
         lon = new std::vector<bin*>();
         lat->push_back(lon);
         for(k = 0; k < binSize; k++){
            bi = new bin(minLat+i, -1*(minLon+j));
            lon->push_back(bi); 
         }
      }
   }
   

   //cout << "lat size: " << grid.size() << "   lon size: " << grid[0]->size() << "   bin size: " << grid[0]->at(0)->size() << "\n";

   cout << "Loading file 1: " << filename << endl;
   cout << flush;
   loadFile(filename, 0);
   cout << "File loaded..." << endl << flush;
   //cout << "Loading file 2: " << filename2 << endl;
   //cout << flush;
   //loadFile(filename2, 1);
   

   // Print specific bins
   for(i = 0; i < degLat; i++){
   //for(i = 0; i < 1; i++){
      for(j = 0; j < degLon; j++){
         if( (minLat+i == 24 || minLat+i == 25) && 
             (-1*(minLon+j) == -84 || -1*(minLon+j) == -85)){
            //cout << "Grid Lat Lon: " << minLat+i << ", " << -1*(minLon+j) << "\n";
         }
         for(k = 0; k < binSize; k++){
            if( (minLat+i == 24 || minLat+i == 25) && 
                (-1*(minLon+j) == -84 || -1*(minLon+j) == -85)){

              //cout << "     bin: " << k << "\n";
              //printBS = true;
            }
            grid[i]->at(j)->at(k)->resolve();
         }
         printBS = false;
      }
   }
}

void gridStruct::loadFile(string filename, int t){
   fstream file;
   char peekC;
   gridPoint * toIns;
   int curLat, curLon, curBin;
   double input;
   double tDeg, tSpeed, tWeight, tsLat, tsLon, tfLat, tfLon, sTime, fTime;

   curLat = 0;
   curLon = 0;
   curBin = 0;

   file.open(filename.c_str(), fstream::in);

   while(!file.eof() && !file.bad()){
      file.get(peekC);

      if(peekC == 'A'){
         file >> dec >> input;
         curLat = degLat - abs((int)(maxLat - input))-1;
      }
      else if(peekC == 'O'){
         file >> dec >> input;
         curLon = degLon - (abs((int)maxLon) - abs((int)input))-1;
      }
      else if(peekC == 'D'){
         file >> dec >> input;
         curBin = (int)floor(input / degPerBin);
      }
      else if(peekC == 'P'){
         file >> dec >> tDeg;
         file >> dec >> tSpeed;
         file >> dec >> tWeight;
         file >> dec >> tsLat;
         file >> dec >> tsLon;
         file >> dec >> tfLat;
         file >> dec >> tfLon;
         if( t == 1 ){
            file >> dec >> sTime;
            file >> dec >> fTime;
         }
         toIns = new gridPoint(tDeg, tSpeed, tWeight, tsLat, tsLon, tfLat, tfLon);
         if(abs(tSpeed > 1000)){
            cout << "Bad speed value read\n";
         }
         grid[curLat]->at(curLon)->at(curBin)->b.push_back(toIns);
      }
   }

   file.close();
   
}

void gridStruct::addPoints(gridPoint * g){
   gridPoint * toAdd;
   Vector2d tDest;
   Vector2d * p1, * p2;
   double bDif, bNew;
   int insLat, insLon, insBin;
   int i;

   p1 = new Vector2d(g->getSLon(), g->getSLat());
   p2 = new Vector2d(g->getFLon(), g->getFLat());

   bDif = findBearing_2(p1, p2);
   bNew = bDif - 90;
   if(bNew < 0){
      bNew += 360;
   }

   //cout << "Old point: ";
   //g->printData();

   //cout << "bNew: " << bNew << endl;
   // Add to left of p
   for(i = 0; i < extraLines; i++){
      tDest = locateDestination_2(g->getSLon(), g->getSLat(), 60*(i+1), bNew);

      // Find insert point into grid structure
      insLat = (int)floor(tDest.y); 
      insLat = (int)(degLat - abs((int)(maxLat - insLat))-1);
      insLon = (int)floor(tDest.x);
      insLon = (int)(degLon - (abs((int)minLon) - abs((int)insLon))-1);
      insBin = (int)floor(bNew / degPerBin);

      // Create point to add
      if(insLat > 0 && insLat < degLat){
         if(insLon > 0 && insLon < degLon){
            toAdd = new gridPoint(g->getBD(), g->getSD(), 0.5, tDest.y, tDest.x, tDest.y, tDest.x);
            //cout << "   Computed point top: ";
            //toAdd->printData();
            //cout << "        store: " << insLat << ", " << insLon << ", " << insBin << endl;
            grid[insLat]->at(insLon)->at(insBin)->b.push_back(toAdd);
         }
      }
   }

   bNew = fmod(bDif + 90, 360);
   //cout << "bNew: " << bNew << endl;
   // Add to left of p
   for(i = 0; i < extraLines; i++){
      tDest = locateDestination_2(g->getSLon(), g->getSLat(), 60*(i+1), bNew);

      // Find insert point into grid structure
      insLat = (int)floor(tDest.y); 
      insLat = (int)(degLat - abs((int)(maxLat - insLat))-1);
      insLon = (int)floor(tDest.x);
      insLon = (int)(degLon - (abs((int)minLon) - abs((int)insLon))-1);
      insBin = (int)floor(bNew / degPerBin);

      // Create point to add
      if(insLat > 0 && insLat < degLat){
         if(insLon > 0 && insLon < degLon){
            toAdd = new gridPoint(g->getBD(), g->getSD(), 0.5, tDest.y, tDest.x, tDest.y, tDest.x);
            //cout << "   Computed point bottom: ";
            //toAdd->printData();
            //cout << "        store: " << insLat << ", " << insLon << ", " << insBin << endl;
            grid[insLat]->at(insLon)->at(insBin)->b.push_back(toAdd);
         }
      }
   }
}

void gridStruct::printGrid(){
   int i, j;
   unsigned int k, l;

   std::vector<std::vector<bin*> *> * lat;
   std::vector<bin*> * lon;
   bin * bi;

   //for(i = 0; i < grid.size(); i++){
      i = 7;
      cout << "Lattitude: " << i+minLat << "    with i: " << i << "\n";
      cout << "    size: " << grid[i]->size() << "\n";
      lat = grid[i];
      //for(j = 0; j < grid[i]->size(); j++){
         j = 10;
         cout << "     Longitude: " << -1*(minLon + j) << "   with j: " << j << "\n";
         lon = lat->at(j);
         for(k = 0; k < lon->size(); k++){
            bi = lon->at(k);
            cout << "          Bin size: " << k*degPerBin << " - " << k*degPerBin + degPerBin-1 << "\n";
            for(l = 0; l < bi->b.size(); l++){
               cout << "                 ";
               bi->b[l]->printData();
            }
         }
      //}
   //}
}
