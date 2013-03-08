/*
 * pathSegment.cpp 
 *
 * Creadted on: Apr 5, 2011
 * Jonathan Cox
 * Clemosn University
 */

#include <fstream>
#include <iostream>
#include <vector>
#include "Vector.h"
#include "geoFunct.h"
#include "pathSegment.h"

int madeIt = 0;

extern int debugPrint;

extern double errorRadR[];

pathSegment::pathSegment(Vector4d * p0, Vector4d * p1, double hour, std::vector<Vector4d*> * e1, std::vector<Vector4d*> * e2, double initBear, double initSpeed){

   double dist;
   double b1, b2, b3;
   double d1, d2, d3;
   int seg;
   Vector2d pos0, pos1;

   Vector2d v1, v2;
   double errDist = 0.0;
   double tempDist = 0.0;
   double timeSeg = 1.0;
   double curSpeed = 0.0;

   //double h[3] = [9, 12, 24];

   bDif = p0->z;
   sDif = p0->w;

   dist = haversine(p0->x, p1->x, p0->y, p1->y);

   pmaxB = pminB = bDif;
   pmaxS = pminS = sDif;
   time = hour*60;
   if(hour == 0){
      cout << "p0: " << *p0 << "  p1: " << *p1 << "  bDif: " << bDif << " sDif: " << sDif << "  time: " << time << endl;
   }

   curSpeed = haversine(p0->x, p1->x, p0->y, p1->y)/3.0;

   samples = 101;

   seg = (int)(hour/3);

   switch((int)hour){
     case 0:
       errDist = errorRadR[1];
       tempDist = dist*3;
       timeSeg = 9;
       break;
     case 9:
       errDist = errorRadR[2];
       tempDist = dist*4;
       timeSeg = 12;
       break;
     case 21:
       errDist = errorRadR[3];
       tempDist = dist*4;
       timeSeg = 12;
       break;
     case 33:
       errDist = errorRadR[4];
       tempDist = dist*4;
       timeSeg = 12;
       break;
     case 45:
       errDist = errorRadR[5];
       tempDist = dist*8;
       timeSeg = 24;
       break;
   }

   cout << "Hour: " << hour << "     ErrorDist: " << errDist << endl << flush;


   //cout << "hour: " << hour << "   Seg: " << seg << endl;
   //cout << "Size of e1: " << e1->size() << " seg: " << seg << endl;
   //cout << "Size of e2: " << e2->size() << " seg: " << seg << endl;
   pos0.set(p0->x, p0->y);
   pos1.set(p1->x, p1->y);
   b1 = findBearing_2(&pos0, &pos1);
   v1.set(e1->at(seg)->x, e1->at(seg)->y);
   v2.set(e1->at(seg+1)->x, e1->at(seg+1)->y);
   b2 = findBearing_2(&v1, &v2);
   v1.set(e2->at(seg)->x, e2->at(seg)->y);
   v2.set(e2->at(seg+1)->x, e2->at(seg+1)->y);
   b3 = findBearing_2(&v1, &v2);

   d1 = dist/3.0;
   d2 = haversine(e1->at(seg)->x, e1->at(seg+1)->x, e1->at(seg)->y, e1->at(seg+1)->y)/3.0;
   d3 = haversine(e2->at(seg)->x, e2->at(seg+1)->x, e2->at(seg)->y, e2->at(seg+1)->y)/3.0;
    
   //  New method - find dif between error cone bearings from previous segment and current segment.
   //  Treat each side differently.
   double e1Pre, e2Pre;
   double e1PreSpeed, e2PreSpeed;
   double predPre;

   //cout << "hour: " << hour << "   seg: " << seg << endl;

   if(seg == 0){
      e1Pre = e2Pre = initBear;
      e1PreSpeed = e2PreSpeed = initSpeed;
   }
   else{
      v1.set(e1->at(seg)->x, e1->at(seg)->y);
      v2.set(e1->at(seg-1)->x, e1->at(seg-1)->y);
      e1Pre = fmod(findBearing_2(&v2, &v1) + 0.0, 360.0);
      //cout << "   e1 v1: " << v1 << "   v2: " << v2 << endl;
      v1.set(e2->at(seg)->x, e2->at(seg)->y);
      v2.set(e2->at(seg-1)->x, e2->at(seg-1)->y);
      e2Pre = fmod(findBearing_2(&v2, &v1) + 0.0, 360.0);
      //cout << "   e2 v1: " << v1 << "   v2: " << v2 << endl;

      e1PreSpeed = haversine(e1->at(seg)->x, e1->at(seg-1)->x, e1->at(seg)->y, e1->at(seg-1)->y)/3.0;
      e2PreSpeed = haversine(e2->at(seg)->x, e2->at(seg-1)->x, e2->at(seg)->y, e2->at(seg-1)->y)/3.0;
   }
   //cout << "v1: " << v1 << "   v2: " << v2 << endl;
   //cout << "b2: " << b2 << " e1Pre: " << e1Pre << endl;
   //cout << "b3: " << b3 << " e2Pre: " << e2Pre << endl;
   //cout << "p0->z: " << p0->z << "    b1: " << b1 << "   e1Pre: " << e1Pre << endl;

   phB1 = findDif(b2, e1Pre);
   phB2 = findDif(b3, e2Pre);

   phS1 = (d2-e1PreSpeed);
   phS2 = (d3-e2PreSpeed);

   if(errDist != 0){
     phS1 = ((tempDist-errDist)/tempDist)*sDif;
     phS1 = ((tempDist-errDist) - (curSpeed*timeSeg))/timeSeg;
     cout << "tempDist: " << tempDist << "    errDist: " << errDist << flush;
     cout << "     curSpeed: " << curSpeed << "    timeSeg: " << timeSeg << endl << flush;
     cout << "     num: " << (tempDist-errDist) - (curSpeed*timeSeg) << endl << flush;
     cout << "PHS1: " << phS1 << endl << flush;
     //phS1 = ((errDist-tempDist)/errDist)*sDif;
     phS2 = ((tempDist+errDist)/tempDist)*sDif;
     phS2 = ((tempDist+errDist) - (curSpeed*timeSeg))/timeSeg;
     //phS2 = ((tempDist+errDist)/errDist)*sDif;
   }
   else{
     phS1 = 0;
     phS2 = 0;
   }

   cout << "Hour: " << hour << endl << flush;
   cout << "   dist: " << tempDist << "      errDist: " << errDist << endl << flush;
   cout << "   phS1 : " << phS1 << "     phS2: " << phS2 << endl << flush;


   bRange = 1.5;
   sRange = 1.0;

   if(phB1 < phB2){
      pminB = bDif - bRange*abs(bDif-phB1);
      pmaxB = bDif + bRange*abs(phB2-bDif);
   }
   else{
      pminB = bDif - bRange*abs(bDif-phB2);
      pmaxB = bDif + bRange*abs(phB1-bDif);
   }
   if(phS1 < phS2){
      pminS = sDif - sRange*abs(phS1);
      pmaxS = sDif + sRange*abs(phS2);
      //pminS = phS1;
      //pmaxS = phS2;
   }
   else{
      pminS = sDif - sRange*abs(phS2);
      pmaxS = sDif + sRange*abs(phS1);
      //pminS = phS2;
      //pmaxS = phS1;
   }

   //sDif = pmaxS;
   //pminS = pmaxS;

   //sDif = pminS;
   //pmaxS = pminS;
   cout << "     sDif: " << sDif << "     pminS: " << pminS << "     pmaxS: " << pmaxS << endl << flush;

   if(hour == 0){  
     cout << "distance with time change: " << (pminS+curSpeed)*timeSeg << endl << flush;
     cout << "CurSpeed: " << curSpeed << "    initSpeed: " << initSpeed << endl << flush;
   }

   //if(madeIt == 0 && time/60 == 24){
      //cout << "pminS: " << pminS << "     pmaxS: " << pmaxS << endl;
      //cout << "    d2: " << d2 << " e1PreSpeed: " << e1PreSpeed << endl;
      //cout << "    d3: " << d3 << " e2PreSpeed: " << e2PreSpeed << endl;
      //cout << "    phS1 : " << phS1 << "    phS2: " << phS2 << endl;
      //cout << "    e1Pre: " << e1Pre << "     e2Pre: " << e2Pre << endl;
      //cout << "    p0: " << *p0 << endl;
      //cout << "  sDif: " << sDif << endl;
   //}

   //cout << "pMinB: " << pminB << "     pMaxB: " << pmaxB << endl;

   genPreBearDif();
   computePreAreaB();
   genPreSpeedDif();
   computePreAreaS();
}

void pathSegment::genPreBearDif(){
   double kSum;
   int i;
   double tWB1, tWB2;
   double sampleVal;

   //tWB = pmaxB-pminB;
   tWB2 = pmaxB-bDif;
   tWB1 = bDif-pminB;

   
   for(i = 0; i < samples; i++){
      if(i < samples/2 ){
         sampleVal = pminB+((double)i/(samples/2))*tWB1;
      }
      //else if(i == 5){
      //   sampleVal = bDif;
      //}
      else{
         sampleVal = bDif+(1.0-((double)(samples-i-1)/(samples/2)))*tWB2;
      }

      
      kSum = findPreKB(sampleVal-bDif, i, tWB1, tWB2);

      if(kSum == kSum){
         //if(time/60 == 24 && madeIt == 0){
            //cout << "   kSum: " << kSum << "     sample: " << sampleVal << endl;
         //}
         preBearF.push_back(kSum);
         preBearPos.push_back(sampleVal);
      }
      else{
         preBearF.push_back(0.0);
         preBearPos.push_back(sampleVal);
      }
   }
}

void pathSegment::genPreSpeedDif(){
   double kSum;
   int i;
   double tWS, tWS2, tWS1;
   double sampleVal;

   //tWS = pmaxS - pminS;
   tWS2 = pmaxS - sDif;
   tWS1 = sDif - pminS; 

   cout << "tWS2 -> " << tWS2 << endl << flush;
   cout << "tWS1 -> " << tWS1 << endl << flush;
   
   for(i = 0; i < samples; i++){
      if(i < samples/2){
         sampleVal = pminS + ((double)i/(samples/2))*tWS1;
      }
      //else if(i ==5){
      //   sampleVal = sDif;
      //}
      else{
         sampleVal = sDif +(1.0-((double)(samples-i-1)/(samples/2)))*tWS2;
      }
      //sampleVal = pminS+((double)i/(double)samples)*tWS;
      
      kSum = findPreKS(sampleVal-sDif, i, tWS1, tWS2);
      
      if(kSum == kSum){
         //cout << "i: " << i << "   kSum: " << kSum << "     sample: " << sampleVal << endl;
         preSpeedF.push_back(kSum);
         preSpeedPos.push_back(sampleVal);
      }
      else{
         preSpeedF.push_back(0.0);
         preSpeedPos.push_back(sampleVal);
      }
   }
}

void pathSegment::computePreAreaB(){
   std::vector<double>::iterator _d;
   double aV, bV;
   double n, sum;

   n = preBearF.size();

   _d = preBearF.begin();
   aV = pminB;
   sum = (*_d);
   //cout << "   area value: " << (*_d) << endl;
   
   for(_d = preBearF.begin()+1; _d != preBearF.end()-1; _d++){
      sum += 2*(*_d);
   }
   _d = preBearF.end()-1;
   sum += (*_d);
   bV = pmaxB;

   if(n != 0){
      pareaB = ((bV-aV)/(2*n))*sum;
      //cout <<  "   Area Under Curve: " << pareaB << endl;
   }
   else{
      pareaB = 0.0;
   }
  
   if(madeIt == 0 && time/60 == 24){
     for(_d = preBearF.begin(); _d != preBearF.end(); _d++){
        //cout << "Pre bear before crap a: " << (*_d) << endl;
     }
   }

   // Normalize area
   if(pareaB != 0.0){
      for(_d = preBearF.begin(); _d != preBearF.end(); _d++){
         (*_d) = (*_d)*(1.0/pareaB);
      }
      if(madeIt == 0 && time/60 == 24){
         for(_d = preBearF.begin(); _d != preBearF.end(); _d++){
            //cout << "Pre bear before crap b: " << (*_d) << endl;
            //cout << "pareaB: " << pareaB << endl;
            //cout << "min: " << pminB << "   max: " << pmaxB << endl;
         }
      }
      _d = preBearF.begin();
      aV = pminB;
      sum = (*_d);
   
      for(_d = preBearF.begin()+1; _d != preBearF.end()-1; _d++){
         sum += 2*(*_d);
      }
      _d = preBearF.end()-1;
      sum += (*_d);
      bV = pmaxB;

      if(n != 0){
         pareaB = ((bV-aV)/(2*n))*sum;
      }
   }

   if(madeIt == 0 && time/60 == 24){
      cout << "Area of predicted bearing curve: " << pareaB << endl;
      cout << " Area of preBearF: " << preBearF.size() << endl;
   }

   // Find area of individual trapazoids
   double k = 1;
   double fa, fb;
   double a;
   sum = 0.0;
   if(pareaB != 0.0){
      for(_d = preBearF.begin()+1; _d != preBearF.end(); _d++){
         aV = pminB + ((k-1)/preBearF.size())*(pmaxB-pminB);
         bV = pminB + (k/preBearF.size())*(pmaxB-pminB);
         fa = (*_d);
         fb = (*(_d-1));
         a = (bV-aV)*((fa+fb)/2.0);
         sum += a;
         preBearDistFunct.push_back(a);
         //cout << " K: " << k << "    a: " << a << endl;
         k++;
      }
   }

   //if(time/60 == 12){
   if(madeIt == 0 && time/60 == 24){
      madeIt = 1;   
      cout << "Making time varying PDF" << endl;
      BearingToFile();
   }
}

void pathSegment::computePreAreaS(){
   std::vector<double>::iterator _d;
   double aV, bV;
   double n, sum;

   n = preSpeedF.size();

   _d = preSpeedF.begin();
   aV = pminS;
   sum = (*_d);
   //cout << "   area value: " << (*_d) << endl;
   
   for(_d = preSpeedF.begin()+1; _d != preSpeedF.end()-1; _d++){
      sum += 2*(*_d);
   }
   _d = preSpeedF.end()-1;
   sum += (*_d);
   bV = pmaxS;
   //cout << "   area value: " << (*_d) << endl;

   if(n != 0){
      pareaS = ((bV-aV)/(2*n))*sum;
      //cout <<  "   Area Under Curve: " << areaB << endl;
   }
   else{
      pareaS = 0.0;
   }

   // Normalize area
   if(pareaS != 0.0){
      for(_d = preSpeedF.begin(); _d != preSpeedF.end(); _d++){
         (*_d) = (*_d)*(1.0/pareaS);
      }
      _d = preSpeedF.begin();
      aV = pminS;
      sum = (*_d);
   
      for(_d = preSpeedF.begin()+1; _d != preSpeedF.end()-1; _d++){
         sum += 2*(*_d);
      }
      _d = preSpeedF.end()-1;
      sum += (*_d);
      bV = pmaxS;

      if(n != 0){
         pareaS = ((bV-aV)/(2*n))*sum;
      }
   }

   // Find area of individual trapazoids
   double k = 1;
   double fa, fb;
   double a;
   sum = 0.0;
   if(pareaS != 0.0){
      for(_d = preSpeedF.begin()+1; _d != preSpeedF.end(); _d++){
         aV = pminS + ((k-1)/preSpeedF.size())*(pmaxS-pminS);
         bV = pminS + (k/preSpeedF.size())*(pmaxS-pminS);
         fa = (*_d);
         fb = (*(_d-1));
         a = (bV-aV)*((fa+fb)/2.0);
         sum += a;
         preSpeedDistFunct.push_back(a);
         k++;
      }
   }
   //cout << "   Speed Sum of parts: " << sum << endl;
}

double pathSegment::findPreKB(double x, int i, double tWB1, double tWB2){
   //static const double a = 1/(phB * sqrt(2*M_PI));
   double a = 0;
   double b = 0;
   double temp = 0;

   if(madeIt == 0 && time/60 == 24){
      //cout << "pminB: " << (pminB+3*bDif)/4 << "    bDif: " << bDif << "   pmaxB: " << 
      //        (pmaxB+3*bDif)/4 << endl;
   }

   if(i < samples/2){
      temp = tWB1/3.0;
      a = 1/(temp * sqrt(2*M_PI));
      b = exp( (-1*(x*x))/(2*temp*temp));
   }
   else{
      temp = tWB2/3.0;
      a = 1/(temp * sqrt(2*M_PI));
      b = exp( (-1*(x*x))/(2*temp*temp));
   }

   if(madeIt == 0 && time/60 == 24){
      //cout << "   temp: " << temp << "   x: " << x << endl;
      //cout << "      a: " << a << "   b: " << b << "   a*b: " << a*b << endl;
   }

   return a*b;
}

double pathSegment::findPreKS(double x, int i, double tWS1, double tWS2){
   double a = 0;
   double b = 0;
   double temp = 0;

   if(i < samples/2){
      temp = tWS1/3.0;
      a = 1/(temp * sqrt(2*M_PI));
      b = exp( (-1*(x*x))/(2*temp*temp));
   }
   else{
      temp = tWS2/3.0;
      a = 1/(temp * sqrt(2*M_PI));
      b = exp( (-1*(x*x))/(2*temp*temp));
   }
   
   return a*b;
}

// Parameter has to be between 0-1
double pathSegment::getPreProbBear(double p){
   std::vector<double>::iterator _d;
   double newP = p*pareaB;
   double aV, bV, n, testD;
   double sum, prevSum;
   double rangeL, rangeT;
   double areaDif, areaRatio;
   double mod;
   int k;

   
   aV = *(preBearF.begin());
   bV = *(preBearF.end()-1);
   k = 0;
   n = preBearF.size();
   sum = prevSum = 0.0;
   for(_d = preBearDistFunct.begin(); _d != preBearDistFunct.end()-1 && preBearDistFunct.size() > 0; _d++){
      prevSum = sum;
      if(n != 0){
         testD = (*_d);
      }
      else{
         testD = 0;
      }
      sum += testD;
      if(sum > p){
         break;
      }
      k++;
   } 

   // Interpolate alonge the area line
   areaDif = sum - prevSum;
   areaRatio = sum - newP;
   if(areaDif != 0 && areaDif == areaDif){
      areaRatio = areaRatio/areaDif;
   }
   else{
      areaRatio = 0.0;
   }

   // Use area interpolation to find a new value
   rangeL = preBearPos[k];
   rangeT = preBearPos[k+1];
   rangeT = rangeT - rangeL;
   mod = 1.0;
   if(rangeT < 0){
      mod = -1.0;
   }
   rangeL = (rangeL + rangeT*areaRatio);


   if(debugPrint == 1){
      cout << "  Bearing: " << rangeL << endl;
      cout << "     min: " << pminB << "    max: " << pmaxB << endl;
      cout << "     p: " << p << endl;
      debugPrint = 0;
   }

   return rangeL;
}

// Parameter has to be between 0-1
double pathSegment::getPreProbSpeed(double p){
   std::vector<double>::iterator _d;
   double newP = p*pareaS;
   double aV, bV, n, testD;
   double sum, prevSum;
   double rangeL, rangeT;
   double areaDif, areaRatio;
   double mod;
   int k;

 
   aV = *(preSpeedF.begin());
   bV = *(preSpeedF.end()-1);
   k = 0;
   n = preSpeedF.size();
   sum = prevSum = 0.0;
   for(_d = preSpeedDistFunct.begin(); _d != preSpeedDistFunct.end()-1 && preSpeedDistFunct.size() > 0; _d++){
      prevSum = sum;
      if(n != 0){
         testD = (*_d);
      }
      else{
         testD = 0;
      }
      sum += testD;
      //sum += a + k*((b-a)/n);
      if(sum > p){
         break;
      }
      k++;
   }

   //cout << "   Sum: " << sum << "    Prev Sum: " << prevSum << "   k: " << k << "   p: " << p << endl;
   // Interpolate alonge the area line
   areaDif = sum - prevSum;
   areaRatio = sum - newP;
   if(areaDif != 0 && areaDif == areaDif){
      areaRatio = areaRatio/areaDif;
   }
   else{
      areaRatio = 0.0;
   }

   // Use area interpolation to find a new value
   rangeL = preSpeedPos[k];
   rangeT = preSpeedPos[k+1];
   //cout << "       rangeL: " << rangeL << "    rangeT: " << rangeT << endl;
   rangeT = rangeT - rangeL;
   mod = 1.0;
   if(rangeT < 0){
      mod = -1.0;
   }
   rangeL = rangeL + rangeT*areaRatio;

   //cout << "       Returning rangeL: " << rangeL << "     min: " << pminS << "     max: " << pmaxS << endl;
   return rangeL;
}

void pathSegment::BearingToFile(){
   fstream file;
   unsigned int i;
   double pos, xMin, xMax;

   file.open("timeVaryPDF.txt", fstream::out);

   cout << "pminB: " << pminB << endl;
   cout << "bDif: " << bDif << endl;
   cout << "pmaxB: " << pmaxB << endl;
   for(i = 0; i < preBearF.size(); i++){
      //cout << "  i: " << i;
      //cout << "   preBearF: " << preBearF[i] << endl;
      //pos = pminB + ((double)i/(double)preBearF.size())*(pmaxB-pminB);
      /*if(i < 5){
         pos = pminB + ((double)i/(double)preBearF.size())*(bDif-pminB);
      }
      else if(i == 5){
         pos = bDif;
      }
      else{
         pos = bDif + ((double)i/(double)preBearF.size())*(pmaxB-bDif);
      }*/
      pos = preBearPos[i];
      file << dec << pos;
      file << " ";
      file << dec << preBearF[i];
      file << endl;
      //cout << "Writing to file: " << pos << " " << bearF[i] << endl;
   }

   file.close();

   file.open("plotBearing2", fstream::out);

   xMin = pminB;// - 0.5*(maxB-minB);
   xMax = pmaxB;// + 0.5*(maxB-minB);
   file << "set nokey" << endl;
   file << "set xrange[";
   file << dec << xMin;
   file << ":";
   file << dec << xMax;
   file << "]" << endl;

   file << "set style line 1 lt 1 lw 3" << endl;
   file << "plot \"timeVaryPDF.txt\" title \"title\" with lines linestyle 1";

   file.close();
}
