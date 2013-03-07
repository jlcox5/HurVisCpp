   /* Jonathan Cox
      Clemson University
      Bin Structure
   */

   #include <math.h>
   #include <limits.h>
   #include <fstream>
   #include <stdlib.h>
   #include "bin.h"
   #include "simulation.h"

   using namespace std;

   extern int counter;
   extern double omega;
   extern int preOnly;
   extern simulation * sim;
   extern double userSig;
   extern double errorRadius[];
   std::vector<double> * preSeg;
   int i;

   extern bool printBS;

   void bin::resolve(){
      std::vector<gridPoint*>::iterator _b;
      double meanB, meanS, tSum, uSum;
      double k = 1.0;

      if(b.size() > 0){
         _b = b.begin();
         maxB = minB = (*_b)->getBD();
         maxS = minS = (*_b)->getSD();

         samples = 11;

         meanB = 0.0;
         meanS = 0.0;
         if(printBS){
           cout << "b size: " << b.size() << endl << flush;
         }
         for(_b = b.begin(); _b != b.end(); _b++){

            if(printBS){
               cout << "     sDel: " << (*_b)->getSD() << "     bDel: " << (*_b)->getBD() << endl;
            }
            if((*_b)->getBD() > maxB){
               maxB = (*_b)->getBD();
            }
            if((*_b)->getBD() < minB){
               minB = (*_b)->getBD();
            }
            if((*_b)->getSD() > maxS){
               maxS = (*_b)->getSD();
            }
            if((*_b)->getSD() < minS){
               minS = (*_b)->getSD();
            }
            meanB += (*_b)->getBD();
            meanS += (*_b)->getSD();
         }
         meanB = meanB/(double)b.size();
         meanS = meanS/(double)b.size();

         if(printBS){
           cout << "meanB : " << meanB << "     meanS: " << meanS << endl << flush;
         }

         tSum = 0.0;
         uSum = 0.0;
         for(_b = b.begin()+1; _b != b.end(); _b++){
            tSum += pow((*_b)->getBD() - meanB, 2);
            uSum += pow((*_b)->getSD() - meanS, 2);
         }
         tSum = (1.0/((double)b.size()-1.0))*tSum;
         uSum = (1.0/((double)b.size()-1.0))*uSum;

         wB = maxB - minB;
         wS = maxS - minS;


         if(wB > 0){
            hB = tSum;
            hB = sqrt(hB);
            hB = k*hB;
         }
         else{
            hB = 0;
         }
         if(wS > 0){
            hS = uSum;
            hS = sqrt(hS);
            hS = k*hS;
         }
         else{
            hS = 0;
         }

         // Added per discussion with Dr. House
         if(printBS){
           cout << "minB: " << minB << "   maxB: " << maxB << endl << flush;
           cout << "minS: " << minS << "   maxS: " << maxS << endl << flush;
         }

         minB = minB - 3.0*(hB);
         maxB = maxB + 3.0*(hB);
         minS = minS - 3.0*(hS);
         maxS = maxS + 3.0*(hS);

         if(printBS){
           cout << "hB: " << hB << "     hS: " << hS << endl << flush;
           cout << "minB: " << minB << "   maxB: " << maxB << endl << flush;
           cout << "minS: " << minS << "   maxS: " << maxS << endl << flush;
         }

         genBearDif();
         computeAreaB();
         genSpeedDif();
         computeAreaS();
     }
     for(i = 0; i < 6; i++){
        preSeg = new std::vector<double>();
        preSegDist.push_back(preSeg);
     }
     for(i = 0; i < 6; i++){
        buildSegDist(i);
     }

     std::vector<double>::iterator _d;

     if(printBS){
       cout << "          Bearing:" << endl << "               Value:        ";
       for( _d = bearPos.begin(); _d != bearPos.end(); _d++){
         cout << " " << (*_d) << ", ";
       }
       cout << endl << "               Segment Area: ";
       for( _d = bearDistFunct.begin(); _d != bearDistFunct.end(); _d++){
         cout << " " << (*_d) << ", ";
       }
       cout << endl;
       cout << "          Speed:" << endl << "               Value:        ";
       for( _d = speedPos.begin(); _d != speedPos.end(); _d++){
         cout << " " << (*_d) << ", ";
       }
       cout << endl << "               Segment Area: ";
       for( _d = speedDistFunct.begin(); _d != speedDistFunct.end(); _d++){
         cout << " " << (*_d) << ", ";
       }
       cout << endl;
     }

}

double bin::findKB(double x){
   static const double a = 1/(hB * sqrt(2*M_PI));
   double b;

   if(hB != 0){
      b = exp( (-1*(x*x))/(2*hB*hB) );
   }
   else{
      b = 0;
   }
   return a*b;
}

double bin::findKS(double x){
   const double a = 1/(hS * sqrt(2*M_PI));
   double b;

   if(hS != 0){
      b = exp( (-1*(x*x))/(2*hS*hS) );
   }
   else{
      b = 0;
   }
   
   return a*b;
}

// Generates the sample values for the KDE
void bin::genBearDif(){
   double wSum;
   double kSum;
   int i;
   double tWB;
   double sampleVal, push;
   std::vector<gridPoint *>::iterator _b;

   tWB = maxB-minB;
   
   for(i = 0; i < samples; i++){
      sampleVal = minB+((double)i/(double)samples)*tWB;
      wSum = kSum = 0;
      for(_b = b.begin(); _b != b.end(); _b++){
         wSum += (*_b)->getW();
         kSum += (*_b)->getW()*findKB(sampleVal-(*_b)->getBD());
      }
      if(kSum == kSum){
         if(wSum <= 0.0){
            cout << "Bad wSum in bearing: " << wSum << endl;
         }
         push = (1.0/wSum)*kSum;
         bearF.push_back((1.0/wSum)*kSum);
         bearPos.push_back(sampleVal);
      }
      else{
         push = 0.0;
         bearF.push_back(0.0);
         bearPos.push_back(sampleVal);
      }
   }
}

void bin::genSpeedDif(){
   double wSum;
   double kSum;
   int i;
   double tWS;
   double sampleVal, push;
   std::vector<gridPoint *>::iterator _b;

   tWS = maxS-minS;
   
   for(i = 0; i < samples; i++){
      sampleVal = minS+((double)i/(double)samples)*tWS;
      wSum = kSum = 0;
      for(_b = b.begin(); _b != b.end(); _b++){
         wSum += (*_b)->getW();
         kSum += (*_b)->getW()*findKS(sampleVal-(*_b)->getSD());
      }
      if(kSum == kSum){
         if(wSum <= 0.0){
            cout << "Bad wSum in speed: " << wSum << endl;
         }
         push = (1.0/wSum)*kSum;
         speedF.push_back((1.0/wSum)*kSum);
         speedPos.push_back(sampleVal);
      }
      else{
         push = 0.0;
         speedF.push_back(0.0);
         speedPos.push_back(sampleVal);
      }
   }
}

void bin::computeAreaB(){
   std::vector<double>::iterator _d;
   double aV, bV;
   double n, sum;

   n = bearF.size();

   _d = bearF.begin();
   aV = minB;
   sum = (*_d);
   
   for(_d = bearF.begin()+1; _d != bearF.end()-1; _d++){
      sum += 2*(*_d);
   }
   _d = bearF.end()-1;
   sum += (*_d);
   bV = maxB;

   if(n != 0){
      areaB = ((bV-aV)/(2*n))*sum;
   }
   else{
      areaB = 0.0;
   }

   // Normalize area
   if(areaB != 0.0){
      for(_d = bearF.begin(); _d != bearF.end(); _d++){
         (*_d) = (*_d)*(1.0/areaB);
      }
      _d = bearF.begin();
      aV = minB;
      sum = (*_d);
   
      for(_d = bearF.begin()+1; _d != bearF.end()-1; _d++){
         sum += 2*(*_d);
      }
      _d = bearF.end()-1;
      sum += (*_d);
      bV = maxB;

      if(n != 0){
         areaB = ((bV-aV)/(2*n))*sum;
      }
   }

   // Find area of individual trapazoids
   double k = 1;
   double fa, fb;
   double a;
   sum = 0.0;
   if(areaB != 0.0){
      for(_d = bearF.begin()+1; _d != bearF.end(); _d++){
         aV = minB + ((k-1)/bearF.size())*(maxB-minB);
         bV = minB + (k/bearF.size())*(maxB-minB);
         fa = (*_d);
         fb = (*(_d-1));
         a = (bV-aV)*((fa+fb)/2.0);
         sum += a;
         bearDistFunct.push_back(a);
         k++;
      }
   }

   // Now assign to the bear DistFunct;
   //cout << "About to write KDE....  " << midPoint << endl;
   /*if(abs(midPoint.x) == 87.5 && abs(midPoint.y) == 21.5){
      cout << "Writing KDE" << endl;
      BearingToFile();
   }*/
}

void bin::computeAreaS(){
   std::vector<double>::iterator _d;
   double aV, bV;
   double n, sum;

   n = speedF.size();

   _d = speedF.begin();
   aV = minS;
   sum = (*_d);
  
   for(_d = speedF.begin()+1; _d != speedF.end()-1; _d++){
      sum += 2*(*_d);
   }
   _d = speedF.end()-1;
   sum += (*_d);
   if( (*_d) != (*_d) ){
      cout << "          Bad D!!!" << endl;
   }
   bV = maxS;

   if(n != 0){
      areaS = ((bV-aV)/(2*n))*sum;
   }
   else{
      areaS = 0.0;
   }
   if(sum != sum || areaS != areaS){
      cout << "      minS: " << minS << "     maxS: " << maxS << endl;
      cout << "      n: " << n << "      aV: " << aV << "      bV: "  << bV << endl;
      cout << "      sum: " << sum << "      areaS: " << areaS << endl;
   }

   // Normalize area
   if(areaS != 0.0){
      for(_d = speedF.begin(); _d != speedF.end(); _d++){
         (*_d) = (*_d)*(1.0/areaS);
      }
      _d = speedF.begin();
      aV = minS;
      sum = (*_d);
   
      for(_d = speedF.begin()+1; _d != speedF.end()-1; _d++){
         sum += 2*(*_d);
      }
      _d = speedF.end()-1;
      sum += (*_d);
      bV = maxS;

      if(n != 0){
         areaS = ((bV-aV)/(2*n))*sum;
      }
   }

   // Find area of individual trapazoids
   double k = 1;
   double fa, fb;
   double a;
   sum = 0.0;
   if(areaS != 0.0){
      for(_d = speedF.begin()+1; _d != speedF.end(); _d++){
         aV = minS + ((k-1)/speedF.size())*(maxS-minS);
         bV = minS + (k/speedF.size())*(maxS-minS);
         fa = (*_d);
         fb = (*(_d-1));
         a = (bV-aV)*((fa+fb)/2.0);
         sum += a;
         speedDistFunct.push_back(a);
         k++;
      }
   }
   if(areaS != areaS){
      cout << "Area under speed curve: " << areaS << "   speedF.size(): " << speedF.size() << endl;
   }
}

double bin::getProbBear3Hour(double p, double min, int findPre, int histOrPre){
   double kdeVal, rangeL, omega;
   double chosen, preBear;
   int r, q;

   kdeVal = 3*getProbBear(p, min, findPre);

   srand(counter);
   counter = (counter*21)%UINT_MAX;
   r = rand()%10000000+1;
   chosen = (double)r/10000000.0;

   globalChosen = chosen;

   q = (int)floor(min/(60*sim->getHours()));
   preBear = sim->advList[sim->getCurrentAdv()]->pre->predPathSeg[q]->getPreProbBear(chosen);

   omega = findOmega(min);

   /*if(preBear > 60){
      cout << "     preBear: " << preBear << endl;
      cout << "        min: " << min << endl;
   }*/
   //omega = 1;
   rangeL = kdeVal*(1.0-omega) + preBear*(omega);
   /*if(rangeL > 360.0){
      cout << "RangeL exploded: " << rangeL << endl;
   }*/

   if(histOrPre == 1){
      return kdeVal;
   }
   else{
      return preBear;
   }

   return rangeL;
}

// Parameter has to be between 0-1
double bin::getProbBear(double p, double min, int findPre){
   std::vector<double>::iterator _d;
   double newP = p*areaB;
   double n, testD;
   double sum, prevSum;
   double rangeL, rangeT;
   double areaDif, areaRatio;
   double mod;
   int k;
   
   k = 0;
   n = bearF.size();
   sum = prevSum = 0.0;
   for(_d = bearDistFunct.begin(); _d != bearDistFunct.end()-1 && bearDistFunct.size() > 0; _d++){
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
   rangeL = bearPos[k];
   rangeT = bearPos[k+1];
   //cout << "   RangeL: " << rangeL << "    RangeT: " << rangeT << "    area ratio: " << areaRatio << endl;
   rangeT = rangeT - rangeL;
   mod = 1.0;
   if(rangeT < 0){
      mod = -1.0;
   }
   rangeL = (rangeL + rangeT*areaRatio);
   rangeL = fmod(rangeL, 360.0);

   //cout << "     Historical Bearing Returning rangeL: " << rangeL << "     min: " << minB << "     max: " << maxB << "  omega: " << omega <<  endl;
   return rangeL;
}

double bin::getProbSpeed3Hour(double p,double min, int findPre, int histOrPre){
   double kdeVal, rangeL, omega;
   double chosen, preSpeed;
   int r, q;

   kdeVal = 3*getProbSpeed(p, min, findPre);

   srand(counter);
   counter = (counter*21)%UINT_MAX;
   r = rand()%10000000+1;
   chosen = (double)r/10000000.0;

   chosen = globalChosen;

   q = (int)floor(min/(60*sim->getHours()));
   preSpeed = sim->advList[sim->getCurrentAdv()]->pre->predPathSeg[q]->getPreProbSpeed(chosen);

   omega = findOmega(min);
   //omega = 1;
   rangeL = kdeVal*(1.0-omega) + preSpeed*(omega);

   if(histOrPre == 1){
      return kdeVal;
   }
   else{
      return preSpeed;
   }

   return rangeL;
}

// Parameter has to be between 0-1
double bin::getProbSpeed(double p,double min, int findPre){
   std::vector<double>::iterator _d;
   double newP = p*areaS;
   double n, testD;
   double sum, prevSum;
   double rangeL, rangeT;
   double areaDif, areaRatio;
   double mod;
   int k;
  
   k = 0;
   n = speedF.size();
   sum = prevSum = 0.0;
   for(_d = speedDistFunct.begin(); _d != speedDistFunct.end()-1 && speedDistFunct.size() > 0; _d++){
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
   if(areaDif != 0){
      areaRatio = areaRatio/areaDif;
   }
   else{
      areaRatio = 0.0;
   }

   // Use area interpolation to find a new value
   rangeL = speedPos[k];
   rangeT = speedPos[k+1];
   rangeT = rangeT - rangeL;
   mod = 1.0;
   if(rangeT < 0){
      mod = -1.0;
   }
   rangeL = (rangeL + rangeT*areaRatio);

   return rangeL;
}

void bin::BearingToFile(){
   fstream file;
   unsigned int i;
   double pos, xMin, xMax;

   file.open("bearingOut.txt", fstream::out);

   for(i = 0; i < bearF.size(); i++){
      //cout << "    minB: " << minB << " + " << ((double)i/(double)bearF.size())*(maxB-minB) << " "; 
      pos = minB + ((double)i/(double)bearF.size())*(maxB-minB);
      file << dec << pos;
      file << " ";
      file << dec << bearF[i];
      file << endl;
      //cout << "Writing to file: " << pos << " " << bearF[i] << endl;
   }

   file.close();

   file.open("plotBearing", fstream::out);

   xMin = minB;// - 0.5*(maxB-minB);
   xMax = maxB;// + 0.5*(maxB-minB);
   file << "set nokey" << endl;
   file << "set xrange[";
   file << dec << xMin;
   file << ":";
   file << dec << xMax;
   file << "]" << endl;

   file << "set style line 1 lt 1 lw 3" << endl;
   file << "set style line 2 lt 2 lw 1" << endl;
   file << "Gauss1(x,mu,sigma) = 1.0/(sigma*sqrt(2*pi)) * exp( -(x-mu)**2 / (2*sigma**2) )" << endl;
   file << "Gauss2(x,mu,sigma) = 0.75/(sigma*sqrt(2*pi)) * exp( -(x-mu)**2 / (2*sigma**2) )" << endl;
   file << "Gauss3(x,mu,sigma) = 0.5/(sigma*sqrt(2*pi)) * exp( -(x-mu)**2 / (2*sigma**2) )" << endl;
   file << "Gauss4(x,mu,sigma) = 0.25/(sigma*sqrt(2*pi)) * exp( -(x-mu)**2 / (2*sigma**2) )" << endl;
   //file << "plot \"bearingOut.txt\" title \"title\" with lines linestyle 1";
   file << "plot ";
   for(i = 0; i < b.size(); i++){
      if(b[i]->getW() == 1.0){
         file << "Gauss1(x, ";
      }
      if(b[i]->getW() == 0.75){
         file << "Gauss2(x, ";
      }
      if(b[i]->getW() == 0.5){
         file << "Gauss3(x, ";
      }
      if(b[i]->getW() == 0.25){
         file << "Gauss4(x, ";
      }
      file << dec << b[i]->getBD();
      file << ", ";
      file << dec << hB;
      file << ") linestyle 2, ";
   }
   //file << "Gauss(x,mu,sigma) = 1.0/(sigma*sqrt(2*pi)) * exp( -(x-mu)**2 / (2*sigma**2) )" << endl;
   file << "\"bearingOut.txt\" title \"title\" with lines linestyle 1";

   file.close();
}


void bin::buildSegDist(int curAdv){
   std::vector<Vector4d*> prePath;
   std::vector<Vector4d*>::iterator p0;
   std::vector<Vector4d*>::iterator p1;
   prePath = sim->advList[curAdv]->pre->predPathThreeHour;
   Vector2d mid;
   Vector2d a, b;
   double dist, bear;

   p1 = prePath.begin()+1;
   for(p0 = prePath.begin(); p0 != prePath.end()-1; p0++){
      a.set((*p0)->x, (*p0)->y);
      b.set((*p1)->x, (*p1)->y);

      dist = haversine(a.x, b.x, a.y, b.y)/2.0;
      bear = findBearing_2(&a, &b);

      mid = locateDestination_2(a.x, a.y, dist, bear);

      //cout << "a: " << a << "   b: " << b << "   mid: " << mid << "   dist: " << dist << "   bear: " << bear << endl;

      dist = haversine(mid.x, midPoint.x, mid.y, midPoint.y); 
      //cout << "    cell Mid: " << midPoint << "    dist to path: " << dist << " with mid: " << mid << endl;
      preSegDist[curAdv]->push_back(dist);
      p1++;
   }
   //cout << "preSegDist size: " << preSegDist.size() << endl;
}

double bin::findOmega(double min){
   int bin = (int)floor(min/((69/(preSegDist[sim->getCurrentAdv()]->size()-1.0)*60)));
   double dx, rho, toRet;
   std::vector<double>::iterator _d;
   dx = preSegDist[sim->getCurrentAdv()]->at(bin);
   rho = findRo(min);

   //double area = sqrt(2*M_PI);
   //double valNear = 2.58;
   double valNear = 1.35;
   //double valFar = 1.71;
   double valFar = 3.0;
   //double height = area/(valNear + valFar);
   double height = 1.0;
   double testX = abs(dx/(sqrt(2)*rho*userSig));
   double ratio = 0.0;

   /*if( dx < 100 ){
      dx = 1;
   }*/


   toRet = exp((-1.0*dx*dx)/(2.0*(rho*rho*userSig*userSig)));
   /*if(testX > valNear/2.0){
      cout << "Initial toRet: " << toRet << endl;
   }*/
   if(toRet != toRet){
      cout << "There's a problem!" << endl;
      cout << "     bin: " << bin << " dx: " << dx << " rho: " << rho << endl;
   }
   //cout << "     min : " << min << endl;
   //cout << "     Omega: " << toRet << " with dx: " << dx << "  rho: " << rho  << "    userSig: " << userSig << " and bin: " << bin << endl;

   if(preOnly > 0){
      toRet = 1.0;
   }
   else{
     // Use constant value for values close to predicted path
     if(testX <= valNear/2.0){
        toRet = height;
     }
     // Use linear interpolation to find other values
     else{
        ratio = testX - (valNear/2.0);
        ratio = pow((valFar - ratio)/valFar, 3);

        toRet = height*ratio;
     }
     
   }
   /*if(testX > valNear/2.0){
      cout << "   TestX: " << testX << "     valNear/2.0: " << valNear/2.0 << "    ratio: " << ratio;
      cout << "    height: " << height << "     new toRet: " << toRet << endl;
   }*/
   return toRet; 
}

double bin::findRo(double min){
   double t = min/60.0 + 3;
   int i;
   double subt, setNHS, f;
   double intRad;
   double ro;   // Find i, subt and setNHS based on the sim->getHours() completed so far

   if(t < 9){
      i = 0;
      subt = 0;
      setNHS = 9;
   }
   else if(t <= 21){
      i = 1;
      subt = 9;
      setNHS = 12;
   }
   else if(t <= 33){
      i = 2;
      subt = 21;
      setNHS = 12;
   }
   else if(t <= 45){
      i = 3;
      subt = 33;
      setNHS = 12;
   }
   else{
      i = 4;
      subt = 45;
      setNHS = 24;
   }

   f = (t - subt)/setNHS;

   if(t != 0){
      intRad = f*errorRadius[i+1] + (1-f)*errorRadius[i];
   }
   else{
      intRad = errorRadius[i];
   }

   ro = intRad/errorRadius[5];
   
   if(ro == 0){
      ro = 0.0001;
   }
   return ro;
}
