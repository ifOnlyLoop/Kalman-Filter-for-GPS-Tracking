/*
 *  This function is coded to reduce the GPS error of a car
 *  given its GPS(from multi satalites) and its position
 *  from other LiDARs if given
 */

#include<bits/stdc++.h>
#include"Kalman.h"

//using namespace Eigen;
using namespace std;

int main()
{
  int n;
  double t0,GPSguess,vguess,gpsVar,vVar,
         t,x,y,vx,vy,ax,ay;
  
  freopen("in.txt", "r", stdin);
  freopen("out.txt", "w", stdout);
 
  cin>>t0>>x0>>y0>>v0>>gpsVar>>vVar>>n;
  Kalman why(t0,GPSguess,vguess,gpsVar,vVar);
  t=0;
  
  while(n--){
  cin>>x>>y>>v>>TH;
  why.handleUpdate(t,x,y,vx,vy,ax,ay);
  }

  fclose (stdin);  // stop reading data from in.txt
  fclose (stdout); // stop writing data from out.txt
}
