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
  int T,n=0;
  double t0,GPSguess,vguess,gpsVar,vVar,
         t,x,y,vx,vy,ax,ay;
  
  freopen("in.txt", "r", stdin);
  freopen("out.txt", "w", stdout);
 
  
  
  
cin>>T;
Kalman why;//(0,0,0,0,0);  
while(T--){ // test cases
  
  cin>>t0>>GPSguess>>vguess>>gpsVar>>vVar>>n;
  why.intialize(t0,GPSguess,vguess,gpsVar,vVar);
  cout<<"GPS vareinace of "<<gpsVar<<"\n\n\n";
  
    while(n--){ // for the sample
      
      cin>>t>>x>>y>>vx>>vy>>ax>>ay;
      why.handleUpdate(t,x,y,vx,vy,ax,ay);
      
    }

}

  

  fclose (stdin);  // stop reading data from in.txt
  fclose (stdout); // stop writing data from out.txt
}
