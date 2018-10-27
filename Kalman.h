/*
 *  This function is coded to reduce the GPS error of a car
 *  given its GPS(from multi satalites) and its position
 *  from other LiDARs if given
 */

#include<bits/stdc++.h>
#include"/usr/include/eigen3/Eigen/Eigen" // include the path from the eigen3 file

using namespace Eigen;
using namespace std;

/*
 * for farther details you may refere to :
 * 1- https://blog.maddevs.io/reduce-gps-data-error-on-android-with-kalman-filter-and-accelerometer-43594faed19c
 * 2- https://pdfs.semanticscholar.org/0b93/eb84ff2f48ea8d9e2770e4b45d30609095b1.pdf
 */

// NOTES to reader
/*
 * SEE LAST LINE of the header to understand the input represntation
 * use EIGEN3 library (it's a must)
 * if you can make the GPS readings in Eigen vector format I'd be happy
 * if not I'll handle the convertion or you can replace it with for loop
 */

class Kalman{

private:

    int i,n; // number of samples/readings

    double
            dt,  // time step/slot
            x,y,vx,vy,ax,ay, // GPS,Speed,Acce
            xVar,yVar,vxVar,vyVar; // Error

    /*
     * Our state space model taken from motion equations
     * X(i)= AX(i-1) + BU + W  (matrix format)
     * x(t+dt)= x(t)+ t v(dt) + 0.05 t^2 a(dt)
     */
    
    Matrix<double,4,4> K,P,A; 
    Matrix<double,4,2> B; 
    Matrix<double,4,1> X,Xp,Xm,Y; 
    Matrix<double,2,1> U;
    Matrix<double,2,4> H; 
    /*
     * A: zero input / state response
     * B: zero state / input response
     * X: state(updated)
     * Xm: states(measured)
     * Xp: states(predicted)
     * Y: states(measured)
     * U: accelaration input
     * States are position and velocity
     * K: kalman gain
     * P: covariance matrix
     */
   
    /*
     * Errors and noises of the Kalman
     */
    Matrix<double,4,4> R; // covariance of observation noise
    Matrix<double,4,4> Q; // covariance of process(algorthem) noise
    Matrix<double,4,1> Z ; // covariance of observation(measuremnt) error
    //Matrix<double,4,1> W ; // covariance of prediction error

    /*
     * Statistics (for future use)
     * row:1 GPSx, row:2 GPSy row:3 speed.
     */
    //Matrix<double,2,2> MEANS; // readings means
    //Matrix<double,2,2> VARS ; // readings variances

protected:
    
public:
    // constructor
    Kalman();
    // next estimation
    void handleUpdate();

};



Kalman::Kalman(){

    // read parameters
    cin>>dt;
    // initialize the states and input coefficients matrix
    A << 1,0,dt,0, 0,1,0,dt, 0,0,1,0, 0,0,0,1;
    B << 0.5*dt*dt,0, 0,0.5*dt*dt, dt,0, 0,dt;

    // input the parameters of error/noise model
    cin>>xVar>>yVar>>vxVar>>vyVar;

    // initialize errors coefficients matrix
    R <<    xVar*xVar,     0,0,0,
         0,   yVar*yVar,     0,0,
         0,0,   vxVar*vxVar,   0,
         0,0,0,   vyVar*vyVar;
    Q <<    xVar*xVar,  0,xVar*vxVar,  0,
         0,   yVar*yVar,  0,yVar*vyVar,
         0,0,   vxVar*vxVar,0,
         0,0,0,   vyVar*vyVar;
    H << 1,0,0,0, 0,1,0,0;
    
    Z.setRandom();
    Z*=0.1;

    K= Matrix<double,4,4>::Zero();
    P<< 6.25,0,0,0, 0,6.25,0,0, 0,0,1,0, 0,0,0,1;
    handleUpdate();

}

void Kalman::handleUpdate(){

    /*
     * kalman filter algo goes here
     */
    cin>>x>>y>>vx>>vy>>ax>>ay>>n;
    X<<x,y,vx,vy; U<<ax,ay;
    // iterate here
    while(n--){
        // predicted state X(i)=A*X(i-1)+B*U(i)+W;
        Xp=A*X+B*U/*+W*/;
        // predicted covariance P(i)=A*X(i-1)P^(-1)+Q;
        P=(A*P)*A.transpose()+Q;
        // measurment Input
        Y=Xm+Z;
        // kalman gain
        K=P*(P+R).inverse();
        // update states
        X=Xp+K*(Y-Xp);
        // update covariance
        P=(Matrix<double,4,4>::Identity()-K)*P;
        // display result
        cout<<X<<"\n\n";/*<<P<<"\n\n"<<K<<"\n\n"*/;
        // next iteration for Xm,U
        cin>>x>>y>>vx>>vy>>ax>>ay;
        Xm<<x,y,vx,vy; U<<ax,ay;

    }

    fclose (stdin); // stop readings data from in.txt
    return;
}



/*
 * Input Understanding
 * 1 time step
 * 0 0 initial position x y
 * 0 0 initial velocity x y
 * 2 0 acceleration x y
 * 5 5 GPS measurement error
 * 1 1 veocity measurement error
 * 4 number if readings (multi satalites)
 * 0 0 1 0 0 2 0 GPS x y SPEED x y  Acce x y
 * .... to end of readings
 *
 * check the output file too
 */


