/*
 * Contains methods and variables for each walking state 1--4
 */

double PIDController(double Desired_Force, double Load_Cell_Force,double Gain_P);
double DesiredForce(double Impedance,double MomentArm);
double KneeAngle(double PVolt);
double MomentArm(double KneeAngle);
double Impedance(double KneeAngle, double Knee_Velocity, double K1, double B, double Theta_E);
double PIDCurrent(double Desired_Current, double Current,double Gain_P,float percent);
double DesiredCurrent (double Impedance,double Angle);
float RateLimiter(float percent_old,float percent);
double KneeDesiredCurrent (double Impedance,double Angle);
