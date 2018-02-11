/*
 * This file contains the math functions and models for calculating states 1--5
 *
 * The Ankle Velocity formula is placed in state 1 since it has a sample time
 * dependent part the function (numerical differentiation).
 *
 */

#include "StateFormulas.h"
#include <math.h>
// #include <double.h>


//const double K1      = 1.0;
//const double K2      = 1.0;
//const double b       = 1.0;
//const double theta_b = 1.0;
//const double Gain    = 1.0f;
#define HARMONIC_DRIVE_RATIO 50 //earlier it was 100
#define BELT 3  // earlier it was 1.31
//#define KNEE_GEAR_RATIO HARMONIC_DRIVE_RATIO*BELT
#define KNEE_GEAR_RATIO 150

const double theta_s0 = 3.6; //deg
const double control_command_lower_limit = 0.3; // volt full range is 0.3 to 5 v
const float k_spring = 4.0;     //Nm/deg
const float torque_const = 37.0;    //mNm/A
double loop_time = 0.001;   // second
double rising_slew_rate = 30;    // voltage/sec
double falling_slew_rate = -30;

// LoadCellForce
double LoadCellForce(double LCellVoltage) {
        return LCellVoltage*445.0+20;
}
//
double KneeAngle(double PVolt){
        //return acos((42.4685-(6.0+0.6*PVolt)*(6.0+0.6*PVolt))/22.5294)*57.2958;
        // from 0 to 2.5 volt to knee_angle (0 to 100 deg).
        return PVolt*132-76;
}

double MomentArm(double KneeAngle){
        //return sin(0.2912-acos((sin(acos((42.4685-(6.0+0.6*PVolt)*(6.0+0.6*PVolt))/22.5294)-1.2796)*7.7984)/((6.0+0.6*PVolt)*(6.0+0.6*PVolt))))*0.1591;
        //3rd polynomial fit
        double poly_coefficient[4] = {2.22320112553112e-08,-9.47330958490815e-06,0.000674466762924094,0.0316020276371335}; //3rd order to 0 order
        return poly_coefficient[0]*pow(KneeAngle,3.0)+poly_coefficient[1]*pow(KneeAngle,2.0)+poly_coefficient[2]*KneeAngle+poly_coefficient[3];
}

double Impedance(double KneeAngle, double Knee_Velocity, double K1, double B, double Theta_E){

        return -1*(K1*(KneeAngle-Theta_E)+B*Knee_Velocity);
}

double DesiredForce(double Impedance,double MomentArm){
        return Impedance/MomentArm;
}

double DesiredCurrent (double Impedance,double Angle){
        double tau_motor, tau_spring = 0, id;
        if (Angle > theta_s0)
                tau_spring = -k_spring*(Angle - theta_s0);
        else
                tau_spring = 0;
        tau_motor = Impedance - tau_spring;
        id = tau_motor/KNEE_GEAR_RATIO/0.9*1000/torque_const;
        return (id);
}

double KneeDesiredCurrent (double Impedance,double Angle){
        double tau_motor, id;

        tau_motor = Impedance;
        //id = tau_motor/150/0.9*1000/37;
        id = tau_motor/KNEE_GEAR_RATIO/0.9*1000/torque_const;

        return (id);
}

double PIDController(double Desired_Force, double Load_Cell_Force,double Gain_P) {
        // control_command is in range (0 to 5V).
        double control_command;
        control_command = Gain_P*(Desired_Force-Load_Cell_Force)/2;
        if (control_command >2.5)
                control_command = 2.5;
        else
        if (control_command <-2.5)
                control_command = -2.5;
        control_command +=2.5; // control_command is in range (0 to 5v)
        if (control_command < control_command_lower_limit)
                control_command = control_command_lower_limit;
        return control_command;
}

double PIDCurrent(double Desired_Current, double Current,double Gain_P,float percent) {
        // control_command is in range (0 to 5V).
        double control_command;
        control_command = Gain_P*(fabs(Desired_Current)-Current);
        control_command = control_command+percent;

        if (control_command >=0.5)
                control_command = 0.5;
        else
        if (control_command <=0)
                control_command = 0;
        return control_command;
}

float RateLimiter(float percent_old,float percent)
{

        float percent_new = percent;
        float rate;

        rate = (percent_new-percent_old)/loop_time;
        if (rate > rising_slew_rate)
                percent_new = loop_time*rising_slew_rate + percent_old;
        else if (rate < falling_slew_rate)
                percent_new = loop_time*falling_slew_rate + percent_old;
        return percent_new;

}
