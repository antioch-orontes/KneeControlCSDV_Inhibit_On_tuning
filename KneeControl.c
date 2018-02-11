//Two loadcell data

#include "KneeControl.h"
#include "StateFormulas.h"
#include "rotation.h"
#include <stdbool.h>
#include <math.h>
#include "pwm.h"

/*Switching parameter*/
#define ES_SWF_switching_heeloff 950   //State 0 to State 0 (Early_stance to Swing_flexion)

#define SWF_SWE_switchin_angle 40  //State 2 to State 3 (Swing_flexion to Swing_extension)

#define SWE_Idle_switching_angle 5  //earlier 4 State 3 to State 4  (Swing_extension to Idle)

#define ES_SWF_switching_heelstrike 1000   //State 4 to State 0  (Idle to Early_stance)



/*State Equilibrium Setup*/
#define ES_equilibrium 8   // earlier 5 position in degree
#define PSW_equilibrium 20    // position in degree -3
#define SWF_equilibrium 50     // position in degree earlier 55
#define SWE_equilibrium 6     //earlier 3  position in degree
#define IDLE_equilibrium 8     // earlier 4 position in degree


/*State 0:  Early Stance Parameter*/
#define EStance_stiffness 1.80     // earlier 1.9 Previous St Eq.= 0 Present St Eq.= 0
#define EStance_damping 0.0005  //earlier .001

/*State 1:  PRE_SWING Parameter*/
#define PRE_SWING_stiffness 0.07               // Previous St Eq.= 0 Present St Eq.= 20
#define PRE_SWING_damping 0.001       

/*State 2:  SW_FLEXION Parameter*/
#define SW_FLEXION_stiffness 0.24     //previos St stiffness 0.22  Previous St switching = 0 Present St Eq.= 60
#define SW_FLEXION_damping 0.005 // previous damping .005

/*State 3:  SW_EXTENSION Parameter*/
#define SW_EXTENSION_stiffness 0.24    //previos St stiffness 0.18 // Previous St switching = 38 Present St Eq.= 2 
#define SW_EXTENSION_damping 0.006

/*State 4:  IDLE Parameter*/
#define IDLE_stiffness 0.45     // Previous St switching = 2 Present St switching = 2
#define IDLE_damping 0.005


enum states{
    ST_EARLY_STANCE,
    ST_PRE_SWING,
    ST_SW_FLEXION,
    ST_SW_EXTENSION,
    IDLE,
};
//enum states state = ST_EARLY_STANCE;

struct st_impedance knee_st_impedance;
enum states state = IDLE;
double impedance = 0;
double desired_force = 0.0, desired_current = 0;
float percent = 0,percent_new = 0,percent_old = 0;  // 0.5 percent means 50% duty cycle
float peak_current = 20.0;  //A
/* Duty cycle range for swing_flexion is 10% to 15%*/
float duty_cycle_max = 0.22; // which means 20 percent
float sw_flexion_pwm_max = 0.15;
float sw_extension_pwm_max = 0.15;
float late_stance_pwm_max = 0.4;
// const number for appropriate time delay  of ST_EARLY_STANCE
double count = 0;
double heel_sensor_toe_off = 180; //earlier it was 0.65
//double swing_flexion_angle = 38;
float tau_friction = 1.0;
float sum_loadcell,diff_loadcell;

struct st_impedance KneeControl(float knee_angle, float knee_velocity,int16_t ac_x,float load_cell1,float load_cell2)
{
    sum_loadcell=load_cell1+load_cell2;
    diff_loadcell=load_cell2-load_cell1;
    //bool cond_HS = ((knee_angle >= 2)&&(knee_velocity >= 1))? true:false; //Heel Strike condition is met.
    //bool  cond_HS = (ac_x > 15000) ? true : false;  //Heel Strike condition is met.
    //printf("%d\n",state);
switch (state)
    {
    // state 0
    case ST_EARLY_STANCE:
        //LATBbits.LATB15 = 1;
         if (sum_loadcell <= ES_SWF_switching_heeloff) //(for oldcell testing)
        {
            state = ST_SW_FLEXION;
            break;
        }
        
        
        
      // if(count >= 600) //650 delay 0.86 sec  (770 for 1 sec) 850
      //  {
         //   state = ST_PRE_SWING;
         //   count=0;
        //    break;
        //}
        
        //CONTROL ACTION  K1 = 2
        
        impedance = Impedance(knee_angle, knee_velocity,EStance_stiffness,EStance_damping, ES_equilibrium); //Impedance( KneeAngle, Knee_Velocity, K1, B,Theta_E)
        //impedance = ( impedance > .65) ? 0 : impedance; //if impedance is <=5 then impedance =5 else impedance =impedance
        
        //printf("\ntau:%f\n",impedance);
        //desired_current = impedance/210/0.9*1000/37;
        desired_current = KneeDesiredCurrent (impedance,knee_angle);

        //percent = PIDCurrent(desired_current, current, 0.0001, percent); // calculate the valve_command from the PID controller
        percent = desired_current/peak_current;
        //printf("\np:%f\n",-percent);
        //limit the duty cycle to 25% range, so the current will in the range of 0 to 25%*20A = 0 to 5A
        // in practical limit to 20% percent

        if (percent >= duty_cycle_max)
            percent = duty_cycle_max;
        else if(percent <= - duty_cycle_max)
            percent = - duty_cycle_max;
        

        //Rate Limiter
        percent_new = RateLimiter(percent_old,percent);
        percent_old = percent_new;

       // if (fabsf (percent_new)<=0.06)
         //   percent_new = 0;
        //printf("\np:%f\n",percent_new);
        if (desired_current <= 0)    //output the pwm command
            KneeExtension (-percent_new);
        else if (desired_current >0)
            KneeFlexion (percent_new);
 

        count ++;

        break;
    //state 1
    case ST_PRE_SWING:
        if (load_cell1 < 132) //(for oldcell testing)
        {
            state = ST_SW_FLEXION;
            break;
        }
        
        //if (knee_angle >=12) //(for oldcell testing)
        //{
        //    state = ST_SW_FLEXION;
        //    break;
        //}
        
        //CONTROL ACTION K1 = 1
        impedance = Impedance(knee_angle, knee_velocity, PRE_SWING_stiffness, PRE_SWING_damping, PSW_equilibrium);
        //printf("\ntau:%f\n",impedance);
        //desired_current = impedance/210/0.9*1000/37;
        desired_current = KneeDesiredCurrent (impedance,knee_angle);

        //percent = PIDCurrent(desired_current, current, 0.0001, percent); // calculate the valve_command from the PID controller
        percent = desired_current/peak_current;
        //printf("\np:%f\n",-percent);
        //limit the duty cycle to 25% range, so the current will in the range of 0 to 25%*20A = 0 to 5A
        // in practical limit to 20% percent

        if (percent >= duty_cycle_max)
            percent = duty_cycle_max;
        else if(percent <= - duty_cycle_max)
            percent = - duty_cycle_max;
        //printf("\np:%f\n",-percent);

        //Rate Limiter
        percent_new = RateLimiter(percent_old,percent);
        percent_old = percent_new;

        if (desired_current < 0)    //output the pwm command
            KneeExtension (-percent_new);
        else if (desired_current >0)
            KneeFlexion (percent_new);
        break;
    //state 2
    case ST_SW_FLEXION:
        if (knee_angle >= SWF_SWE_switchin_angle)
        {
            state = ST_SW_EXTENSION;
            //state = IDLE;
            //Stop(); //stop the motor
            break;
        }

        //CONTROL ACTION  0.36
        impedance = Impedance(knee_angle, knee_velocity, SW_FLEXION_stiffness, SW_FLEXION_damping, SWF_equilibrium); //
        //impedance = ((impedance <= 4) && (impedance >= 0.0)) ? 4 : impedance;

        //impedance = Impedance(knee_angle, 0.0, 1, 0.01, -20.0);
        //printf("\ntau:%f\n",impedance);
        //desired_current = impedance/210/0.9*1000/37;
        desired_current = KneeDesiredCurrent (impedance,knee_angle);
        //printf("\nid:%f\n",desired_current);
        //percent = PIDCurrent(desired_current, current, 0.0001, percent); // calculate the valve_command from the PID controller
        percent = desired_current/peak_current;
        //printf("\npercent:%f\n",percent);
        //limit the duty cycle to 25% range, so the current will in the range of 0 to 25%*20A = 0 to 5A
        // in practical limit to 20% percent

        if (percent >= sw_flexion_pwm_max)
            percent = sw_flexion_pwm_max;
        else if(percent <= - sw_flexion_pwm_max)
            percent = - sw_flexion_pwm_max;
        //printf("\np:%f\n",-percent);

        percent_new = RateLimiter(percent_old,percent);
        percent_old = percent_new;
        
        //printf("\np:%f\n",percent_new);
        
        if (desired_current < 0)    //output the pwm command
            KneeExtension (-percent_new);
        else if (desired_current >0)
            KneeFlexion (percent_new);
        break;
      //state 3
      case ST_SW_EXTENSION:
          //TRISBbits.TRISB15 = 1;
        //if (heel_sensor >= heel_sensor_toe_off)
        if (knee_angle <= SWE_Idle_switching_angle)
        {
            //Stop(); //stop the motor

            //LATBbits.LATB15 = 0;
            //__delay_ms(50);
            state = IDLE;
            //state = ST_EARLY_STANCE;
            break;
        }

        //CONTROL ACTION 1.5

        impedance = Impedance(knee_angle, knee_velocity, SW_EXTENSION_stiffness, SW_EXTENSION_damping, SWE_equilibrium);
        //impedance = ((impedance <=-0.1) && (impedance >= -3.7)) ? -3.7 : impedance;
        
        //printf("\ntau:%f\n",impedance);
        //desired_current = impedance/210/0.9*1000/37;
        desired_current = KneeDesiredCurrent (impedance,knee_angle);

        //percent = PIDCurrent(desired_current, current, 0.0001, percent); // calculate the valve_command from the PID controller
        percent = desired_current/peak_current;
        //printf("\np:%f\n",-percent);
        //limit the duty cycle to 25% range, so the current will in the range of 0 to 25%*20A = 0 to 5A
        // in practical limit to 20% percent

        if (percent >= sw_extension_pwm_max)
            percent = sw_extension_pwm_max;
        else if(percent <= - sw_extension_pwm_max)
            percent = - sw_extension_pwm_max;
        //printf("\np:%f\n",-percent);

        //Rate Limiter
        percent_new = RateLimiter(percent_old,percent);
        percent_old = percent_new;


        if (desired_current < 0)    //output the pwm command
            KneeExtension (-percent_new);
        else if (desired_current >0)
            KneeFlexion (percent_new);
        break;


      
        //state 4
      case IDLE:
          
       // Stop(); //stop the motor
        if (sum_loadcell >= ES_SWF_switching_heelstrike) 

        {
            state = ST_EARLY_STANCE;
            break;
        }
        
        
                //CONTROL ACTION  K1 = 2
        
        impedance = Impedance(knee_angle, knee_velocity, IDLE_stiffness,IDLE_damping, IDLE_equilibrium); //Impedance( KneeAngle, Knee_Velocity, K1, B,Theta_E)
        
        desired_current = KneeDesiredCurrent (impedance,knee_angle);

        percent = desired_current/peak_current;
        if (percent >= duty_cycle_max)
            percent = duty_cycle_max;
        else if(percent <= - duty_cycle_max)
            percent = - duty_cycle_max;
        

        //Rate Limiter
        percent_new = RateLimiter(percent_old,percent);
        percent_old = percent_new;

       // if (fabsf (percent_new)<=0.06)
         //   percent_new = 0;
        //printf("\np:%f\n",percent_new);
        if (desired_current <= 0)    //output the pwm command
            KneeExtension (-percent_new);
        else if (desired_current >0)
            KneeFlexion (percent_new);
        
        break;
     
    }
//  printf("tau:%f,",impedance);
//    return state;
    
    knee_st_impedance.st = state;
    knee_st_impedance.impedance = impedance;
    knee_st_impedance.percent_new = percent_new;
    return knee_st_impedance;
    
    
}
 