/*******************************************************
   Name: Knee Control State Transition Parameters
   Date: 02/13/2018
   Created by: Md Rejwanul Haque
   Latest revisor: Cosmo Chou
   Comments: Cleaned code, updated state transition variables, added att'l state.
 *****************************************************/

#include "KneeControl.h"
#include "StateFormulas.h"
#include "rotation.h"
#include <stdbool.h>
#include <math.h>
#include "pwm.h"

// Switching Parameters
// State 4 (Idle Stance) to State 0 (Early Stance) characterized by a heelstrike.
#define heelstrike_sum 950      //(Optional) Sum of load cell readings must exceed this value to register heelstrike.
#define heelstrike_diff 40      //Difference of load cell readings must fall below this value to register heelstrike. (Difference value decreases for heel-biased loads.)
#define heelstrike_anglemax 5   //Angle of knee must be within this range to register heelstrike.
#define heelstrike_anglemin 0

// State 0 (Early Stance) to State 1 (Pre-Swing Stance) characterized by maximum knee flexion after absorbing heelstrike load.
#define stanceflex_angle 10     //Angle of knee must be greater than this value to trigger transition.
#define stanceflex_sum 965      //(Optional) Sum of load cell readings must be less than this value to trigger transition (force registered decreases after initial impact.)
#define stanceflex_diff 25      //(Optional) Difference of load cell readings must be less than this value to trigger transition (Difference value increases for toe-biased loads.)

// State 1 (Pre-Swing Stance) to State 2 (Swing Flexion) characterized by a toe-off.
#define toeoff_sum 950        //(Optional) Sum of load cell readings must exceed this value to register heelstrike. (Axial force increases during toe-off as user pushes off from ground.)
#define toeoff_diff 80        //Difference of load cell readings must exceed this value to register heelstrike. (Difference value increases for toe-biased loads.)
#define toeoff_anglemax 10    //Angle of knee must be within this range to register toe-off.
#define toeoff_anglemin 0

// State 2 (Swing Flexion) to State 3 (Swing Extension) characterized by knee exceeding a flexion threshold.
// For future models: Is it possible to vary flexion soft ceiling angle as a function of walking speed?
#define swingflex_angle 40    //Angle of knee must exceed this value to transition to extension.
#define swingflex_sum 950     //Sum of load cell readings must be less than this value to transition. As leg is now airborne and sustaining no load, axial force should decrease.
#define swingflex_diff 60     //As leg flexes, it approaches being parallel with the ground. The moment about the leg induced the gravity causes the difference between the two load cells to increase.

//State 3 (Swing Extension) to State 4 (Idle Stance) characterized by knee exceeding an extension threshold.
#define swingext_angle 5      //Angle of knee must fall below this value to transition.
#define swingext_sum 925      //As leg swings back down, centripetal force caused by swing acceleration causes 'ghost' axial force to register. Knee decelerates as it nears equilibrium, thus must fall below this value to transition.
#define swingext_diff 50      //As leg swings back down, difference must decrease past this value to account for leg approaching perpendicularity with the ground. (Just realized that this only applies on level surfaces. May need to reconsider methodology, test on inclines.)

/*

   Old values preserved here.

 #define ES_SWF_switching_heeloff 950   //State 0 to State 0 (Early_stance to Swing_flexion)
 #define SWF_SWE_switchin_angle 40  //State 2 to State 3 (Swing_flexion to Swing_extension)
 #define SWE_Idle_switching_angle 5  //earlier 4 State 3 to State 4  (Swing_extension to Idle)
 #define ES_SWF_switching_heelstrike 1000   //State 4 to State 0  (Idle to Early_stance)

 */

//State Equilibrium Setup
//Target equilibriums for each state defined below.
#define ES_equilibrium 10     // State 0: Early Stance
#define PSW_equilibrium 8     // State 1: Pre-Swing Stance
#define SWF_equilibrium 40    // State 2: Swing Flexion
#define SWE_equilibrium 5     // State 3: Swing Extension
#define IDLE_equilibrium 5    // State 4: Idle Stance

//State Parameter Setup
//Target stiffness and damping for each state defined below.
// State 0: Early Stance
#define ES_stiffness 1.50
#define ES_damping 0.0005
// State 1: Pre-Swing Stance
#define PSW_stiffness 0.6
#define PSW_damping 0.001
// State 2: Swing Flexion
#define SWF_stiffness 0.24
#define SWF_damping 0.005
// State 3: Swing Extension
#define SWE_stiffness 0.22
#define SWE_damping 0.006
// State 4: Idle Stance
#define IDLE_stiffness 0.40
#define IDLE_damping 0.006

enum states {
        ST_EARLY_STANCE,
        ST_PRE_SWING,
        ST_SW_FLEXION,
        ST_SW_EXTENSION,
        IDLE,
};

struct st_impedance knee_st_impedance;
enum states state = IDLE;
double impedance = 0;
double desired_force = 0.0;
double desired_current = 0;
float percent = 0;
float percent_new = 0;
float percent_old = 0;  // Duty cycle percentage defined from 0 to 1.
float peak_current = 20.0;  // Current defined in amperes.
float duty_cycle_max = 0.22;
float sw_flexion_pwm_max = 0.15;
float sw_extension_pwm_max = 0.15;
float late_stance_pwm_max = 0.4;

// Constant number for appropriate time delay of early stance.

double count = 0;
float tau_friction = 1.0;
float sum_loadcell,diff_loadcell;

struct st_impedance KneeControl(float knee_angle, float knee_velocity,int16_t ac_x,float load_cell1,float load_cell2)
{
        sum_loadcell=load_cell1+load_cell2;
        diff_loadcell=load_cell2-load_cell1;
        switch (state)
        {

        // State 0: Early Stance
        case ST_EARLY_STANCE:
                if (knee_angle <= stanceflex_angle)
                {
                        state = ST_PRE_SWING;
                        break;
                }
                impedance = Impedance(knee_angle, knee_velocity,ES_stiffness,ES_damping, ES_equilibrium);
                desired_current = KneeDesiredCurrent (impedance,knee_angle);
                percent = desired_current/peak_current;
                if (percent >= duty_cycle_max)
                        percent = duty_cycle_max;
                else if(percent <= -duty_cycle_max)
                        percent = -duty_cycle_max;
                //Rate Limiter
                percent_new = RateLimiter(percent_old,percent);
                percent_old = percent_new;
                if (desired_current <= 0)
                        KneeExtension (-percent_new);
                else if (desired_current >0)
                        KneeFlexion (percent_new);
                count++;
                break;

        // State 1: Pre-Swing Stance
        case ST_PRE_SWING:
                if (diff_loadcell >= toeoff_diff)
                {
                        state = ST_SW_FLEXION;
                        break;
                }
                impedance = Impedance(knee_angle, knee_velocity, PSW_stiffness, PSW_damping, PSW_equilibrium);
                desired_current = KneeDesiredCurrent (impedance,knee_angle);
                percent = desired_current/peak_current;
                if (percent >= duty_cycle_max)
                        percent = duty_cycle_max;
                else if(percent <= -duty_cycle_max)
                        percent = -duty_cycle_max;
                //Rate Limiter
                percent_new = RateLimiter(percent_old,percent);
                percent_old = percent_new;
                if (desired_current <= 0)
                        KneeExtension (-percent_new);
                else if (desired_current >0)
                        KneeFlexion (percent_new);
                break;

        // State 2: Swing Flexion
        case ST_SW_FLEXION:
                if (knee_angle >= swingflex_angle)
                {
                        state = ST_SW_EXTENSION;
                        break;
                }
                impedance = Impedance(knee_angle, knee_velocity, SWF_stiffness, SWF_damping, SWF_equilibrium);
                desired_current = KneeDesiredCurrent (impedance,knee_angle);
                percent = desired_current/peak_current;
                if (percent >= sw_flexion_pwm_max)
                        percent = sw_flexion_pwm_max;
                else if(percent <= -sw_flexion_pwm_max)
                        percent = -sw_flexion_pwm_max;
                //Rate Limiter
                percent_new = RateLimiter(percent_old,percent);
                percent_old = percent_new;
                if (desired_current <= 0)
                        KneeExtension (-percent_new);
                else if (desired_current >0)
                        KneeFlexion (percent_new);
                break;

        // State 3: Swing Extension
        case ST_SW_EXTENSION:
                if (knee_angle <= swingext_angle)
                {
                        state = IDLE;
                        break;
                }
                impedance = Impedance(knee_angle, knee_velocity, SWE_stiffness, SWE_damping, SWE_equilibrium);
                desired_current = KneeDesiredCurrent (impedance,knee_angle);
                percent = desired_current/peak_current;
                if (percent >= sw_extension_pwm_max)
                        percent = sw_extension_pwm_max;
                else if(percent <= -sw_extension_pwm_max)
                        percent = -sw_extension_pwm_max;
                //Rate Limiter
                percent_new = RateLimiter(percent_old,percent);
                percent_old = percent_new;
                if (desired_current <= 0)
                        KneeExtension (-percent_new);
                else if (desired_current >0)
                        KneeFlexion (percent_new);
                break;

        // State 4: Idle Stance
        case IDLE:
                if (diff_loadcell <= heelstrike_diff)
                {
                        state = ST_EARLY_STANCE;
                        break;
                }
                impedance = Impedance(knee_angle, knee_velocity, IDLE_stiffness,IDLE_damping, IDLE_equilibrium);
                desired_current = KneeDesiredCurrent (impedance,knee_angle);
                percent = desired_current/peak_current;
                if (percent >= duty_cycle_max)
                        percent = duty_cycle_max;
                else if(percent <= -duty_cycle_max)
                        percent = -duty_cycle_max;
                //Rate Limiter
                percent_new = RateLimiter(percent_old,percent);
                percent_old = percent_new;
                if (desired_current <= 0)
                        KneeExtension (-percent_new);
                else if (desired_current >0)
                        KneeFlexion (percent_new);
                break;
        }

        knee_st_impedance.st = state;
        knee_st_impedance.impedance = impedance;
        knee_st_impedance.percent_new = percent_new;
        return knee_st_impedance;
}
