/*******************************************************
   Name: Knee Prothesis (right) Can Bus v. 2.26.2
   Date: 02/13/2018
   Created by: Md Rejwanul Haque
   Latest revisor: Cosmo Chou
   Comments: Cleaned code and updated state transition variables.
   Next revision: Address torque discontinuities. Eliminate unnecessary code.
 *****************************************************/
/* Device header file */
#include <p33FJ128MC802.h>
#if defined(__XC16__)
#include <xc.h>
#elif defined(__C30__)
#if defined(__dsPIC33E__)
#include <p33Exxxx.h>
#elif defined(__dsPIC33F__)
#include <p33Fxxxx.h>
#endif
#endif
#include <stdint.h>  // Includes uint16_t definition
#include <stdbool.h> // Includes true/false definition
#include <stdio.h>
#include "system.h" // System funct/params, like osc/peripheral config
#include <libpic30.h>
#include "AnalogIO.h"      // ADC + DAC I/O Library
#include <float.h>         // Floating point math definition
#include "StateFormulas.h" // Contains methods and variables for each walking state 1-4
#include "i2c_emem.h"
#include "MPU6050.h"
#include <math.h>
#include "libas.h"
#include "pwm.h"
#include "KneeControl.h" //Includes the knee walking controller
#include "Globals.h"     //Include Globals.h here only, Externals.h in all other files.
#define gear_ratio 150
#define torque_const 36.9
/*
   Functions that are defined in other .c files.
   The first word o the function name matches the name of the .c file.
   EX:  InitOsc() is in Init.c)
 */
extern void InitOsc(void);
extern void TimersInit(void);
extern void InitDma3(void);
extern void InitDma2(void);
extern void InitCAN1(void);
extern void CanSend(UINT16 ID);
extern INT16 CanParse(void);
extern void CANSendAll(float flt_data_1, float flt_data_2, float flt_data_3, int int_data_2);
// Functions located in Main.c
INT16 main(void);
void IdleTasks(void);
// This is the start up osc.
_FOSCSEL(FNOSC_FRCPLL &IESO_ON);
// Clock Switching is enabled and Fail Safe Clock Monitor is disabled
// OSC2 Pin Function: OSC2 is GPIO
// Primary Oscillator Mode: Internal RC Osc
_FOSC(FCKSM_CSECME &OSCIOFNC_OFF &POSCMD_NONE);
_FICD(ICS_PGD1 &JTAGEN_OFF);
_FWDT(FWDTEN_OFF); // Watchdog Timer disabled
INT16 main(void)
{
        UINT16 ledMask;
        ledMask = 0;
        InitOsc(); //functions starting with "Init" are in Init.c
        InitCAN1();
        InitDma3();
        InitDma2();
        TimersInit(); //Functions starting with "Timers" are in Timers.c
        /* Set register bits that control global interrupts.
           See section 7 of dsPIC datasheet */
        INTCON1bits.NSTDIS = 0; // Disable nested interrupts
        SRbits.IPL = 0;         // Set interrupt priority, see section 7.3
        double u1, y1, LoadCell1, LoadCell2, sumloadcell, diffloadcell;
        struct imu_data imuBase;
        float current = 0;
        double measured_torque = 0;
        extern I2CEMEM_DRV i2cmem;
        float a = 0.85; //parameter for digital filter
        int16_t AcX1 = 0;
        int st; // State number
        struct st_impedance my_st_impedance;
        double desired_torque;
        double percent = 0;
        double duty_cycle = 0;
        bool odd = true;
        float angle = 0, angle_old = 0, knee_velocity;
        double angular_velocity = 0;
        double old_angular_velocity = 0;
        double fc = 10;
        double tau = 1 / (2 * 3.14 * fc); //0.0159
        double T = 0.001;                 // Cycle time when frequency has been fixed to 1000Hz
        // Configure the oscillator for the device.
        ConfigureOscillator(); // Must be called at beginning of main routine
        ConfigureAnalog();     // Must be called before while loop to use ReadAnalogSample(...)
        InitEncoder(9, 7, 12); //pin 9 (RA2) as ouput CLK; pin 12 (RA4) as ouput CS ;pin 7 RB3/AN5 as input DI
        init_DIR();            // Initialize the Direction Pin
        init_PWM();            // Initialize the PWM module
        Init_Inhibit();
        __delay_ms(2000);
        Open_Inhibit();
        y1 = Samp2Voltage(ReadAnalogSample(0)); //read voltage from channel 0 (AN0)  is the motor current
        angle_old = GetKneeAngle();
        LoadCell1 = Samp2Voltage(ReadAnalogSample(2));
        LoadCell2 = Samp2Voltage(ReadAnalogSample(4));
        __delay_ms(100);
        IEC0bits.T1IE = 1; // Enable Timer1 interrupt
        double knee_v_filter = 0;
        float t_old = 0;
        float t_now = 0;
        float dt = 2e-3; // control cycle time in us
        while (1)
        {
                angle = GetKneeAngle(); // New range -6.45 to 97.89
                /*Digital Practical Differentiator for Angular Velocity Measurement*/
                angular_velocity = (2 * (angle - angle_old) + (2 * tau - T) * old_angular_velocity) / (T + 2 * tau);
                old_angular_velocity = angular_velocity;
                angle_old = angle;
                /* current monitor */
                u1 = Samp2Voltage(ReadAnalogSample(0)); //read voltage from channel 0 (AN0) PIN2
                y1 = a * y1 + (1 - a) * u1;             //0.855 0.145
                current = 6.4 * y1;                     // current monitor scaling = 6.4 A/V
                /* Read Load Cells*/
                LoadCell1 = ReadAnalogSample(2);
                LoadCell2 = ReadAnalogSample(4);
                sumloadcell = LoadCell1 + LoadCell2;
                diffloadcell = LoadCell2 - LoadCell1;
                my_st_impedance = KneeControl(angle, angular_velocity, AcX1, LoadCell1, LoadCell2);
                st = my_st_impedance.st;
                desired_torque = -my_st_impedance.impedance;
                percent = my_st_impedance.percent_new;
                measured_torque = 0.9 * current * gear_ratio * torque_const / 1000; //torque constant = 36.9 mNm/A
                duty_cycle = 100 * percent;
                CANSendAll(angle, desired_torque, LoadCell1, LoadCell2);
                //CANSendAll(float flt_data_1, float flt_data_2, float flt_data_3, int int_data_2);
                /*range of data: CANSendAll(flt_data_1= -3276.7 to 3276.7, flt_data_2 = -3276.7 to 3276.7, int_data_1 = 0 to 65535,int_data_2 = 0 to 65535)*/
                IdleTasks(); //asynchronous tasks go here (receiving can messages)
        }
}
void IdleTasks(void)
{ //perform tasks that do not need to execute at a particular rate here
        //example parsing CAN messages.
        while (gTimers[MAINTIMER] > 0)
        {
                CanParse();
                //LATAbits.LATA3 = 0;  //set pin 10 as low to see how long MCU is idle
        }
        gTimers[MAINTIMER] = 1; //reset the timer flag for next time slice.
}
