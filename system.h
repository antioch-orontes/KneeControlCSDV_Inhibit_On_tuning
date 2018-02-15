/******************************************************************************/
/* System Level #define Macros                                                */
/******************************************************************************/

/* TODO Define system operating frequency */

/* Microcontroller MIPs (FCY) [FCY = F_OSC / 2] */
// #define SYS_FREQ        7370000L
#define SYS_FREQ        79840000LL // 79.84Mhz
#define FCY             SYS_FREQ/2  //39.92Mhz

/******************************************************************************/
/* System Function Prototypes                                                 */
/******************************************************************************/

/* Custom oscillator configuration funtions, reset source evaluation
   functions, and other non-peripheral microcontroller initialization functions
   go here. */

void ConfigureOscillator(void); /* Handles clock switching/osc initialization */
