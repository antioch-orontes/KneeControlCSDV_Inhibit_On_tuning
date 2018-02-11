

#include <stdint.h>        /* Includes uint16_t definition                    */

struct st_impedance
{
        int st;
        float impedance;
        float percent_new;
};
struct st_impedance KneeControl(float knee_angle, float knee_velocity,int16_t ac_x, float load_cell1, float load_cell2);
