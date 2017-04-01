
 #include <string.h>
 #include <stdint.h>
 #include <limits.h>

 #include "odometry.h"
 #include "pid.h"

volatile int* pio_n = (volatile int*)(0x01000000);

#define M_PI           3.14159265358979323846

typedef struct generic_mapping
{
    uint8_t reset;
    uint8_t pgm_id;
    uint8_t arg[2];
} __attribute__((packed)) generic_mapping_t;

#define PGM_ODOMETRY 0x00
#define PGM_PID 0x01
#define PGM_TRAJECTORY 0x02



int main()
{
    generic_mapping_t* generic = (generic_mapping_t*)pio_n;

    while (1)
    {
        switch (generic->pgm_id)
        {
            case PGM_ODOMETRY:
                odometry_main((void*)generic);
                break;
            case PGM_PID:
                pid_main((void*)generic);
                break;
            case PGM_TRAJECTORY:
                trajectory_main((void*)generic);
                break;        
        }
    }
}
