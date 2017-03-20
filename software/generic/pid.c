
 #include <string.h>
 #include <stdint.h>
 #include <limits.h>

 #include "tools.h"

#define M_PI           3.14159265358979323846

typedef struct pid_mapping
{
    uint32_t cfg0;
    uint32_t cfg1;
} __attribute__((packed)) pid_mapping_t;


int pid_main(void* data)
{
    pid_mapping_t* pid = (pid_mapping_t*)data;

    uint32_t ts = 0;
    uint8_t out_inverted = 0;
    uint32_t id = 0;

    float x = 0;
    float Kp = 0.01;
    float Ki2 = 0.001;
    float x_integral = 0;
    float x_min = -65535;
    float x_max = 65535;
    float scale = 0.01;

    char chr;    

    float c;
    float v;
    int32_t vi;
    float e;
    float sat = 0;
    uint8_t debug = 0;


    for(;;) {


        chr = jtaguart_getchar();
        if (chr != 0)
        {
            switch (chr)
            {

                case 'd':
                    if (debug)
                        debug--;
                    break;

                case 'D':
                    debug++;
                    break;

                case 's':
                    scale/=10.0;
                    print_float(scale,1);
                    break;
                case 'S':
                    scale*=10.0;
                    print_float(scale,1);
                    break; 

                case 'k':
                    Kp-=scale;
                    print_float(Kp,1);
                    break;
                case 'K':
                    Kp+=scale;
                    print_float(Kp,1);
                    break; 


                case 'i':
                    Ki2-=scale;
                    print_float(Ki2,1);
                    break;
                case 'I':
                    Ki2+=scale;
                    print_float(Ki2,1);
                    break; 


                case 'c':
                    c-=scale;
                    print_float(c,1);
                    break;

                case 'C':
                    c+=scale;
                    print_float(c,1);
                    break;

                case 'o':
                    out_inverted=~out_inverted;
                    if (out_inverted)
                        jtaguart_puts("Output inverted\n");               
                    else
                        jtaguart_puts("Output normal\n");                                   
                    break;

            }
        }


        ts_start(&ts);

        if (1)//(pio_0_get_data(&pio_in))
        {


            id++;

            
/*
if ((sat < 0 && e < 0) || (sat > 0 && e > 0))
 ; 
-- do nothing if there is saturation, and error is in the same direction;
-- if you're careful you can implement as "if (sat*e > 0)"
--
else
 x_integral = x_integral + Ki2*e;
(x_integral,sat) = satlimit(x_integral, x_minimum, x_maximum);
x = limit(Kp*e + x_integral, x_minimum, x_maximum)


*/
            if (id%100000 == 0)
                c = -1*c;

            if (debug == 2)
                print_int((int)c,1);

            vi = int24_to_int32(0x01) - 0;

            v = (float)vi;

            if (debug == 2)
                print_int((int)v,1);

            e = c-v;

            if (debug == 2)
                print_int((int)e,1);

            if (sat*e > 0)
            {

            }
            else
            {
                x_integral = x_integral + Ki2*e;
                satlimit(x_integral,x_min,x_max, &x_integral,&sat); 
            }

 
            if (debug == 2)
                print_int((int)x_integral,1);
           
            x = limit(Kp*e+x_integral,x_min,x_max);

            if (debug == 2)
                print_int((int)x,1);


            //if (out_inverted)
            //    *pio_0 = -(int)x;
            //else
            //    *pio_0 = (int)x;                


            ts_stop(&ts);

            if (debug == 1)
                print_int((int)ts,1);       

        }      

	}
}
