
 #include <string.h>
 #include <stdint.h>
 #include <limits.h>

 #include "tools.h"



typedef struct pid_mapping
{
    uint8_t reset;
    uint8_t pgm_id;
    uint8_t arg[2];
    uint16_t period_cfg;
    uint16_t period_latest;
    uint8_t enable;
    uint8_t override;
    uint16_t unused;
    float P;
    float I;
    float D;
    float speed;
    float acc;
    uint32_t sat;
    union {
        float f_measure;
        uint32_t measure;
    };
    union {
        float f_target;
        uint32_t target;
    };
    int32_t output;
} __attribute__((packed)) pid_mapping_t;


int pid_main(void* data)
{
    pid_mapping_t* regs = (pid_mapping_t*)data;

    //uint32_t ts = 0;
    uint8_t out_inverted = 0;
    uint32_t id = 0;

    uint8_t enabled = 0;
    float x = 0;
    float Kp = 6500;
    float Ki2 = 0.000;
    float x_integral = 0;
    float x_min = -15000;
    float x_max = 15000;
    float scale = 100.00;

    char chr;    

    float c;
    float v;
    int32_t vi;
    float e = 0;
    float sat = 0;
    uint8_t debug = 0;

    uint32_t ts[2];
    #define TS_UPDATE 0
    #define TS_DURATION 1

/*
    delay(10000);
    while(1){
        delay(100000000);
        if (pid->arg[0] == 0)
        {
            print_float(pid->f_measure,1);
            pid->output = (int32_t)((float)pid->f_measure*10000);
        }
        else    
        {
            print_int(pid->measure,1);
            pid->output = 0;
        }
    }
*/

    float f_target = 0;
    uint32_t target = 0;


    ts_start(&ts[TS_UPDATE]);

    for(;;) {



        //ts_start(&ts);

        if (enabled == 1)//(regs->period_cfg != 0)
        {
            if (ts_is_elapsed(ts[TS_UPDATE],(uint32_t)5000))//regs->period_cfg))
            {
                ts_start(&ts[TS_DURATION]);
                ts[TS_UPDATE] = ts[TS_DURATION];

                //id++;

                
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
                //if (id%100000 == 0)
                //    c = -1*c;

                if (regs->arg[0] == 0)
                {
                    if (debug == 2)
                    {
                        print_float(f_target,1);
                        print_float(regs->f_measure,1);
                    }
                    e = f_target - regs->f_measure;
                }
                else    
                {
                    if (debug == 2)
                    {
                        print_int((int)target,1);
                        print_int((int)regs->measure,1);
                    }

                    vi = (int32_t)((uint32_t)(target-regs->measure));
                    e = (float)vi;
                }
                //vi = 0;//int24_to_int32(0x01) - 0;



                //if (debug == 2)
                //    print_int((int)v,1);

                //e = c-v;

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


                if (out_inverted)
                    regs->output = -(int32_t)x;
                else
                    regs->output = (int32_t)x;



                ts_stop(&ts[TS_DURATION]);

                if (debug == 1)
                    print_int((int)ts,1);       
  
            }
        } else {
            ts_start(&ts[TS_UPDATE]);
            regs->output = 0;
        }


        chr = jtaguart_getchar();
        if (chr != 0)
        {
            switch (chr)
            {

                case 'r':
                    // reset the current module
                    return 0;

                case 'e':
                    enabled = 0;                    
                    x_integral = 0;
                    sat = 0;

                    jtaguart_puts("----- PID DISABLED ----\n");
                    break;

                case 'E':
                    enabled = 1;
                    if (regs->arg[0] == 0)
                    {
                        f_target=regs->f_measure;
                        print_float(f_target,1);
                    } else {
                        target=regs->measure;
                        print_int(target,1);
                    }
                    jtaguart_puts("----- PID ENABLED ----\n");
                    break;

                case 'p':
                    jtaguart_puts("----- PID Internals ----\n");
                    if (regs->arg[0] == 0)
                    {
                        print_float(f_target,1);                        
                        print_float(regs->f_measure,1);                        
                    } else {
                        print_int(target,1);                        
                        print_int(regs->measure,1); 
                    }

                    print_float(e,1);
                    print_float(x_integral,1);
                    print_float(sat,1);
                    print_float(x,1);
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


                case 't':
                    if (regs->arg[0] == 0)
                    {
                        f_target-=scale;
                        print_float(f_target,1);
                    } else {
                        target-=(uint32_t)scale;
                        print_int(target,1);
                    }
                    break;

                case 'T':
                    if (regs->arg[0] == 0)
                    {
                        f_target+=scale;
                        print_float(f_target,1);
                    } else {
                        target+=(uint32_t)scale;
                        print_int(target,1);
                    }
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
	}
}

