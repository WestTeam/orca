
 #include <string.h>
 #include <stdint.h>
 #include <limits.h>

 #include "tools.h"

#define M_PI           3.14159265358979323846

#define DEG(x) ((x) * (180.0 / M_PI))

typedef struct odo_mapping
{
    uint8_t reset;
    uint8_t pgm_id;
    uint8_t arg[2];
    float c_wheel_axe_mm;
    float c_wheel_mm_per_tick[2];
    float m_wheel_axe_mm;
    float m_wheel_mm_per_tick[2];
    uint16_t period_cfg;
    uint16_t period_latest;
    union {
        uint16_t c_qei_value[2];
        float odo_sum_c_distance;
    };
    union {
        uint16_t m_qei_value[2];
        float odo_sum_c_angle;
    };
    float odo_sum_m_distance;
    float odo_sum_m_angle;
    uint8_t odo_pos_valid;
    uint8_t odo_pos_id;
    int16_t odo_pos_teta;
    uint16_t odo_pos_x;
    uint16_t odo_pos_y;
} __attribute__((packed)) odo_mapping_t;


#define LEFT 0
#define RIGHT 1

typedef struct wheel_config
{
    float axe_mm;
    float axe_mm_inv; // to avoid live division
    float mm_per_tick[2]; // used to compute quickly the distance
} wheel_config_t;


typedef struct odo_state
{
    // INPUT
    uint16_t qei_latest[2];

    // INTERNALS
    int32_t  qei_sum[2]; // sum from the init
    int32_t  qei_sum_latest[2]; // latest processed value for position

    // CONFIGS
    wheel_config_t wheel;

    // OUTPUT RAW
    float dist_sum;
    float angle_sum;

} odo_state_t;

typedef struct odo_data
{
/*
    // INPUT
    uint16_t c_qei_latest[2];
    uint16_t m_qei_latest[2];

    // INTERNALS
    int32_t  c_qei_sum[2]; // sum from the init
    int32_t  m_qei_sum[2];
    int32_t  c_qei_sum_latest[2]; // latest processed value for position
    int32_t  m_qei_sum_latest[2];

    // CONFIGS
    wheel_config_t c_wheel;
    wheel_config_t m_wheel;

    // OUTPUT RAW
    float c_dist_sum;
    float m_dist_sum;

    float c_angle_sum;
    float m_angle_sum;
*/
    odo_state_t cs;
    odo_state_t ms;


    
    // OUTPUT PROCESSED
    float x;
    float y;
    float teta_rad; // modulo PI
    int16_t teta_deg; // (1 = 0.1deg)
    uint8_t id;


} odo_data_t;


void odometry_init(odo_mapping_t* regs, odo_data_t* data)
{
    // init structures
    memset((void*)data,0,sizeof(*data));
    memset((void*)regs,0,sizeof(*regs));

    // read current QEI
    data->ms.qei_latest[0] = regs->m_qei_value[0];
    data->ms.qei_latest[1] = regs->m_qei_value[1];

    data->cs.qei_latest[0] = regs->c_qei_value[0];
    data->cs.qei_latest[1] = regs->c_qei_value[1];

    data->ms.wheel.axe_mm = 274.0;
    data->ms.wheel.axe_mm_inv = 1.0/data->ms.wheel.axe_mm;
    data->ms.wheel.mm_per_tick[0] = 1.08*72.0*M_PI/(1024.0*728.0/45.0);
    data->ms.wheel.mm_per_tick[1] = 1.08*72.0*M_PI/(1024.0*728.0/45.0);

    data->cs.wheel.axe_mm = 234.0;
    data->cs.wheel.axe_mm_inv = 1.0/data->cs.wheel.axe_mm;
    data->cs.wheel.mm_per_tick[0] = 32.0*M_PI/(1024.0);
    data->cs.wheel.mm_per_tick[1] = 32.0*M_PI/(1024.0);
 
}

void odo_update_state(odo_state_t* state,uint16_t* regs_qei, float* regs_sum_dist, float *regs_sum_angle)
{
    uint16_t qei[2];

    qei[0] = regs_qei[0];
    qei[1] = regs_qei[1];
    if (qei[0] != state->qei_latest[0] || qei[1] != state->qei_latest[1])
    {
        int32_t diff[2];
        // compute diff on each
        diff[0] = -(int32_t)(((int16_t)(qei[0]-state->qei_latest[0])));
        diff[1] = (int32_t)(((int16_t)(qei[1]-state->qei_latest[1])));

        // save current value for next computation
        state->qei_latest[0] = qei[0];
        state->qei_latest[1] = qei[1];

        // update the total sum on each side
        state->qei_sum[0] += (int32_t)diff[0];
        state->qei_sum[1] += (int32_t)diff[1];

        // update SUM distance
        state->dist_sum += ((float)diff[0]*state->wheel.mm_per_tick[0] + (float)diff[1]*state->wheel.mm_per_tick[1])*0.5;

        // output the value
        *regs_sum_dist = state->dist_sum;

        // update SUM angle
        state->angle_sum = ((float)state->qei_sum[1]*state->wheel.mm_per_tick[1]-(float)state->qei_sum[0]*state->wheel.mm_per_tick[0])*state->wheel.axe_mm_inv;

        // output the value
        *regs_sum_angle = state->angle_sum;
    } 

}

float rad_mod_pi(float teta)
{
    float ret = teta;
    while (ret < -M_PI)
        ret += (M_PI*2);

    while (ret > M_PI)
        ret -= (M_PI*2);

    return ret;
}


int odometry_main(void* data)
{
    odo_mapping_t* regs = (odo_mapping_t*)data;
    odo_data_t odo;

    uint32_t ts[2];
    #define TS_UPDATE 0
    #define TS_DURATION 1

    uint32_t period_latest = 0;

    odometry_init(regs,&odo);

    print_float(__ieee754_atan2f(__atanf(__ieee754_sqrtf(__fabsf(__cosf(-M_PI/2)))),2.0),1);
    print_float(__cosf(-M_PI/4),1);
    print_float(__cosf(-3.0*M_PI/4.0),1);
    print_float(__cosf(-M_PI),1);

    print_float(__sinf(-M_PI/2),1);
    print_float(__sinf(-M_PI/4),1);
    print_float(__sinf(-3.0*M_PI/4.0),1);
    print_float(__sinf(-M_PI),1);


    ts_start(&ts[TS_UPDATE]);

    // odometry v2;
    for(;;) {

        // update motor encoders 
        odo_update_state(&odo.ms,&regs->m_qei_value[0],&regs->odo_sum_m_distance,&regs->odo_sum_m_angle);
        // update coding wheel encoders
        odo_update_state(&odo.cs,&regs->c_qei_value[0],&regs->odo_sum_c_distance,&regs->odo_sum_c_angle);


        // check if timer is elapsed to run the position update
        if (1)//(regs->period_cfg != 0)
        {
            if (ts_is_elapsed(ts[TS_UPDATE],(uint32_t)500000))//regs->period_cfg))
            {


/*
 - sl (diff): delta distance parcourue roue gauche
 - sr (diff): delta distance parcourue roue droite
 - alpha = teta(i-1) + (sr-sl)/(2L)
 - d = (sr+sl)/2
 - x(i) = x(i-1) + d.cos(alpha)
 - y(i) = y(i-1) + d.sin(alpha)
 - teta(i) = (sum(sr) - sum(sl))/L
*/
                if (odo.cs.qei_sum[0] != odo.cs.qei_sum_latest[0] || odo.cs.qei_sum[1] != odo.cs.qei_sum_latest[1])
                {
                    ts_start(&ts[TS_DURATION]);
                    ts[TS_UPDATE] = ts[TS_DURATION];
                                    
                    // processing

                    int32_t diff[2];
                    // compute diff on each
                    diff[0] = odo.cs.qei_sum[0]-odo.cs.qei_sum_latest[0];
                    diff[1] = odo.cs.qei_sum[1]-odo.cs.qei_sum_latest[1];

                    float dist;
                    float alpha;

                    dist = ((float)diff[0]*odo.cs.wheel.mm_per_tick[0] + (float)diff[1]*odo.cs.wheel.mm_per_tick[1])*0.5;
                    alpha = odo.teta_rad + (dist)*odo.cs.wheel.axe_mm_inv;
                    alpha = rad_mod_pi(alpha);

                    odo.x += dist * __cosf(alpha);
                    odo.y += dist * __sinf(alpha);

                    odo.teta_rad = rad_mod_pi(odo.cs.angle_sum);
                    odo.teta_deg = (int16_t)(DEG(odo.teta_rad)*10.0);

                    regs->odo_pos_valid = 0;
                    regs->odo_pos_id = odo.id++;
                    regs->odo_pos_teta = odo.teta_deg;
                    regs->odo_pos_x = (int16_t)odo.x;
                    regs->odo_pos_y = (int16_t)odo.y;
                    regs->odo_pos_valid = 1;
                    
                    odo.cs.qei_sum_latest[0] = odo.cs.qei_sum[0];
                    odo.cs.qei_sum_latest[1] = odo.cs.qei_sum[1];


                    ts_stop(&ts[TS_DURATION]);
                    period_latest = (uint16_t)ts[TS_DURATION];
                    regs->period_latest = period_latest;
                }

                // check config changes (if any)
            }
            
        } else {
            ts_start(&ts[TS_UPDATE]);
        }


        // DEBUG
        char chr;

        chr = jtaguart_getchar();
        if (chr != 0)
        {
            switch (chr)
            {

                case 'p':
                    jtaguart_puts("----- Motor  Wheel ----\n");
                    jtaguart_puts("QEI VALUE:\n");
                    print_int((int32_t)odo.ms.qei_latest[0],1);
                    print_int((int32_t)odo.ms.qei_latest[1],1);
                    jtaguart_puts("QEI M SUM:\n");
                    print_int((int32_t)odo.ms.qei_sum[0],1);
                    print_int((int32_t)odo.ms.qei_sum[1],1);
                    jtaguart_puts("Distance:\n");
                    print_float(odo.ms.dist_sum,1);
                    jtaguart_puts("Angle:\n");
                    print_float(odo.ms.angle_sum,1);
                    jtaguart_puts("----- Coding Wheel ----\n");
                    jtaguart_puts("QEI VALUE:\n");
                    print_int((int32_t)odo.cs.qei_latest[0],1);
                    print_int((int32_t)odo.cs.qei_latest[1],1);
                    jtaguart_puts("QEI M SUM:\n");
                    print_int((int32_t)odo.cs.qei_sum[0],1);
                    print_int((int32_t)odo.cs.qei_sum[1],1);
                    jtaguart_puts("Distance:\n");
                    print_float(odo.cs.dist_sum,1);
                    jtaguart_puts("Angle:\n");
                    print_float(odo.cs.angle_sum,1);
                    jtaguart_puts("--------- POS --------\n");
                    jtaguart_puts("X/Y/Angle(rad)/Angle(Deg):\n");
                    print_float(odo.x,1);
                    print_float(odo.y,1);
                    print_float(odo.teta_rad,1);
                    print_float((float)odo.teta_deg/10.0,1);
                    jtaguart_puts("Period:\n");
                    print_int((int32_t)period_latest,1);
                    break;

                case 'r':
                    return 0;

            }
        }        


//        print_int(regs->m_qei_value[0],1);
//        print_int(regs->m_qei_value[1],1);
//        print_int(regs->c_qei_value[0],1);
//        print_int(regs->c_qei_value[1],1);

/*

        qei0 = regs->c_qei_value[0];
        qei1 = regs->c_qei_value[1];

        if (qei0_last != qei0 || qei1_last != qei1)
        {

            ts_start(&ts);

            jtaguart_puts("Odo Regs:");
            print_float(regs->m_wheel_axe_mm,1);
            print_float(regs->m_wheel_mm_per_tick[0],1);
            print_float(regs->m_wheel_mm_per_tick[1],1);

            print_float(regs->c_wheel_axe_mm,1);
            print_float(regs->c_wheel_mm_per_tick[0],1);
            print_float(regs->c_wheel_mm_per_tick[1],1);

            //print_int(regs->m_qei_value[0],1);
            //print_int(regs->m_qei_value[1],1);
            //print_int(regs->c_qei_value[0],1);
            //print_int(regs->c_qei_value[1],1);


            left_cm_per_tick  = regs->c_wheel_mm_per_tick[0]*M_PI/(1024.0*728.0/45.0)/10.0; //32
            right_cm_per_tick = regs->c_wheel_mm_per_tick[1]*M_PI/(1024.0*728.0/45.0)/10.0;
            entrax_cm = regs->c_wheel_axe_mm/10.0;

            qei0_diff = qei0-qei0_last;
            qei1_diff = -(qei1-qei1_last);

            qei0_last = qei0;
            qei1_last = qei1;

            print_float(regs->c_wheel_axe_mm,1);
            print_float(regs->c_wheel_mm_per_tick[0],1);
            print_float(regs->c_wheel_mm_per_tick[1],1);

            
            print_int(qei0_diff,0);
            jtaguart_putc(':');
            print_int(qei1_diff,1);
            

            {
	            float diff_left_cm = (float)qei0_diff*left_cm_per_tick;
	            float diff_right_cm = (float)qei1_diff*right_cm_per_tick;

                float distance = (diff_left_cm+diff_right_cm)/2.0;
                float angle    = (diff_right_cm-diff_left_cm)/entrax_cm;

		        a_rad+=angle;
		        x = x + __cosf(a_rad) * (distance) ;
		        y = y + __sinf(a_rad) * (distance) ;

	            if (a_rad < -M_PI)
		            a_rad += (M_PI*2);
	            else if (a_rad > (M_PI))
		            a_rad -= (M_PI*2);

                a_deg = (int16_t)(DEG(a_rad));

                regs->odo_pos_valid = 0;
                regs->odo_pos_id = id_pos++;
                regs->odo_pos_teta = a_deg;
                regs->odo_pos_x = (int16_t)x;
                regs->odo_pos_y = (int16_t)y;
                regs->odo_pos_valid = 1;

                ts_stop(&ts);
                print_int((int)ts,1);    
                
                print_float(x,0);
                jtaguart_putc(':');
                print_float(y,0);
                jtaguart_putc(':');
                print_float(a_rad,0);
                jtaguart_putc(':');
                print_int(a_deg,1);                

            }
        }*/
    }

    return 0;
}
