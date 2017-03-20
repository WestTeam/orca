
 #include <string.h>
 #include <stdint.h>
 #include <limits.h>

 #include "tools.h"

#define M_PI           3.14159265358979323846

typedef struct odo_mapping
{
    uint32_t cfg0;
    uint32_t cfg1;
    float m_wheel_axe_mm;
    float m_wheel_diam_mm_left;
    float m_wheel_diam_mm_rigth;
    float c_wheel_axe_mm;
    float c_wheel_diam_mm_left;
    float c_wheel_diam_mm_rigth;
    uint16_t period_cfg;
    uint16_t period_latest;
    union {
        uint16_t m_qei_value[2];
        float odo_sum_m_distance;
    };
    union {
        uint16_t c_qei_value[2];
        float odo_sum_m_angle;
    };
    float odo_sum_c_distance;
    float odo_sum_c_angle;
    uint8_t odo_pos_valid;
    uint8_t odo_pos_id;
    uint16_t odo_pos_teta;
    uint16_t odo_pos_x;
    uint16_t odo_pos_y;
} __attribute__((packed)) odo_mapping_t;


    int odometry_main(void* data)
{
    odo_mapping_t* odo = (odo_mapping_t*)data;

    uint32_t ts = 0;

    uint16_t qei0;
    uint16_t qei1;

    uint16_t qei0_last = 0;
    uint16_t qei1_last = 0;

    int16_t qei0_diff;
    int16_t qei1_diff;

    float entrax_cm;// = 27.4;
    float left_cm_per_tick;//  = 32.0*M_PI/1024.0/10.0; //32
    float right_cm_per_tick;// = 32.0*M_PI/1024.0/10.0; //32.1

    float x = 0;
    float y = 0;
    float a_rad = 0;
    int16_t a_deg;

    uint8_t id_pos = 0;

    #define DEG(x) ((x) * (180.0 / M_PI))

    // odometry v2;
    for(;;) {

//        print_int(odo->m_qei_value[0],1);
//        print_int(odo->m_qei_value[1],1);
//        print_int(odo->c_qei_value[0],1);
//        print_int(odo->c_qei_value[1],1);

        qei0 = odo->c_qei_value[0];
        qei1 = odo->c_qei_value[1];

        if (qei0_last != qei0 || qei1_last != qei1)
        {

            ts_start(&ts);

            jtaguart_puts("Odo Regs:");
            print_float(odo->m_wheel_axe_mm,1);
            print_float(odo->m_wheel_diam_mm_left,1);
            print_float(odo->m_wheel_diam_mm_rigth,1);

            print_float(odo->c_wheel_axe_mm,1);
            print_float(odo->c_wheel_diam_mm_left,1);
            print_float(odo->c_wheel_diam_mm_rigth,1);

            //print_int(odo->m_qei_value[0],1);
            //print_int(odo->m_qei_value[1],1);
            //print_int(odo->c_qei_value[0],1);
            //print_int(odo->c_qei_value[1],1);


            left_cm_per_tick  = odo->c_wheel_diam_mm_left*M_PI/(1024.0*728.0/45.0)/10.0; //32
            right_cm_per_tick = odo->c_wheel_diam_mm_rigth*M_PI/(1024.0*728.0/45.0)/10.0;
            entrax_cm = odo->c_wheel_axe_mm/10.0;

            qei0_diff = qei0-qei0_last;
            qei1_diff = -(qei1-qei1_last);

            qei0_last = qei0;
            qei1_last = qei1;

            print_float(odo->c_wheel_axe_mm,1);
            print_float(odo->c_wheel_diam_mm_left,1);
            print_float(odo->c_wheel_diam_mm_rigth,1);

            
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

                odo->odo_pos_valid = 0;
                odo->odo_pos_id = id_pos++;
                odo->odo_pos_teta = a_deg;
                odo->odo_pos_x = (int16_t)x;
                odo->odo_pos_y = (int16_t)y;
                odo->odo_pos_valid = 1;

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
        }
    }

}
