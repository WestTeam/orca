
//#include <string.h>
//#include <stdint.h>
//#include <limits.h>

#include <tools.h>
#include <i2c.h>
#include <i2c_color.h>
#include <i2c_proximity.h>

volatile uint32_t* pio_n = (volatile uint32_t*)(0x01000000);


typedef struct
{
    uint8_t reset;
    uint8_t pgm_id;
    uint8_t arg[2];
    // ADC 1278 Muxed
    uint8_t  adc_drdy;  // in // 1
    uint8_t  adc_reset; // out
    uint8_t  adc_valid; // 
    uint8_t  adc_id;
    uint32_t adc_value; // 2
    // Color Sensor
    uint8_t  color_valid; // 3
    uint8_t  reserved2[3];
    uint16_t R; //4
    uint16_t G;
    uint16_t B; //5
    uint16_t C;
    // Distance sensors
    uint8_t  dist_valid; // 6
    uint8_t  dist_id;
    uint8_t  reserved3[2];
    uint32_t dist_value; // 7   
} __attribute__((packed)) low_level_mapping_t;


int main()
{
    volatile low_level_mapping_t* regs = (low_level_mapping_t*)pio_n;
    uint8_t error = 0;
    jtaguart_puts("LL Init\n");

    uint8_t buf[2];

    uint16_t R,G,B,C;


    //delay(200000000);

    i2c_init: i2c_master_configure(i2c_master_0);

    //int i;
    //buf[0] = 0xFF;
    //for (i=0;i<4;i++)
    //    i2c_master_tx(i2c_master_0,i,buf,2);

    error = i2c_color_init(i2c_master_0);


    if (error)
    {
        jtaguart_puts("color sensor: init NOT OK\n");
        goto i2c_init;
    } else {
        jtaguart_puts("color sensor: init OK\n");
    }


    // reset I2C 1
    regs->adc_reset = 0;
    delay(10000);
    regs->adc_reset = 1;

    i2c_master_configure(i2c_master_1);


    error = i2c_proximity_init(i2c_master_1);

    if (error)
    {
        jtaguart_puts("proximity sensor: init NOT OK\n");
       // goto i2c_init;
    } else {
        jtaguart_puts("proximity sensor: init OK\n");
    }




    uint8_t dist[8];
    char chr;
    uint32_t color_error_count = 0;
    uint32_t prox_error_count = 0;

    for (;;)
    {
        if (regs->adc_drdy)
        {
            error = i2c_color_get(i2c_master_0,&R,&G,&B,&C);
            regs->R = R;
            regs->G = G;
            regs->B = B;
            regs->C = C;

            if (error == 0)
            {
                regs->color_valid = 1;
            } else {
                color_error_count++;
            }
        }

        error = i2c_proximity_get(i2c_master_1,dist);
        if (error)
        {
            prox_error_count++;
            dist[0] = 0;
        }
        else
        {
            regs->dist_valid = 0;
            delay(1);
            regs->dist_id = 0;
            regs->dist_value = dist[0];
            regs->dist_valid = 1;
            delay(1);
        }
    
        if (prox_error_count > 1000)
        {
            prox_error_count = 0;
            // reset I2C 1
            regs->adc_reset = 0;
            delay(10000);
            regs->adc_reset = 1;
            i2c_proximity_init(i2c_master_1);
        }


        chr = jtaguart_getchar();
        if (chr != 0)
        {
            switch (chr)
            {
                case 'p':
                    jtaguart_puts("----- DEBUG INFO ----\n");
                    print_int(regs->adc_drdy,1);
                    print_int(color_error_count,1);
                    print_int(prox_error_count,1);
                    jtaguart_puts("G=");
                    print_int(G,0);
                    jtaguart_puts(" |R=");
                    print_int(R,0);
                    jtaguart_puts(" |B=");
                    print_int(B,0);
                    jtaguart_puts(" |C=");
                    print_int(C,0);
                    jtaguart_puts(" |Dist=");
                    print_int(dist[0],1);
                    break;
            }
        }

    }

}
