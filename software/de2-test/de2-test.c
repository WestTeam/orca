
 #include <string.h>
 #include <stdint.h>
 #include <limits.h>

volatile int* pio_0 = (volatile int*)(0x01000020);
volatile int* pio_1 = (volatile int*)(0x01000010);
volatile int* pio_2 = (volatile int*)(0x01000030);
volatile int* pio_3 = (volatile int*)(0x01000040);
volatile int* uart = (volatile int*)(0x01000070);


inline unsigned get_time()
{
	int val;
	asm volatile ("csrr %0,mtime":"=r"(val));
	return val;
}
inline void delay(int cycles)
{
	unsigned start=get_time();
	while(get_time() - start < cycles);
}



inline void jtaguart_putc(char c)
{

	//while((uart[1]&0xffff0000) == 0){
	//}//uart fifo full
	uart[0]=c;

}
inline void jtaguart_puts(char* s)
{
	while(*s){
		jtaguart_putc(*s++);
	}

}

inline char jtaguart_getchar()
{
    unsigned int reg = uart[0];
    
    if ((reg>>15)&0x1==1)
        return (reg&0xff);
    else
        return '\0';

}



inline void ts_start(unsigned int *ts)
{
    *ts = get_time();
}

inline void ts_stop(unsigned int *ts)
{
    *ts = get_time()-*ts;
}


uint8_t pio_0_get_data(int32_t *data)
{
    static uint8_t state = 0;

    int32_t pio_in = pio_0[0];
    if (((pio_in >> 31) & 0x1) != state)
    {
        state = (~state) & 0x1;
        *data = pio_in & 0x7fffffff;
        return 1;
    }
    return 0;    
}


int32_t int24_to_int32(int32_t data)
{
    int32_t ret;
    if (((data >> 23)&0x1) == 0)
    {
        ret = data & 0x00ffffff;
    }
    else
    {
        data &= 0x007fffff;
        ret = 0xff800000 | (data);
    }    
    return ret;
}


/*
*
 * limit(x, min, max) does the following:
 * if x is between min and max, return x
 * if x < min, return min
 * if x > max, return max
--
*/
float limit(float x, float min, float max) 
{
    if (x < min) //(x_integral >= x_min && x_integral <= x_max)
    {
        return min;
    }
    else if (x > max)
    {
        return max;
    }
    return x;
}



/*

-- satlimit(x, min, max) does the following:
 * if x is between min and max, return (x,0)
 * if x < min, return (min, -1)
 * if x > max, return (max, +1)
 */

void satlimit(float x, float min, float max, float* x_integral,float* sat) 
{
    if (x < min) //(x_integral >= x_min && x_integral <= x_max)
    {
        *x_integral = min;
        *sat = -1; 
    }
    else if (x > max)
    {
        *x_integral = max;
        *sat = 1; 
    }
    else
    {
        *x_integral = x;
        *sat = 0; 
    }
}

 
size_t strlen(const char *s) {
    size_t i;
    for (i = 0; s[i] != '\0'; i++) ;
    return i;
}


 /* reverse:  reverse string s in place */
 void reverse(char s[])
 {
     int i, j;
     char c;
 
     for (i = 0, j = strlen(s)-1; i<j; i++, j--) {
         c = s[i];
         s[i] = s[j];
         s[j] = c;
     }
 }





// Implementation of itoa()
char* itoa(int num, char* str, int base)
{
    int i = 0;
    unsigned char isNegative = 0;
 
    /* Handle 0 explicitely, otherwise empty string is printed for 0 */
    if (num == 0)
    {
        str[i++] = '0';
        str[i] = '\0';
        return str;
    }
 
    // In standard itoa(), negative numbers are handled only with 
    // base 10. Otherwise numbers are considered unsigned.
    if (num < 0 && base == 10)
    {
        isNegative = 1;
        num = -num;
    }
 
    // Process individual digits
    while (num != 0 && i < 10)
    {
        int rem = num % base;
        str[i++] = (rem > 9)? (rem-10) + 'a' : rem + '0';
        num = num/base;
    }
 
    // If number is negative, append '-'
    if (isNegative)
        str[i++] = '-';
 
    str[i] = '\0'; // Append string terminator
 
    // Reverse the string
    reverse(str);
 
    return str;
}




void print_int(int i,uint8_t ret)
{
    char buf[32];

    itoa(i,buf,10);
    int s = strlen(buf);
    if (ret)
        buf[s] = '\n';
    buf[s+1] = '\0';
    jtaguart_puts(buf);
}

void print_float(float f)
{
    int a = (int)f;
    print_int(a,0);
    jtaguart_putc('.');
    int b = (int)(f*1000)-a*1000;
    print_int(b/100%10,0);
    print_int(b/10%10,0);
    print_int(b%10,1);

    
}


int main()
{
    uint32_t i = 0;
    uint32_t id = 0;
    int32_t  out = 0;

    char chr;    

    float c;
    float v;
    float e;
    float sat = 0;
    float x = 0;
    float Kp = 0.01;
    float Ki2 = 0.001;
    float x_integral = 0;
    float x_min = -65535;
    float x_max = 65535;
    float scale = 0.01;

    uint8_t debug = 0;
    int32_t pio_in;
    int32_t vi;
    int32_t vi_offset = 0;

    uint32_t ts = 0;
    char buf[32];
    uint8_t out_inverted = 0;
    c=30000;


    // init (search ADC offset)
    *pio_0 = 0;

    int32_t sum = 0;
    #define INIT_SAMPLE_COUNT (1<<18)
    for (i=0;i<INIT_SAMPLE_COUNT;i++)
    {
        if (pio_0_get_data(&pio_in))
        {
            sum += int24_to_int32(pio_in);
        }
    }
    vi_offset = sum / INIT_SAMPLE_COUNT;

    print_int(vi_offset,1);

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
                    print_float(scale);
                    break;
                case 'S':
                    scale*=10.0;
                    print_float(scale);
                    break; 

                case 'k':
                    Kp-=scale;
                    print_float(Kp);
                    break;
                case 'K':
                    Kp+=scale;
                    print_float(Kp);
                    break; 


                case 'i':
                    Ki2-=scale;
                    print_float(Ki2);
                    break;
                case 'I':
                    Ki2+=scale;
                    print_float(Ki2);
                    break; 


                case 'c':
                    c-=scale;
                    print_float(c);
                    break;

                case 'C':
                    c+=scale;
                    print_float(c);
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

        if (pio_0_get_data(&pio_in))
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

            vi = int24_to_int32(pio_in) - vi_offset;

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


            if (out_inverted)
                *pio_0 = -(int)x;
            else
                *pio_0 = (int)x;                


            ts_stop(&ts);

            if (debug == 1)
                print_int((int)ts,1);       

        }      

	}
}
