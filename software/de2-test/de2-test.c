
 #include <string.h>
 #include <stdint.h>
 #include <limits.h>

volatile int* pio_0 = (volatile int*)(0x01000000);
//volatile int* pio_1 = (volatile int*)(0x01000010);
//volatile int* pio_2 = (volatile int*)(0x01000030);
//volatile int* pio_3 = (volatile int*)(0x01000040);
volatile int* uart = (volatile int*)(0x01000400);


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

	while((uart[1]&0xffff0000) == 0){
	}//uart fifo full
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

void print_float(float f,uint8_t ret)
{
    int a = (int)f;
    int b = (int)(f*1000)-a*1000;

    if (a == 0 && b < 0)
    {
        jtaguart_putc('-');
    }
    if (b < 0)
        b = -1*b;

    print_int(a,0);
    jtaguart_putc('.');
    print_int(b/100%10,0);
    print_int(b/10%10,0);
    print_int(b%10,ret);

    
}


#define M_PI           3.14159265358979323846

typedef union
{
  float value;
  uint32_t word;
} ieee_float_shape_type;

/* Get a 32 bit int from a float.  */
#ifndef GET_FLOAT_WORD
# define GET_FLOAT_WORD(i,d)					\
do {								\
  ieee_float_shape_type gf_u;					\
  gf_u.value = (d);						\
  (i) = gf_u.word;						\
} while (0)
#endif

/* Set a float from a 32 bit int.  */
#ifndef SET_FLOAT_WORD
# define SET_FLOAT_WORD(d,i)					\
do {								\
  ieee_float_shape_type sf_u;					\
  sf_u.word = (i);						\
  (d) = sf_u.value;						\
} while (0)
#endif

static const float
one =  1.0000000000e+00, /* 0x3f800000 */
C1  =  4.1666667908e-02, /* 0x3d2aaaab */
C2  = -1.3888889225e-03, /* 0xbab60b61 */
C3  =  2.4801587642e-05, /* 0x37d00d01 */
C4  = -2.7557314297e-07, /* 0xb493f27c */
C5  =  2.0875723372e-09, /* 0x310f74f6 */
C6  = -1.1359647598e-11; /* 0xad47d74e */

float __kernel_cosf(float x, float y)
{
	float a,hz,z,r,qx;
	int32_t ix;
	GET_FLOAT_WORD(ix,x);
	ix &= 0x7fffffff;			/* ix = |x|'s high word*/
	if(ix<0x32000000) {			/* if x < 2**27 */
	    if(((int)x)==0) return one;		/* generate inexact */
	}
	z  = x*x;
	r  = z*(C1+z*(C2+z*(C3+z*(C4+z*(C5+z*C6)))));
	if(ix < 0x3e99999a) 			/* if |x| < 0.3 */
	    return one - ((float)0.5*z - (z*r - x*y));
	else {
	    if(ix > 0x3f480000) {		/* x > 0.78125 */
		qx = (float)0.28125;
	    } else {
	        SET_FLOAT_WORD(qx,ix-0x01000000);	/* x/4 */
	    }
	    hz = (float)0.5*z-qx;
	    a  = one-qx;
	    return a - (hz - (z*r-x*y));
	}
}


static const float
half =  5.0000000000e-01,/* 0x3f000000 */
S1  = -1.6666667163e-01, /* 0xbe2aaaab */
S2  =  8.3333337680e-03, /* 0x3c088889 */
S3  = -1.9841270114e-04, /* 0xb9500d01 */
S4  =  2.7557314297e-06, /* 0x3638ef1b */
S5  = -2.5050759689e-08, /* 0xb2d72f34 */
S6  =  1.5896910177e-10; /* 0x2f2ec9d3 */



float __kernel_sinf(float x, float y, int iy)
{
	float z,r,v;
	int32_t ix;
	GET_FLOAT_WORD(ix,x);
	ix &= 0x7fffffff;			/* high word of x */
	if(ix<0x32000000)			/* |x| < 2**-27 */
	  {
	    //math_check_force_underflow (x);
	    if ((int) x == 0)
	      return x;		/* generate inexact */
	  }
	z	=  x*x;
	v	=  z*x;
	r	=  S2+z*(S3+z*(S4+z*(S5+z*S6)));
	if(iy==0) return x+v*(S1+z*r);
	else      return x-((z*(half*y-v*r)-y)-v*S1);
}


float __sinf(float x)
{
	float y[2],z=0.0;
	int32_t n, ix;

	GET_FLOAT_WORD(ix,x);

    //print_int(ix,1);

    /* |x| ~< pi/4 */
	ix &= 0x7fffffff;
	if(ix <= 0x3f490fd8) return __kernel_sinf(x,z,0);

    /* sin(Inf or NaN) is NaN */
	else if (ix>=0x7f800000) {
	  //if (ix == 0x7f800000)
	    //__set_errno (EDOM);
	  return x-x;
	}

    /* argument reduction needed */
	else {
        //PI/4   = 1061752795
        //PI/2   = 1070141403
        //3*PI/4 = 1075235812
        //PI     = 1078530011
        if (ix < 1070141403)
        {
            return __kernel_cosf(M_PI/2-x,0.0);
        } 
        else if (ix < 1075235812)
        {
            return __kernel_cosf(x-M_PI/2,0.0);
        }
        else
        {
            return __kernel_sinf(M_PI-x,z,0);
        }       
        /*
	    n = __ieee754_rem_pio2f(x,y);
	    switch(n&3) {
		case 0: return  __kernel_sinf(y[0],y[1],1);
		case 1: return  __kernel_cosf(y[0],y[1]);
		case 2: return -__kernel_sinf(y[0],y[1],1);
		default:
			return -__kernel_cosf(y[0],y[1]); 
	    } */
	}
}

float __cosf(float x)
{
	float y[2],z=0.0;
	int32_t n,ix;

	GET_FLOAT_WORD(ix,x);

    /* |x| ~< pi/4 */
	ix &= 0x7fffffff;
	if(ix <= 0x3f490fd8) return __kernel_cosf(x,z);

    /* cos(Inf or NaN) is NaN */
	else if (ix>=0x7f800000) {
	  //if (ix == 0x7f800000)
	    //__set_errno (EDOM);
	  return x-x;
	}

    /* argument reduction needed */
	else {
        //PI/4   = 1061752795
        //PI/2   = 1070141403
        //3*PI/4 = 1075235812
        //PI     = 1078530011
        if (ix < 1070141403)
        {
            return __kernel_sinf(M_PI/2-x,0.0,0);
        } 
        else if (ix < 1075235812)
        {
            return __kernel_sinf(x-M_PI/2,0.0,0);
        }
        else
        {
            return __kernel_cosf(M_PI-x,0.0);
        }       


        /*
	    n = 0;//__ieee754_rem_pio2f(x,y);
	    switch(n&3) {
		case 0: return  __kernel_cosf(y[0],y[1]);
		case 1: return -__kernel_sinf(y[0],y[1],1);
		case 2: return -__kernel_cosf(y[0],y[1]);
		default:
		        return  __kernel_sinf(y[0],y[1],1);
	    } */
	}
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

    delay(50000000*5);
/*
    __sinf(M_PI/4-0.0001);
    __sinf(M_PI/4);
    __sinf(M_PI/4+0.0001);

    __sinf(M_PI/2-0.0001);
    __sinf(M_PI/2);
    __sinf(M_PI/2+0.0001);

    __sinf(3*M_PI/4-0.0001);
    __sinf(3*M_PI/4);
    __sinf(3*M_PI/4+0.0001);

    __sinf(M_PI-0.0001);
    __sinf(M_PI);
    __sinf(M_PI+0.0001);

    jtaguart_puts("end\n");
    

    ts_start(&ts);
    for (i=0;i<3141;i++)
    {
        float f;
        f = __cosf(0.001*(float)i);
        print_float(f*1000.0);
        //delay(5000);
    }    
    ts_stop(&ts);
    print_int((int)ts,1);    
*/

    
    uint16_t qei0;
    uint16_t qei1;

    uint16_t qei0_last = 0;
    uint16_t qei1_last = 0;

    int16_t qei0_diff;
    int16_t qei1_diff;

    float entrax_cm = 27.4;
    float left_cm_per_tick  = 32.0*M_PI/1024.0/10.0; //32
    float right_cm_per_tick = 32.0*M_PI/1024.0/10.0; //32.1

    //float x;
    float y;
    float a_rad = 0;
    int16_t a_deg;


    #define DEG(x) ((x) * (180.0 / M_PI))

    // odometry
    for(;;) {



        pio_in = pio_0[0];
        
        qei0 = pio_in & 0xFFFF;
        qei1 = (pio_in >> 16) & 0xFFFF;

        if (qei0_last != qei0 || qei1_last != qei1)
        {
            ts_start(&ts);

            qei0_diff = qei0-qei0_last;
            qei1_diff = -(qei1-qei1_last);

            qei0_last = qei0;
            qei1_last = qei1;

            /*
            print_int(qei0_diff,0);
            jtaguart_putc(':');
            print_int(qei1_diff,1);
            */

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
