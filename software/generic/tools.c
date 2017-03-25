
 #include <string.h>
 #include <stdint.h>
 #include <limits.h>

// #include "tools.h"


volatile int* uart = (volatile int*)(0x01000400);

inline uint32_t get_time()
{
	int val;
	asm volatile ("csrr %0,mtime":"=r"(val));
	return val;
}
void delay(int cycles)
{
	unsigned start=get_time();
	while(get_time() - start < cycles);
}



void jtaguart_putc(char c)
{

	while((uart[1]&0xffff0000) == 0){
	}//uart fifo full
	uart[0]=c;

}

void jtaguart_puts(char* s)
{
	while(*s){
		jtaguart_putc(*s++);
	}

}

char jtaguart_getchar()
{
    unsigned int reg = uart[0];
    
    if (((reg>>15)&0x1)==1)
        return (reg&0xff);
    else
        return '\0';

}



void ts_start(uint32_t *ts)
{
    *ts = get_time();
}

void ts_stop(uint32_t *ts)
{
    *ts = get_time()-*ts;
}

uint8_t ts_is_elapsed(uint32_t ts_start, uint32_t period)
{
    if ((get_time()-ts_start) >= period)
        return 1;
    else
        return 0;
}


/*
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
}*/


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
	//float y[2];
    float z=0.0;
	int32_t ix;
    //int32_t n;

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
	//float y[2];
    float z=0.0;
	int32_t ix;
    //int32_t n;

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

void *memset(void *dst, int value, size_t size)
{
    if (size)
    {
        uint8_t* d = dst;
        do
        {
            *d++=value;
        } while (--size);
    }
    return dst;
}


