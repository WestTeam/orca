
 #include <string.h>
 #include <stdint.h>
 #include <limits.h>

// #include "tools.h"




void delay(int cycles);



void jtaguart_putc(char c);

void jtaguart_puts(char* s);


char jtaguart_getchar();



void ts_start(uint32_t *ts);


void ts_stop(uint32_t *ts);



uint8_t pio_0_get_data(int32_t *data);



int32_t int24_to_int32(int32_t data);



/*
*
 * limit(x, min, max) does the following:
 * if x is between min and max, return x
 * if x < min, return min
 * if x > max, return max
--
*/
float limit(float x, float min, float max) ;




/*

-- satlimit(x, min, max) does the following:
 * if x is between min and max, return (x,0)
 * if x < min, return (min, -1)
 * if x > max, return (max, +1)
 */

void satlimit(float x, float min, float max, float* x_integral,float* sat);


 
size_t strlen(const char *s);



 /* reverse:  reverse string s in place */
 void reverse(char s[]);





// Implementation of itoa()
char* itoa(int num, char* str, int base);





void print_int(int i,uint8_t ret);

void print_float(float f,uint8_t ret);



#define M_PI           3.14159265358979323846

float __kernel_cosf(float x, float y);



float __kernel_sinf(float x, float y, int iy);


float __sinf(float x);


float __cosf(float x);



