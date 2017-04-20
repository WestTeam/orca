
 #include <string.h>
 #include <stdint.h>
 #include <limits.h>

// #include "tools.h"


#define ABS(x) __fabsf(x)

void delay(int cycles);



void jtaguart_putc(char c);
void jtaguart_puts(char* s);
char jtaguart_getchar();

void uart_rs232_configure(uint16_t divisor);
void uart_rs232_tx(uint8_t data);



void ts_start(uint32_t *ts);
void ts_stop(uint32_t *ts);
uint32_t ts_freq_to_cycles(uint16_t freq_hz);
uint16_t ts_cycles_to_freq(uint32_t cycles);
uint8_t ts_is_elapsed(uint32_t ts_start, uint32_t period);





//int32_t int24_to_int32(int32_t data);



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

float __ieee754_sqrtf(float x);
float __fabsf(float x);

float __atanf(float x);

float
__ieee754_atan2f (float y, float x);
