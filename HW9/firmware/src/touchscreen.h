#ifndef _TOUCHSCREEN_H    /* Guard against multiple inclusion */
#define _TOUCHSCREEN_H

void XPT2046_read(unsigned short *x, unsigned short *y, unsigned int *z);

#endif 

