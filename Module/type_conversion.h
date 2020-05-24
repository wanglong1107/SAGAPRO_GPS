#ifndef _TYPE_CONVERSION_H
#define _TYPE_CONVERSION_H

void buf2float(float *tfloat, char *buf);

void buf2float2(float *tfloat, char *buf);

void buf2long(long *tfloat, char *buf);

void buf2longlong(long long *llong, char *buf);

void buf2int(int *tint, char *buf);

void buf2double(double *tdouble, char *buf);

void float2buf(char *buf, float *tfloat);

void int2buf(char *buf, int *tint);

void ulong2buf(char *buf, unsigned long *tulong);

void longlong2buf(char *buf, long long *llong);

void double2buf(char *buf, double *tdouble);

#endif //end of file