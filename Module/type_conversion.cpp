#include "type_conversion.h"

void buf2float2(float *tfloat, char *buf)
{
    int i;
    char * p1 = (char *)tfloat;
    char * p3 = buf+3;

    for(i=0; i<4; i++)
    {
        *p1 = *p3;
        p1++;
        p3--;
    }
}

void buf2float(float *tfloat, char *buf)
{
    int i;
    char * p1 = (char *)tfloat;
    char * p3 = buf;

    for(i=0; i<4; i++)
    {
        *p1 = *p3;
        p1++;
        p3++;
    }
}

void buf2long(long *tfloat, char *buf)
{
    int i;
    char * p1 = (char *)tfloat;
    char * p3 = buf;

    for(i=0; i<4; i++)
    {
        *p1 = *p3;
        p1++;
        p3++;
    }
}

void buf2longlong(long long *llong, char *buf)
{
    int i;
    char * p1 = (char *)llong;
    char * p3 = buf;

    for(i=0; i<8; i++)
    {
        *p1 = *p3;
        p1++;
        p3++;
    }
}

void buf2double(double *tdouble, char *buf)
{
    int i;
    char * p1 = (char *)tdouble;
    char * p3 = buf;

    for(i=0; i<8; i++)
    {
        *p1 = *p3;
        p1++;
        p3++;
    }
}

void buf2int(int *tint, char *buf)
{
    int i;
    char * p1 = (char *)tint;
    char * p3 = buf;

    for(i=0; i<4; i++)
    {
        *p1 = *p3;
        p1++;
        p3++;
    }
}

void float2buf(char *buf,float *tfloat)
{
    int i;
    char * p1 = (char *)tfloat;
    char * p3 = buf;

    for(i=0; i<4; i++)
    {
        *p3 = *p1;
        p1++;
        p3++;
    }
}

void int2buf(char *buf,int *tint)
{
    int i;
    char * p1 = (char *)tint;
    char * p3 = buf;

    for(i=0; i<4; i++)
    {
        *p3 = *p1;
        p1++;
        p3++;
    }
}

void ulong2buf(char *buf,unsigned long *tulong)
{
    int i;
    char * p1 = (char *)tulong;
    char * p3 = buf;

    for(i=0; i<4; i++)
    {
        *p3 = *p1;
        p1++;
        p3++;
    }
}

void longlong2buf(char *buf, long long *llong)
{
    int i;
    char * p1 = (char *)llong;
    char * p3 = buf;

    for(i=0; i<8; i++)
    {
        *p3 = *p1;
        p1++;
        p3++;
    }
}

void double2buf(char *buf, double *tdouble)
{
    int i;
    char * p1 = (char *)tdouble;
    char * p3 = buf;

    for(i=0; i<8; i++)
    {
        *p3 = *p1;
        p1++;
        p3++;
    }
}