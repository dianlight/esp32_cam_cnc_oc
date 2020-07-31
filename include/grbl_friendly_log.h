#pragma once
#include <stdio.h>
//#include <string.h>

#define MAX_STRING_LENGTH   255 

int grbl_friendly_logging_vprintf(const char *str, va_list l)
{
    static bool first = true;
    int16_t cc = 0;
    char nout[MAX_STRING_LENGTH];

    int16_t tot = vsnprintf(nout,MAX_STRING_LENGTH,str,l);

    for (u8_t i = 0; i < strlen(nout); i++)
    {
        if (first)
        {
            putchar('(');
            cc++;
            first = false;
        }
        switch (nout[i])
        {
        case '(':
            putchar('[');
            cc++;
            break;
        case ')':
            putchar(']');
            cc++;
            break;
        case 0x0D:
            break;
        case 0x0A:
            putchar(')');
//            putchar('\n');
            putchar(nout[i]);
            cc+=3;
            first = true;
            break;
        default:
            putchar(nout[i]);
            cc++;
            break;
        }
    }

    if(tot > MAX_STRING_LENGTH-1){
        putchar(')');
        putchar('\n');
        cc+=2;
    }

    return cc;
}