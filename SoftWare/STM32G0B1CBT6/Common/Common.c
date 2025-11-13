#include "Common.h"
#include <ctype.h>

void StrNum_split(char* arr, char* Chars, char* Number) 
{
	int i, j = 0, k = 0;
	for (i = 0; i < strlen(arr); i++) 
	{
		if (isdigit(arr[i])) 
		{
			Number[j++] = arr[i];
		} 
		else if (isalpha(arr[i])) 
		{
			Chars[k++] = arr[i];
		}
	}
	Number[j] = '\0';
	Chars[k] = '\0';
}

float bytefloat(unsigned char a)
{
    unsigned int l;
    float f;

    l = a;
    f = *((float *)(&l));

    return f;
}

float byte1float(unsigned char a, unsigned char b)
{

    unsigned int l;
    float f;

    l = a;
    l &= 0xff;
    l |= ((long) b << 8);
    f = *((float *)(&l));

    return f;
}

float byte2float(unsigned char a, unsigned char b, unsigned char c, unsigned char d)
{

    unsigned int l;
    float f;

    l = a;
    l &= 0xff;
    l |= ((long) b << 8);
    l &= 0xffff;
    l |= ((long) c << 16);
    l &= 0xffffff;
    l |= ((long) d << 24);
    f = *((float *)(&l));

    return f;
}

long byte1int(unsigned char  a, unsigned char  b)
{
    long l;

    l = a;
    l &= 0xff;
    l |= ((long) b << 8);

    return l;
}

long byte2int(unsigned char  a, unsigned char  b, unsigned char  c, unsigned char  d)
{
    long l;

    l = a;
    l &= 0xff;
    l |= ((long) b << 8);
    l &= 0xffff;
    l |= ((long) c << 16);
    l &= 0xffffff;
    l |= ((long) d << 24);

    return l;
}
