#include <stdio.h>

int main(int argc, char *argv[])
{
    unsigned char p[4] = {0xc1, 0xe6, 0x02, 0x43};
    printf("%02x %02x %02x %02x\n", p[3], p[2], p[1], p[0]);
    float *pfloat = (float*)p;
    printf("%f\n", *pfloat);
    return 0;
}
