#include <stdio.h>

int main(int argc, char *argv[])
{
    float input;
    scanf("%f", &input);
    char *pchar = (char*)&input;
    char byte[4];
    for (unsigned int i=0; i<sizeof(float); i++) {
        byte[i] = *pchar;
        pchar++;
    }
    printf("%02x %02x %02x %02x\n", byte[3], byte[2], byte[1], byte[0]);
    return 0;
}
