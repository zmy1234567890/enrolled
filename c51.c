#include <reg52.h>
#define uchar unsigned char
#define uint unsigned int

sbit dula = P2^6;
sbit wela = P2^7;
sbit led1 = P1^0;
sbit key = P3^3;

uchar code table[] = {
    0x3f,0x06,0x5b,0x4f,
    0x66,0x6d,0x7d,0x07,
    0x7f,0x6f,0x77,0x7c,
    0x39,0x5e,0x79,0x71
};

uchar num1 = 0, num2 = 0, shi = 0, ge = 0, num = 0;

void delayms(uint xms)
{
    uint i, j;
    for(i = xms; i > 0; i--)
        for(j = 110; j > 0; j--);
}

void display(uchar shi, uchar ge)
{
    dula = 1;
    P0 = table[shi];
    dula = 0;

    wela = 1;
    P0 = 0xfe;
    wela = 0;
    delayms(5);
    P0 = 0x00;  // 清除避免残影

    dula = 1;
    P0 = table[ge];
    dula = 0;

    wela = 1;
    P0 = 0xfd;
    wela = 0;
    delayms(5);
    P0 = 0x00;
}

void main()
{
    TMOD = 0x11;

    TH0 = (65536 - 48872) / 256;
    TL0 = (65536 - 48872) % 256;
    TH1 = (65536 - 48872) / 256;
    TL1 = (65536 - 48872) % 256;

    EA = 1;
    ET0 = 1;
    ET1 = 1;

    TR0 = 0; // 初始不启动定时器
    TR1 = 0;

    while(1)
    {
        if(key == 0)
        {
            delayms(20);
            if(key == 0)
            {
                TR0 = 1;
                TR1 = 1;
                while(!key);  // 等待松手
            }
        }

        display(shi, ge);
    }
}

void T0_time(void) interrupt 1 using 1
{
    TH0 = (65536 - 48872) / 256;
    TL0 = (65536 - 48872) % 256;
    num1++;
    if(num1 == 4)
    {
        num1 = 0;
        led1 = ~led1;
    }
}

void T1_time(void) interrupt 3 using 2
{
    TH1 = (65536 - 48872) / 256;
    TL1 = (65536 - 48872) % 256;
    num2++;
    if(num2 == 60)  // 大约1秒
    {
        num2 = 0;
        num++;
        if(num == 60)
            num = 0;

        shi = num / 10;
        ge = num % 10;
    }
}
