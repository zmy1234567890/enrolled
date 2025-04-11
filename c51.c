#include <reg52.h>

#define uchar unsigned char
#define uint unsigned int

sbit dula = P2^6;
sbit wela = P2^7;
sbit key = P3^3;

uchar code table[] = {
    0x3f, 0x06, 0x5b, 0x4f,
    0x66, 0x6d, 0x7d, 0x07,
    0x7f, 0x6f, 0x77, 0x7c,
    0x39, 0x5e, 0x79, 0x71
};

uint cnt = 0;              // 计数器，单位 0.01s，最大值 5999（59.99s）
bit running = 0;           // 启动标志位

void delayms(uint ms);
void display(uint value);
void init_timer();

void main()
{
    init_timer();
    while(1)
    {
        if(key == 0)  // 检测按键按下
        {
            delayms(20);
            if(key == 0)
            {
                running = ~running; // 切换运行状态
                while(!key);        // 等待松手
            }
        }
        display(cnt);
    }
}

void init_timer()
{
    TMOD = 0x01;               // 定时器0方式1
    TH0 = (65536 - 10000) / 256;  // 10ms定时
    TL0 = (65536 - 10000) % 256;
    EA = 1;
    ET0 = 1;
    TR0 = 1;
}

void timer0_isr() interrupt 1
{
    TH0 = (65536 - 10000) / 256;
    TL0 = (65536 - 10000) % 256;

    if(running)
    {
        cnt++;
        if(cnt > 5999)
            cnt = 0;
    }
}

// value: 0~5999 显示 ss.ss 格式
void display(uint value)
{
    uchar bai, shi, ge, fen;
    uchar digits[4];

    fen = value / 1000;                // 十位秒
    bai = (value / 100) % 10;          // 个位秒
    shi = (value / 10) % 10;           // 十分位
    ge = value % 10;                   // 百分位

    digits[0] = table[fen];               // 第一位：十位秒
    digits[1] = table[bai] | 0x80;        // 第二位：个位秒 + 小数点
    digits[2] = table[shi];              // 第三位：十分位
    digits[3] = table[ge];               // 第四位：百分位

    // 逐位显示
    wela = 1; P0 = 0xFE; wela = 0; dula = 1; P0 = digits[0]; dula = 0; delayms(1); P0 = 0x00;
    wela = 1; P0 = 0xFD; wela = 0; dula = 1; P0 = digits[1]; dula = 0; delayms(1); P0 = 0x00;
    wela = 1; P0 = 0xFB; wela = 0; dula = 1; P0 = digits[2]; dula = 0; delayms(1); P0 = 0x00;
    wela = 1; P0 = 0xF7; wela = 0; dula = 1; P0 = digits[3]; dula = 0; delayms(1); P0 = 0x00;
}

void delayms(uint x)
{
    uint i, j;
    for(i = x; i > 0; i--)
        for(j = 110; j > 0; j--);
}

