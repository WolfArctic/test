#include "fuzzycontrol.h"

fuzzycontroller::fuzzycontroller()
{
    e_upper = 9.5;
    //e_lower = -9.5;
    e_lower = -19.5;
    de_upper = 0.65;
    //de_lower = -0.65;
    de_lower = -0.95;
    u_upper = 1.5;
    //u_lower = -2.5; //-5
    u_lower = -8; 
    accel_plus = 0.15;
    accel_minus = -0.15;
    accel_mode = 0;
    accel_mode_last = 0;
}

fuzzycontroller::~fuzzycontroller()
{

}

//实现模糊控制
double fuzzycontroller::realize(double e, double de, double accel,double relspeed)   // relspeed :e  reldist % :de
{
    // double fuzzycontroller::realize(double e,double de) {  // relspeed :e  reldist % :de
    if (e >= e_upper)
    {
        e = e_upper;
    }
    if (e <= e_lower)
    {
        e = e_lower;
    }
    if (de >= de_upper)
    {
        de = de_upper;
    }
    if (de <= de_lower)
    {
        de  = de_lower;
    }
    double u_e[N], u_de[N], u_u[N];
    double u;
    int M = 3; // "trimf" 三角函数有三个参数
    Ke = 1;
    Kde = 10;
    Ku = 1;
    e = Ke * e;
    de = Kde * de;

    for(int i = 0; i < N; i++)
    {
        u_e[i] = trimf(e, e_mf_paras[i * M], e_mf_paras[i * M + 1], e_mf_paras[i * M + 2]);//e模糊化，计算它的隶属度
        cout << FBLU("i") << i <<  FBLU("u_e") <<  u_e[i] << endl;
    }

    for(int i = 0; i < N; i++)
    {
        u_de[i] = trimf(de, de_mf_paras[i * M], de_mf_paras[i * M + 1], de_mf_paras[i * M + 2]);//de模糊化，计算它的隶属度                                                 //存储被激活的模糊子集的下标，可以减小计算量
        cout << FBLU("i") << i <<  FBLU("u_de") <<  u_de[i] << endl;
    }

    double den = 0;
    double num = 0;
    // double accel = 0;
    if (accel >= accel_plus) {
        accel_mode = 1;
    }
    else if (accel <= accel_minus) {
      accel_mode = -1;
    }
    else if (accel_mode_last == 1 && accel < (accel_plus - 0.15)) {
      accel_mode = 0;
    }
    else if (accel_mode_last == -1 && accel > (accel_minus + 0.15)) {
      accel_mode = 0;
    }
     

    accel_mode_last = accel_mode;

    if (accel_mode == 1 && fabs(relspeed) < 1.0)
    {
        for(int m = 0; m < N; m++)
        {
            for(int n = 0; n < N; n++)
            {
                num += u_e[m] * u_de[n] * ruleMatrix_plus[m][n];
                den += u_e[m] * u_de[n];
            }
        }
    }
    else if (accel_mode == -1 && fabs(relspeed) < 1.0)
    {
        for(int m = 0; m < N; m++)
        {
            for(int n = 0; n < N; n++)
            {
                num += u_e[m] * u_de[n] * ruleMatrix_minus[m][n];
                den += u_e[m] * u_de[n];
            }
        }
    }
    else
    {
        for(int m = 0; m < N; m++)
        {
            for(int n = 0; n < N; n++)
            {
                num += u_e[m] * u_de[n] * ruleMatrix[m][n];
                den += u_e[m] * u_de[n];
            }
        }
    }

    cout << FRED("num") << num << endl;
    cout << FRED("den") << den << endl;
    u = num / den;
    cout << "u_before" << u << endl;

    if(u <= NFB)
    {
        cout << "NFB" << endl;
        u = u_mf_paras[0];
    }
    else if(u <= NHB)
    {
        cout << "NHB" << endl;
        u = u_mf_paras[3] + (u - NFB) / (NHB - NFB) * (u_mf_paras[4] - u_mf_paras[3]);
    }
    else if(u <= NXB)
    {
        cout << "NXB" << endl;
        u = u_mf_paras[6] + (u - NHB) / (NXB - NHB) * (u_mf_paras[7] - u_mf_paras[6]);
    }
    else if(u <= NLB)
    {
        cout << "NLB" << endl;
        u = u_mf_paras[9] + (u - NXB) / (NLB - NXB) * (u_mf_paras[10] - u_mf_paras[9]);
    }
    else if(u <= NMB)
    {
        cout << "NMB" << endl;
        u = u_mf_paras[12] + (u - NLB) / (NMB - NLB) * (u_mf_paras[13] - u_mf_paras[10]);
    }
    else if(u <= NSB)
    {
        cout << "NSB" << endl;
        u = u_mf_paras[15] + (u - NMB) / (NSB - NMB) * (u_mf_paras[16] - u_mf_paras[15]);
    }
    else if(u <= NVB)
    {
        cout << "NVB" << endl;
        u = u_mf_paras[18] + (u - NSB) / (NVB - NSB) * (u_mf_paras[19] - u_mf_paras[18]);
    }
    else if (u <= NB)
    {
        cout << "NB" << endl;
        u = u_mf_paras[21] + (u - NVB) / (NB - NVB) * (u_mf_paras[22] - u_mf_paras[21]);
    }
    else if (u <= NM)
    {
        cout << "NM" << endl;
        u = u_mf_paras[24] + (u - NB) / (NM - NB) * (u_mf_paras[25] - u_mf_paras[24]);
    }
    else if (u <= NS)
    {
        u = u_mf_paras[27] + (u - NM) / (NS - NM) * (u_mf_paras[28] - u_mf_paras[27]);
        cout << "NS" << endl;
        // cout << "$$$$$$$$$$$$" << (NS - u)/(NS - NM)*(u_mf_paras[10] - u_mf_paras[9]) << endl;
        // cout << "$$$$$$$$$$$$" << u_mf_paras[9] + (NS - u)/(NS - NM)*(u_mf_paras[10] - u_mf_paras[9]) << endl;
        // cout << "$$$$$$$$$$$$" << u << endl;
    }
    else if (u <= ZO)
    {
        u = u_mf_paras[30] + (u - NS) / (ZO - NS) * (u_mf_paras[31] - u_mf_paras[30]);
        cout << "ZO" << endl;

    }
    else if (u <= PS)
    {
        u = u_mf_paras[33] + (u - ZO) / (PS - ZO) * (u_mf_paras[34] - u_mf_paras[33]);
        cout << "PS" << endl;
    }
    else if (u <= PM)
    {
        u = u_mf_paras[36] + (u - PS) / (PM - PS) * (u_mf_paras[37] - u_mf_paras[36]);
        cout << "PM" << endl;
    }
    else if (u <= PB)
    {
        u = u_mf_paras[39] + (u - PM) / (PB - PM) * (u_mf_paras[40] - u_mf_paras[39]);
        cout << "PB" << endl;
    }
    else if (u <= PVB)
    {
        u = u_mf_paras[42] + (u - PB) / (PVB - PB) * (u_mf_paras[43] - u_mf_paras[42]);
        cout << "PVB01" << endl;
    }
    else if (u > PVB)
    {
        u = u_mf_paras[44];
        cout << "PVB" << endl;
    }
    cout << "u_upper" << u_upper << endl;
    cout << "u_lower" << u_lower << endl;
    if(u >= u_upper)
    {
        u = u_upper;
    }
    else if(u <= u_lower)
    {
        u = u_lower;
    }
    cout << "u" << u << endl;
    cout << "$$$$$$$$$$$$$$$$$$$$$$$" << endl;
    return u;
}

//三角隶属度函数
double fuzzycontroller::trimf(double x, double a, double b, double c)
{

    double u;

    if (x > a && x <= b)
    {
        u = (x - a) / (b - a);
    }
    else if (x > b && x <= c)
    {
        u = (c - x) / (c - b);
    }
    else
        u = 0.0;

    return u;
}
//正态隶属度函数
double fuzzycontroller::gaussmf(double x, double ave, double sigma)
{
    double u;

    if(sigma < 0)
    {
        cout << "In gaussmf, sigma must larger than 0" << endl;
    }
    u = exp(-pow(((x - ave) / sigma), 2));

    return u;
}
//梯形隶属度函数
double fuzzycontroller::trapmf(double x, double a, double b, double c, double d)
{
    double u;

    if(x >= a && x < b)
    {
        u = (x - a) / (b - a);
    }
    else if(x >= b && x < c)
    {
        u = 1;
    }
    else if(x >= c && x <= d)
    {
        u = (d - x) / (d - c);
    }
    else
    {
        u = 0;
    }

    return u;
}