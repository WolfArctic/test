#ifndef _LOG_H_
#define _LOG_H_

typedef struct
{
std::string m_name;//变量名
double m_value;//值, 具体的值需要根据类型去申请空间
std::string m_type;//类型
}Log_s;

#endif