#include "test.h"

#pragma region[类 Clock]

// Clock::Clock(int hour, int minute, int second)
// {
// 	hour_ = hour;
// 	minute_ = minute;
// 	second_ = second;
// 	cout << "Clock::Clock" << endl;
// }
Clock::Clock(int hour, int minute, int second)
	: hour_(hour), minute_(minute), second_(second) //初始化列表
{
	cout << "Clock::Clock" << endl;
}
Clock::~Clock()
{
	cout << "Clock::~Clock" << endl;
}

void Clock::Display()
{
	cout << hour_ << " : " << minute_ << " : " << second_ << endl;
}

void Clock::Update()
{
	second_++;
	if (second_ == 60)
	{
		minute_++;
		second_ = 0;
	}
	if (minute_ == 60)
	{
		hour_++;
		minute_ = 0;
	}
	if (hour_ == 24)
	{
		hour_ = 0;
	}
}

int Clock::GetHour()
{
	return hour_;
}
int Clock::GetMinute()
{
	return minute_;
}
int Clock::GetSecond()
{
	return second_;
}

void Clock::SetHour(int hour)
{
	hour_ = hour;
}
void Clock::SetMinute(int minute)
{
	minute_ = minute;
}
void Clock::SetSecond(int second)
{
	second_ = second;
}
#pragma endregion[类 Clock]

#pragma region[类 Test]
///	实现时给出inline关键字
// inline
// inline int Test::Add(int a, int b)
// {
// 	return a + b;
// }

// void Test::Init()
// {
// 	x_ = 0;
// 	y_ = 0;
// 	z_ = 0;
// }

// void Test::Init(int x)
// {
// 	x_ = x;
// 	y_ = 0;
// 	z_ = 0;
// }

// void Test::Init(int x, int y)
// {
// 	x_ = x;
// 	y_ = y;
// 	z_ = 0;
// }

// void Test::Init(int x, int y, int z)
// {
// 	x_ = x;
// 	y_ = y;
// 	z_ = z;
// }

void Test::Init(int x, int y, int z)
{
	x_ = x;
	y_ = y;
	z_ = z;
}

void Test::Display()
{
	cout << "x_ = " << x_ << " y_ = " << y_ << " z_ = " << z_ << endl;
}

#pragma endregion[类 Test]

#pragma region[类 Test1]
/// 不带参数的构造函数被称为默认构造函数,构造函数是自动被调用的
Test1::Test1() : num_(0)
{
	// num_ = 0;
	cout << "Initializing Default" << endl;
}

Test1::Test1(int num) : num_(num)
{
	// num_ = num;
	cout << "Initializing " << num_ << endl;
}

Test1::Test1(const Test1 &other) : num_(other.num_)
{
	// num_=other.num_;
	cout << "Initializing with other" << num_ << endl;
}

void Test1::Display()
{
	cout << "num = " << num_ << endl;
}

Test1::~Test1()
{
	cout << "Destroy " << num_ << endl;
}

Test1 &Test1::operator=(const Test1 &other)
{
	num_ = other.num_;
	cout << "Test1::operator= " << endl;

	return *this;
}

#pragma endregion[类 Test1]

#pragma region[类 String]
String::String(char *str)
{
	int len = strlen(str) + 1;
	str_ = new char[len];
	memset(str_, 0, len);
	strcpy(str_, str);
}
String::~String()
{
	delete[] str_;
}
void String::Display()
{
	cout<<str_<<endl;
}
#pragma endregion[类 String]

//