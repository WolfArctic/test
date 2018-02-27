#ifndef _TEST_H_
#define _TEST_H_
#include <iostream>
#include <string.h>
using namespace std;

class Clock
{
  public:
	Clock(int hour = 0, int minute = 0, int second = 0);
	~Clock();

	void Display();
	void Update();

	int GetHour();
	int GetMinute();
	int GetSecond();

	void SetHour(int hour);
	void SetMinute(int minute);
	void SetSecond(int second);

  private:
	int hour_;
	int minute_;
	int second_;
};

class Test
{
  public:
	int Add(int a, int b); //声明时可以不给出inline ，实现时给出,也可以用下面的定义方式
						   // int Add(int a, int b)
						   // {
						   // 	return a + b;
						   // }

	// void Init();
	// void Init(int x);
	// void Init(int x, int y);
	// void Init(int x, int y, int z);
	void Init(int x = 0, int y = 0, int z = 0);

	void Display();

  private:
	int x_;
	int y_;
	int z_;

  protected: //类继承的时候才用，表现在继承和派生时不一样
};

class String
{
  public:
	String(char *str = "");
	~String();
	
	void Display();

  private:
	char *str_;
};

#pragma region[前向声明类]
class B; //前向声明
class A
{
  public:
	A(void);
	~A(void);

	B *b_; //前向声明的类不能实例化对象，只能定义指针或者引用
};
class B
{
  public:
	B(void);
	~B(void);

	A a_;
};
#pragma endregion[前向声明类]

#pragma region[构造函数与析构函数]

#pragma region[构造函数]
class Test1
{
  public:
	Test1();
	~Test1(); //析构函数不能被重载，没有参数

	Test1(int num);
	// explicit Test1(int num);//这个关键字可以阻止编译器进行隐式转换
	Test1(const Test1 &other); ///拷贝构造函数，所接受的参数是对象的引用
	void Display();

	Test1 &operator=(const Test1 &other);

  private:
	int num_;
};
#pragma endregion[构造函数]

#pragma endregion[构造函数与析构函数]
#endif //_TEST_H_
