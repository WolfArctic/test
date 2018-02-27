#include "test.h"
#include <stdio.h>

#pragma region[First]

#pragma region[bool 类型的学习]
// int main()
// {
// 	Test t;
// 	t.print();

// 	//bool 类型的学习
// 	// bool result;
// 	// result = true;
// 	// result = 100;
// 	// cout<<"result = "<<result<<endl;
// 	return 0;
// }
#pragma endregion[bool类型的学习]

#pragma region[const 类型的学习]
// #define STR(a) #a
// #define CAT(a,b) a##b
// int main()
// {
// 	//const 的学习
// 	//一般形式
// 	//const 数据类型 常量名 = 常量值
// 	//数据类型 const 常量名 = 常量值
// 	//注意：定义时必须初始化，初始化以后不能再被赋值
// 	// const float PI = 3.1415926f;
// 	// int const a = 100;

// 	// int b = 22;
// 	// const int * p;//const在*号左边表示*p是一个常量,即内容是一个常量,经由*p不能更改指针所指向的内容
// 	// p = &b;
// 	//*p = 200;  Error，常量不能被重新赋值

// 	//int * const p2;//Error const 在*号右边，表示p2是一个常量，必须初始化
// 	// int * const p2 = &b;
// 	// int c = 100;
// 	// //p2 = &c; Error,不能重新指向了
// 	// *p2 = 200;

// 	//高层次的编程尽量用const enum inline替换#define
// 	//在底层编程#define是很灵活的
// 	// int xy = 100;
// 	// cout<<STR(ABCD)<<endl;//#ABCD <==> "ABCD"
// 	// cout<<CAT(x,y)<<endl;//x##y <==> xy
// 	return 0;
// }
#pragma endregion[const类型的学习]

#pragma region[struct 对齐的学习]
//结构体对齐的规则
//第一个成员与结构体变量的偏移量为0
//其他成员要对齐到某个数的整数倍地址
//某个数称之为对齐数，其取值于编译器预设的一个对齐整数与该成员大小的较小值
//结构体总大小为最大对齐数的整数倍；
// #pragma pack(8)//更改编译器默认的对齐数
// struct test
// {
// 	char a;
// 	double b;
// 	char c;
// };
// #pragma pack()
// int main()
// {
// 	//结构体对齐和内存对齐
// 	// test tt;
// 	// char *p = (char*)&tt;
// 	// printf("p = %p\n",p);
// 	// p = &tt.a;
// 	// printf("p = %p\n",p);
// 	// cout<<sizeof(test)<<endl;//求结构体大小
// 	return 0;
// }
#pragma endregion[结构体对齐的学习]

#pragma region[域运算符、new、delete 运算]
// int var = 100;
// int main()
// {
// 	//域运算符、new、delete运算符、重载、name managling与extern “C”、带默认参数的函数
// 	//域运算符 :: 其作用是对与局部变量同名的全局变量进行访问
// 	int var = 200;
// 	cout<<"var = "<<var<<endl;
// 	cout<<"var = "<<::var<<endl;

// 	//new 运算符可以用于创建堆空间 ，等价于c语言中的malloc，成功返回首地址，失败直接抛出异常
// 	//语法：
// 	//指针变量 = new 数据类型
// 	//指针变量 = new 数据类型[长度]
// 	int* p = new int(33);//分配一个整数空间4个字节 ,并初始化为33
// 	cout<<*p<<endl;

// 	int* p2 = new int[10];//分配连续的10个整数空间40个字节，不能在此初始化，可以用for

// 	delete p;
// 	delete[] p2;
// }
#pragma endregion[域运算符、new、delete 运算符]

#pragma region[overload 重载]
// //函数重载又称为函数的多态性
// //如果返回类型不同而函数名相同、形参也相同的话，是非法重载
// void fun(int a, int b)
// {
// 	cout<<" int fun "<<endl;
// }
// void fun(double a, double b)
// {
// 	cout<<" double fun "<<endl;
// }
// int main()
// {
// 	fun(3,4);
// 	fun(3.3,4.0);
// 	return 0;
// }
#pragma endregion[overload 重载]

#pragma region[name managling 名字改编]
// int main()
// {
// 	return 0;
// }
#pragma endregion[]

#pragma region[extern "C"实现混合编程]
//extern "C"表示不进行名字改编
//如下可以c和C++都能使用
// #ifdef _CPLUSCPLUS_
// extern "C"
// {
// #endif
// 	void fun();
// 	void fun2();
// #ifdef _CPLUSCPLUS_
// }
// #endif
// int main()
// {
// 	return 0;
// }
#pragma endregion[]

#pragma region[带默认形参值的函数]
//默认值的定义必须从右到左
//调用时从左到右进行匹配
//注意不要产生二义性
// int fun(int a, int b = 5)
// {
// 	return a + b;
// }

// int fun(int a, int b, int c);

// int main()
// {
// 	cout << fun(3) << endl;
// 	cout << fun(3, 4) << endl;
// 	return 0;
// }

// int fun(int a, int b, int c)
// {
// 	return a + b + c;
// }
#pragma endregion[]

#pragma region[引用 &]
/*引用是给一个变量起别名,引用没有独立的空间，要与所引用的变量共享空间
因此对引用所做的改变就是对其所引用的变量做的改变
格式：类型 &引用名 = 变量名 例如 int a = 1;int &b = a;
b是a的别名，因此a和b是同一个单元
注意引用时一定要初始化，指明该引用变量是谁的别名,另外引用一经初始化就不能重新指向其他变量
在实际应用中，引用一般用作参数传递与返回值*/

// int main()
// {
// 	int val = 100;
// 	// int &reval;  Error 引用必须初始化
// 	int &reval = val;
// 	cout << "val 1 = " << val << endl;
// 	reval = 200; //实际上改变的是val的值
// 	cout << "val 2 = " << val << endl;
// 	cout << "reval = " << reval << endl;

// 	int val2 = 500;
// 	// int &reval=val2; Error
// 	reval = val2; //Right 但不代表将reval引用至val2这个变量
// 	cout << "reval = " << reval << endl;
// 	return 0;
// }
#pragma endregion[引用 &]

#pragma region[引用const]
/*const 引用是指向const对象的引用
例如：
const int ival = 1024;
const int &refval = ival; ok :both reference and object are const
int &ref2 = ival; 		Error: nonconst reference to a const object
*/
// int main()
// {
// 	const int val = 1024;
// 	const int &refval = val;
// 	// int &ref2 = val; Error
// 	// refval = 200; Error 常量不能被赋值

// 	int val2 = 1024;
// 	const int &ref3 = val2; //const 可以引用至非const类型变量，当然引用后不能被修改了
// 	// ref3 = 300; Error

// 	double val3 = 3.14;
// 	const int &ref4 = val3;//等价于 int temp = val3;const int &ref4 = temp;
// 	cout << "ref4 = " << ref4 << endl;
// 	cout << "val3 = " << val3 << endl;
// 	// int &ref5 = val3;非const不会有上面的两步等价
// 	return 0;
// }
#pragma endregion[引用const]

#pragma region[引用作为参数传递]
/*引用传递方式是在函数定义时在形参前面加上引用运算符”&“ 例如 ： swap(int &a,int &b);
按值传递方式容易理解，但是形参值的改变不能对实参产生影响
地址传递方式通过形参的改变使相应的实参改变，但程序容易产生错误且难以阅读
引用作为参数对形参的任何操作都能改变相应的实参数据，又使函数调用显得方便、自然
*/
/*回顾：参数传递
值传递 ： 形参不能改变实参
指针（地址）传递：形参可以改变实参
*/
// void swap(int &x, int &y)
// {
// 	int temp;
// 	temp = x;
// 	x = y;
// 	y = temp;
// }

// int main()
// {
// 	int a, b;
// 	a = 20;
// 	b = 10;
// 	cout << "a = " << a << " b = " << b << endl;
// 	swap(a, b);//相当于在函数调用时引用被初始化 x = a，y = b
// 	cout << "a = " << a << " b = " << b << endl;
// 	return 0;
// }
#pragma endregion[引用作为参数传递]

#pragma region[引用作为函数返回值]
/*引用的另一个作用是用于返回引用的函数
函数返回引用的一个主要目的是可以将函数放在赋值运算符的左边
注意：不能返回对局部变量的引用。
*/
// int a[] = {0, 1, 2, 3, 4};
// int &index(int i)
// {
// 	return a[i]; //a是全局变量，因此可以使用函数返回引用
// }

// int &add(int a, int b)
// {
// 	int sum;
// 	sum = a + b;
// 	return sum;//局部变量在函数返回时就销毁了
// }

// int main()
// {
// 	index(3) = 100; //引用作为函数返回值，使得函数可以放在赋值运算符的左边  //函数返回引用是在函数返回时被初始化为a[3]
// 	cout << "a[3] = " << a[3] << endl;

// 	int n = add(3, 4);
// 	int &n2 = add(5, 6);//n2是引用没有独立的空间，依赖于它所引用的变量，如果n2所引用的变量生命期结束了，那么也就是说n2的值不确定
// 	cout << "n2 = " << n2 << endl;
// 	cout << "n = " << n << endl;
// 	cout << "n2 = " << n2 << endl;
// 	return 0;
// }
#pragma endregion[引用作为函数返回值]

#pragma region[引用与指针的区别]
/*引用访问一个变量是直接访问，而指针是间接访问
引用是一个变量别名，本身不单独分配自己的内存空间，而指针有着自己的内存空间
引用一经初始化不能再引用其他变量，而指针可以
尽可能使用引用，不得已时使用指针
*/
/*值传递    实参要初始化形参要分配空间，将实参内容拷贝到形参
引用传递    实参初始化形参时不分配空间
指针传递    本质是值传递，要分配空间的，如果修改指针的地址，单纯用指针传递也是不行的 要用指针的指针** 或者用指针的引用*&
*/
#pragma endregion[引用与指针的区别]

#pragma region[内联函数]
/*关键字 inline
 当程序执行函数调用时，为了提高效率应当将代码很短，又频繁调用的函数声明为内联函数
*/
// inline int max(int a, int b)
// {
// 	return a > b ? a : b;
// }
// int main()
// {
// 	return 0;
// }
#pragma endregion[内联函数]

#pragma region[内联函数与带参数宏的区别]
/*
 内联函数调用时要求实参和形参的类型一致，另外内联函数会先对实参表达式进行求值，然后传递给形参；
 而宏调用时只用实参简单地替换形参。
 内联函数是在编译的时候、在调用的地方将代码展开的，而宏则是在预处理时进行替换的
 在C++中建议采用inline函数来替换带参数的宏
*/
/*
 宏有两个功能
 常量						C++建议用const、enum替换
 带参数的宏（类似于函数调用） C++建议用inline替换
 C++高层次编程建议用const、enum、inline替换宏
 低层次编程宏很灵活
*/

// #define MAX(a, b) (a) > (b) ? (a) : (b)
// inline int max(int a, int b)
// {
// 	return a > b ? a : b;
// }
// int main()
// {
// 	int a = 8, b = 9;
// 	MAX(a, b);
// 	return 0;
// }

#pragma endregion[内联函数与带参数宏的区别]

#pragma region[新的类型转换运算符]
/*旧式转型
 (T)expr
 T(expr)
*/
/*新式转型
  const_cast<T>(expr)
  static_cast<T>(expr)
  reinterpret_cast<T>(expr)
  dynamic_cast<T>(expr)执行”安全向下“转型操作，也就是说支持运行时识别指针或所指向的对象，这是唯一无法用旧式语法来进行的转型操作
  尽可能避免使用强制转换（显示转换）
  如果无法避免，推荐使用新式类型转换
*/
/*const_cast<T>(expr)
 用来移除对象的常量性（cast away the constness）
 一般用于指针或者引用,不能用于对象
 使用const_cast去除const限定的目的不是为了修改它的内容，通常是为了函数能够接受这个实际参数
 */
/*static_cast<T>(expr)
 编译器隐式执行的任何类型转换都可以由static_cast完成
 当一个较大的算术类型赋值给较小的类型时，可以用static_cast进行强制转换
 可以将void*指针转换为某一类型的指针
 可以将基类指针指向派生类指针
 无法将const转换为nonconst，这个只有const_cast才可以办得到

 编译器的隐式转换由编译器自动完成，是安全的 如将int a；short b； a=b;
 */
/*reinterpret_cast<T>(expr)
 通常为操作数的位模式提供较低层的重新解释，也就是说将数据以二进制存在形式的重新解释
 */
/**/

// void fun(int &val)
// {
// 	cout << "fun " << val << endl;
// }
// int main()
// {
// 	// const int val = 100;
// 	// // int n = const_cast<int>(val);
// 	// int n = val;

// 	// // int *p = &val; //&val 的类型是 const int *  Error
// 	// int *p = const_cast<int *>(&val);
// 	// *p = 200; //使用const_cast去除const限定的目的不是为了修改它的内容,实际上也不能真正修改val的值
// 	// cout << "&val 地址 = " << &val << endl;
// 	// cout << "p 地址 = " << p << endl;
// 	// cout << "*p = " << *p << endl;
// 	// cout << "val = " << val << endl;

// 	// const int val2 = 200;
// 	// // int &refval = val2; error
// 	// int &refval = const_cast<int &>(val2);
// 	// refval = 300; //使用const_cast去除const限定的目的不是为了修改它的内容,实际上也不能真正修改val的值
// 	// cout << "val2 = " << val2 << endl;
// 	// cout << "refval = " << refval << endl;

// 	// // fun(val2);Error
// 	// fun(const_cast<int &>(val2));

// 	// int n = static_cast<int>(3.14);
// 	// cout << "n = " << n << endl;

// 	int i;
// 	char *p = "This is an example.";
// 	i = reinterpret_cast<int>(p);
// 	//cout << "i = " << i << endl;

// 	return 0;
// }
#pragma endregion[新的类型转换运算符]

#pragma endregion[First]

#pragma region[面向对象]

//程序 = 算法 + 数据结构

#pragma region[类声明]
// int main()
// {
// 	Clock c;
// 	c.Init(10, 10, 59);
// 	c.Display();
// 	//c.second_ += 1;
// 	c.Update();

// 	c.SetHour(11);
// 	c.Display();
// 	return 0;
// }
#pragma endregion[]

#pragma region[内联函数]
///	内联函数可以提高效率，要求内联函数比较短小，编译时直接嵌入到调用的地方
/// 如果函数中含有swtich或者for等，编译器不会以内联函数来解析
#pragma endregion[内联函数]

#pragma region[成员函数的重载及其缺省参数]
// int main()
// {
// 	Test t;
// 	t.Init();
// 	t.Display();
// 	return 0;
// }
#pragma endregion[成员函数的重载及其缺省参数]

#pragma region[类与结构体]
///	class与struct的区别：在未指定访问权限时，class默认是私有的，struct默认是公有的

// struct S
// {
// 	int x_;
// 	int y_;
// 	int z_;
// 	void Init(int x, int y, int z)
// 	{
// 		x_ = x;
// 		y_ = y;
// 		z_ = z;
// 	}

// 	void Display()
// 	{
// 		cout << "x_ = " << x_ << " y_ = " << y_ << " z_ = " << z_ << endl;
// 	}
// };

// class C
// {
// 	int x_;
// 	int y_;
// 	int z_;
// 	void Init(int x, int y, int z)
// 	{
// 		x_ = x;
// 		y_ = y;
// 		z_ = z;
// 	}

// 	void Display()
// 	{
// 		cout << "x_ = " << x_ << " y_ = " << y_ << " z_ = " << z_ << endl;
// 	}
// };

// int main()
// {
// 	// S s;
// 	// s.Init(10, 20, 30);
// 	// s.Display();

// 	// S s = {10, 20, 30}; ///结构体可以这样初始化
// 	// s.Display();

// 	// C c;
// 	// c.Init(10,20,30);
// 	// c.Display();

// 	// C c={10,20,30}; Error 类不能用这种方式对私有成员初始化

// 	return 0;
// }
#pragma endregion[类与结构体]

#pragma region[隐含的this 指针]
///	成员函数有一个隐含的附加形参，即指向该对象的指针，this
///	使用this指针保证了每个对象可以拥有不同的数据结构，但处理这些成员的代码可以被所有对象共享

#pragma endregion[隐含的this 指针]

#pragma region[类作用域]

// class Test2
// {
//   public:
// 	int num_;
// };
// // num_ = 20; Error类作用域只在类中可见
// int num_ = 20; //与类中的num_是不用的作用域，这个是 1、文件作用域

// int add(int a, int b); //a和b是3、函数原型作用域
// int test();

// int main()
// {

// 	int num_ = 30; //2、块作用域，只作用在花括号以内
// 	cout << num_ << endl;
// 	cout << ::num_ << endl;
// 	test();
// 	return 0;
// }
// int add(int a, int b)
// {
// 	return a + b; //块作用域
// }

// int test()
// {
// LABEL1://函数作用域
// 	cout << "label 1" << endl;
// 	goto LABEL3;
// LABEL2:
// 	cout << "label 2" << endl;
// 	goto LABEL1;
// LABEL3:
// 	cout << "label 3" << endl;
// 	goto LABEL2;
// }

#pragma endregion[类作用域]

#pragma region[前向声明]
///	C++中类必须先定义，才能实例化
/// 两个类需要相互引用形成一个“环形”引用时，无法先定义使用。这时候需要用到前向声明
/// 前向声明的类不能实例化

#pragma endregion[前向声明]

#pragma region[嵌套类]
///	外围类需要使用嵌套类对象作为底层实现，并且该嵌套类只用于外围类的实现，且同时可以对用户隐藏该底层实现。
// class Outer
// {
// 	class Inner
// 	{
// 	  public:
// 		void Fun();
// 		// {
// 		// 	cout << "Inner Fun" << endl;
// 		// }
// 	};

//   public:
// 	Inner obj_;
// 	void Fun()
// 	{
// 		cout << "Outer::Fun" << endl;
// 		obj_.Fun();
// 	}
// };
// void Outer::Inner::Fun()
// {
// 	cout << "Inner Fun" << endl;
// }
// int main()
// {
// 	Outer o;
// 	o.Fun();
// 	return 0;
// }
#pragma endregion[嵌套类]

#pragma region[局部类]
///	类定义在函数体内，称为局部类（local class）。
///	局部类只在定义它的局部域内可见
///	局部类的成员函数必须被定义在类体中
///	局部类中不能有静态成员

// class Outer
// {
// 	class Inner
// 	{
// 	  public:
// 		void Fun();
// 		// {
// 		// 	cout << "Inner Fun" << endl;
// 		// }
// 	};

//   public:
// 	Inner obj_;
// 	void Fun()
// 	{
// 		cout << "Outer::Fun" << endl;
// 		obj_.Fun();
// 	}
// };
// void Outer::Inner::Fun()
// {
// 	cout << "Inner Fun" << endl;
// }
// void Fun()
// {
// 	class LocalClass
// 	{
// 	  public:
// 		int num_;
// 		void Init(int num)///	必须在函数内部实现
// 		{
// 			num_ = num;
// 		}
// 		void Display()
// 		{
// 			cout << "num = " << num_ << endl;
// 		}
// 		// static int num_; Error 局部类里面不能有静态成员
// 	};

// 	LocalClass lc;///	局部类只能在定义它的函数内部使用
// 	lc.Init(10);
// 	lc.Display();
// }
// int main()
// {
// 	Outer o;
// 	o.Fun();
// 	Fun();
// 	return 0;
// }
#pragma endregion[局部类]

#pragma endregion[面向对象]

#pragma region[构造函数与析构函数]

#pragma region[构造函数]
/// 构造函数是特殊的成员函数
/// 创建类类型的新对象，系统会自动调用构造函数
/// 构造函数是为了保证对象的每个数据成员都被正确初始化
/// 构造函数可以被重载
// int main()
// {
// 	Test1 t1;
// 	t1.Display();
// 	Test1 t2(10);
// 	t2.Display();

// 	Test1 *t3 = new Test1(20);// new operator
// 	t3->Display();

// 	delete t3;//delate 同时调用了析构函数
// 	return 0;
// }
#pragma endregion[构造函数]

#pragma region[全局对象的构造先于main 函数]

// Test1 t(10);
// int main()
// {
// 	cout << "Entering main" << endl;
// 	cout << "Exiting main" << endl;
// 	return 0;
// }
#pragma endregion[全局对象的构造先于main 函数]

#pragma region[析构函数与数组的关系]
// int main()
// {
// 	cout << "Entering main" << endl;
// 	Test1 t[2] = {10, 20};

// 	Test1 *t2 = new Test1(2);
// 	delete t2;

// 	Test1 *t3 = new Test1[2];
// 	delete[] t3;
// 	cout << "Exiting main" << endl;
// 	return 0;
// }
#pragma endregion[析构函数与数组的关系]

#pragma region[]
// int main()
// {
// 	Clock c(10,10,10);
// 	c.Display();
// 	//c.second_ += 1;
// 	c.Update();

// 	c.SetHour(11);
// 	c.Display();
// 	return 0;
// }
#pragma endregion[]

#pragma region[转换构造函数]
///	单个参数的构造函数称之为转换构造函数，作用是初始化加类型转换
///	将其他类型转换为类类型（构造函数的另一个作用，第一个作用是初始化）
///	类的构造函数只有一个参数是非常危险的，因为编译器可以使用这种构造函数把参数的类型隐式转换为类类型
// int main()
// {
// 	Test1 t(10); /// 带一个参数的构造函数，充当的依然是普通构造函数的功能，并没有进行类型转换
// 	t = 20;		 //含义是将20一个整数赋值给一个对象t，因此这个时候1、将整型转换成类类型，将调用转换构造函数（生成一个临时对象）
// 		// 2、将临时对象赋值给t对象，调用的是=运算符
// 	return 0;
// }
#pragma endregion[转换构造函数]

#pragma region[赋值与初始化的区别]
// int main()
// {
// 	Test1 t = 10; //等价于Test1 t(10)，表示初始化
// 	t = 20;		  //赋值操作

// 	Test1 t2;
// 	t = t2; //赋值操作 t.operator=(t2)

// 	return 0;
// }
#pragma endregion[赋值与初始化的区别]

#pragma region[构造函数初始化列表]
// int main()
// {
// 	Clock c(10,10,10);
// 	c.Display();
// 	return 0;
// }
#pragma endregion[构造函数初始化列表]

#pragma region[对象成员及其初始化]
//对象成员，如果该类所对应的类没有默认构造函数，对象成员的初始化也只能在构造函数初始化列表中进行
// class Object
// {
//   public:
// 	Object(int num) : num_(num)
// 	{
// 		cout << "Object " << num_ << endl;
// 	}
// 	~Object()
// 	{
// 		cout << "~Object " << num_ << endl;
// 	}

//   private:
// 	int num_;
// };
// class Container
// {
//   public:
// 	Container(int obj1 = 0, int obj2 = 0) : obj1_(obj1), obj2_(obj2)
// 	{
// 		cout << "Container " << endl;
// 	}
// 	~Container()
// 	{
// 		cout << "~Container " << endl;
// 	}

//   private:
// 	Object obj1_;
// 	Object obj2_;
// };
// int main()
// {
// 	Container c(10,20);
// 	return 0;
// }
#pragma endregion[对象成员及其初始化]

#pragma region[const 成员和引用成员的初始化]
//对象成员，如果该类所对应的类没有默认构造函数，对象成员的初始化也只能在构造函数初始化列表中进行
//const成员的初始化必须在构造函数初始化列表中进行
//引用成员的初始化必须在构造函数的初始化列表中进行
// class Object
// {
//   public:
// 	Object(int num = 0) : num_(num), kNum_(num),refNum_(num_)
// 	{
// 		cout << "Object " << num_ << endl;
// 		cout << "Object " << kNum_ << endl;
// 	}
// 	~Object()
// 	{
// 		cout << "~Object " << num_ << endl;
// 	}

//   private:
// 	int num_;
// 	const int kNum_; //const成员的初始化必须在构造函数初始化列表中进行
// 	int &refNum_;//引用成员的初始化必须在构造函数的初始化列表中进行
// };
// int main()
// {
// 	Object obj(10);
// 	return 0;
// }
#pragma endregion[const 成员和引用成员的初始化]

#pragma region[枚举 enum]
// class Object
// {
//   public:
// 	enum E_TYPE //枚举常量对所有对象都有const性
// 	{
// 		TYPE_A = 100,
// 		TYPE_B = 200
// 	};

//   public:
// 	Object(int num = 0) : num_(num), kNum_(num), refNum_(num_)
// 	{
// 		cout << "Object " << num_ << endl;
// 		cout << "Object " << kNum_ << endl;
// 	}
// 	~Object()
// 	{
// 		cout << "~Object " << num_ << endl;
// 	}

// 	void DisplaykNum()
// 	{
// 		cout << "The kNum = " << kNum_ << endl;
// 	}

//   private:
// 	int num_;
// 	const int kNum_; //常量性只对声明其的对象有效
// 	int &refNum_;
// };
// int main()
// {
// 	Object obj1(10);
// 	Object obj2(20);
// 	obj1.DisplaykNum();
// 	obj2.DisplaykNum();
// 	cout << "TYPE_A = " << Object::TYPE_A << endl;
// 	cout << "TYPE_A = " << obj1.TYPE_B << endl;
// 	cout << "TYPE_A = " << obj2.TYPE_B << endl;
// 	return 0;
// }
#pragma endregion[枚举 enum]

#pragma region[拷贝构造函数]
/// 功能：使用一个已经存在的对象来初始化一个新的同一类型的对象
/// 声明：只有一个参数并且参数为该类对象的引用
/// 如果类中没有说明拷贝构造函数，则系统自动生成一个缺省复制构造函数，作为该类的公有成员
// void TestFun1(const Test1 t) //会调用拷贝构造函数
// {
// }
// void TestFun2(const Test1 &t) //不会调用拷贝构造函数
// {
// }
// Test1 TestFun3(const Test1 &t)
// {
// 	return t; //返回后就销毁了 也会调用拷贝构造函数创建一个临时对象出来
// }

// const Test1 &TestFun4(const Test1 &t)//返回的是const引用，传入的也是const对象
// {
// 	return t;
// }
// Test1 &TestFun5(const Test1 &t)//返回的是const引用，传入的也是const对象 和TestFun4一样
// {
// 	return const_cast<Test1&>(t);
// }
// int main()
// {
// 	Test1 t(10);
// 	// Test1 t2(t);//用t对象初始化t2对象，将调用拷贝构造函数
// 	// Test1 t2=t;//等价于Test1 t2(t);

// 	// TestFun1(t);
// 	// TestFun2(t);
// 	// TestFun3(t);
// 	TestFun4(t);
// 	return 0;
// }
#pragma endregion[拷贝构造函数]

#pragma region[深拷贝与浅拷贝]
int main()
{
	String s1("AAA");
	s1.Display();
	// String s2 = s1;//是系统提供的默认的拷贝构造函数，实施的是浅拷贝，等价与s2.str_ = s1.str_
	// s2.Display();

	return 0;
}
#pragma endregion[深拷贝与浅拷贝]

#pragma endregion[构造函数与析构函数]
