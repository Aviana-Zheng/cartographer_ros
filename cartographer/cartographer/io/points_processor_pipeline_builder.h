/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef CARTOGRAPHER_IO_POINTS_PROCESSOR_PIPELINE_BUILDER_H_
#define CARTOGRAPHER_IO_POINTS_PROCESSOR_PIPELINE_BUILDER_H_

#include <string>
#include <unordered_map>
#include <vector>

#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/io/file_writer.h"
#include "cartographer/io/points_processor.h"
#include "cartographer/mapping/proto/trajectory.pb.h"

namespace cartographer {
namespace io {

// Builder to create a points processor pipeline out of a Lua configuration.
// You can register all built-in PointsProcessors using
// 'RegisterBuiltInPointsProcessors'. Non-built-in PointsProcessors must define
// a name and a factory method for building itself from a
// LuaParameterDictionary. See the various built-in PointsProcessors for
// examples.
class PointsProcessorPipelineBuilder {
 public:
  // FactoryFunction是一个函数指针
  // 返回值是std::unique_ptr<PointsProcessor>
  // 输入是common::LuaParameterDictionary*, PointsProcessor* next
  using FactoryFunction = std::function<std::unique_ptr<PointsProcessor>(
      common::LuaParameterDictionary*, PointsProcessor* next)>;

  PointsProcessorPipelineBuilder();

  PointsProcessorPipelineBuilder(const PointsProcessorPipelineBuilder&) =
      delete;
  PointsProcessorPipelineBuilder& operator=(
      const PointsProcessorPipelineBuilder&) = delete;

  // Register a new PointsProcessor type uniquly identified by 'name' which will
  // be created using 'factory'.
  void Register(const std::string& name, FactoryFunction factory);

  std::vector<std::unique_ptr<PointsProcessor>> CreatePipeline(
      common::LuaParameterDictionary* dictionary) const;

 private:
  // unordered_map: unordered_map内部实现了一个哈希表（也叫散列表，
  // 通过把关键码值映射到Hash表中一个位置来访问记录，查找的时间复杂度可达到O(1)，
  // 其在海量数据处理中有着广泛应用）。因此，其元素的排列顺序是无序的
  std::unordered_map<std::string, FactoryFunction> factories_;
};

// Register all 'PointsProcessor' that ship with Cartographer with this
// 'builder'.
void RegisterBuiltInPointsProcessors(
    const mapping::proto::Trajectory& trajectory,
    FileWriterFactory file_writer_factory,
    PointsProcessorPipelineBuilder* builder);

}  // namespace io
}  // namespace cartographer

#endif  // CARTOGRAPHER_IO_POINTS_PROCESSOR_PIPELINE_BUILDER_H_

/*C/C++函数指针(typedef简化定义)
https://www.cnblogs.com/renyuan/p/5268094.html

定义一个指向函数的指针用如下的形式，以上面的test()为例：
int (*fp)(int a);//这里就定义了一个指向函数(这个函数的参数仅仅为一个int类型)的指针

#include <iostream>
#include <string>
using namespace std;

int test(int a);

void main(int argc,char* argv[])   
{
    cout<<test<<endl;//显示函数地址
    int (*fp)(int a);
    fp=test;//将函数test的地址赋给函数学指针fp
    cout<<fp(5)<<"|"<<(*fp)(10)<<endl;
    // 上面的输出fp(5),这是标准c++的写法,(*fp)(10)这是兼容c语言的标准写法,
    // 两种同意,但注意区分,避免写的程序产生移植性问题!
    cin.get();
}

int test(int a)
{
    return a;
}
**************************************************************************************
typedef定义可以简化函数指针的定义，在定义一个的时候感觉不出来，但定义多了就知道方便了，
上面的代码改写成如下的形式：

#include <iostream>
#include <string>
using namespace std;

int test(int a);

void main(int argc,char* argv[])   
{
    cout<<test<<endl;
    typedef int (*fp)(int a);//注意,这里不是生命函数指针,而是定义一个函数指针的类型,这个类型是自己定义的,类型名为fp
    fp fpi;//这里利用自己定义的类型名fp定义了一个fpi的函数指针!
    fpi=test;
    cout<<fpi(5)<<"|"<<(*fpi)(10)<<endl;
    cin.get();
}

int test(int a)
{
    return a;
}
**********************************************************
利用函数指针，我们可以构成指针数组，更明确点的说法是构成指向函数的指针数组，
这么说可能就容易理解的多了：
#include <iostream>   
#include <string>   
using namespace std;

void t1(){cout<<"test1";}
void t2(){cout<<"test2";}
void t3(){cout<<"test3";}
void main(int argc,char* argv[])     
{
    void* a[]={t1,t2,t3};
    cout<<"比较t1()的内存地址和数组a[0]所存储的地址是否一致"<<t1<<"|"<<a[0]<<endl;

    cout<<a[0]();//错误!指针数组是不能利用数组下标操作调用函数的

    typedef void (*fp)();//自定义一个函数指针类型
    fp b[]={t1,t2,t3}; //利用自定义类型fp把b[]定义趁一个指向函数的指针数组
    b[0]();//现在利用指向函数的指针数组进行下标操作就可以进行函数的间接调用了;
    cin.get();
}
****************************************************************************
 仔细看上面的例子可能不用我多说大家也会知道是怎么一会事情了,最后我们做一个重点小结,
 只要记住这一点,对于理解利用函数指针构成数组进行函数间接调用就很容易了!
    void* a[]={t1,t2,t3};
    cout<<"比较t1()的内存地址和数组a[0]所存储的地址是否一致"<<t1<<"|"<<a[0]<<endl;

    cout<<a[0]();//错误!指针数组是不能利用数组下标操作调用函数的

    上面的这一小段中的错误行，为什么不能这么调用呢？

前一篇教程我们已经说的很清楚了，不过在这里我们还是复习一下概念，
指针数组元素所保存的只是一个内存地址，既然只是个内存地址就不可能进行a[0]()这样地址带括号的操作，

而函数指针不同它是一个例外，函数指针只所以这么叫它就是因为它是指向函数指向内存的代码区的指针
，它被系统授予允许与()括号操作的权利，进行间接的函数调用，既然函数指针允许这么操作，
那么被定义成函数指针的数组就一定是可以一样的操作的
*/

/*std::function与std::bind使用总结
https://blog.csdn.net/weixin_41167925/article/details/117920374

C++中函数指针的用途非常广泛，例如回调函数，接口类的设计等，但函数指针始终不太灵活，
它只能指向全局或静态函数，对于类成员函数、lambda表达式或其他可调用对象就无能为力了，
因此，C++11推出了std::function与std::bind这两件大杀器。

std::function vs 函数指针
C++函数指针相信大家用的很多了，用法最广泛的应该就是先定义函数指针的类型，
然后在声明一个函数指针的变量作为另一个函数的入参，以此作为回调函数，如下列代码所示：

typedef void (*PrintFinCallback)();
void print(const char *text, PrintFinCallback callback) {
    printf("%s\n", text);
    callback();
}

void printFinCallback() {
    cout << "hhh" << endl;
}
print("test", printFinCallback);


毫无疑问，函数指针的用法非常简单，但是它只能指向全局或静态函数，这有点太不灵活了，
而且我们都知道在C/C++中，全局的东西都很可怕，稍有不慎就会被篡改或随便调用。
幸好，在C++11之后，我们多了一种选择，std::function，使用它时需要引入头文件functional。
std::function可以说是函数指针的超集，它除了可以指向全局和静态函数，还可以指向彷函数，
lambda表达式，类成员函数，甚至函数签名不一致的函数，可以说几乎所有可以调用的对象都可以当做
std::function，当然对于后两个需要使用std::bind进行配合，而至于指向其他类型可以参考以下代码：

typedef std::function<void ()> PrintFinFunction;
void print(const char *text, PrintFinFunction callback) {
    printf("%s\n", text);
    if (callback)
        callback();
}
// 普通函数
void printFinCallback() {
    cout << "Normal callback" << endl;
}
// 类静态函数
class Test {
public:
    static void printFinCallback() {
        cout << "Static callback" << endl;
    }
};
// 仿函数，重载()运算符
struct Functor {
    void operator() () {
        cout << "Functor callback" << endl;
    }
};
print("test 1", printFinCallback);
print("test 2", Test::printFinCallback);
print("test 3", Functor());
print("test 4", [] () {
	cout << "Lambda callback" << endl;
});


当然，任何东西都会有优缺点，std::function填补了函数指针的灵活性，但会对调用性能有一定损耗，
经测试发现，在调用次数达10亿次时，函数指针比直接调用要慢2秒左右，
而std::function要比函数指针慢2秒左右，这么少的损耗如果是对于调用次数并不高的函数，
替换成std::function绝对是划得来的。

std::function与std::bind双剑合璧
刚才也说道，std::function可以指向类成员函数和函数签名不一样的函数，其实，这两种函数都是一样的，
因为类成员函数都有一个默认的参数，this，作为第一个参数，
这就导致了类成员函数不能直接赋值给std::function，这时候我们就需要std::bind了，
简言之，std::bind的作用就是转换函数签名，将缺少的参数补上，将多了的参数去掉，
甚至还可以交换原来函数参数的位置，具体用法如下列代码所示：

typedef std::function<void (int)> PrintFinFunction;
void print(const char *text, PrintFinFunction callback) {
    printf("%s\n", text);
    if (callback)
        callback(0);
}
// 类成员函数
class Test {
public:
    void printFinCallbackInter(int res) {
        cout << "Class Inter callback" << endl;
    }
};
// 函数签名不一样的函数
void printFinCallback2(int res1, int res2) {
    cout << "Different callback " << res1 << " " << res2 << endl;
}
Test testObj;
auto callback5 = std::bind(&Test::printFinCallbackInter, testObj, std::placeholders::_1);
print("test 5", callback5); //函数模板只有一个参数，这里需要补充this参数
auto callback6 = std::bind(&printFinCallback2, std::placeholders::_1, 100);
print("test 6", callback6); //这里需要补充第二个参数


std::bind的用法就是第一个参数是要被指向的函数的地址，
为了区分，这里std::bind语句的左值函数为原函数，右值函数为新函数，
那么std::bind方法从第二个参数起，都是新函数所需要的参数，缺一不可，
而我们可以使用std::placeholders::_1或std::placeholders::_2等等来使用原函数的参数，
_1就是原函数的第一个参数，如此类推。

值得注意的有两点：
一旦bind补充了缺失的参数，那么以后每次调用这个function时，那些原本缺失的参数都是一样的，
举个栗子，上面代码中callback6，我们每次调用它的时候，第二个参数都只会是100。
正因为第一点，所以假如我们是在iOS程序中使用std::bind传入一个缺失参数，
那么我们转化后的那个function会持有那些缺失参数，这里我们需要防止出现循环引用导致内存泄漏。
*/