# C++11 并发指南三(std::mutex 详解)

## `<mutex> `头文件介绍

### Mutex 系列类(四种)

- `std::mutex`，最基本的 Mutex 类。
- `std::recursive_mutex`，递归 Mutex 类。
- `std::time_mutex`，定时 Mutex 类。
- `std::recursive_timed_mutex`，定时递归 Mutex 类。

### Lock 类（两种）

- `std::lock_guard`，与 `Mutex RAII` 相关，方便线程对互斥量上锁。
- `std::unique_lock`，与 `Mutex RAII` 相关，方便线程对互斥量上锁，但提供了更好的上锁和解锁控制。

### 其他类型

- `std::once_flag`
- `std::adopt_lock_t`
- `std::defer_lock_t`
- `std::try_to_lock_t`

### 函数

- `std::try_lock`，尝试同时对多个互斥量上锁。
- `std::lock`，可以同时对多个互斥量上锁。
- `std::call_once`，如果多个线程需要同时调用某个函数，`call_once `可以保证多个线程对该函数只调用一次。

## std::mutex 介绍

下面以`std::mutex `为例介绍 C++11 中的互斥量用法。

`std::mutex` 是C++11 中最基本的互斥量，`std::mutex` 对象提供了独占所有权的特性——即不支持递归地对` std::mutex `对象上锁，而 `std::recursive_lock` 则可以递归地对互斥量对象上锁。

### std::mutex 的成员函数

- 构造函数，`std::mutex`不允许拷贝构造，也不允许` move` 拷贝，最初产生的 `mutex `对象是处于` unlocked `状态的。
- `lock()`，调用线程将锁住该互斥量。线程调用该函数会发生下面 3 种情况：
  1.   如果该互斥量当前没有被锁住，则调用线程将该互斥量锁住，直到调用 `unlock`之前，该线程一直拥有该锁。
  2.   如果当前互斥量被其他线程锁住，则当前的调用线程被阻塞住。
  3.  如果当前互斥量被当前调用线程锁住，则会产生死锁(`deadlock`)。
- `unlock()`， 解锁，释放对互斥量的所有权。
- `try_lock()`，尝试锁住互斥量，如果互斥量被其他线程占有，则当前线程也不会被阻塞。线程调用该函数也会出现下面 3 种情况:
  1.  如果当前互斥量没有被其他线程占有，则该线程锁住互斥量，直到该线程调用 `unlock `释放互斥量。
  2.  如果当前互斥量被其他线程锁住，则当前调用线程返回` false`，而并不会被阻塞掉。
  3.  如果当前互斥量被当前调用线程锁住，则会产生死锁(`deadlock`)。

下面给出一个与 std::mutex 的小例子

```c++
#include <iostream>       // std::cout
#include <thread>         // std::thread
#include <mutex>          // std::mutex

volatile int counter(0); // non-atomic counter
std::mutex mtx;           // locks access to counter

void attempt_10k_increases() {
    for (int i=0; i<10000; ++i) {
        if (mtx.try_lock()) {   // only increase if currently not locked:
            ++counter;
            mtx.unlock();
        }
    }
}

int main (int argc, const char* argv[]) {
    std::thread threads[10];
    for (int i=0; i<10; ++i)
        threads[i] = std::thread(attempt_10k_increases);

    for (auto& th : threads) th.join();
    std::cout << counter << " successful increases of the counter.\n";

    return 0;
}
```



### std::recursive_mutex 介绍

`std::recursive_mutex `与` std::mutex` 一样，也是一种可以被上锁的对象，但是和 `std::mutex`  不同的是，`std::recursive_mutex`  允许同一个线程对互斥量多次上锁（即递归上锁），来获得对互斥量对象的多层所有权，`std::recursive_mutex  `释放互斥量时需要调用与该锁层次深度相同次数的 `unlock()`，可理解为` lock() `次数和` unlock()`  次数相同，除此之外，`std::recursive_mutex` 的特性和 `std::mutex `大致相同。

### std::time_mutex 介绍

`std::time_mutex` 比` std::mutex `多了两个成员函数，`try_lock_for()`，`try_lock_until()`。

`try_lock_for` 函数接受一个时间范围，表示在这一段时间范围之内线程如果没有获得锁则被阻塞住（与 `std::mutex` 的  `try_lock() `不同，`try_lock `如果被调用时没有获得锁则直接返回  `false`），如果在此期间其他线程释放了锁，则该线程可以获得对互斥量的锁，如果超时（即在指定时间内还是没有获得锁），则返回 `false`。

`try_lock_until `函数则接受一个时间点作为参数，在指定时间点未到来之前线程如果没有获得锁则被阻塞住，如果在此期间其他线程释放了锁，则该线程可以获得对互斥量的锁，如果超时（即在指定时间内还是没有获得锁），则返回 `false`。

下面的小例子说明了 `std::time_mutex `的用法

```c++
#include <iostream>       // std::cout
#include <chrono>         // std::chrono::milliseconds
#include <thread>         // std::thread
#include <mutex>          // std::timed_mutex

std::timed_mutex mtx;

void fireworks() {
  // waiting to get a lock: each thread prints "-" every 200ms:
  while (!mtx.try_lock_for(std::chrono::milliseconds(200))) {
    std::cout << "-";
  }
  // got a lock! - wait for 1s, then this thread prints "*"
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  std::cout << "*\n";
  mtx.unlock();
}

int main ()
{
  std::thread threads[10];
  // spawn 10 threads:
  for (int i=0; i<10; ++i)
    threads[i] = std::thread(fireworks);

  for (auto& th : threads) th.join();

  return 0;
}
```

### std::recursive_timed_mutex 介绍

和` std:recursive_mutex `与 `std::mutex `的关系一样，`std::recursive_timed_mutex `的特性也可以从` std::timed_mutex` 推导出来，感兴趣的同鞋可以自行查阅。

### std::lock_guard 介绍

与 `Mutex RAII `相关，方便线程对互斥量上锁。例子

```c++
#include <iostream>       // std::cout
#include <thread>         // std::thread
#include <mutex>          // std::mutex, std::lock_guard
#include <stdexcept>      // std::logic_error

std::mutex mtx;

void print_even (int x) {
    if (x%2==0) std::cout << x << " is even\n";
    else throw (std::logic_error("not even"));
}

void print_thread_id (int id) {
    try {
        // using a local lock_guard to lock mtx guarantees unlocking on destruction / exception:
        std::lock_guard<std::mutex> lck (mtx);
        print_even(id);
    }
    catch (std::logic_error&) {
        std::cout << "[exception caught]\n";
    }
}

int main ()
{
    std::thread threads[10];
    // spawn 10 threads:
    for (int i=0; i<10; ++i)
        threads[i] = std::thread(print_thread_id,i+1);

    for (auto& th : threads) th.join();

    return 0;
}
```

### std::unique_lock 介绍

与 Mutex RAII 相关，方便线程对互斥量上锁，但提供了更好的上锁和解锁控制。例子

```c++
#include <iostream>       // std::cout
#include <thread>         // std::thread
#include <mutex>          // std::mutex, std::unique_lock

std::mutex mtx;           // mutex for critical section

void print_block (int n, char c) {
    // critical section (exclusive access to std::cout signaled by lifetime of lck):
    std::unique_lock<std::mutex> lck (mtx);
    for (int i=0; i<n; ++i) {
        std::cout << c;
    }
    std::cout << '\n';
}

int main ()
{
    std::thread th1 (print_block,50,'*');
    std::thread th2 (print_block,50,'$');

    th1.join();
    th2.join();

    return 0;
}
```



# C++11 并发指南三(Lock 详解)

C++11 标准为我们提供了两种基本的锁类型，分别如下：

- std::lock_guard，与 Mutex RAII 相关，方便线程对互斥量上锁。
- std::unique_lock，与 Mutex RAII 相关，方便线程对互斥量上锁，但提供了更好的上锁和解锁控制。

另外还提供了几个与锁类型相关的 Tag 类，分别如下:

- `std::adopt_lock_t`，一个空的标记类，定义如下：

```
struct adopt_lock_t {};
```

 该类型的常量对象`adopt_lock`（`adopt_lock` 是一个常量对象，定义如下：

```
constexpr adopt_lock_t adopt_lock {}; // constexpr 是 C++11 中的新关键字）
通常作为参数传入给 unique_lock 或 lock_guard 的构造函数。 
```

- `std::defer_lock_t`，一个空的标记类，定义如下：

```
struct defer_lock_t {};
```

 该类型的常量对象 `defer_lock`（`defer_lock` 是一个常量对象，定义如下：

```
constexpr defer_lock_t defer_lock {}; // constexpr 是 C++11 中的新关键字）
通常作为参数传入给 unique_lock 或 lock_guard 的构造函数。 
```

- `std::try_to_lock_t`，一个空的标记类，定义如下：

```
struct try_to_lock_t {};
```

 该类型的常量对象 `try_to_lock`（`try_to_lock` 是一个常量对象，定义如下：

```
constexpr try_to_lock_t try_to_lock {}; // constexpr 是 C++11 中的新关键字）
```

通常作为参数传入给 `unique_lock` 或 `lock_guard` 的构造函数。

后面我们会详细介绍以上三种 `Tag` 类型在配合` lock_gurad` 与 `unique_lock` 使用时的区别。

## std::lock_guard 介绍

`std::lock_gurad` 是 C++11 中定义的模板类。定义如下：

```
template < class Mutex>  
class lock_guard;
```

`lock_guard` 对象通常用于管理某个锁(`Lock`)对象，因此与` Mutex RAII `相关，方便线程对互斥量上锁，即在某个 ` lock_guard` 对象的声明周期内，它所管理的锁对象会一直保持上锁状态；而 `lock_guard`  的生命周期结束之后，它所管理的锁对象会被解锁(注：类似 `shared_ptr` 等智能指针管理动态分配的内存资源 )。

模板参数` Mutex` 代表互斥量类型，例如` std::mutex` 类型，它应该是一个基本的 `BasicLockable  `类型，标准库中定义几种基本的 `BasicLockable` 类型，分别 `std::mutex`, `std::recursive_mutex`,  `std::timed_mutex`，`std::recursive_timed_mutex `(以上四种类型均已在上一篇博客中介绍)以及  `std::unique_lock`(本文后续会介绍` std::unique_lock`)。(注：`BasicLockable`  类型的对象只需满足两种操作，`lock `和` unlock`，另外还有` Lockable `类型，在 `BasicLockable `类型的基础上新增了`  try_lock` 操作，因此一个满足 `Lockable` 的对象应支持三种操作：`lock`，`unlock `和 `try_lock`；最后还有一种  `TimedLockable `对象，在` Lockable `类型的基础上又新增了` try_lock_for` 和` try_lock_until ` 两种操作，因此一个满足 `TimedLockable` 的对象应支持五种操作：`lock`,` unlock`,` try_lock`, ` try_lock_for`, `try_lock_until`)。

在 `lock_guard` 对象构造时，传入的 `Mutex `对象(即它所管理的` Mutex `对象)会被当前线程锁住。在`lock_guard ` 对象被析构时，它所管理的` Mutex `对象会自动解锁，由于不需要程序员手动调用` lock` 和 `unlock` 对` Mutex ` 进行上锁和解锁操作，因此这也是最简单安全的上锁和解锁方式，尤其是在程序抛出异常后先前已被上锁的 `Mutex ` 对象可以正确进行解锁操作，极大地简化了程序员编写与` Mutex `相关的异常处理代码。

 值得注意的是，`lock_guard `对象并不负责管理 `Mutex` 对象的生命周期，`lock_guard` 对象只是简化了` Mutex`  对象的上锁和解锁操作，方便线程对互斥量上锁，即在某个 `lock_guard` 对象的声明周期内，它所管理的锁对象会一直保持上锁状态；而  `lock_guard `的生命周期结束之后，它所管理的锁对象会被解锁。

### std::lock_guard 构造函数

lock_guard 构造函数如下表所示：

|                     |                                                  |
| ------------------- | ------------------------------------------------ |
| locking (1)         | `explicit lock_guard (mutex_type& m); `          |
| adopting (2)        | `lock_guard (mutex_type& m, adopt_lock_t tag); ` |
| copy` [deleted](3)` | `lock_guard (const lock_guard&) = delete;`       |

1. locking 初始化
   - lock_guard 对象管理 Mutex 对象 m，并在构造时对 m 进行上锁（调用 m.lock()）。
2. adopting初始化
   - lock_guard 对象管理 Mutex 对象 m，与 locking 初始化(1) 不同的是， Mutex 对象 m 已被当前线程锁住。
3. 拷贝构造
   - lock_guard 对象的拷贝构造和移动构造(move construction)均被禁用，因此 lock_guard 对象不可被拷贝构造或移动构造。

我们来看一个简单的例子

```c++
#include <iostream>       // std::cout
#include <thread>         // std::thread
#include <mutex>          // std::mutex, std::lock_guard, std::adopt_lock

std::mutex mtx;           // mutex for critical section

void print_thread_id (int id) {
  mtx.lock();
  std::lock_guard<std::mutex> lck(mtx, std::adopt_lock);
  std::cout << "thread #" << id << '\n';
}

int main ()
{
  std::thread threads[10];
  // spawn 10 threads:
  for (int i=0; i<10; ++i)
    threads[i] = std::thread(print_thread_id,i+1);

  for (auto& th : threads) th.join();

  return 0;
}
```

在 `print_thread_id `中，我们首先对 `mtx` 进行上锁操作(`mtx.lock()`;)，然后用 `mtx` 对象构造一个` lock_guard `对象(`std::lock_guard<std::mutex> lck(mtx, std::adopt_lock)`;)，注意此时 `Tag` 参数为` std::adopt_lock`，表明当前线程已经获得了锁，此后` mtx` 对象的解锁操作交由 `lock_guard` 对象` lck` 来管理，在` lck `的生命周期结束之后，`mtx` 对象会自动解锁。

`lock_guard `最大的特点就是安全易于使用，请看下面例子([参考](http://www.cplusplus.com/reference/mutex/lock_guard/))，在异常抛出的时候通过` lock_guard `对象管理的 `Mutex `可以得到正确地解锁。

```c++
#include <iostream>       // std::cout
#include <thread>         // std::thread
#include <mutex>          // std::mutex, std::lock_guard
#include <stdexcept>      // std::logic_error

std::mutex mtx;

void print_even (int x) {
  if (x%2==0) std::cout << x << " is even\n";
  else throw (std::logic_error("not even"));
}

void print_thread_id (int id) {
  try {
    // using a local lock_guard to lock mtx guarantees unlocking on destruction / exception:
    std::lock_guard<std::mutex> lck (mtx);
    print_even(id);
  }
  catch (std::logic_error&) {
    std::cout << "[exception caught]\n";
  }
}

int main ()
{
  std::thread threads[10];
  // spawn 10 threads:
  for (int i=0; i<10; ++i)
    threads[i] = std::thread(print_thread_id,i+1);

  for (auto& th : threads) th.join();

  return 0;
}
```

## std::unique_lock 介绍

但是` lock_guard `最大的缺点也是简单，没有给程序员提供足够的灵活度，因此，C++11 标准中定义了另外一个与 `Mutex  RAII `相关类 `unique_lock`，该类与 `lock_guard` 类相似，也很方便线程对互斥量上锁，但它提供了更好的上锁和解锁控制。

顾名思义，`unique_lock `对象以独占所有权的方式（ `unique owership`）管理 `mutex `对象的上锁和解锁操作，所谓独占所有权，就是没有其他的 `unique_lock` 对象同时拥有某个` mutex `对象的所有权。

在构造(或移动(`move`)赋值)时，`unique_lock `对象需要传递一个 `Mutex `对象作为它的参数，新创建的 `unique_lock` 对象负责传入的 `Mutex `对象的上锁和解锁操作。

 `std::unique_lock `对象也能保证在其自身析构时它所管理的 `Mutex `对象能够被正确地解锁（即使没有显式地调用` unlock`  函数）。因此，和` lock_guard `一样，这也是一种简单而又安全的上锁和解锁方式，尤其是在程序抛出异常后先前已被上锁的 `Mutex  `对象可以正确进行解锁操作，极大地简化了程序员编写与` Mutex `相关的异常处理代码。

 值得注意的是，`unique_lock `对象同样也不负责管理 `Mutex `对象的生命周期，`unique_lock `对象只是简化了 `Mutex  `对象的上锁和解锁操作，方便线程对互斥量上锁，即在某个` unique_lock`  对象的声明周期内，它所管理的锁对象会一直保持上锁状态；而 `unique_lock` 的生命周期结束之后，它所管理的锁对象会被解锁，这一点和 ` lock_guard `类似，但` unique_lock `给程序员提供了更多的自由，我会在下面的内容中给大家介绍 `unique_lock `的用法。

另外，与 `lock_guard` 一样，模板参数 `Mutex `代表互斥量类型，例如`std::mutex `类型，它应该是一个基本的`  BasicLockable` 类型，标准库中定义几种基本的` BasicLockable `类型，分别` std::mutex`,  `std::recursive_mutex`, `std::timed_mutex`，`std::recursive_timed_mutex`  (以上四种类型均已在上一篇博客中介绍)以及 `std::unique_lock`(本文后续会介绍  `std::unique_lock`)。(注：`BasicLockable`类型的对象只需满足两种操作，`lock` 和` unlock`，另外还有`  Lockable` 类型，在 `BasicLockable` 类型的基础上新增了` try_lock `操作，因此一个满足` Lockable`  的对象应支持三种操作：`lock`，`unlock `和 `try_lock`；最后还有一种 `TimedLockable` 对象，在 `Lockable  `类型的基础上又新增了` try_lock_for` 和 `try_lock_until `两种操作，因此一个满足 `TimedLockable ` 的对象应支持五种操作：`lock`, `unlock`, `try_lock`, `try_lock_for`, `try_lock_until`)。

### std::unique_lock 构造函数

`std::unique_lock `的构造函数的数目相对来说比 `std::lock_guard `多，其中一方面也是因为  `std::unique_lock` 更加灵活，从而在构造` std::unique_lock  `对象时可以接受额外的参数。总地来说，`std::unique_lock `构造函数如下：

|                    |                                                              |
| ------------------ | ------------------------------------------------------------ |
| default (1)        | `unique_lock() noexcept; `                                   |
| locking (2)        | `explicit unique_lock(mutex_type& m); `                      |
| try-locking (3)    | `unique_lock(mutex_type& m, try_to_lock_t tag); `            |
| deferred (4)       | `unique_lock(mutex_type& m, defer_lock_t tag) noexcept; `    |
| adopting (5)       | `unique_lock(mutex_type& m, adopt_lock_t tag); `             |
| locking for (6)    | `template <class Rep, class Period> unique_lock(mutex_type& m, const chrono::duration<Rep,Period>& rel_time); ` |
| locking until (7)  | `template <class Clock, class Duration> unique_lock(mutex_type& m, const chrono::time_point<Clock,Duration>& abs_time); ` |
| copy [deleted] (8) | `unique_lock(const unique_lock&) = delete; `                 |
| move (9)           | `unique_lock(unique_lock&& x);`                              |

下面我们来分别介绍以上各个构造函数：

1. 默认构造函数

   新创建的 unique_lock 对象不管理任何 Mutex 对象。 

2. locking 初始化

   新创建的 unique_lock 对象管理 Mutex 对象 m，并尝试调用 m.lock() 对 Mutex 对象进行上锁，如果此时另外某个 unique_lock 对象已经管理了该 Mutex 对象 m，则当前线程将会被阻塞。

3. try-locking 初始化

   新创建的 unique_lock 对象管理 Mutex 对象 m，并尝试调用 m.try_lock() 对 Mutex 对象进行上锁，但如果上锁不成功，并不会阻塞当前线程。

4. deferred 初始化

   新创建的 unique_lock 对象管理 Mutex 对象 m，但是在初始化的时候并不锁住 Mutex 对象。 m 应该是一个没有当前线程锁住的 Mutex 对象。

5. adopting 初始化

   新创建的 unique_lock 对象管理 Mutex 对象 m， m 应该是一个已经被当前线程锁住的 Mutex 对象。(并且当前新创建的 unique_lock 对象拥有对锁(Lock)的所有权)。

6. locking 一段时间(duration)

   新创建的 unique_lock 对象管理 Mutex 对象 m，并试图通过调用 m.try_lock_for(rel_time) 来锁住 Mutex 对象一段时间(rel_time)。

7. locking 直到某个时间点(time point)

   新创建的 unique_lock 对象管理 Mutex 对象m，并试图通过调用 m.try_lock_until(abs_time) 来在某个时间点(abs_time)之前锁住 Mutex 对象。

8. 拷贝构造 [被禁用]

   unique_lock 对象不能被拷贝构造。

9. 移动(move)构造

   新创建的 unique_lock 对象获得了由 x 所管理的 Mutex 对象的所有权(包括当前 Mutex 的状态)。调用 move 构造之后， x 对象如同通过默认构造函数所创建的，就不再管理任何 Mutex 对象了。

综上所述，由 (2) 和 (5) 创建的 unique_lock 对象通常拥有 Mutex 对象的锁。而通过 (1) 和 (4) 创建的则不会拥有锁。通过 (3)，(6) 和 (7) 创建的 unique_lock 对象，则在 lock 成功时获得锁。

关于unique_lock 的构造函数，请看下面例子([参考](http://www.cplusplus.com/reference/mutex/unique_lock/unique_lock/))：

```c++
#include <iostream>       // std::cout
#include <thread>         // std::thread
#include <mutex>          // std::mutex, std::lock, std::unique_lock
                          // std::adopt_lock, std::defer_lock
std::mutex foo,bar;

void task_a () {
  std::lock (foo,bar);         // simultaneous lock (prevents deadlock)
  std::unique_lock<std::mutex> lck1 (foo,std::adopt_lock);
  std::unique_lock<std::mutex> lck2 (bar,std::adopt_lock);
  std::cout << "task a\n";
  // (unlocked automatically on destruction of lck1 and lck2)
}

void task_b () {
  // foo.lock(); bar.lock(); // replaced by:
  std::unique_lock<std::mutex> lck1, lck2;
  lck1 = std::unique_lock<std::mutex>(bar,std::defer_lock);
  lck2 = std::unique_lock<std::mutex>(foo,std::defer_lock);
  std::lock (lck1,lck2);       // simultaneous lock (prevents deadlock)
  std::cout << "task b\n";
  // (unlocked automatically on destruction of lck1 and lck2)
}


int main ()
{
  std::thread th1 (task_a);
  std::thread th2 (task_b);

  th1.join();
  th2.join();

  return 0;
}
```

### std::unique_lock 移动(move assign)赋值操作

`std::unique_lock `支持移动赋值(`move assignment`)，但是普通的赋值被禁用了，

|                    |                                                         |
| ------------------ | ------------------------------------------------------- |
| move (1)           | `unique_lock& operator= (unique_lock&& x) noexcept; `   |
| copy [deleted] (2) | `unique_lock& operator= (const unique_lock&) = delete;` |

移动赋值(`move assignment`)之后，由` x `所管理的 `Mutex `对象及其状态将会被新的 `std::unique_lock` 对象取代。

如果被赋值的对象之前已经获得了它所管理的 `Mutex `对象的锁，则在移动赋值(`move assignment`)之前会调用` unlock `函数释放它所占有的锁。

 调用移动赋值(`move assignment`)之后，` x` 对象如同通过默认构造函数所创建的，也就不再管理任何` Mutex` 对象了。请看下面例子([参考](http://www.cplusplus.com/reference/mutex/unique_lock/operator=/))：

```c++
#include <iostream>       // std::cout
#include <thread>         // std::thread
#include <mutex>          // std::mutex, std::unique_lock

std::mutex mtx;           // mutex for critical section

void print_fifty (char c) {
  std::unique_lock<std::mutex> lck;         // default-constructed
  lck = std::unique_lock<std::mutex>(mtx);  // move-assigned
  for (int i=0; i<50; ++i) { std::cout << c; }
  std::cout << '\n';
}

int main ()
{
  std::thread th1 (print_fifty,'*');
  std::thread th2 (print_fifty,'$');

  th1.join();
  th2.join();

  return 0;
}
```

### std::unique_lock 主要成员函数

本节我们来看看 std::unique_lock 的主要成员函数。由于` std::unique_lock `比 `std::lock_guard `操作灵活，因此它提供了更多成员函数。具体分类如下：

1. 上锁/解锁操作：`lock`，`try_lock`，`try_lock_for`，`try_lock_until`和`unlock`
2. 修改操作：移动赋值(`move assignment`)(前面已经介绍过了)，交换(`swap`)（与另一个 `std::unique_lock`  对象交换它们所管理的` Mutex` 对象的所有权），释放(`release`)（返回指向它所管理的` Mutex `对象的指针，并释放所有权）
3. 获取属性操作：`owns_lock`（返回当前 `std::unique_lock `对象是否获得了锁）、`operator bool()`与  `owns_lock `功能相同，返回当前 `std::unique_lock` 对象是否获得了锁）、`mutex`（返回当前  `std::unique_lock `对象所管理的` Mutex` 对象的指针）。

#### **std::unique_lock::lock**请看下面例子([参考](http://www.cplusplus.com/reference/mutex/unique_lock/lock/))：

上锁操作，调用它所管理的` Mutex `对象的 `lock `函数。如果在调用 `Mutex `对象的 `lock `函数时该` Mutex `对象已被另一线程锁住，则当前线程会被阻塞，直到它获得了锁。

该函数返回时，当前的 `unique_lock` 对象便拥有了它所管理的` Mutex `对象的锁。如果上锁操作失败，则抛出` system_error `异常

```c++
// unique_lock::lock/unlock
#include <iostream>       // std::cout
#include <thread>         // std::thread
#include <mutex>          // std::mutex, std::unique_lock, std::defer_lock

std::mutex mtx;           // mutex for critical section

void print_thread_id (int id) {
  std::unique_lock<std::mutex> lck (mtx,std::defer_lock);
  // critical section (exclusive access to std::cout signaled by locking lck):
  lck.lock();
  std::cout << "thread #" << id << '\n';
  lck.unlock();
}

int main ()
{
  std::thread threads[10];
  // spawn 10 threads:
  for (int i=0; i<10; ++i)
    threads[i] = std::thread(print_thread_id,i+1);

  for (auto& th : threads) th.join();

  return 0;
}
```

#### **std::unique_lock::try_lock_for**

上锁操作，调用它所管理的 Mutex 对象的 try_lock_for 函数，如果上锁成功，则返回 true，否则返回 false。

请看下面例子([参考](http://www.cplusplus.com/reference/mutex/unique_lock/try_lock_for/))：

```c++
#include <iostream>       // std::cout
#include <chrono>         // std::chrono::milliseconds
#include <thread>         // std::thread
#include <mutex>          // std::timed_mutex, std::unique_lock, std::defer_lock

std::timed_mutex mtx;

void fireworks () {
  std::unique_lock<std::timed_mutex> lck(mtx,std::defer_lock);
  // waiting to get a lock: each thread prints "-" every 200ms:
  while (!lck.try_lock_for(std::chrono::milliseconds(200))) {
    std::cout << "-";
  }
  // got a lock! - wait for 1s, then this thread prints "*"
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  std::cout << "*\n";
}

int main ()
{
  std::thread threads[10];
  // spawn 10 threads:
  for (int i=0; i<10; ++i)
    threads[i] = std::thread(fireworks);

  for (auto& th : threads) th.join();

  return 0;
}
```

#### **std::unique_lock::try_lock_until**

上锁操作，调用它所管理的 Mutex 对象的 try_lock_for 函数，如果上锁成功，则返回 true，否则返回 false。

请看下面例子([参考](http://www.cplusplus.com/reference/mutex/unique_lock/try_lock_until/))：

```c++
#include <iostream>       // std::cout
#include <chrono>         // std::chrono::milliseconds
#include <thread>         // std::thread
#include <mutex>          // std::timed_mutex, std::unique_lock, std::defer_lock

std::timed_mutex mtx;

void fireworks () {
  std::unique_lock<std::timed_mutex> lck(mtx,std::defer_lock);
  // waiting to get a lock: each thread prints "-" every 200ms:
  while (!lck.try_lock_for(std::chrono::milliseconds(200))) {
    std::cout << "-";
  }
  // got a lock! - wait for 1s, then this thread prints "*"
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  std::cout << "*\n";
}

int main ()
{
  std::thread threads[10];
  // spawn 10 threads:
  for (int i=0; i<10; ++i)
    threads[i] = std::thread(fireworks);

  for (auto& th : threads) th.join();

  return 0;
}
```

#### **std::unique_lock::unlock**

解锁操作，调用它所管理的 Mutex 对象的 unlock 函数。

请看下面例子(参考)：

```c++
#include <iostream>       // std::cout
#include <thread>         // std::thread
#include <mutex>          // std::mutex, std::unique_lock, std::defer_lock

std::mutex mtx;           // mutex for critical section

void print_thread_id (int id) {
  std::unique_lock<std::mutex> lck (mtx,std::defer_lock);
  // critical section (exclusive access to std::cout signaled by locking lck):
  lck.lock();
  std::cout << "thread #" << id << '\n';
  lck.unlock();
}

int main ()
{
  std::thread threads[10];
  // spawn 10 threads:
  for (int i=0; i<10; ++i)
    threads[i] = std::thread(print_thread_id,i+1);

  for (auto& th : threads) th.join();

  return 0;
}
```

#### **std::unique_lock::release**

返回指向它所管理的 Mutex 对象的指针，并释放所有权。

请看下面例子([参考](http://www.cplusplus.com/reference/mutex/unique_lock/release/))：

```c++
#include <iostream>       // std::cout
#include <vector>         // std::vector
#include <thread>         // std::thread
#include <mutex>          // std::mutex, std::unique_lock

std::mutex mtx;
int count = 0;

void print_count_and_unlock (std::mutex* p_mtx) {
  std::cout << "count: " << count << '\n';
  p_mtx->unlock();
}

void task() {
  std::unique_lock<std::mutex> lck(mtx);
  ++count;
  print_count_and_unlock(lck.release());
}

int main ()
{
  std::vector<std::thread> threads;
  for (int i=0; i<10; ++i)
    threads.emplace_back(task);

  for (auto& x: threads) x.join();

  return 0;
}
```

#### **std::unique_lock::owns_lock**

返回当前 std::unique_lock 对象是否获得了锁。

请看下面例子(参考)：

```c++
#include <iostream>       // std::cout
#include <vector>         // std::vector
#include <thread>         // std::thread
#include <mutex>          // std::mutex, std::unique_lock, std::try_to_lock

std::mutex mtx;           // mutex for critical section

void print_star () {
  std::unique_lock<std::mutex> lck(mtx,std::try_to_lock);
  // print '*' if successfully locked, 'x' otherwise: 
  if (lck.owns_lock())
    std::cout << '*';
  else                    
    std::cout << 'x';
}

int main ()
{
  std::vector<std::thread> threads;
  for (int i=0; i<500; ++i)
    threads.emplace_back(print_star);

  for (auto& x: threads) x.join();

  return 0;
}
```

#### std::unique_lock::operator bool()
与 owns_lock 功能相同，返回当前 std::unique_lock 对象是否获得了锁。

请看下面例子([参考](http://www.cplusplus.com/reference/mutex/unique_lock/operator_bool/))：

```c++
#include <iostream>       // std::cout
#include <vector>         // std::vector
#include <thread>         // std::thread
#include <mutex>          // std::mutex, std::unique_lock, std::try_to_lock

std::mutex mtx;           // mutex for critical section

void print_star () {
  std::unique_lock<std::mutex> lck(mtx,std::try_to_lock);
  // print '*' if successfully locked, 'x' otherwise: 
  if (lck)
    std::cout << '*';
  else                    
    std::cout << 'x';
}

int main ()
{
  std::vector<std::thread> threads;
  for (int i=0; i<500; ++i)
    threads.emplace_back(print_star);

  for (auto& x: threads) x.join();

  return 0;
}
```

#### std::unique_lock::mutex
返回当前 std::unique_lock 对象所管理的 Mutex 对象的指针。

请看下面例子([参考](http://www.cplusplus.com/reference/mutex/unique_lock/mutex/))：

```c++
#include <iostream>       // std::cout
#include <thread>         // std::thread
#include <mutex>          // std::mutex, std::unique_lock, std::defer_lock

class MyMutex : public std::mutex {
  int _id;
public:
  MyMutex (int id) : _id(id) {}
  int id() {return _id;}
};

MyMutex mtx (101);

void print_ids (int id) {
  std::unique_lock<MyMutex> lck (mtx);
  std::cout << "thread #" << id << " locked mutex " << lck.mutex()->id() << '\n';
}

int main ()
{
  std::thread threads[10];
  // spawn 10 threads:
  for (int i=0; i<10; ++i)
    threads[i] = std::thread(print_ids,i+1);

  for (auto& th : threads) th.join();

  return 0;
}
```



# C++11 并发指南五(std::condition_variable 详解)

前面三讲《[C++11 并发指南二(std::thread 详解)](http://www.cnblogs.com/haippy/p/3236136.html)》，《[C++11 并发指南三(std::mutex 详解)](http://www.cnblogs.com/haippy/p/3237213.html)》分别介绍了 `std::thread`，`std::mutex`，`std::future` 等相关内容，相信读者对 C++11  中的多线程编程有了一个最基本的认识，本文将介绍 C++11 标准中 <condition_variable>  头文件里面的类和相关函数。

`<condition_variable > `头文件主要包含了与条件变量相关的类和函数。相关的类包括  `std::condition_variable` 和  `std::condition_variable_any`，还有枚举类型`std::cv_status`。另外还包括函数  `std::notify_all_at_thread_exit()`，下面分别介绍一下以上几种类型。

## std::condition_variable 类介绍

`std::condition_variable` 是条件变量，更多有关条件变量的定义参考[维基百科](http://en.wikipedia.org/wiki/Monitor_(synchronization))。Linux 下使用 `Pthread` 库中的 `pthread_cond_*() `函数提供了与条件变量相关的功能， Windows 则参考 [MSDN](http://msdn.microsoft.com/en-us/library/windows/desktop/ms682052(v=vs.85).aspx)。

当` std::condition_variable `对象的某个 `wait `函数被调用的时候，它使用 `std::unique_lock`(通过 `std::mutex`) 来锁住当前线程。当前线程会一直被阻塞，直到另外一个线程在相同的 `std::condition_variable`  对象上调用了` notification `函数来唤醒当前线程。

std::condition_variable 对象通常使用`std::unique_lock<std::mutex>`来等待，如果需要使用另外的 lockable 类型，可以使用 std::condition_variable_any 类，本文后面会讲到  std::condition_variable_any 的用法。

首先我们来看一个简单的[例子](http://www.cplusplus.com/reference/condition_variable/condition_variable/)

```c++
#include <iostream>                // std::cout
#include <thread>                // std::thread
#include <mutex>                // std::mutex, std::unique_lock
#include <condition_variable>    // std::condition_variable

std::mutex mtx; // 全局互斥锁.
std::condition_variable cv; // 全局条件变量.
bool ready = false; // 全局标志位.

void do_print_id(int id)
{
    std::unique_lock <std::mutex> lck(mtx);
    while (!ready) // 如果标志位不为 true, 则等待...
        cv.wait(lck); // 当前线程被阻塞, 当全局标志位变为 true 之后,
    // 线程被唤醒, 继续往下执行打印线程编号id.
    std::cout << "thread " << id << '\n';
}

void go()
{
    std::unique_lock <std::mutex> lck(mtx);
    ready = true; // 设置全局标志位为 true.
    cv.notify_all(); // 唤醒所有线程.
}

int main()
{
    std::thread threads[10];
    // spawn 10 threads:
    for (int i = 0; i < 10; ++i)
        threads[i] = std::thread(do_print_id, i);

    std::cout << "10 threads ready to race...\n";
    go(); // go!

  for (auto & th:threads)
        th.join();

    return 0;
}
```

执行结果如下：

```c++
concurrency ) ./ConditionVariable-basic1 
10 threads ready to race...
thread 1
thread 0
thread 2
thread 3
thread 4
thread 5
thread 6
thread 7
thread 8
thread 9
```

好了，对条件变量有了一个基本的了解之后，我们来看看` std::condition_variable `的各个成员函数。

## **std::condition_variable 构造函数**

|                    |                                                            |
| ------------------ | ---------------------------------------------------------- |
| default (1)        | `condition_variable(); `                                   |
| copy [deleted] (2) | `condition_variable (const condition_variable&) = delete;` |

`std::condition_variable `的拷贝构造函数被禁用，只提供了默认构造函数。

## **std::condition_variable::wait() 介绍**

|                   |                                                              |
| ----------------- | ------------------------------------------------------------ |
| unconditional (1) | `void wait (unique_lock<mutex>& lck); `                      |
| predicate (2)     | `template <class Predicate>  void wait (unique_lock<mutex>& lck, Predicate pred);` |

`std::condition_variable` 提供了两种 `wait() `函数。当前线程调用` wait() `后将被阻塞(此时当前线程应该获得了锁（`mutex`），不妨设获得锁 `lck`)，直到另外某个线程调用 `notify_* `唤醒了当前线程。

在线程被阻塞时，该函数会自动调用 `lck.unlock() `释放锁，使得其他被阻塞在锁竞争上的线程得以继续执行。另外，一旦当前线程获得通知(`notified`，通常是另外某个线程调用 `notify_*` 唤醒了当前线程)，`wait() `函数也是自动调用` lck.lock()`，使得` lck `的状态和 `wait` 函数被调用时相同。

在第二种情况下（即设置了 `Predicate`），只有当 `pred`条件为 `false `时调用 `wait() `才会阻塞当前线程，并且在收到其他线程的通知后只有当 `pred `为` true` 时才会被解除阻塞。因此第二种情况类似以下代码：

```
while (!pred()) wait(lck);
```

请看下面例子（[参考](http://www.cplusplus.com/reference/condition_variable/condition_variable/wait/)）：

```c++
#include <iostream>                // std::cout
#include <thread>                // std::thread, std::this_thread::yield
#include <mutex>                // std::mutex, std::unique_lock
#include <condition_variable>    // std::condition_variable

std::mutex mtx;
std::condition_variable cv;

int cargo = 0;
bool shipment_available()
{
    return cargo != 0;
}

// 消费者线程.
void consume(int n)
{
    for (int i = 0; i < n; ++i) {
        std::unique_lock <std::mutex> lck(mtx);
        cv.wait(lck, shipment_available);
        std::cout << cargo << '\n';
        cargo = 0;
    }
}

int main()
{
    std::thread consumer_thread(consume, 10); // 消费者线程.

    // 主线程为生产者线程, 生产 10 个物品.
    for (int i = 0; i < 10; ++i) {
        while (shipment_available())
            // yield()调用线程放弃执行，回到准备状态，重新分配cpu资源。所以调用该方法后，可能执行其他线程，也可能还是执行该线程
            std::this_thread::yield();
        std::unique_lock <std::mutex> lck(mtx);
        cargo = i + 1;
        cv.notify_one();
    }

    consumer_thread.join();

    return 0;
}
```

程序执行结果如下：

```c++
concurrency ) ./ConditionVariable-wait 
1
2
3
4
5
6
7
8
9
10
```

## **std::condition_variable::wait_for() 介绍**

|                   |                                                              |
| ----------------- | ------------------------------------------------------------ |
| unconditional (1) | `template <class Rep, class Period>  cv_status wait_for (unique_lock<mutex>& lck,                      const chrono::duration<Rep,Period>& rel_time); ` |
| predicate (2)     | `template <class Rep, class Period, class Predicate>       bool wait_for (unique_lock<mutex>& lck,                      const chrono::duration<Rep,Period>& rel_time, Predicate pred);` |

与` std::condition_variable::wait() `类似，不过 `wait_for `可以指定一个时间段，在当前线程收到通知或者指定的时间 `rel_time` 超时之前，该线程都会处于阻塞状态。而一旦超时或者收到了其他线程的通知，`wait_for` 返回，剩下的处理步骤和 `wait() `类似。

另外，`wait_for` 的重载版本（`predicte(2)`）的最后一个参数` pred `表示 `wait_for `的预测条件，只有当 `pred `条件为 `false `时调用 `wait() `才会阻塞当前线程，并且在收到其他线程的通知后只有当 `pred` 为 `true `时才会被解除阻塞，因此相当于如下代码：

```c++
return wait_until (lck, chrono::steady_clock::now() + rel_time, std::move(pred));
```

请看下面的例子（[参考](http://www.cplusplus.com/reference/condition_variable/condition_variable/wait_for/)），下面的例子中，主线程等待 th 线程输入一个值，然后将 th 线程从终端接收的值打印出来，在 th 线程接受到值之前，主线程一直等待，每个一秒超时一次，并打印一个 "."：

```c++
#include <iostream>           // std::cout
#include <thread>             // std::thread
#include <chrono>             // std::chrono::seconds
#include <mutex>              // std::mutex, std::unique_lock
#include <condition_variable> // std::condition_variable, std::cv_status

std::condition_variable cv;

int value;

void do_read_value()
{
    std::cin >> value;
    cv.notify_one();
}

int main ()
{
    std::cout << "Please, enter an integer (I'll be printing dots): \n";
    std::thread th(do_read_value);

    std::mutex mtx;
    std::unique_lock<std::mutex> lck(mtx);
    while (cv.wait_for(lck,std::chrono::seconds(1)) == std::cv_status::timeout) {
        std::cout << '.';
        std::cout.flush(); // 强行将缓冲区的数据清空
    }

    std::cout << "You entered: " << value << '\n';

    th.join();
    return 0;
}
```

## **std::condition_variable::wait_until 介绍**

|                   |                                                              |
| ----------------- | ------------------------------------------------------------ |
| unconditional (1) | `template <class Clock, class Duration>  cv_status wait_until (unique_lock<mutex>& lck,                        const chrono::time_point<Clock,Duration>& abs_time); ` |
| predicate (2)     | `template <class Clock, class Duration, class Predicate>       bool wait_until (unique_lock<mutex>& lck,                        const chrono::time_point<Clock,Duration>& abs_time,                        Predicate pred);` |

与 `std::condition_variable::wait_for `类似，但是 `wait_until `可以指定一个时间点，在当前线程收到通知或者指定的时间点` abs_time` 超时之前，该线程都会处于阻塞状态。而一旦超时或者收到了其他线程的通知，`wait_until` 返回，剩下的处理步骤和 `wait_until() `类似。

另外，`wait_until` 的重载版本（`predicte(2)`）的最后一个参数` pred` 表示 `wait_until `的预测条件，只有当` pred `条件为 `false` 时调用` wait() `才会阻塞当前线程，并且在收到其他线程的通知后只有当 `pred `为` true `时才会被解除阻塞，因此相当于如下代码：

```c++
while (!pred())
  if ( wait_until(lck,abs_time) == cv_status::timeout)
    return pred();
return true;
```

## **std::condition_variable::notify_one() 介绍**

唤醒某个等待(wait)线程。如果当前没有等待线程，则该函数什么也不做，如果同时存在多个等待线程，则唤醒某个线程是不确定的(unspecified)。

请看下例（[参考](http://www.cplusplus.com/reference/condition_variable/condition_variable/notify_one/)）：

```c++
#include <iostream>                // std::cout
#include <thread>                // std::thread
#include <mutex>                // std::mutex, std::unique_lock
#include <condition_variable>    // std::condition_variable

std::mutex mtx;
std::condition_variable cv;

int cargo = 0; // shared value by producers and consumers

void consumer()
{
    std::unique_lock < std::mutex > lck(mtx);
    while (cargo == 0)
        cv.wait(lck);
    std::cout << cargo << '\n';
    cargo = 0;
}

void producer(int id)
{
    std::unique_lock < std::mutex > lck(mtx);
    cargo = id;
    cv.notify_one();
}

int main()
{
    std::thread consumers[10], producers[10];

    // spawn 10 consumers and 10 producers:
    for (int i = 0; i < 10; ++i) {
        consumers[i] = std::thread(consumer);
        producers[i] = std::thread(producer, i + 1);
    }

    // join them back:
    for (int i = 0; i < 10; ++i) {
        producers[i].join();
        consumers[i].join();
    }

    return 0;
}
```

## **std::condition_variable::notify_all() 介绍**

唤醒所有的等待(wait)线程。如果当前没有等待线程，则该函数什么也不做。请看下面的例子：

```c++
#include <iostream>                // std::cout
#include <thread>                // std::thread
#include <mutex>                // std::mutex, std::unique_lock
#include <condition_variable>    // std::condition_variable

std::mutex mtx; // 全局互斥锁.
std::condition_variable cv; // 全局条件变量.
bool ready = false; // 全局标志位.

void do_print_id(int id)
{
    std::unique_lock <std::mutex> lck(mtx);
    while (!ready) // 如果标志位不为 true, 则等待...
        cv.wait(lck); // 当前线程被阻塞, 当全局标志位变为 true 之后,
    // 线程被唤醒, 继续往下执行打印线程编号id.
    std::cout << "thread " << id << '\n';
}

void go()
{
    std::unique_lock <std::mutex> lck(mtx);
    ready = true; // 设置全局标志位为 true.
    cv.notify_all(); // 唤醒所有线程.
}

int main()
{
    std::thread threads[10];
    // spawn 10 threads:
    for (int i = 0; i < 10; ++i)
        threads[i] = std::thread(do_print_id, i);

    std::cout << "10 threads ready to race...\n";
    go(); // go!

  for (auto & th:threads)
        th.join();

    return 0;
}
```

##  std::condition_variable_any 介绍

与 `std::condition_variable` 类似，只不过 `std::condition_variable_any `的  `wait `函数可以接受任何` lockable `参数，而 `std::condition_variable `只能接受  `std::unique_lock<std::mutex> `类型的参数，除此以外，和` std::condition_variable`  几乎完全一样。

## std::cv_status 枚举类型介绍

|                       |                                                              |
| --------------------- | ------------------------------------------------------------ |
| cv_status::no_timeout | wait_for 或者 wait_until 没有超时，即在规定的时间段内线程收到了通知。 |
| cv_status::timeout    | wait_for 或者 wait_until 超时。                              |

## std::notify_all_at_thread_exit

函数原型为：

```c++
void notify_all_at_thread_exit (condition_variable& cond, unique_lock<mutex> lck);
```

当调用该函数的线程退出时，所有在 cond 条件变量上等待的线程都会收到通知。请看下例（[参考](http://www.cplusplus.com/reference/condition_variable/notify_all_at_thread_exit/)）：

```c++
#include <iostream>           // std::cout
#include <thread>             // std::thread
#include <mutex>              // std::mutex, std::unique_lock
#include <condition_variable> // std::condition_variable

std::mutex mtx;
std::condition_variable cv;
bool ready = false;

void print_id (int id) {
  std::unique_lock<std::mutex> lck(mtx);
  while (!ready) cv.wait(lck);
  // ...
  std::cout << "thread " << id << '\n';
}

void go() {
  std::unique_lock<std::mutex> lck(mtx);
  std::notify_all_at_thread_exit(cv,std::move(lck));
  ready = true;
}

int main ()
{
  std::thread threads[10];
  // spawn 10 threads:
  for (int i=0; i<10; ++i)
    threads[i] = std::thread(print_id,i);
  std::cout << "10 threads ready to race...\n";

  std::thread(go).detach();   // go!

  for (auto& th : threads) th.join();

  return 0;
}
```

好了，到此为止，<condition_variable> 头文件中的两个条件变量类（std::condition_variable 和  std::condition_variable_any）、枚举类型（std::cv_status）、以及辅助函数（std::notify_all_at_thread_exit()）都已经介绍完了。

从下一章开始我会逐步开始介绍 <atomic> 头文件中的内容，后续的文章还会介绍 C++11 的内存模型，涉及内容稍微底层一些，希望大家能够保持兴趣，学完 C++11 并发编程，