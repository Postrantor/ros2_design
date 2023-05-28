---
tip: translate by openai@2023-05-28 11:35:34
...
---
    layout: default
    title: Introduction to Real-time Systems
    permalink: articles/realtime_background.html
    abstract: This article is a brief survey of real-time computing requirements and methods to achieve real-time performance.
    published: true
    author: Jackie Kay
    date_written: 2016-01
    last_modified: 2016-05
    Authors: {{ page.author }}
    Date Written: {{ page.date_written }}
    Last Modified: {% if page.last_modified %}{{ page.last_modified }}{% else %}{{ page.date_written }}{% endif %}
---


This document seeks to summarize the requirements of real-time computing and the challenges of implementing real-time performance.

> 这份文件旨在总结实时计算的要求和实现实时性能的挑战。

It also lays out options for how ROS 2 could be structured to enforce real-time compatibility.

> 它还提出了如何结构化ROS 2以确保实时兼容性的选择。


Robotic systems need to be responsive.

> 机器人系统需要具有反应能力。

In mission critical applications, a delay of less than a millisecond in the system can cause a catastrophic failure.

> .

在任务关键应用中，系统延迟小于1毫秒就可能导致灾难性失败。

For ROS 2 to capture the needs of the robotics community, the core software components must not interfere with the requirements of real-time computing.

> 为了让ROS 2满足机器人社区的需求，核心软件组件必须不会影响实时计算的要求。

## Definition of Real-time Computing


The definition of real-time computing requires the definition of a few other key terms:

> 实时计算的定义需要定义其他一些关键术语：


- Determinism: A system is deterministic if it always produces the same output for a known input. The output of a nondeterministic system will have random variations.

> -决定论：如果一个系统对已知输入总是产生相同的输出，则该系统是决定性的。非决定性系统的输出将具有随机变化。


- Deadline: A deadline is the finite window of time in which a certain task must be completed.

> 截止日期：截止日期是一个有限的时间窗口，在此窗口内必须完成某项任务。


- Quality of Service: The overall performance of a network. Includes factors such as bandwith, throughput, availability, jitter, latency, and error rates.

> 服务质量：网络的整体性能。包括带宽、吞吐量、可用性、抖动、延迟和错误率等因素。


Real-time software guarantees correct computation at the correct time.

> 实时软件保证正确的计算在正确的时间。


*Hard real-time* software systems have a set of strict deadlines, and missing a deadline is considered a system failure.

> *硬实时*软件系统有一套严格的截止日期，错过截止日期被视为系统故障。

Examples of hard real-time systems: airplane sensor and autopilot systems, spacecrafts and planetary rovers.

> 例子：飞机传感器和自动驾驶系统、航天器和行星漫游者等硬实时系统。


*Soft real-time* systems try to reach deadlines but do not fail if a deadline is missed.

> 软实时系统试图达到截止日期，但如果错过截止日期则不会失败。

However, they may degrade their quality of service in such an event to improve responsiveness.

> 然而，在这种情况下，他们可能会降低服务质量以提高响应能力。

Examples of soft real-time systems: audio and video delivery software for entertainment (lag is undesirable but not catastrophic).

> 例子：娱乐音频和视频传输软件（滞后不太理想但也不会导致灾难性后果）


*Firm real-time systems* treat information delivered/computations made after a deadline as invalid.

> *实时系统*对超过期限交付/计算的信息视为无效。

Like soft real-time systems, they do not fail after a missed deadline, and they may degrade QoS if a deadline is missed (1).

> 就像软实时系统一样，如果错过了期限，它们不会失败，如果错过了期限，可能会降低QoS（1）。

Examples of firm real-time systems: financial forecast systems, robotic assembly lines (2).

> 例子：实时系统：金融预测系统，机器人装配线（2）。


Real-time computer systems are often associated with low-latency systems.

> 实时计算机系统通常与低延迟系统有关。

Many applications of real-time computing are also low-latency applications (for example, automated piloting systems must be reactive to sudden changes in the environment).

> 许多实时计算的应用也是低延迟应用（例如，自动驾驶系统必须对环境的突变作出反应）。

However, a real-time system is not defined by low latency, but by a deterministic schedule: it must be guaranteed that the system finishes a certain task by a certain time.

> 然而，实时系统不是由低延迟来定义的，而是由确定的调度来定义的：必须保证系统在某一时刻完成某项任务。

Therefore, it is important that the latency in the system be measurable and a maximum allowable latency for tasks be set.

> 因此，重要的是系统中的延迟可以可测量，并且设定最大允许的任务延迟。


A real-time computer system needs both an operating system that operates in real-time and user code that delivers deterministic execution.

> 一个实时计算机系统需要一个实时操作系统和能够提供确定性执行的用户代码。

Neither deterministic user code on a non-real-time operating system or nondeterministic code on a real-time operating system will result in real-time performance.

> 无论是在非实时操作系统上的确定性用户代码，还是在实时操作系统上的非确定性代码，都不会导致实时性能。


Some examples of real-time environments:

> 一些实时环境的例子：


- The `RT_PREEMPT` Linux kernel patch, which modifies the Linux scheduler to be fully preemptible (3).

> - `RT_PREEMPT` Linux内核补丁，修改Linux调度程序，使其完全可抢占（3）。


- Xenomai, a POSIX-compliant co-kernel (or hypervisor) that provides a real-time kernel cooperating with the Linux kernel. The Linux kernel is treated as the idle task of the real-time kernel's scheduler (the lowest priority task).

> Xenomai是一种符合POSIX标准的共内核（或虚拟机），它提供一个实时内核，与Linux内核协同工作。Linux内核被视为实时内核调度器的空闲任务（最低优先级任务）。


- RTAI, an alternative co-kernel solution.

> RTAI，一种替代的共内核解决方案。


- QNX Neutrino, a POSIX-compliant real-time operating system for mission-critical systems.

> QNX Neutrino，一种符合POSIX标准的实时操作系统，用于关键任务系统。

## Best Practices in Real-time Computing


In general, an operating system can guarantee that the tasks it handles for the developer, such as thread scheduling, are deterministic, but the OS may not guarantee that the developer's code will run in real-time.

> 一般而言，操作系统可以保证它为开发人员处理的任务，例如线程调度，是确定的，但操作系统可能不能保证开发人员的代码能够实时运行。

Therefore, it is up to the developer to know what the determinstic guarantees of an existing system are, and what she must do to write hard real-time code on top of the OS.

> 因此，开发人员必须了解现有系统的确定性保证，以及她必须做什么来在操作系统之上编写硬实时代码。


In this section, various strategies for developing on top of a real-time OS are explored, since these strategies might be applicable to ROS 2.

> 在本节中，将探讨在实时操作系统上开发的各种策略，因为这些策略可能适用于ROS 2。

The patterns focus on the use case of C/C++ development on Linux-based real-time OS's (such as `RT_PREEMPT`), but the general concepts are applicable to other platforms.

> 这些模式主要关注基于Linux的实时操作系统（比如`RT_PREEMPT`）上的C/C++开发的用例，但是这些概念也适用于其他平台。

Most of the patterns focus on workarounds for blocking calls in the OS, since any operation that involves blocking for an indeterminate amount of time is nondeterministic.

> 大多数模式都集中在操作系统中阻止呼叫的解决方案上，因为任何涉及阻塞不定时间的操作都是不确定的。


It is a common pattern to section real-time code into three parts; a non real-time safe section at the beginning of a process that preallocates memory on the heap, starts threads, etc., a real-time safe section (often implemented as a loop), and a non-real-time safe "teardown" section that deallocates memory as necessary, etc.

> 这是一个常见的模式，将实时代码分为三个部分：非实时安全部分位于过程的开头，在堆上预分配内存、启动线程等；实时安全部分（通常实现为循环）；非实时安全的“拆卸”部分，必要时释放内存等。

The "real-time code path" refers to the middle section of the execution.

> "实时代码路径"指的是执行的中间部分。

### Memory management


Proper memory management is critical for real-time performance.

> 正确的内存管理对实时性能至关重要。

In general, the programmer should avoid page faults in the real-time code path.

> 一般来说，程序员应该避免实时代码路径中的页面故障。

During a page fault, the CPU pauses all computation and loads the missing page from disk into RAM (or cache, or registers).

> 在页面错误时，CPU暂停所有计算，并将丢失的页面从磁盘加载到RAM（或缓存或寄存器）中。

Loading data from disk is a slow and unpredictable operation.

> 从磁盘加载数据是一个缓慢而不可预测的操作。

However, page faults are necessary or else the computer will run out of memory.

> 然而，页面错误是必要的，否则计算机会耗尽内存。

The solution is to avoid pagefaults.

> 解决方案是避免页面错误。


Dynamic memory allocation can cause poor real-time performance.

> 动态内存分配可能会降低实时性能。

Calls to `malloc/new` and `free/delete` will probably result in pagefaults.

> 调用malloc/new和free/delete可能会导致页面故障。

Additionally, the heap allocates and frees memory blocks in such a way that leads to memory fragmentation, which creates poor performance for reads and writes, since the OS may have to scan for an indeterminate amount of time for a free memory block.

> 此外，堆内存分配和释放内存块的方式导致内存碎片化，这会降低读写性能，因为操作系统可能需要扫描不确定的时间来寻找一个空闲的内存块。

#### Lock memory, prefault stack

```cpp
    if (mlockall(MCL_CURRENT|MCL_FUTURE) == -1) {
      perror("mlockall failed");
      exit(-2);
    }
    unsigned char dummy[MAX_SAFE_STACK];
    
    memset(dummy, 0, MAX_SAFE_STACK);
```


[`mlockall`](http://linux.die.net/man/2/mlockall) is a Linux system call for locking the process's virtual address space into RAM, preventing the memory that will be accessed by the process from getting paged into swap space.

> mlockall是Linux系统调用，用于将进程的虚拟地址空间锁定到RAM中，以防止进程访问的内存被页面交换到交换空间。


This code snippet, when run at the beginning of a thread's lifecycle, ensures that no pagefaults occur while the thread is running.

> 这段代码片段，在线程生命周期开始时运行，可以确保线程运行期间不会发生页面故障。
`mlockall` locks the stack for the thread.

The [`memset`](http://linux.die.net/man/3/memset) call pre-loads each block of memory of the stack into the cache, so that no pagefaults will occur when the stack is accessed (3).

> `memset`调用预先将堆栈的每个内存块加载到缓存中，因此在访问堆栈时不会发生页错误（3）。

#### Allocate dynamic memory pool

```cpp
    if (mlockall(MCL_CURRENT | MCL_FUTURE))
      perror("mlockall failed:");
    
    /* Turn off malloc trimming.*/
    mallopt(M_TRIM_THRESHOLD, -1);
    
    /* Turn off mmap usage. */
    mallopt(M_MMAP_MAX, 0);
    
    page_size = sysconf(_SC_PAGESIZE);
    buffer = malloc(SOMESIZE);
    
    for (i=0; i < SOMESIZE; i+=page_size) {
      buffer[i] = 0;
    }
    free(buffer);
```


The intro to this section stated that dynamic memory allocation is usually not real-time safe.

> 本节的介绍指出，动态内存分配通常不是实时安全的。

However, this code snippet shows how to make dynamic memory allocation real-time safe (mostly).

> 然而，这段代码片段展示了如何使动态内存分配实时安全（大部分情况下）。

It locks the virtual address space to a fixed size, disallows returning deallocated memory to the kernel via `sbrk`, and disables `mmap`.

> 它将虚拟地址空间锁定到一个固定的大小，不允许通过`sbrk`将已释放的内存返回给内核，并禁用`mmap`。

This effectively locks a pool of memory in the heap into RAM, which prevents page faults due to `malloc` and `free` (3).

> 这有效地将堆中的一块内存锁定到RAM中，从而防止由`malloc`和`free`（3）引起的页面故障。


Pros:

> 优点：


- Can use malloc/new, free/delete, and even STL containers

> 可以使用malloc/new、free/delete，甚至是STL容器。


Cons:

> 缺点：


- Platform/implementation dependent

> 依赖平台/实施的

- Must accurately predict bounded memory size for the process!

> 必须准确预测进程的有限内存大小！

- Using STL containers is therefore dangerous (unbounded sizes)

> 使用STL容器因此很危险（无界大小）

- In practice, only works for processes with small memory footprint

> 在实践中，只适用于具有小内存占用的进程。

#### Custom real-time safe memory allocators


The default allocator on most operating systems is not optimized for real-time safety.

> 大多数操作系统的默认分配器不适用于实时安全。

However, there is another strategy that is an exception to the "avoid dynamic memory allocation" rule.

> 然而，还有一种策略是对“避免动态内存分配”规则的例外。

Research into alternative dynamic memory allocators is a rich research topic (8).

> 研究可替代的动态内存分配器是一个丰富的研究课题（8）。


One such alternative allocator is TLSF (Two-Level Segregate Fit).

> 一种替代分配器是TLSF（双级分离适配）。

It is also called the O(1) allocator, since the time cost of `malloc`, `free`, and `align` operations under TLSF have a constant upper bound.

> 它也被称为O(1)分配器，因为TLSF下的`malloc`、`free`和`align`操作的时间成本具有恒定的上界。

It creates a low level of fragmentation.

> 它可以创造低水平的碎片化。

The disadvantages of TLSF are that it is not thread safe and that its current implementation is architecture specific: it assumes the system can make 4-byte aligned accesses.

> TLSF的缺点是它不是线程安全的，而且它目前的实现是特定于架构的：它假设系统可以进行4字节对齐的访问。


Pros:

> 优点：


- Can safely allocate memory in a program with memory bounds that are unknown at runtime or compile

> 在运行时或编译时未知的内存边界的程序中可以安全地分配内存。

- Advantages vary depending on the choice of allocator

> 优势取决于选择的分配器。


Cons:

> 缺点：


- Implementations of custom allocators may not be well tested, since they are less widely used

> 定制分配器的实现可能没有得到很好的测试，因为它们的使用范围较小。

- Extra dependency, potentially more code complexity

> - 额外的依赖，可能会带来更多的代码复杂性

- Drawbacks vary depending on the choice of allocator

> 取決於分配器的選擇，缺點也會有所不同。

#### Global variables and (static) arrays


Global variables are preallocated at the start of a process, thus assigning and accessing them is real-time safe.

> 全局变量在进程开始时预先分配，因此赋值和访问它们是实时安全的。

However, this strategy comes with the many disadvantages of using global variables.

> 然而，这种策略有许多使用全局变量的缺点。

#### Cache friendliness for pointer and vtable accesses


Classes with many levels of inheritance may not be real-time safe because of vtable overhead access.

> 类具有多个继承层次的类可能不是实时安全的，因为会有vtable开销访问。

When executing an inherited function, the program needs to access the data used in the function, the vtable for the class, and the instructions for the function, which are all stored in different parts of memory, and may or may not be stored in cache together (5).

> 当执行继承函数时，程序需要访问函数中使用的数据、类的vtable以及函数的指令，它们都存储在不同的内存部分，有可能会一起存储在缓存中（5）。


In general, C++ patterns with poor cache locality are not well-suited to real-time environments.

> 一般来说，缓存局部性差的C++模式不适合实时环境。

Another such pattern is the opaque pointer idiom (PIMPL), which is convenient for ABI compatibility and speeding up compile times.

> 另一种这样的模式是不透明指针惯用法（PIMPL），它对于ABI兼容性和加速编译时间很方便。

However, bouncing between the memory location for the object and its private data pointer causes the cache to "spill" as it loads one chunk of memory and then another, unrelated chunk for almost every function in the PIMPLized object.

> 然而，在对象的内存位置和其私有数据指针之间来回跳转会导致缓存“溢出”，因为它为几乎每个PIMPL化对象的函数加载一块内存，然后又加载另一块不相关的内存。

#### Exceptions


Handling exceptions can incur a large performance penalty.

> 处理异常会导致性能损失很大。

Running into an exception tends to push a lot of memory onto the stack, which is often a limited resource in real-time programming.

> 遇到异常往往会在堆栈上压入大量内存，这在实时编程中往往是一种有限的资源。

But if exceptions are used properly, they should not be a concern to real-time programmers (since they indicate a place in the program with undefined behavior and are integral to debugging) (6).

> 如果异常被正确使用，它们不应该成为实时程序员的担忧（因为它们指示程序中有未定义行为的地方，并且是调试的重要组成部分）(6)。

#### Know your problem


Different programs have different memory needs, thus memory management strategies vary between applications.

> 不同的程序有不同的内存需求，因此内存管理策略因应用程序而有所不同。


- Required memory size known at compile time

> 所需内存大小在编译时已知


  - Example: publishing a message of a fixed size.

> 例如：发布固定大小的消息。

  - Solution: use stack allocation with fixed-size objects.

> 解决方案：使用具有固定大小对象的堆栈分配。


- Required memory size known at runtime, before real-time execution.

> 需要在实时执行之前，在运行时确定内存大小。


  - Example: publishing a message of a size specified on the command line.

> 例如：根据命令行指定的大小发布消息

  - Preallocate variable size objects on the heap once required size is known, then execute real-time code.

> 预先在堆上分配变量大小的对象，一旦确定了所需的大小，就可以执行实时代码。


- Required memory size computed during real-time

> 所需内存大小由实时计算得出


  - Example: a message received by the robot's sensors determines the size of the messages it publishes.

> 例如：机器人传感器接收到的消息决定它发布消息的大小。


  - Multiple solutions exist

> 多种解决方案存在

    - [Object pools](https://en.wikipedia.org/wiki/Object_pool_pattern)
    - [TLSF O(1) memory allocation](http://www.gii.upv.es/tlsf/)
    - Use stack allocation and fail if allocated memory is exceeded

### Device I/O


Interacting with physical devices (disk I/O, printing to the screen, etc.) may introduce unacceptable latency in the real-time code path, since the process is often forced to wait on slow physical phenomena.

> 与物理设备交互（磁盘I/O，打印到屏幕等）可能会在实时代码路径中引入不可接受的延迟，因为该过程通常被迫等待缓慢的物理现象。

Additionally, many I/O calls such as `fopen` result in pagefaults.

> 另外，许多I/O调用，比如`fopen`，会导致页面错误。


Keep disk reads/writes at the beginning or end of the program, outside of the RT code path.

> 在实时代码路径之外，将磁盘读/写操作放在程序的开头或结尾处。


Spin up threads that are not scheduled in real-time to print output to the screen.

> 启动不按实时调度的线程，将输出打印到屏幕上。

### Multithreaded Programming and Synchronization


Real-time computation requirements change the typical paradigm of multithreaded programming.

> 实时计算要求改变了多线程编程的典型范式。

Program execution may not block asynchronously, and threads must be scheduled deterministically.

> 程序执行不能异步阻塞，线程必须以确定性方式调度。

A real-time operating system will fulfill this scheduling requirement, but there are still pitfalls for the developer to fall into.

> 一个实时操作系统可以满足这一调度要求，但开发人员仍然有可能陷入陷阱。

This section provides guidelines for avoiding these pitfalls.

> 这一节提供了避免这些陷阱的指南。

#### Thread creation guidelines


Create threads at the start of the program.

> 在程序开始时创建线程。

This confines the nondeterministic overhead of thread allocation to a defined point in the process.

> 这将线程分配的非确定性开销限制在过程的一个明确点上。


Create high priority (but not 99) threads with a FIFO, Round Robin, or Deadline scheduler (see POSIX [sched](http://man7.org/linux/man-pages/man7/sched.7.html) API).

> 使用FIFO，Round Robin或Deadline调度器（参见POSIX [sched](http://man7.org/linux/man-pages/man7/sched.7.html) API）创建高优先级（但不是99）线程。

#### Avoid priority inversion


Priority inversion can occur on a system with a preemptive task scheduler and results in deadlock.

> 优先级反转可能发生在具有抢占式任务调度器的系统上，并导致死锁。

It occurs when: a low-priority task acquires a lock and is then pre-empted by a medium-priority task, then a high-priority task acquires the lock held by the low-priority task.

> 这种情况发生时：一个低优先级的任务获得锁定，然后被一个中等优先级的任务抢占，然后一个高优先级的任务获得由低优先级任务持有的锁定。


The three tasks are stuck in a triangle: the high-priority task is blocked on the low-priority task, which is blocked on the medium-priority task because it was preempted by a task with a higher priority, and the medium-priority task is also blocked on a task with a higher priority.

> 三个任务被困在一个三角形中：高优先级任务被低优先级任务阻塞，而低优先级任务又被更高优先级的任务抢占，中优先级任务也被更高优先级的任务阻塞。


Here are the some solutions to priority inversion:

> 这里有一些解决优先级反转的解决方案：


- Don't use blocking synchronization primitives

> 不要使用阻塞同步原语。

- Disable preemption for tasks holding locks (can lead to jitter)

> 禁用持有锁的任务的抢占（可能导致抖动）

- Increase priority of task holding a lock

> 增加任务持有锁的优先级

- Use priority inheritance: a task that owns a lock inherits the priority of a task that tries to acquire the lock

> 使用优先级继承：拥有锁的任务继承试图获取锁的任务的优先级。

- Use lock-free data structures and [algorithms](http://www.1024cores.net/home/lock-free-algorithms)

> 使用无锁数据结构和[算法](http://www.1024cores.net/home/lock-free-algorithms)

#### Timing shots


One real-time synchronization technique is when a thread calculates its next "shot" (the start of its next execution period).

> 一种实时同步技术是，当一个线程计算它的下一次“射击”（它下一个执行周期的开始）时。

For example, if a thread is required to provide an update every 10 milliseconds, and it must complete an operation that takes 3-6 milliseconds, the thread should get the time before the operation, do the operation, and then wait for the remaining 7-4 milliseconds, based on the time measured after the operation.

> 例如，如果一个线程需要每10毫秒提供一次更新，并且必须完成一个需要3-6毫秒的操作，那么线程应该在操作之前获取时间，完成操作，然后根据操作后测量的时间等待剩余的7-4毫秒。


The most important consideration for the developer is to use a high precision timer, such as `nanosleep` on Linux platforms, while waiting.

> 开发者最重要的考虑是在等待时使用高精度计时器，比如Linux平台上的`nanosleep`。

Otherwise the system will experience drift.

> 否则系统会出现漂移。

#### Spinlocks


Spinlocks tend to cause clock drift.

> 自旋锁往往会导致时钟漂移。

The developer should avoid implementing his own spinlocks.

> 开发者应该避免实现自己的自旋锁。

The RT Preempt patch replaces much of the kernel's spinlocks with mutexes, but this might not be guaranteed on all platforms.

> RT Preempt补丁用互斥量取代了内核的大部分自旋锁，但这在所有平台上可能无法保证。

#### Avoid fork


[`fork`](http://linux.die.net/man/3/memset) is not real-time safe because it is implemented using [copy-on-write](https://en.wikipedia.org/wiki/Copy-on-write).

> `[Fork](http://linux.die.net/man/3/memset)不是实时安全的，因为它是使用[Copy-on-Write](https://en.wikipedia.org/wiki/Copy-on-write)实现的。`

This means that when a forked process modifies a page of memory, it gets its own copy of that page.

> 这意味着，当一个分叉进程修改一页内存时，它会得到自己的该页副本。

This leads to page faults!

> 这导致了页面错误！


Page faults should be avoided in real-time programming, so Linux `fork`, as well as programs that call `fork`, should be avoided.

> 在实时编程中应该避免页面错误，因此应该避免使用Linux的`fork`以及调用`fork`的程序。

## Testing and Performance Benchmarking

### cyclictest


[`cyclictest`](http://manpages.ubuntu.com/manpages/trusty/man8/cyclictest.8.html) is a simple Linux command line tool for measuring the jitter of a real-time environment.

> cyclictest是一款简单的Linux命令行工具，用于测量实时环境的抖动。

It takes as input a number of threads, a priority for the threads, and a scheduler type.

> 它接受线程数量、线程优先级和调度器类型作为输入。

It spins up `n` threads that sleep regular intervals (the sleep period can also be specified from the command line) (7).

> 它启动`n`个线程，它们以定期间隔（可以从命令行指定睡眠周期）进行休眠（7）。


For each thread, `cyclictest` measures the time between when the thread is supposed to wake up and when it actually wakes up.

> 对于每个线程，`cyclictest`测量线程应该唤醒和实际唤醒之间的时间。

The variability of this latency is the scheduling jitter in the system.

> 这个延迟的可变性是系统中的调度抖动。

If there are processes with non-deterministic blocking behavior running in the system, the average latency will grow to a large number (on the order of milliseconds), since the scheduler cannot meet the deadlines of the periodically sleeping threads profiled in the program.

> 如果系统中运行有具有非确定性阻塞行为的进程，平均延迟会增加到一个较大的数字（以毫秒为单位），因为调度程序无法满足程序中分析的定期休眠线程的截止日期。

### Instrumenting code for testing


A more precise way to measure the scheduling jitter in a program is to instrument the periodic real-time update loop of existing code to record scheduling jitter.

> 更精确地测量程序中的调度抖动，可以通过对现有代码的定期实时更新循环进行检测，以记录调度抖动。


A proposed header for a minimal library for real-time code instrumentation can be found here: [rttest.h](https://github.com/ros2/realtime_support/blob/master/rttest/include/rttest/rttest.h).

> 可以在这里找到一个用于实时代码检测的最小库的建议头文件：[rttest.h](https://github.com/ros2/realtime_support/blob/master/rttest/include/rttest/rttest.h)。

### Pagefaults


The Linux system call [`getrusage`](http://linux.die.net/man/2/getrusage) returns statistics about many resource usage events relevant to real-time performance, such as minor and major pagefaults, swaps, and block I/O.

> Linux系统调用[`getrusage`](http://linux.die.net/man/2/getrusage)返回与实时性能相关的许多资源使用事件的统计信息，例如次要和主要页面错误、交换和块I / O。

It can retrieve these statistics for an entire process or for one thread.

> 它可以为整个进程或单个线程检索这些统计信息。

Thus it is simple to instrument code to check for these events.

> 因此，检查这些事件很容易进行代码检测。

In particular, `getrusage` should be called right before and right after the real-time section of the code, and these results should be compared, since `getrusage` collects statistics about the entire duration of the thread/process.

> 特别是，应该在代码的实时部分之前和之后调用`getrusage`，并且应该比较这些结果，因为`getrusage`收集有关线程/进程整个持续时间的统计信息。


Collecting these statistics gives an indication of what events could cause the latency/scheduling jitter measured by the previously described methods.

> 采集这些统计数据可以指示什么事件可能导致由先前描述的方法测量的延迟/调度抖动。

## Design Guidelines for ROS 2

## Implementation strategy


With judicious application of the performance patterns and benchmarking tests proposed in this document, implementing real-time code in C/C++ is feasible.

> 通过本文档提出的性能模式和基准测试的明智应用，实现C/C++中的实时代码是可行的。

The question of how ROS 2 will achieve real-time compatibility remains.

> 问题是ROS 2如何实现实时兼容性仍然存在。


It is acceptable for the setup and teardown stages of the ROS node lifecycle to not be real-time safe.

> 可以接受ROS节点生命周期的设置和拆卸阶段不是实时安全的。

However, interacting with ROS interfaces, particularly within an intra-process context, should be real-time safe, since these actions could be on the real-time code path of a process.

> 然而，与ROS接口交互，特别是在进程内部上下文中，应该是实时安全的，因为这些操作可能会在进程的实时代码路径上。


There are a few possible strategies for the real-time "hardening" of existing and future ROS 2 code:

> 有几种可能的策略可以用于实时“加固”现有和未来的ROS 2代码：


- Create a configuration option for the stack to operate in "real-time friendly" mode.

> 创建一个配置选项，使堆栈以"实时友好"模式运行。


  - Pros:

> .

优点：

    - Could allow user to dynamically switch between real-time and non-real-time modes.


  - Cons:

> 缺点:

    - Refactoring overhead.
      Integrating real-time code with existing code may be intractable.


- Implement a new real-time stack (rclrt, rmwrt, etc.) designed with real-time computing in mind.

> 实现一个新的实时堆栈（rclrt、rmwrt等），旨在实时计算。


  - Pros:

> 优点：

    - Easier to design and maintain.

    - Real-time code is "quarantined" from existing code.
      Can fully optimize library for real-time application.


  - Cons:

> 缺点：

    - More packages to write and maintain.
    - Potentially less convenient for the user.


- Give the option for real-time safety up to a certain point in the stack, and implement a real-time safe language wrapper (rclrt or rclc)

> 提供到堆栈某一特定点的实时安全选项，并实现实时安全语言包装器（rclrt 或 rclc）。


  - Pros:

> 优点：

    - Existing code is designed for this refactoring to be fairly easy
    - User can provide memory allocation strategy to rcl/rmw to ensure deterministic operation
    - Synchronization happens at the top the language/OS-specific layer, so refactoring rcl/rmw is easier
    - May be easier to support multiple embedded platforms with different wrappers


  - Cons:

> 缺点：

    - More code to test for real-time safety
    - More flexibility for user-developer may mean more complexity


The third option is most appealing because it represents the least amount of work for the most number of benefits.

> 第三个选项最具吸引力，因为它代表最少的工作量，最多的好处。

## Sources


1. Stefan Petters, [Presentation on Real-Time Systems](http://www.cse.unsw.edu.au/~cs9242/08/lectures/09-realtimex2.pdf)

> 1. 斯蒂芬·彼特（Stefan Petters）的[实时系统演示](http://www.cse.unsw.edu.au/~cs9242/08/lectures/09-realtimex2.pdf)

2. [Differences between hard real-time, soft real-time, and firm real-time](http://stackoverflow.com/questions/17308956/differences-between-hard-real-time-soft-real-time-and-firm-real-time), Stack Overflow

> 2. [硬实时、软实时和稳实时的区别](http://stackoverflow.com/questions/17308956/differences-between-hard-real-time-soft-real-time-and-firm-real-time), Stack Overflow

3. [Real-Time Linux Wiki](https://rt.wiki.kernel.org/)

> .

3. [Real-Time Linux 维基](https://rt.wiki.kernel.org/)

4. [Real-time operating system, Wikipedia](https://en.wikipedia.org/wiki/Real-time_operating_system#Memory_allocation)

> 现时操作系统，维基百科（https://en.wikipedia.org/wiki/Real-time_operating_system#Memory_allocation）：内存分配

5. Scott Salmon, [How to make C++ more real-time friendly](http://www.embedded.com/design/programming-languages-and-tools/4429790/2/How-to-make-C--more-real-time-friendly)

> 5. Scott Salmon，[如何让C++更加实时友好](http://www.embedded.com/design/programming-languages-and-tools/4429790/2/How-to-make-C--more-real-time-friendly)

6. Stack Overflow, [Are Exceptions still undesirable in Realtime environment?](http://stackoverflow.com/questions/5257190/are-exceptions-still-undesirable-in-realtime-environment)

> 「6. Stack Overflow，[在实时环境中异常仍然不受欢迎吗？](http://stackoverflow.com/questions/5257190/are-exceptions-still-undesirable-in-realtime-environment)」的简体中文翻译是：

Stack Overflow，[在实时环境中异常是否仍然不受欢迎？](http://stackoverflow.com/questions/5257190/are-exceptions-still-undesirable-in-realtime-environment)

7. Pavel Moryc, [Task jitter measurement under RTLinux operating system](https://fedcsis.org/proceedings/2007/pliks/48.pdf)

> 7. 帕维尔·莫里奇（Pavel Moryc），[在RTLinux操作系统下的任务抖动测量](https://fedcsis.org/proceedings/2007/pliks/48.pdf)

8. Alfons Crespo, Ismael Ripoll and Miguel Masmano, [Dynamic Memory Management for Embedded Real-Time Systems](http://www.gii.upv.es/tlsf/files/papers/tlsf_slides.pdf)

> 8. Alfons Crespo、Ismael Ripoll 和 Miguel Masmano，《嵌入式实时系统的动态内存管理》（http://www.gii.upv.es/tlsf/files/papers/tlsf_slides.pdf）

9. Miguel Masmno, [TLSF: A New Dynamic Memory Allocator for Real-Time Systems](http://www.gii.upv.es/tlsf/files/ecrts04_tlsf.pdf)

> 9. Miguel Masmno，《TLSF：实时系统的新动态内存分配器》（http://www.gii.upv.es/tlsf/files/ecrts04_tlsf.pdf）
