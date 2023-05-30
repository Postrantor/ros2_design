---
tip: translate by openai@2023-05-29 23:24:42
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

This document seeks to summarize the requirements of real-time computing and the challenges of implementing real-time performance. It also lays out options for how ROS 2 could be structured to enforce real-time compatibility.

> 本文旨在总结实时计算的要求和实现实时性能的挑战。它还提出了如何结构化 ROS 2 以确保实时兼容性的选择。

Robotic systems need to be responsive. In mission critical applications, a delay of less than a millisecond in the system can cause a catastrophic failure. For ROS 2 to capture the needs of the robotics community, the core software components must not interfere with the requirements of real-time computing.

> 系统需要响应迅速。在关键任务中，系统延迟不到一毫秒就可能导致灾难性失败。为了满足机器人社区的需求，ROS 2 的核心软件组件必须不影响实时计算的要求。

## Definition of Real-time Computing

The definition of real-time computing requires the definition of a few other key terms:

> 实时计算的定义需要定义一些其他关键术语：

- Determinism: A system is deterministic if it always produces the same output for a known input. The output of a nondeterministic system will have random variations.

- Deadline: A deadline is the finite window of time in which a certain task must be completed.

- Quality of Service: The overall performance of a network. Includes factors such as bandwith, throughput, availability, jitter, latency, and error rates.

Real-time software guarantees correct computation at the correct time.

> 实时软件保证正确的计算在正确的时间。

_Hard real-time_ software systems have a set of strict deadlines, and missing a deadline is considered a system failure. Examples of hard real-time systems: airplane sensor and autopilot systems, spacecrafts and planetary rovers.

> *硬实时*软件系统具有一套严格的最后期限，而错过最后期限被视为系统失败。硬实时系统的例子：飞机传感器和自动驾驶系统、航天器和行星漫游者。

_Soft real-time_ systems try to reach deadlines but do not fail if a deadline is missed. However, they may degrade their quality of service in such an event to improve responsiveness. Examples of soft real-time systems: audio and video delivery software for entertainment (lag is undesirable but not catastrophic).

> 软实时系统试图达到截止日期，但如果未能达到截止日期也不会失败。但是，如果发生这种情况，它们可能会降低服务质量以提高响应性。软实时系统的例子：用于娱乐的音频和视频传输软件(滞后不受欢迎但不会造成灾难性后果)。

_Firm real-time systems_ treat information delivered/computations made after a deadline as invalid. Like soft real-time systems, they do not fail after a missed deadline, and they may degrade QoS if a deadline is missed (1). Examples of firm real-time systems: financial forecast systems, robotic assembly lines (2).

> *实时系统*处理超过期限交付/完成的信息无效。与软实时系统一样，它们在错过期限后不会失败，如果错过期限可能会降低 QoS(1)。实时系统的例子：金融预测系统，机器人装配线(2)。

Real-time computer systems are often associated with low-latency systems. Many applications of real-time computing are also low-latency applications (for example, automated piloting systems must be reactive to sudden changes in the environment). However, a real-time system is not defined by low latency, but by a deterministic schedule: it must be guaranteed that the system finishes a certain task by a certain time. Therefore, it is important that the latency in the system be measurable and a maximum allowable latency for tasks be set.

> 实时计算机系统通常与低延迟系统有关。实时计算的许多应用也是低延迟应用(例如，自动驾驶系统必须对环境的突然变化作出反应)。但是，实时系统不是由低延迟定义的，而是由确定的计划定义的：必须保证系统在某个时间完成某项任务。因此，重要的是系统的延迟可以测量，并且可以为任务设定最大允许延迟。

A real-time computer system needs both an operating system that operates in real-time and user code that delivers deterministic execution. Neither deterministic user code on a non-real-time operating system or nondeterministic code on a real-time operating system will result in real-time performance.

> 一个实时计算机系统需要实时操作系统和提供确定性执行的用户代码。非实时操作系统上的确定性用户代码或实时操作系统上的非确定性代码都不会导致实时性能。

Some examples of real-time environments:

> 一些实时环境的例子：

- The `RT_PREEMPT` Linux kernel patch, which modifies the Linux scheduler to be fully preemptible (3).
- Xenomai, a POSIX-compliant co-kernel (or hypervisor) that provides a real-time kernel cooperating with the Linux kernel. The Linux kernel is treated as the idle task of the real-time kernel's scheduler (the lowest priority task).
- RTAI, an alternative co-kernel solution.
- QNX Neutrino, a POSIX-compliant real-time operating system for mission-critical systems.

## Best Practices in Real-time Computing

In general, an operating system can guarantee that the tasks it handles for the developer, such as thread scheduling, are deterministic, but the OS may not guarantee that the developer's code will run in real-time. Therefore, it is up to the developer to know what the determinstic guarantees of an existing system are, and what she must do to write hard real-time code on top of the OS.

> 一般来说，操作系统可以保证它为开发者处理的任务(如线程调度)是确定性的，但操作系统可能不能保证开发者的代码可以实时运行。因此，开发者需要了解现有系统的确定性保证以及如何在操作系统上编写硬实时代码。

In this section, various strategies for developing on top of a real-time OS are explored, since these strategies might be applicable to ROS 2. The patterns focus on the use case of C/C++ development on Linux-based real-time OS's (such as `RT_PREEMPT`), but the general concepts are applicable to other platforms. Most of the patterns focus on workarounds for blocking calls in the OS, since any operation that involves blocking for an indeterminate amount of time is nondeterministic.

> 在本节中，探索了在实时操作系统上进行开发的各种策略，因为这些策略可能适用于 ROS 2。模式重点关注基于 Linux 的实时操作系统(例如`RT_PREEMPT`)上的 C / C ++开发用例，但一般概念适用于其他平台。大多数模式都集中在操作系统中阻止调用的解决方案上，因为任何涉及阻止不确定时间的操作都是不确定的。

It is a common pattern to section real-time code into three parts; a non real-time safe section at the beginning of a process that preallocates memory on the heap, starts threads, etc., a real-time safe section (often implemented as a loop), and a non-real-time safe "teardown" section that deallocates memory as necessary, etc. The "real-time code path" refers to the middle section of the execution.

> 这是一种常见的模式，将实时代码分为三个部分；一个在进程开始时的非实时安全部分，它在堆上预分配内存，启动线程等；一个实时安全部分(通常实现为一个循环)，以及一个非实时安全的“拆卸”部分，根据需要释放内存等。“实时代码路径”是指执行的中间部分。

### Memory management

Proper memory management is critical for real-time performance. In general, the programmer should avoid page faults in the real-time code path. During a page fault, the CPU pauses all computation and loads the missing page from disk into RAM (or cache, or registers). Loading data from disk is a slow and unpredictable operation. However, page faults are necessary or else the computer will run out of memory. The solution is to avoid pagefaults.

> 正确的内存管理对实时性能至关重要。一般来说，程序员应该避免在实时代码路径中出现页面故障。在页面故障期间，CPU 会暂停所有计算，并将缺失的页面从磁盘加载到 RAM(或缓存或寄存器)中。从磁盘加载数据是一个缓慢且不可预知的操作。然而，页面故障是必要的，否则计算机将耗尽内存。解决方案是避免页面故障。

Dynamic memory allocation can cause poor real-time performance. Calls to `malloc/new` and `free/delete` will probably result in pagefaults. Additionally, the heap allocates and frees memory blocks in such a way that leads to memory fragmentation, which creates poor performance for reads and writes, since the OS may have to scan for an indeterminate amount of time for a free memory block.

> 动态内存分配可能会导致实时性能不佳。调用`malloc/new`和`free/delete`可能会导致页面错误。此外，堆内存以这种方式分配和释放内存块，导致内存碎片化，这会导致读写性能变差，因为操作系统可能需要扫描不确定时间来查找空闲内存块。

#### Lock memory, prefault stack

```c
if (mlockall(MCL_CURRENT|MCL_FUTURE) == -1) {
  perror("mlockall failed");
  exit(-2);
}
unsigned char dummy[MAX_SAFE_STACK];

memset(dummy, 0, MAX_SAFE_STACK);
```

[`mlockall`](http://linux.die.net/man/2/mlockall) is a Linux system call for locking the process's virtual address space into RAM, preventing the memory that will be accessed by the process from getting paged into swap space.

> `mlockall`是 Linux 系统调用，用于将进程的虚拟地址空间锁定到 RAM 中，以防止进程访问的内存被页面到交换空间。

This code snippet, when run at the beginning of a thread's lifecycle, ensures that no pagefaults occur while the thread is running. `mlockall` locks the stack for the thread. The [`memset`](http://linux.die.net/man/3/memset) call pre-loads each block of memory of the stack into the cache, so that no pagefaults will occur when the stack is accessed (3).

> 这段代码片段，在线程生命周期开始时运行，可以确保线程运行期间不会发生页面错误。`mlockall`锁定线程的堆栈。[`memset`](http://linux.die.net/man/3/memset)调用预加载堆栈的每个内存块到缓存中，这样在访问堆栈时就不会发生页面错误(3)。

#### Allocate dynamic memory pool

```c
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

The intro to this section stated that dynamic memory allocation is usually not real-time safe. However, this code snippet shows how to make dynamic memory allocation real-time safe (mostly). It locks the virtual address space to a fixed size, disallows returning deallocated memory to the kernel via `sbrk`, and disables `mmap`. This effectively locks a pool of memory in the heap into RAM, which prevents page faults due to `malloc` and `free` (3).

> 本节介绍开始时指出，动态内存分配通常不是实时安全的。但是，这段代码片段展示了如何使动态内存分配变得实时安全(基本上)。它将虚拟地址空间锁定到一个固定的大小，不允许通过`sbrk`将已释放的内存返回给内核，并禁用`mmap`。这有效地将堆中的一个内存池锁定到 RAM 中，从而防止由于`malloc`和`free`而导致的页面故障(3)。

Pros:

> 优点：

- Can use malloc/new, free/delete, and even STL containers

Cons:

> 缺点：

- Platform/implementation dependent
- Must accurately predict bounded memory size for the process!
- Using STL containers is therefore dangerous (unbounded sizes)
- In practice, only works for processes with small memory footprint

#### Custom real-time safe memory allocators

The default allocator on most operating systems is not optimized for real-time safety. However, there is another strategy that is an exception to the "avoid dynamic memory allocation" rule. Research into alternative dynamic memory allocators is a rich research topic (8).

> 大多数操作系统上的默认分配器不适合实时安全。但是，有另一种策略是对“避免动态内存分配”规则的例外。研究可替代的动态内存分配器是一个丰富的研究课题(8)。

One such alternative allocator is TLSF (Two-Level Segregate Fit). It is also called the O(1) allocator, since the time cost of `malloc`, `free`, and `align` operations under TLSF have a constant upper bound. It creates a low level of fragmentation. The disadvantages of TLSF are that it is not thread safe and that its current implementation is architecture specific: it assumes the system can make 4-byte aligned accesses.

> 一种替代分配器是 TLSF(双级分离适配器)。它也被称为 O(1)分配器，因为 TLSF 下的 malloc，free 和 align 操作的时间成本具有恒定的上限。它可以产生低水平的碎片。 TLSF 的缺点是它不是线程安全的，而且它的当前实现是特定于架构的：它假设系统可以进行 4 字节对齐的访问。

Pros:

> 优点：

- Can safely allocate memory in a program with memory bounds that are unknown at runtime or compile
- Advantages vary depending on the choice of allocator

Cons:

> 缺点：

- Implementations of custom allocators may not be well tested, since they are less widely used
- Extra dependency, potentially more code complexity
- Drawbacks vary depending on the choice of allocator

#### Global variables and (static) arrays

Global variables are preallocated at the start of a process, thus assigning and accessing them is real-time safe. However, this strategy comes with the many disadvantages of using global variables.

> 全局变量在进程开始时就已经预先分配，因此赋值和访问它们是实时安全的。但是，这种策略也伴随着使用全局变量的许多缺点。

#### Cache friendliness for pointer and vtable accesses

Classes with many levels of inheritance may not be real-time safe because of vtable overhead access. When executing an inherited function, the program needs to access the data used in the function, the vtable for the class, and the instructions for the function, which are all stored in different parts of memory, and may or may not be stored in cache together (5).

> 类有多个继承层次可能不能实时安全，因为 vtable 访问开销。在执行继承函数时，程序需要访问该函数中使用的数据、类的 vtable 以及函数指令，这些都存储在不同的内存位置，可能或可能不会一起存储在缓存中(5)。

In general, C++ patterns with poor cache locality are not well-suited to real-time environments. Another such pattern is the opaque pointer idiom (PIMPL), which is convenient for ABI compatibility and speeding up compile times. However, bouncing between the memory location for the object and its private data pointer causes the cache to "spill" as it loads one chunk of memory and then another, unrelated chunk for almost every function in the PIMPLized object.

> 一般来说，缓存局部性差的 C++模式不适合实时环境。另一种这样的模式是不透明指针习语(PIMPL)，它可以方便地实现 ABI 兼容性和加速编译时间。但是，在对象和其私有数据指针之间来回跳转，会导致缓存在加载一块内存和另一块不相关内存时“泄漏”，几乎每个 PIMPL 化对象的函数都会出现这种情况。

#### Exceptions

Handling exceptions can incur a large performance penalty. Running into an exception tends to push a lot of memory onto the stack, which is often a limited resource in real-time programming. But if exceptions are used properly, they should not be a concern to real-time programmers (since they indicate a place in the program with undefined behavior and are integral to debugging) (6).

> 处理异常可能会产生巨大的性能损失。遇到异常往往会将大量的内存压入堆栈，这在实时编程中通常是有限的资源。但如果正确使用异常，它们不应该成为实时程序员的担忧(因为它们指示程序中未定义行为的位置，并且是调试的重要组成部分)(6)。

#### Know your problem

Different programs have different memory needs, thus memory management strategies vary between applications.

> 不同的程序有不同的内存需求，因此内存管理策略在应用程序之间有所不同。

- Required memory size known at compile time

  - Example: publishing a message of a fixed size.
  - Solution: use stack allocation with fixed-size objects.

- Required memory size known at runtime, before real-time execution.

  - Example: publishing a message of a size specified on the command line.
  - Preallocate variable size objects on the heap once required size is known, then execute real-time code.

- Required memory size computed during real-time

  - Example: a message received by the robot's sensors determines the size of the messages it publishes.

  - Multiple solutions exist

    - [Object pools](https://en.wikipedia.org/wiki/Object_pool_pattern)
    - [TLSF O(1) memory allocation](http://www.gii.upv.es/tlsf/)
    - Use stack allocation and fail if allocated memory is exceeded

### Device I/O

Interacting with physical devices (disk I/O, printing to the screen, etc.) may introduce unacceptable latency in the real-time code path, since the process is often forced to wait on slow physical phenomena. Additionally, many I/O calls such as `fopen` result in pagefaults.

> 与物理设备交互(磁盘 I/O、向屏幕打印等)可能会在实时代码路径中引入不可接受的延迟，因为进程通常被迫等待缓慢的物理现象。此外，许多 I/O 调用，如`fopen`会导致页面错误。

Keep disk reads/writes at the beginning or end of the program, outside of the RT code path. Spin up threads that are not scheduled in real-time to print output to the screen.

> 保持程序开头或结尾的磁盘读/写操作在实时代码路径之外。启动不受实时调度的线程来输出到屏幕上。

### Multithreaded Programming and Synchronization

Real-time computation requirements change the typical paradigm of multithreaded programming. Program execution may not block asynchronously, and threads must be scheduled deterministically. A real-time operating system will fulfill this scheduling requirement, but there are still pitfalls for the developer to fall into. This section provides guidelines for avoiding these pitfalls.

> 实时计算要求改变了多线程编程的典型范式。程序执行可能不会异步阻塞，线程必须以确定性的方式调度。实时操作系统将满足这一调度要求，但是开发人员仍然有可能陷入陷阱。本节提供了避免这些陷阱的指导方针。

#### Thread creation guidelines

Create threads at the start of the program. This confines the nondeterministic overhead of thread allocation to a defined point in the process.

> 在程序开始时创建线程。这将线程分配的非确定性开销限制在进程的一个定义点。

Create high priority (but not 99) threads with a FIFO, Round Robin, or Deadline scheduler (see POSIX [sched](http://man7.org/linux/man-pages/man7/sched.7.html) API).

> 使用 FIFO、Round Robin 或 Deadline 调度器创建高优先级(但不是 99)线程(参见 POSIX [sched](http://man7.org/linux/man-pages/man7/sched.7.html) API)。

#### Avoid priority inversion

Priority inversion can occur on a system with a preemptive task scheduler and results in deadlock. It occurs when: a low-priority task acquires a lock and is then pre-empted by a medium-priority task, then a high-priority task acquires the lock held by the low-priority task.

> 优先级反转可以发生在具有抢占式任务调度程序的系统上，并导致死锁。它发生的情况是：低优先级任务获取锁，然后被中优先级任务抢占，然后高优先级任务获取由低优先级任务持有的锁。

The three tasks are stuck in a triangle: the high-priority task is blocked on the low-priority task, which is blocked on the medium-priority task because it was preempted by a task with a higher priority, and the medium-priority task is also blocked on a task with a higher priority.

> 三项任务被困在一个三角形中：高优先级任务被低优先级任务阻塞，而低优先级任务又被优先级更高的任务所取代，中优先级任务也被优先级更高的任务阻塞。

Here are the some solutions to priority inversion:

> 这里有一些解决优先级反转的解决方案：

- Don't use blocking synchronization primitives
- Disable preemption for tasks holding locks (can lead to jitter)
- Increase priority of task holding a lock
- Use priority inheritance: a task that owns a lock inherits the priority of a task that tries to acquire the lock
- Use lock-free data structures and [algorithms](http://www.1024cores.net/home/lock-free-algorithms)

#### Timing shots

One real-time synchronization technique is when a thread calculates its next "shot" (the start of its next execution period). For example, if a thread is required to provide an update every 10 milliseconds, and it must complete an operation that takes 3-6 milliseconds, the thread should get the time before the operation, do the operation, and then wait for the remaining 7-4 milliseconds, based on the time measured after the operation.

> 一种实时同步技术是，当一个线程计算它的下一个“射击”(它下一次执行周期的开始)时。例如，如果线程需要每 10 毫秒提供一次更新，并且它必须完成一个需要 3-6 毫秒的操作，那么线程应该在操作之前获取时间，然后执行操作，然后根据操作后测量的时间等待剩余的 7-4 毫秒。

The most important consideration for the developer is to use a high precision timer, such as `nanosleep` on Linux platforms, while waiting. Otherwise the system will experience drift.

> 开发者最重要的考虑是在等待时使用高精度定时器，例如 Linux 平台上的`nanosleep`。否则，系统将出现漂移。

#### Spinlocks

Spinlocks tend to cause clock drift. The developer should avoid implementing his own spinlocks. The RT Preempt patch replaces much of the kernel's spinlocks with mutexes, but this might not be guaranteed on all platforms.

> 自旋锁往往会导致时钟漂移。开发人员应避免实现自己的自旋锁。RT Preempt 补丁将内核的大部分自旋锁替换为互斥锁，但这在所有平台上可能不能保证。

#### Avoid fork

[`fork`](http://linux.die.net/man/3/memset) is not real-time safe because it is implemented using [copy-on-write](https://en.wikipedia.org/wiki/Copy-on-write). This means that when a forked process modifies a page of memory, it gets its own copy of that page. This leads to page faults!

> `fork`不是实时安全的，因为它使用[copy-on-write](https://en.wikipedia.org/wiki/Copy-on-write)实现。这意味着当一个 fork 进程修改一页内存时，它会得到自己的一份该页的副本。这就导致了页面故障！

Page faults should be avoided in real-time programming, so Linux `fork`, as well as programs that call `fork`, should be avoided.

> 在实时编程中应避免页面错误，因此应避免使用 Linux 的`fork`，以及调用`fork`的程序。

## Testing and Performance Benchmarking

### cyclictest

[`cyclictest`](http://manpages.ubuntu.com/manpages/trusty/man8/cyclictest.8.html) is a simple Linux command line tool for measuring the jitter of a real-time environment. It takes as input a number of threads, a priority for the threads, and a scheduler type. It spins up `n` threads that sleep regular intervals (the sleep period can also be specified from the command line) (7).

> cyclictest 是一个用于测量实时环境抖动的简单的 Linux 命令行工具。它接受线程数、线程优先级和调度类型作为输入。它启动`n`个线程，它们以固定的间隔睡眠(睡眠期也可以从命令行指定)(7)。

For each thread, `cyclictest` measures the time between when the thread is supposed to wake up and when it actually wakes up. The variability of this latency is the scheduling jitter in the system. If there are processes with non-deterministic blocking behavior running in the system, the average latency will grow to a large number (on the order of milliseconds), since the scheduler cannot meet the deadlines of the periodically sleeping threads profiled in the program.

> 对于每个线程，`cyclictest`测量线程应该唤醒和实际唤醒之间的时间差。这种延迟的可变性就是系统中的调度抖动。如果系统中运行着具有非确定性阻塞行为的进程，平均延迟会增加到一个较大的数字(毫秒级)，因为调度程序无法满足程序中周期性睡眠线程的截止日期。

### Instrumenting code for testing

A more precise way to measure the scheduling jitter in a program is to instrument the periodic real-time update loop of existing code to record scheduling jitter.

> 一种更精确地测量程序中调度抖动的方法是，对现有代码的定期实时更新循环进行检测，以记录调度抖动。

A proposed header for a minimal library for real-time code instrumentation can be found here: [rttest.h](https://github.com/ros2/realtime_support/blob/master/rttest/include/rttest/rttest.h).

> 在这里可以找到一个用于实时代码检测的最小库的建议头文件：[rttest.h](https://github.com/ros2/realtime_support/blob/master/rttest/include/rttest/rttest.h)。

### Pagefaults

The Linux system call [`getrusage`](http://linux.die.net/man/2/getrusage) returns statistics about many resource usage events relevant to real-time performance, such as minor and major pagefaults, swaps, and block I/O. It can retrieve these statistics for an entire process or for one thread. Thus it is simple to instrument code to check for these events. In particular, `getrusage` should be called right before and right after the real-time section of the code, and these results should be compared, since `getrusage` collects statistics about the entire duration of the thread/process.

> Linux 系统调用[`getrusage`](http://linux.die.net/man/2/getrusage)可以返回与实时性能相关的许多资源使用情况的统计数据，例如次要和主要的页面错误、交换和块 I / O。它可以为整个进程或一个线程检索这些统计数据。因此，很容易对代码进行检测以检查这些事件。特别是，应该在实时代码的前后调用`getrusage`，并将这些结果进行比较，因为`getrusage`收集线程/进程的整个持续时间的统计数据。

Collecting these statistics gives an indication of what events could cause the latency/scheduling jitter measured by the previously described methods.

> 收集这些统计数据可以指示哪些事件可能会导致先前描述的方法测量的延迟/调度抖动。

## Design Guidelines for ROS 2

## Implementation strategy

With judicious application of the performance patterns and benchmarking tests proposed in this document, implementing real-time code in C/C++ is feasible. The question of how ROS 2 will achieve real-time compatibility remains.

> 通过明智地应用本文档中提出的性能模式和基准测试，实现 C/C++ 中的实时代码是可行的。 ROS 2 如何实现实时兼容性的问题仍然存在。

It is acceptable for the setup and teardown stages of the ROS node lifecycle to not be real-time safe. However, interacting with ROS interfaces, particularly within an intra-process context, should be real-time safe, since these actions could be on the real-time code path of a process.

> 可以接受 ROS 节点生命周期的设置和拆卸阶段不是实时安全的。但是，与 ROS 接口交互，特别是在进程内上下文中，应该是实时安全的，因为这些操作可能是进程的实时代码路径。

There are a few possible strategies for the real-time "hardening" of existing and future ROS 2 code:

> 有几种可能的策略可以用来对现有和未来的 ROS 2 代码进行实时“硬化”:

- Create a configuration option for the stack to operate in "real-time friendly" mode.

  - Pros:

    - Could allow user to dynamically switch between real-time and non-real-time modes.

  - Cons:

    - Refactoring overhead.
      Integrating real-time code with existing code may be intractable.

- Implement a new real-time stack (rclrt, rmwrt, etc.) designed with real-time computing in mind.

  - Pros:

    - Easier to design and maintain.

    - Real-time code is "quarantined" from existing code.
      Can fully optimize library for real-time application.

  - Cons:

    - More packages to write and maintain.
    - Potentially less convenient for the user.

- Give the option for real-time safety up to a certain point in the stack, and implement a real-time safe language wrapper (rclrt or rclc)

  - Pros:

    - Existing code is designed for this refactoring to be fairly easy
    - User can provide memory allocation strategy to rcl/rmw to ensure deterministic operation
    - Synchronization happens at the top the language/OS-specific layer, so refactoring rcl/rmw is easier
    - May be easier to support multiple embedded platforms with different wrappers

  - Cons:

    - More code to test for real-time safety
    - More flexibility for user-developer may mean more complexity

The third option is most appealing because it represents the least amount of work for the most number of benefits.

> 第三个选项最具吸引力，因为它代表了最少的工作量，却能带来最多的好处。

## Sources

1. Stefan Petters, [Presentation on Real-Time Systems](http://www.cse.unsw.edu.au/~cs9242/08/lectures/09-realtimex2.pdf)
2. [Differences between hard real-time, soft real-time, and firm real-time](http://stackoverflow.com/questions/17308956/differences-between-hard-real-time-soft-real-time-and-firm-real-time), Stack Overflow
3. [Real-Time Linux Wiki](https://rt.wiki.kernel.org/)
4. [Real-time operating system, Wikipedia](https://en.wikipedia.org/wiki/Real-time_operating_system#Memory_allocation)
5. Scott Salmon, [How to make C++ more real-time friendly](http://www.embedded.com/design/programming-languages-and-tools/4429790/2/How-to-make-C--more-real-time-friendly)
6. Stack Overflow, [Are Exceptions still undesirable in Realtime environment?](http://stackoverflow.com/questions/5257190/are-exceptions-still-undesirable-in-realtime-environment)
7. Pavel Moryc, [Task jitter measurement under RTLinux operating system](https://fedcsis.org/proceedings/2007/pliks/48.pdf)
8. Alfons Crespo, Ismael Ripoll and Miguel Masmano, [Dynamic Memory Management for Embedded Real-Time Systems](http://www.gii.upv.es/tlsf/files/papers/tlsf_slides.pdf)
9. Miguel Masmno, [TLSF: A New Dynamic Memory Allocator for Real-Time Systems](http://www.gii.upv.es/tlsf/files/ecrts04_tlsf.pdf)

> 1. 斯特凡·彼特斯(Stefan Petters)的[实时系统演示](http://www.cse.unsw.edu.au/~cs9242/08/lectures/09-realtimex2.pdf)
> 2. [硬实时、软实时和坚实实时之间的区别](http://stackoverflow.com/questions/17308956/differences-between-hard-real-time-soft-real-time-and-firm-real-time)，Stack Overflow
> 3. [Real-Time Linux Wiki](https://rt.wiki.kernel.org/)
> 4. [实时 Linux 维基现时操作系统，维基百科](https://en.wikipedia.org/wiki/Real-time_operating_system#Memory_allocation)：内存分配
> 5. Scott Salmon，[如何让 C++更加实时友好](http://www.embedded.com/design/programming-languages-and-tools/4429790/2/How-to-make-C--more-real-time-friendly)
> 6. Stack Overflow，在实时环境中异常仍然不受欢迎吗？
> 7. 帕维尔·莫里奇(Pavel Moryc)，《基于 RTLinux 操作系统的任务抖动测量》([Task jitter measurement under RTLinux operating system](https://fedcsis.org/proceedings/2007/pliks/48.pdf))
> 8. Alfons Crespo、Ismael Ripoll 和 Miguel Masmano，[嵌入式实时系统的动态内存管理](http://www.gii.upv.es/tlsf/files/papers/tlsf_slides.pdf)
> 9. 米格尔·马斯芒诺(Miguel Masmno)，《TLSF：实时系统的新型动态内存分配器》([TLSF: A New Dynamic Memory Allocator for Real-Time Systems](http://www.gii.upv.es/tlsf/files/ecrts04_tlsf.pdf))
