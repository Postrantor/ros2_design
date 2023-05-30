---
tip: translate by openai@2023-05-29 23:08:09
layout: default
title: Proposal for Implementation of Real-time Systems in ROS 2
permalink: articles/realtime_proposal.html
abstract: Proposal for a test-driven approach to the real-time performance requirement in ROS 2.
published: true
author: Jackie Kay
date_written: 2016-01
last_modified: 2016-01
Authors: {{ page.author }}
Date Written: {{ page.date_written }}
Last Modified: {% if page.last_modified %}{{ page.last_modified }}{% else %}{{ page.date_written }}{% endif %}
---

## Requirements and Implementation of Real-Time Systems

### System requirements

The requirements of a real-time system vary depending on the use case.

> 实时系统的要求根据使用情况而有所不同。

At its core, the real-time requirement of a system has two components:

> 系统的实时要求的核心有两个组成部分：

- Latency

  - Update period (also known as deadline)
  - Predictability

- Failure mode

  - How to react to a missed deadline

Language referring to hard/soft/firm real-time systems generally refers to the failure mode. A hard real-time system treats a missed deadline as a system failure. A soft real-time system tries to meet deadlines but does not fail when a deadline is missed. A firm real-time system discards computations made by missed deadlines and may degrade its performance requirement in order to adapt to a missed deadline.

> 语言指的是硬/软/稳定实时系统通常是指失败模式。硬实时系统将错过的最后期限视为系统失败。软实时系统试图满足最后期限，但在错过最后期限时不会失败。稳定实时系统会抛弃由错过的最后期限所做的计算，并且可能降低其性能要求以适应错过的最后期限。

Though in this document we consider the failure mode orthogonal to latency characteristics, a hard real-time failure mode is often associated with high predictability (low jitter).

> 尽管在本文档中我们考虑到了与延迟特性正交的失效模式，但硬实时失效模式通常与高可预测性(低抖动)相关。

Safety-critical systems often require a hard real-time system with high predictability.

> 安全关键系统通常需要具有高可预测性的硬实时系统。

### Implementation choices: computer architecture and OS

Processors vary in capability, number of cores, memory size, and instruction set architectures, some of which are better suited to real-time performance than others. The selection of computer architecture may be due to cost, size, or energy efficiency, which could set a constraint for the rest of the system. A single robot may contain multiple computers with a variety of processor types, from the main control board to sensors.

> 处理器在能力、核心数量、内存大小和指令集架构方面各不相同，其中一些更适合实时性能。计算机架构的选择可能是由于成本、尺寸或能效，这可能为系统的其余部分设定了约束。一个机器人可能包含多台计算机，其中有各种处理器类型，从主控板到传感器。

Another major consideration when designing a real-time system is the choice of operating system. A "hard" real-time operating system is truly deterministic. "Soft" real-time operating systems are designed to meet most deadlines with low jitter, but are nondeterministic. Most conventional user-facing operating systems are not designed with fairness and predictability as the first priorities, often because they have to support graphical displays and interactivity. Here are some features critical to predictable performance that are often exposed to the user in an RTOS, but not in a conventional OS (see Wikipedia, [Real-time operating system](https://en.wikipedia.org/wiki/Real-time_operating_system)). For more information about why these features matter, refer to the [introductory article on real-time systems](http://design.ros2.org/articles/realtime_background.html).

> 在设计实时系统时，另一个重要考虑因素是操作系统的选择。"硬"实时操作系统是真正的确定性。"软"实时操作系统旨在以低抖动满足大多数最终期限，但不具有确定性。大多数常规面向用户的操作系统不是以公平性和可预测性为首要考虑，通常是因为它们必须支持图形显示和交互性。以下是一些对可预测性能至关重要的功能，通常在 RTOS 中向用户暴露，但在常规操作系统中不是这样(参见维基百科，[实时操作系统](https://en.wikipedia.org/wiki/Real-time_operating_system))。有关这些功能为什么重要的更多信息，请参阅[实时系统介绍文章](http://design.ros2.org/articles/realtime_background.html)。

- Task scheduling

  - Expose thread priority and deterministic schedulers that incorporate priority
  - Allow full thread preemption

- Concurrency and synchronization primitives

  - Floored semaphores (allow a higher priority thread to take control of locked semaphore)

- Interrupt handling

  - Disable interrupts
  - Mask interrupts (temporarily suspend interrupts in a critical section)

- Memory allocation

  - Avoid pagefaults
  - Avoid nondeterministic heap allocation algorithms

Some real-time systems have a non real-time component, requiring inter-communication between the real-time and non-real-time components. An example from the ROS 1 world is real-time Orocos components communicating with ROS nodes. Another example is real-time code with a graphical or user interaction component.

> 一些实时系统有一个非实时组件，需要实时组件和非实时组件之间进行交流。来自 ROS 1 世界的一个例子是实时 Orocos 组件与 ROS 节点通信。另一个例子是具有图形或用户交互组件的实时代码。

Scheduling non-real-time threads alongside real-time threads on the same core can introduce nondeterminism into a system. The non-real-time threads must have a lower priority than real-time threads in order to not interfere with real-time deadlines. The real-time threads should never be blocked on the non-real-time threads and should never be pre-empted by non-real-time threads.

> 在同一个核心上调度非实时线程和实时线程可能会使系统出现不确定性。非实时线程的优先级必须低于实时线程，以免影响实时的截止时间。实时线程不应该被阻塞在非实时线程上，也不应该被非实时线程抢占。

Real-time operating systems have a variety of methods for enforcing these constraints. For example, a Linux RT_PREEMPT system could use the [SCHED_IDLE](http://man7.org/linux/man-pages/man2/sched_setscheduler.2.html) scheduling policy for non-real-time threads. Systems like QNX and Xenomai give the option to partition real-time and non-real-time components onto different cores, or even different kernels. Partitioning is the safest approach to real-time and non-real-time intercommunication because it minimizes the change of interference from non-real-time threads, with the added cost of latency due to coordinating data between the partitions.

> 实时操作系统有多种方法来执行这些约束。例如，Linux RT_PREEMPT 系统可以使用[SCHED_IDLE](http://man7.org/linux/man-pages/man2/sched_setscheduler.2.html)调度策略来处理非实时线程。像 QNX 和 Xenomai 这样的系统提供了将实时和非实时组件分区到不同的内核或不同的内核上的选项。**分区是实时和非实时交互的最安全的方法，因为它可以最大程度地减少非实时线程的干扰，同时增加由于在分区之间协调数据而导致的延迟成本**。

Another important constraint is which processor architectures are supported by which operating systems, and which RTOS is an appropriate choice due to performance constraints of the architecture. For example, an architecture that doesn't support SIMD instructions cannot take full advantage of parallelism, but if the system doesn't run an OS that supports threading, or doesn't have multiple cores, this constraint doesn't matter.

> 另一个重要的约束是哪些处理器架构受哪些操作系统的支持，以及由于架构的性能约束，哪个实时操作系统是一个合适的选择。例如，不支持 SIMD 指令的架构无法充分利用并行性，但是如果系统不运行支持线程的操作系统，或者没有多个内核，则此约束不重要。

## Goal for ROS 2

In order to support real-time systems in ROS 2, we propose an iterative approach for identifying limitations in the API that force the code path into nondeterministic/unpredictable behavior and performance bottlenecks.

> 为了支持 ROS 2 中的实时系统，我们提出了一种迭代方法，用于**识别强制将代码路径转换为非确定/不可预测行为和性能瓶颈的 API 的限制**。

1. Select two use cases with real-time requirements that represent two extremes of the space described above, for example:

> 1. 选择两个具有实时要求的用例，代表上述空间的两个极端，例如：

- An STM32 embedded board running no operating system or a hard real-time OS with 1 kHZ update frequency with 3% allowable jitter.
- An x86 computer running Xenomai or RT_PREEMPT that receives the sensor data over Ethernet or USB and processes the output in soft or firm real-time, but also runs separate non-RT processes for user interaction.

> 一块运行无操作系统或硬实时操作系统的 STM32 嵌入式板，更新频率为 1kHz，允许的抖动为 3%。
> 一台运行 Xenomai 或 RT_PREEMPT 的 x86 计算机，通过以太网或 USB 接收传感器数据，并以软实时或硬实时处理输出，同时还运行独立的非实时进程以进行用户交互。

2. Implement the system with ROS 2.
3. Profile the performance of the system.
4. Identify solutions to improve the performance of the system, e.g. by exposing more concurrency primitives through the appropriate abstraction.
5. Adapt the ROS 2 API or underlying libraries to implement these solutions and to improve the experience of writing real-time code.
6. Refactor the system for the new API if necessary and repeat.

> 实现用 ROS 2.0 系统。
> 对系统性能进行分析。
> 确定改善系统性能的解决方案，例如通过适当的抽象来暴露更多的并发原语。
> 适配 ROS 2 API 或底层库来实现这些解决方案，并改善编写实时代码的体验。
> 如有必要，重构系统以适应新的 API，然后重复此步骤。

## Software architecture challenges and solutions

Wherever ROS 2 libraries expose sources of nondeterminism, there should be a path that exposes an alternative. This section evaluates the current state of ROS 2 and rclcpp, predicts challenges to fulfilling real-time requirements, and presents possible solutions.

> 无论 ROS 2 库在哪里暴露不确定性的来源，都应该有一条路径暴露出另一种选择。本节将评估 ROS 2 和 rclcpp 的当前状态，预测满足实时要求的挑战，并提出可能的解决方案。

### Memory Allocation

Poor memory management can create performance issues due to page faults and nondeterministic algorithms for dynamically allocating memory. Although C++ has an advantage over many programming languages because memory management is exposed, the default memory allocator for operator `new` and dynamically sized data structures is not well-suited to real-time applications (see [Composing High-Performance Memory Allocators](https://people.cs.umass.edu/~emery/pubs/berger-pldi2001.pdf)). Dynamically-sized data structures in the C++ standard library offer an allocator interface, such as `std::vector` (see [cppreference.com](http://en.cppreference.com/w/cpp/concept/Allocator)). C++11 greatly improved this interface by introducing [`std::allocator_traits`](http://en.cppreference.com/w/cpp/memory/allocator_traits), which constructs an allocator compatible with the standard allocator interface from a minimal set of requirements To allow for optimal performance in `rclcpp`, the client library should expose custom allocators for publishers, subscribers, etc. that are propagated to standard library structures used by these entities.

> 糟糕的内存管理会由于页面故障和非确定性算法动态分配内存而导致性能问题。尽管 C++比许多编程语言更有优势，因为它暴露了内存管理，但是操作符`new`和动态调整大小的数据结构的默认内存分配器不适合实时应用程序(参见[组成高性能内存分配器](https://people.cs.umass.edu/~emery/pubs/berger-pldi2001.pdf))。C++标准库中的动态大小数据结构提供了一个分配器接口，比如`std::vector`(参见[cppreference.com](http://en.cppreference.com/w/cpp/concept/Allocator))。C++11 通过引入[`std::allocator_traits`](http://en.cppreference.com/w/cpp/memory/allocator_traits)大大改善了这个接口，它从最小的要求中构造一个与标准分配器接口兼容的分配器，以便在`rclcpp`中获得最佳性能，客户端库应为发布者、订阅者等暴露自定义分配器，这些分配器会传播到这些实体使用的标准库结构中。

As of the writing of this document (January 2016), selecting custom allocators in `rclcpp` is almost fully implemented. The [`realtime_support`](https://github.com/ros2/realtime_support) package will offer real-time memory allocators and example code for real-time ROS 2. An example of the ROS 2 allocator API using the Two Level Segregate Fit allocator is currently available [in realtime_support](https://github.com/ros2/realtime_support/blob/master/tlsf_cpp/example/allocator_example.cpp).

> 截至本文档(2016 年 1 月)的撰写，在`rclcpp`中选择自定义分配器几乎已经完全实现。[`realtime_support`](https://github.com/ros2/realtime_support) 包将提供实时内存分配器和实时 ROS 2 的示例代码。使用 Two Level Segregate Fit 分配器的 ROS 2 分配器 API 示例目前可以在[realtime_support](https://github.com/ros2/realtime_support/blob/master/tlsf_cpp/example/allocator_example.cpp)中找到。

The following work remains to be done:

> 以下工作仍有待完成：

- Improve the interface in rclcpp such that the user need only provide an "allocate" function, a "deallocate" function, and an optional pointer to the allocator state (this interface is currently outlined in the rcl API).
- Custom allocators for services/clients.
- Custom allocators for parameters.
- Fully implement and test the allocator interface in rcl, rclc, and rmw.

### Scheduling and synchronization

Parallelism is valuable for real-time systems because it can reduce latency. Even in single-core architectures, concurrency is important to us because asynchronous patterns are common in ROS and ROS 2.

> 并行计算对于实时系统很有价值，因为它可以减少延迟。
> 即使是单核架构，并发对我们来说也很重要，因为 ROS 和 ROS 2 中常见的异步模式。

The typical approaches to synchronizing concurrent access to data tend to decrease the predictability of a system and increase the complexity. For example, mutexes introduce the possibility of deadlocks when used improperly. As mentioned above, many real-time operating systems offer alternative synchronization primitives for real-time safety.

> 典型的同步访问数据的方法往往会降低系统的可预测性，增加复杂性。例如，当不当使用互斥量时，可能会引起死锁。如上所述，许多实时操作系统提供实时安全的替代同步原语。

Guidelines for synchronization in ROS 2:

> ROS 2 同步指南：

- Prefer atomics to mutexes when possible.

  - Many atomic types are not implemented as lock-free due to architectural limitations, can check in C++ with [`std::atomic_is_lock_free`](http://en.cppreference.com/w/cpp/atomic/atomic_is_lock_free)

- Thoroughly test multithreaded code for deadlock and livelock at the earliest opportunity.

  - Consider compiling multithreaded regression tests with the [Clang ThreadSanitizer](http://clang.llvm.org/docs/ThreadSanitizer.html).

- Hide synchronization primitives behind a layer of abstraction that wraps standard library classes such as `std::mutex` and `std::condition_variable` as the default behavior.

  - Inject alternative synchronization primitives from RTOS-specific APIs where applicable.

As of the time of writing, the Executor and the IntraProcessManager in rclcpp are designed to partially follow the last guideline. However, the abstractions are messy (particularly for the IntraProcessManager). There is no real-time multithreaded Executor that uses "real-time safe" synchronization primitives, similarly there is no such IntraProcessManager--there is only the abstraction path that would allow such alternatives to be implemented. There are also synchronization primitives that are not hidden behind an abstraction layer for the global interrupt handler and for parameters.

> **截至撰写时，rclcpp 中的 Executor 和 IntraProcessManager 被设计为部分遵循最后的指南**。但是，抽象层面比较混乱(尤其是对 IntraProcessManager 而言)。没有使用“实时安全”同步原语的实时多线程执行器，同样也没有这样的 IntraProcessManager-只有实现这种替代方案的抽象路径。还有一些同步原语没有隐藏在抽象层面，用于全局中断处理程序和参数。

### Middleware selection and QoS

If the underlying middleware used in ROS 2 is not suitable for real-time performance, the entire system will fail to meet the real-time requirement. Network communication and discovery, particularly asynchronous communication, introduces a nondeterministic element into software. Though some real-time operating systems partially mitigate this (e.g. disabling interrupts), careful selection and use of a middleware is important.

> 如果 ROS 2 中使用的底层中间件不适合实时性能，整个系统将无法满足实时要求。网络通信和发现，**特别是异步通信，会给软件带来不确定性**。虽然一些实时操作系统可以部分缓解这种情况(例如禁用中断)，但仍然需要仔细选择和使用中间件。

The embedded component of the system proposed will likely use `freertps` as the underlying middleware. The "desktop" system can be matrix-tested using the various DDS implementations supported by ROS 2. This allows us to compare the real-time performance of the different DDS implementations.

> 系统提议的嵌入式组件可能会使用`freertps`作为底层中间件。可以使用 ROS 2 支持的各种 DDS 实现对“桌面”系统进行矩阵测试。这样可以比较不同 DDS 实现的实时性能。

Quality of service is always an important consideration with DDS. Some DDS QoS options were designed with real-time performance in mind, such as the DEADLINE policy (see opendds.org, [QoS Policy Usages](http://www.opendds.org/qosusages.html)). It may be convenient to expose these policies and provide a "real-time" QoS profile in rmw as a suggestion to ROS 2 users. Benchmarking and profiling may provide data to support the introduction of more QoS options into the rmw API.

> 服务质量对于 DDS 总是一个重要的考虑因素。一些 DDS QoS 选项是专门为实时性能而设计的，例如 DEADLINE 策略(参见 opendds.org，[QoS 策略用法](http://www.opendds.org/qosusages.html))。在 rmw 中暴露这些策略并为 ROS 2 用户提供“实时”QoS 配置文件可能很方便。基准测试和性能分析可以提供数据以支持将更多的 QoS 选项引入 rmw API。

### Component lifecycle

It should be noted that the planned component lifecycle features are useful for real-time programming, since it is common that real-time systems have a initialization phase in which the real-time constraint is relaxed, in which it is safe to preallocate memory, change thread priority, etc. Distinguishing between initialization and execution could lead to better self-verification of real-time constraints during the execution phase (see the ROS 2 design article on [managed nodes](http://design.ros2.org/articles/node_lifecycle.html)).

> 应该指出，计划的组件生命周期特性对实时编程很有用，因为实时系统通常有一个初始化阶段，在这个阶段，实时约束被放松，在这个阶段可以安全地预先分配内存，改变线程优先级等。区分初始化和执行可能会导致在执行阶段更好地自我验证实时约束(参见 ROS 2 设计文章[管理节点](http://design.ros2.org/articles/node_lifecycle.html))。

## Testing

The proposed system must be tested for correctness and expected behavior. The performance requirements of the system, such as latency, must also be benchmarked, and the inner workings of the system must be accessible, in order to identify bottlenecks. Therefore we propose a mix of automated unit tests, performance benchmarking, and profiling/introspection tools.

> 系统必须经过正确性和预期行为的测试。系统的性能要求，如延迟，也必须进行基准测试，系统的内部工作原理必须可访问，以便识别瓶颈。因此，我们提出采用自动化单元测试、性能基准测试和分析/内省工具的混合方法。

### Benchmarking

The system will be subjected to varying levels of stress to benchmark "best" and "worst"-case performance statistics.

> 系统将受到不同程度的压力，以便对“最佳”和“最差”性能统计进行基准测试。

Stress can be applied "externally" via polluting the cache and running many garbage processes alongside the test system (if on an OS that supports processes). It could also be added "internally" (to ROS 2) by arbitrarily adding more publishers, subscribers, clients, services, etc.

> 应用压力可以"外部"地通过污染缓存并在测试系统中运行许多垃圾进程(如果是支持进程的操作系统)来实现。它也可以"内部"(对 ROS 2)通过任意添加更多的发布者、订阅者、客户端、服务等来添加。

Possible performance metrics include:

> 可能的绩效指标包括：

- Minimum, maximum, and average latency

  - Latency of the update loop
  - Message round trip time over the middleware

- Standard deviation of latency
- Number of missed deadlines/overruns
- Number of messages received vs. number of messages published

`rttest` is an existing library developed to instrument the update loop of a real-time process and report on various metrics described here. However, it contains code specific to Linux pthreads in its current state. If it is to be used in this work, the library needs to be generalized to support other operating systems.

> `rttest`是一个现有的库开发的，用于启动实时过程的更新循环，并报告此处描述的各种指标。但是，它包含特定于 Linux Pthreads 的代码。如果要在这项工作中使用，则需要将图书馆概括以支持其他操作系统。

### Profiling Tools

In addition to gathering performance statistics, it is useful to introspect the conditions under which the statistics were gathered to pinpoint areas where the code needs improvement.

> 除了收集性能统计数据外，还有必要深入了解统计数据被收集的条件，以确定需要改进的代码区域。

Two well-established open source command line profiling tools with orthogonal methodologies are:

> 两种成熟的开源命令行分析工具，采用正交方法:

- `perf`: counts system-wide hardware events, e.g. instructions executed, cache misses, etc. (see [perf wiki](https://perf.wiki.kernel.org/index.php/Main_Page))

  - Linux only.

- `gprof`: recompiles source code with instrumentation to count time spent in each function for a particular program, with option to format output in a call graph (see [GNU gprof documentation](https://sourceware.org/binutils/docs/gprof/)).

  - GNU and C/C++ only.

However, these particular tools may not be well suited to an RTOS or a bare metal embedded system: `perf` is specialized for the Linux kernel and `gprof` requires code to be compiled and linked with `gcc`. These tools could be used to profile ROS 2 code on a patched Linux kernel like RT_PREEMPT, but there may be performance quirks in another RTOS that are not captured on Linux. When this proposal is implemented, if changes are made to improve performance is made based on profiling data from Linux but no overall improvement is seen in the embedded system, then an alternative approach to profiling should be investigated. For example, the [RTEMS](https://devel.rtems.org/wiki/Developer/Tracing) real-time operating system offers a tracing tool that could be used for profiling (8).

> 然而，这些特定的工具可能不适合实时操作系统或裸机嵌入式系统：`perf`专为 Linux 内核而设计，而`gprof`则需要使用`gcc`编译和链接代码。这些工具可以用来对 RT_PREEMPT 等补丁 Linux 内核上的 ROS 2 代码进行分析，但可能无法捕捉另一种 RTOS 中的性能特性。如果在实施此建议后，根据 Linux 上的分析数据进行改进，但在嵌入式系统中没有看到整体性能的改善，则应该探索另一种分析方法。例如，[RTEMS](https://devel.rtems.org/wiki/Developer/Tracing)实时操作系统提供了一种可用于分析的跟踪工具(8)。

### Verification

In addition to performance testing, which can be thought of as a kind of regression testing, any change made to ROS 2 with the intent to improve performance in the real-time test system should be verified with an automated test. For example, the allocator pipeline has a test case for publishers/subscribers with custom allocators that fails if calls are made to the default system allocator during the real-time execution phase. A similar test could be written for the proposed goal of abstracting synchronization primitives, though the implementation may be more challenging since the allocator pipeline was helped by the ability to override global new in C++.

> 除了性能测试，可以被认为是一种回归测试，任何为改善 ROS 2 在实时测试系统中的性能而做出的更改都应该用自动化测试来验证。例如，分配器管线有一个用于带有自定义分配器的发布者/订阅者的测试用例，如果在实时执行阶段调用默认系统分配器，则会失败。可以编写类似的测试用例来实现抽象同步原语的提出目标，但实现可能更具挑战性，因为分配器管线受益于 C++中可以覆盖全局新的能力。

The expected response of a hard or firm real-time system to missed deadlines should also be thoroughly tested.

> 对于硬实时系统或硬实时系统错过期限的预期响应也应当进行彻底的测试。

## Timeline

This proposal represents a short burst of work to develop the initial test system, followed by extended maintenance as testing identifies performance bottlenecks.

> 这个提案代表了一个短期的工作突破，用于开发初始测试系统，随后通过测试识别性能瓶颈进行延长维护。

Since a key part of this proposal is the development of a hard real-time embedded system, this work depends on the development of `freertps` and `rmw_freertps` and the porting of `rclc` to STM32 or another microcontroller type.

> 由于本提案的一个关键部分是开发一个实时嵌入式系统，因此这项工作依赖于`freertps`和`rmw_freertps`的开发以及将`rclc`移植到 STM32 或其他微控制器类型。

- Alpha 4 (2/12/15): Create initial system and test suite (2-3 weeks).
- Every release thereafter: Iterate on the API, refactor and expand test suite as new features are added to core ROS 2 libraries (1-2 weeks each release cycle).
