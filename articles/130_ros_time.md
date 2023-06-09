---
tip: translate by openai@2023-05-30 22:46:29
layout: default
title: Clock and Time
permalink: articles/clock_and_time.html
abstract:
  This article describes the ROS primitives to support programming which can run both in real time as well as simulated time which may be faster or slower.
published: true
author: '[Tully Foote](https://github.com/tfoote)'
date_written: 2018-07
last_modified: 2015-12
Authors: {{ page.author }}
Date Written: {{ page.date_written }}
Last Modified: {% if page.last_modified %}{{ page.last_modified }}{% else %}{{ page.date_written }}{% endif %}
---
## Background

Many robotics algorithms inherently rely on timing as well as synchronization. To this end we require that nodes running in the ROS network have a synchronized system clock such that they can accurately report timestamps for events.

> 许多机器人算法内在地依赖于时序以及同步。为此，我们要求在 ROS 网络中运行的节点具有同步的系统时钟，以便它们可以准确地报告事件的时间戳。

There are however several use cases where being able to control the progress of the system is important.

> 然而，有几种使用情况需要能够控制系统进度是很重要的。

### Real Time vs real-time computing

In this article the term 'real time' is used to express the true rate of progression of time. This is not connected to 'real-time' computing with deterministic deadlines.

> 在本文中，“实时”一词用来表达时间的真实进度。这与具有确定期限的“实时”计算无关。

## Use cases requiring time abstraction

When playing back logged data it is often very valuable to support accelerated, slowed, or stepped control over the progress of time. This control can allow you to get to a specific time and pause the system so that you can debug it in depth. It is possible to do this with a log of the sensor data, however if the sensor data is out of synchronization with the rest of the system it will break many algorithms.

> 当回放已记录的数据时，支持加速、减速或步进控制时间进度通常非常有价值。此控制可让您达到特定时间并暂停系统，以便深入调试。可以通过传感器数据的日志实现这一点，但是如果传感器数据与系统的其他部分不同步，它将破坏许多算法。

Another important use case for using an abstracted time source is when you are running logged data against a simulated robot instead of a real robot. Depending on the simulation characteristics, the simulator may be able to run much faster than real time or it may need to run much slower. Running faster than real time can be valuable for high level testing as well allowing for repeated system tests. Slower than real time simulation is necessary for complicated systems where accuracy is more important than speed. Often the simulation is the limiting factor for the system and as such the simulator can be a time source for faster or slower playback. Additionally if the simulation is paused the system can also pause using the same mechanism.

> 另一个使用抽象时间源的重要用例是，当您将已记录的数据与模拟机器人而不是真实机器人进行比较时。根据仿真特性，仿真器可以比实际时间运行得更快，也可能需要运行得更慢。比实际时间更快的运行对于高级测试也很有价值，可以重复系统测试。比实际时间更慢的仿真对于复杂系统是必要的，其中精度比速度更重要。通常，仿真是系统的限制因素，因此仿真器可以成为更快或更慢回放的时间源。此外，如果仿真被暂停，系统也可以使用相同的机制暂停。

## Approach

To provide a simplified time interface we will provide a ROS time and duration datatype. To query for the latest time a ROS Clock interface will be provided. A TimeSource can manage one or more Clock instances.

> 为了提供简化的时间界面，我们将提供 ROS 时间和持续时间数据类型。要查询最新时间，将提供 ROS 时钟界面。TimeSource 可以管理一个或多个时钟实例。

## Clock

### Challenges in using abstracted time

There are many algorithms for synchronization and they can typically achieve accuracies which are better than the latency of the network communications between devices on the network. However, these algorithms take advantage of assumptions about the constant and continuous nature of time.

> 有许多用于同步的算法，它们通常可以达到比网络通信延迟更好的准确性。但是，这些算法利用了关于时间恒定和连续性的假设。

An important aspect of using an abstracted time is to be able to manipulate time. In some cases, speeding up, slowing down, or pausing time entirely is important for debugging.

> 一个重要的方面使用抽象时间是能够操纵时间。在某些情况下，加快、减慢或完全暂停时间对于调试是很重要的。

The ability to support pausing time requires that we not assume that the time values are always increasing.

> .

能够支持暂停时间的能力要求我们不要假定时间值总是在增加。

When communicating the changes in time propagation, the latencies in the communication network becomes a challenge. Any change in the time abstraction must be communicated to the other nodes in the graph, but will be subject to normal network communication latency. This inaccuracy is proportional to the latency of communications and also proportional to the increase in the rate at which simulated time advances compared to real time (the "real time factor"). If very accurate timestamping is required when using the time abstraction, it can be achieved by slowing down the real time factor such that the communication latency is comparatively small.

> 当传达时间传播的变化时，通信网络中的延迟成为一个挑战。任何时间抽象的变化都必须传达给图中的其他节点，但会受到正常的网络通信延迟的影响。这种不准确性与通信延迟成正比，并且与模拟时间与实际时间（“实时因子”）的增加速率成正比。如果在使用时间抽象时需要非常准确的时间戳，可以通过减慢实时因子来实现，以使通信延迟相对较小。

The final challenge is that the time abstraction must be able to jump backwards in time, a feature that is useful for log file playback. This behavior is similar to a system clock after a negative date change, and requires developers using the time abstraction to make sure their algorithm can deal with the discontinuity. Appropriate APIs must be provided for to the developer API to enable notifications of jumps in time, both forward and backwards.

> 最后的挑战是，时间抽象必须能够向后跳跃时间，这是用于日志文件回放的有用功能。这种行为类似于负日期变更后的系统时钟，需要使用时间抽象的开发人员确保他们的算法可以处理不连续性。必须为开发人员 API 提供适当的 API，以启用时间跳跃的通知，包括向前和向后。

### Time Abstractions

There will be at least three versions of these abstractions with the following types, `SystemTime`, `SteadyTime` and `ROSTime`. These choices are designed to parallel the [`std::chrono`][] [`system_clock`][] and [`steady_clock`][]. It is expected that the default choice of time will be to use the `ROSTime` source, however the parallel implementations supporting `steady_clock` and `system_clock` will be maintained for use cases where the alternate time source is required.

> 预计默认的时间选择将是使用 `ROSTime` 源，但是为了满足需要使用替代时间源的使用场景，将维护支持 `steady_clock` 和 `system_clock` 的并行实现。这些抽象将至少有三个版本，类型为 `SystemTime`、`SteadyTime` 和 `ROSTime`，这些选择旨在与[`std::chrono`][]的[`system_clock`][]和[`steady_clock`][]并行。

#### System Time

For convenience in these cases we will also provide the same API as above, but use the name `SystemTime`. `SystemTime` will be directly tied to the system clock.

> 在这些情况下，为了方便起见，我们也将提供相同的 API，但使用名称“SystemTime”。“SystemTime”将直接与系统时钟相关联。

#### ROS Time

The `ROSTime` will report the same as `SystemTime` when a ROS Time Source is not active. When the ROS time source is active `ROSTime` will return the latest value reported by the Time Source. `ROSTime` is considered active when the parameter `use_sim_time` is set on the node.

> 当 ROS 时间源未激活时，`ROSTime` 将与 `SystemTime` 报告相同。当 ROS 时间源处于活动状态时，`ROSTime` 将返回时间源报告的最新值。当节点上的参数 `use_sim_time` 设置为活动状态时，`ROSTime` 被认为是活动的。

#### Steady Time

Example use cases for this include hardware drivers which are interacting with peripherals with hardware timeouts.

> 例如，与硬件超时相关的外围设备交互的硬件驱动程序就是此功能的一个用例。

In nodes which require the use of `SteadyTime` or `SystemTime` for interacting with hardware or other peripherals it is expected that they do a best effort to isolate any `SystemTime` or `SteadyTime` information inside their implementation and translate external interfaces to use the ROS time abstraction when communicating over the ROS network.

> 在需要使用 `SteadyTime` 或 `SystemTime` 与硬件或其他外设交互的节点中，期望它们尽最大努力将 `SystemTime` 或 `SteadyTime` 信息隔离在实现内部，并将外部接口在通过 ROS 网络进行通信时转换为 ROS 时间抽象。

### ROS Time Source

#### Default Time Source

To implement the time abstraction the following approach will be used.

> 使用以下方法来实现时间抽象。

The time abstraction can be published by one source on the `/clock` topic. The topic will contain the most up to date time for the ROS system. If a publisher exists for the topic, it will override the system time when using the ROS time abstraction. If `/clock` is being published, calls to the ROS time abstraction will return the latest time received from the `/clock` topic. If time has not been set it will return zero if nothing has been received. A time value of zero should be considered an error meaning that time is uninitialized.

> 时间抽象可以在 `/clock` 主题上由一个源发布。该主题将包含 ROS 系统的最新时间。如果主题存在发布者，则在使用 ROS 时间抽象时会覆盖系统时间。如果正在发布 `/clock`，则对 ROS 时间抽象的调用将返回从 `/clock` 主题接收到的最新时间。如果没有设置时间，如果没有收到任何内容，则返回零。时间值为零应被视为错误，表示时间未初始化。

If the time on the clock jumps backwards, a callback handler will be invoked and be required to complete before any calls to the ROS time abstraction report the new time. Calls that come in before that must block. The developer has the opportunity to register callbacks with the handler to clear any state from their system if necessary before time will be in the past.

> 如果时钟上的时间向后跳跃，将调用回调处理程序，并在调用 ROS 时间抽象报告新时间之前必须完成。在此之前收到的调用必须阻塞。开发人员有机会使用处理程序注册回调，以便在时间过去之前清除系统中的任何状态。

The frequency of publishing the `/clock` as well as the granularity are not specified as they are application specific.

> 发布 `/clock` 的频率以及粒度并未详细说明，因为这些都是应用特定的。

##### No Advanced Estimating Clock By Default

There are more advanced techniques which could be included to attempt to estimate the propagation properties and extrapolate between time ticks. However all of these techniques will require making assumptions about the future behavior of the time abstraction. And in the case that playback or simulation is instantaneously paused, it will break any of these assumptions. There are techniques which would allow potential interpolation, however to make these possible it would require providing guarantees about the continuity of time into the future. For more accuracy the progress of time can be slowed, or the frequency of publishing can be increased. Tuning the parameters for the `/clock` topic lets you trade off time for computational effort and/or bandwidth.

> 有更先进的技术可以用来估计传播特性并在时间刻度之间进行外推。但是，所有这些技术都需要对时间抽象的未来行为做出假设。如果回放或模拟瞬间暂停，它将打破这些假设。有技术可以实现潜在的插值，但是要实现这一点，需要对未来时间的连续性提供保证。为了更准确，可以减慢时间的进度，或者增加发布的频率。调整 `/clock` 主题的参数可以让您在时间和计算工作量/带宽之间进行权衡。

#### Custom Time Source

It is possible that the user may have access to an out of band time source which can provide better performance than the default source the `/clock` topic. It might be possible that for their use case a more advanced algorithm would be needed to propagate the simulated time with adequate precision or latency with restricted bandwidth or connectivity. The user will be able to switch out the time source for the instance of their Time object as well as have the ability to override the default for the process.

> 可能用户可以访问一个超出常规时间源，它可以提供比默认源（/clock 主题）更好的性能。对于他们的用例，可能需要一个更先进的算法来传播模拟时间，以达到足够的精度或延迟，并在受限的带宽或连接性能下。用户将能够为 Time 对象的实例切换时间源，以及有能力覆盖进程的默认值。

It is possible to use an external time source such as GPS as a ROSTime source, but it is recommended to integrate a time source like that using standard NTP integrations with the system clock since that is already an established mechanism and will not need to deal with more complicated changes such as time jumps.

> 可以使用外部时间源（如 GPS）作为 ROSTime 源，但建议使用标准 NTP 集成与系统时钟来集成这样的时间源，因为这已经是一种成熟的机制，不需要处理更复杂的变化，如时间跳跃。

For the current implementation a `TimeSource` API will be defined such that it can be overridden in code. If in the future a common implementation is found that would be generally useful it could be extended to optionally dynamically select the alternative TimeSource via a parameter similar to enabling the simulated time.

> 对于当前的实现，将定义一个 `TimeSource` API，以便可以在代码中重写它。如果将来发现一个通用的实现，可以通过类似于启用模拟时间的参数来选择可选的替代 TimeSource。

## Implementation

The `SystemTime`, `SteadyTime`, and `ROSTime` API's will be provided by each client library in an idiomatic way, but they may share a common implementation, e.g. provided by `rcl`. However, if a client library chooses to not use the shared implementation then it must implement the functionality itself.

> 系统时间、稳定时间和 ROSTime API 将由每个客户端库以惯用的方式提供，但它们可能共享一个公共实现，例如由 rcl 提供。但是，如果客户端库选择不使用共享实现，则必须自行实现该功能。

`SteadyTime` will be typed differently than the interchangable `SystemTime` and `ROSTime`. This is because SystemTime and ROSTime have a common base class with runtime checks that they are valid to compare against each other. However SteadyTime is never comparable to SystemTime or ROSTime so it would actually have a separate implementation of the same API. Thus you could get protection from misusing them at compile time (in compiled languages) instead of only catching it at runtime.

### Public API

The implementation from client library will provide `Time`, `Duration`, and `Rate` datatypes, for all three time source abstractions.

> 客户端库的实现将提供 `时间`、`持续时间` 和 `速率` 数据类型，用于所有三种时间源抽象。

The `Clock` will support a `sleep_for` function as well as a `sleep_until` method using a `Duration` or `Time` argument respectively. The implementation will also provide a `Timer` object which will provide periodic callback functionality for all the abstractions.

> 时钟也将提供 `sleep_for` 函数和 `sleep_until` 方法，分别使用 `Duration` 或 `Time` 参数。实现还将提供一个 `Timer` 对象，为所有抽象提供周期性回调功能。

It will also support registering callbacks for before and after a time jump. The first callback will be to allow proper preparations for a time jump. The latter will allow code to respond to the change in time and include the new time specifically as well as a quantification of the jump.

> 它还将支持在时间跳跃之前和之后注册回调。第一个回调将允许正确准备时间跳跃。后者将允许代码响应时间的变化，并特别包括新的时间以及跳跃的量化。

Any API which is blocking will allow a set of flags to indicate the appropriate behavior in case of time jump. This will allow the user to choose to error immediately on a time jump or choose to ignore. When registering a callback for jumps a filter for the minimum backwards or forwards distance will be possible and well as whether a clock change is to be included.

> 任何阻塞的 API 都将允许一组标志来指示时间跳跃时的适当行为。这将允许用户选择立即出错或选择忽略。在注册回调跳跃时，将可以过滤最小的向前或向后距离，以及是否包括时钟更改。

### RCL implementation

In `rcl` there will be datatypes and methods to implement each of the three time abstractions for each of the core datatypes. However at the `rcl` level the implementation will be incomplete as it will not have a threading model and will rely on the higher level implementation to provide any threading functionality which is required by sleep methods. It also will require appropriate threading to support the reception of `TimeSource` data.

> 在 `rcl` 中，将会有数据类型和方法来实现每个核心数据类型的三种时间抽象。但是在 `rcl` 层面，实现将是不完整的，因为它不具备线程模型，并且需要更高级别的实现来提供任何睡眠方法所需的线程功能。它还需要适当的线程来支持 `TimeSource` 数据的接收。

It will provide implementations parallel to the public datastructures and storage which the client library can depend upon and to which it can delegate. The underlying datatypes will also provide ways to register notifications, however it is the responsibility of the client library implementation to collect and dispatch user callbacks.

> 它将提供与客户端库可以依赖并可以委托的公共数据结构和存储的实现。底层数据类型也将提供注册通知的方式，但是收集和调度用户回调的责任在客户端库实现中。

```
<div class="alert alert-warning" markdown="1">
  <b>TODO:</b> Enumerate the <code>rcl</code> datastructures and methods here.
</div>
```

## References

The default time source is modeled on the ROS Clock and ROS Time system used in ROS 1.0. For more information on the implementation in ROS 1.0 see:

> 默认的时间源是基于 ROS Clock 和 ROS Time 系统，这是 ROS 1.0 所使用的。要了解 ROS 1.0 中的实现，请参阅：

```
- [ROS Clock Documentation](http://wiki.ros.org/Clock)
- [rospy Time Documentation](http://wiki.ros.org/rospy/Overview/Time)
- [roscpp Time Documentation](http://wiki.ros.org/roscpp/Overview/Time)

[`std::chrono`]: http://en.cppreference.com/w/cpp/chrono
[`steady_clock`]: http://en.cppreference.com/w/cpp/chrono/steady_clock
[`system_clock`]: http://en.cppreference.com/w/cpp/chrono/system_clock
```
