---
tip: translate by openai@2023-05-29 15:50:06
layout: default
title: Managed nodes
abstract:
  This article describes the concept of a node with a managed life cycle. It aims to document some of the options for supporting manage d-life cycle nodes in ROS 2. It has been written with consideration for the existing design of the ROS 2 C++ client library, and in particular the current design of executors.
  本文描述了具有托管生命周期的节点的概念。它旨在记录ROS2中支持管理D寿命周期节点的一些选项。它已被考虑在ROS 2 C ++客户库库的现有设计中，尤其是当前的执行者设计。
author: '[Geoffrey Biggs](https://github.com/gbiggs) [Tully Foote](https://github.com/tfoote)'
date_written: 2015-06
last_modified: 2021-02
published: true
Authors: 
Date Written: 
Last Modified:
---
> [!NOTE]
> 在 action 中也有提到这类 `Introspection tools`，考虑也实现一套，还是 ros2 中已经具备？
> 用于对 error 状态的节点进行检查，可以用于调试，而不是直接销毁。

> [!NOTE]
> 可以注意的是，在 action 中也有状态机的使用，感觉完全可以参考。

---

## Background

A managed life cycle for nodes allows greater control over the state of ROS system.

> **一个节点的管理生命周期可以更好地控制 ROS 系统的状态。**

It will allow roslaunch to ensure that all components have been instantiated correctly before it allows any component to begin executing its behaviour. It will also allow nodes to be restarted or replaced on-line.

> 它将允许 roslaunch 确保所有组件都已正确实例化，然后才允许任何组件开始执行其行为。它还允许在线重新启动或替换节点。

The most important concept of this document is that a managed node presents a known interface, executes according to a known life cycle state machine, and otherwise can be considered a black box. This allows freedom to the node developer on how they provide the managed life cycle functionality, while also ensuring that any tools created for managing nodes can work with any compliant node.

> 本文档最重要的概念是，**管理节点提供已知的接口，按照已知的生命周期状态机执行，否则可以视为黑匣子**。这使得节点开发者可以自由提供管理生命周期功能，同时也确保可以用于管理节点的任何工具都能与任何符合标准的节点一起工作。

## Life cycle

![The proposed node life cycle state machine](../img/node_lifecycle/life_cycle_sm.png)

There are 4 primary states:

- `Unconfigured`
- `Inactive`
- `Active`
- `Finalized`

To transition out of a primary state requires action from an external supervisory process, with the exception of an error being triggered in the `Active` state. There are also 6 transition states which are intermediate states during a requested transition.

> 要从主要状态转换出来，需要外部监督过程的行动，除非在“活动”状态中触发错误。还有 6 个转换状态，这些状态是请求转换期间的中间状态。

- `Configuring`
- `CleaningUp`
- `ShuttingDown`
- `Activating`
- `Deactivating`
- `ErrorProcessing`

In the transitions states logic will be executed to determine if the transition is successful. Success or failure shall be communicated to lifecycle management software through the lifecycle management interface. There are 7 transitions exposed to a supervisory process, they are:

> 在转换状态中，将执行逻辑以确定转换是否成功。**成功或失败的结果将通过生命周期管理接口传达给生命周期管理软件**。有 7 个转换暴露给**监督过程**，它们是：

- `create`
- `configure`
- `cleanup`
- `activate`
- `deactivate`
- `shutdown`
- `destroy`

The behavior of each state is as defined below.

> 每个状态的行为如下定义。

### Primary State: Unconfigured

This is the life cycle state the node is in immediately after being instantiated. This is also the state in which a node may be retuned to after an error has happened. In this state there is expected to be no stored state.

> **这是节点实例化后立即处于的生命周期状态。在发生错误后，也可以将节点恢复到此状态。在此状态下，不会有任何存储的状态。**

#### Valid transition out

- The node may transition to the `Inactive` state via the `configure` transition.
- The node may transition to the `Finalized` state via the `shutdown` transition.

> 节点可以通过 `configure` 转换进入 `Inactive` 状态。
> 节点可以通过 `shutdown` 转换进入 `Finalized` 状态。

### Primary State: Inactive

This state represents a node that is not currently performing any processing.

> 这个状态表示当前没有正在执行任何处理的节点。

The main purpose of this state is to allow a node to be (re-)configured (changing configuration parameters, adding and removing topic publications/subscriptions, etc) without altering its behavior while it is running.

> 这个状态的主要目的是**允许节点在运行时更改配置参数、添加和删除主题发布/订阅等，而不改变其行为**。

> [!NOTE]
> 通过让节点处于 `inactive` 状态，可以更改 QoS？
> 这里提到了可以添加、删除 topic/pub/sub 等，结合前面对 QoS 的动态配置问题，是为了避免丢失数据，这样的话，就可以避免这个问题；
> 另外的，QoS 需要 sub、pub 相互匹配，这里也可以相对容易的实现，如，要更改 pub 的 qos，必须将对应的 sub 也置于 `inactive` 状态？

While in this state, the node will not receive any execution time to read topics, perform processing of data, respond to functional service requests, etc. In the inactive state, any data that arrives on managed topics will not be read and or processed. Data retention will be subject to the configured QoS policy for the topic. Any managed service requests to a node in the inactive state will not be answered (to the caller, they will fail immediately).

> 在这种状态下，节点将不会收到任何执行时间来读取主题、执行数据处理、响应功能服务请求等。
> 在非活动状态下，任何到达受管理主题的数据都不会被读取或处理。
> **数据保留将受主题配置的 QoS 策略的约束。**
> 对处于非活动状态的节点发出的任何托管服务请求都不会得到回应(对调用者来说，它们会立即失败)。

> [!NOTE]
> 这里提到，数据保留收到 topic 的 QoS 策略约束？！
> 就是这里的 topic 也是保留数据的，也是实体，底层应该是对应 dds 中的 topic 行为，能力

#### Valid transitions out of Inactive

- A node may transition to the `Finalized` state via the `shutdown` transition.
- A node may transition to the `Unconfigured` state via the `cleanup` transition.
- A node may transition to the `Active` state via the `activate` transition.

> 一个节点可以通过“关机”转换来进入“已完成”状态。
> 一个节点可以通过“清理”转换来进入“未配置”状态。
> 一个节点可以通过“激活”转换进入“活动”状态。

> [!NOTE]
> 这些状态之间的切换感觉就像线程(进程)状态一样...

### Primary State: `Active`

This is the main state of the node's life cycle. While in this state, the node performs any processing, responds to service requests, reads and processes data, produces output, etc. If an error that cannot be handled by the node/system occurs in this state, the node will transition to `ErrorProcessing`.

> 这是节点生命周期的主要状态。在此状态下，节点会执行任何处理，响应服务请求，读取和处理数据，产生输出等。**如果在此状态发生节点/系统无法处理的错误，节点将转换到“错误处理”状态。**

#### Valid transitions out of Active

- A node may transition to the `Inactive` state via the `deactivate` transition.
- A node may transition to the `Finalized` state via the `shutdown` transition.

> 一个节点可以通过“deactivate”转换进入“Inactive”状态。
> 一个节点可以通过“关机”转换来进入“已完成”状态。

### Primary State: `Finalized`

The `Finalized` state is the state in which the node ends immediately before being destroyed. This state is always terminal the only transition from here is to be destroyed.

> 最终状态是节点在被销毁之前立即结束的状态。这个状态总是终止的，从这里唯一的转换是被销毁。

This state exists to support debugging and introspection. A node which has failed will remain visible to system introspection and may be potentially introspectable by debugging tools instead of directly destructing. If a node is being launched in a respawn loop or has known reasons for cycling it is expected that the supervisory process will have a policy to automatically destroy and recreate the node.

> 这个状态是为了支持调试和内省。**一个失败的节点仍然可以被系统内省看到，并可能被调试工具检查，而不是直接摧毁。** 如果一个节点正在重新启动循环中或者有已知的重复原因，预期监督进程将有一个自动销毁和重建节点的策略。

> [!NOTE]
> 这里给出了一个合理使用 lifecycle 的方法！
> 值得借鉴使用！

#### Valid transitions out of Finalized

- A node may be deallocated via the `destroy` transition.

> 一个节点可以通过 `destroy` 转换来释放。

### Transition State: `Configuring`

In this transition state the node's `onConfigure` callback will be called to allow the node to load its configuration and conduct any required setup.

> 在这个转换状态下，将调用节点的 `onConfigure` 回调，以允许节点加载其配置并进行任何必要的设置。

The configuration of a node will typically involve those tasks that must be performed once during the node's life time, such as obtaining permanent memory buffers and setting up topic publications/subscriptions that do not change.

> 节点的配置通常涉及那些必须在节点的生命周期中仅执行一次的任务，例如获取永久内存缓冲区并设置不会更改的主题发布/订阅。

The node uses this to set up any resources it must hold throughout its life (irrespective of if it is active or inactive). As examples, such resources may include topic publications and subscriptions, memory that is held continuously, and initialising configuration parameters.

> **节点使用这个来设置它必须在其整个生命周期中保持的任何资源(无论它是否处于活动或非活动状态)**。例如，这些资源可能包括主题发布和订阅、持续保存的内存以及初始化配置参数。

> [!NOTE]
> 在这里设置是整个生命周期？
> 那 QoS 应该不算在里面！

#### Valid transitions out of Configuring

- If the `onConfigure` callback succeeds the node will transition to `Inactive`
- If the `onConfigure` callback results in a failure code (TODO specific code) the node will transition back to `Unconfigured`.
- If the `onConfigure` callback raises or results in any other result code the node will transition to `ErrorProcessing`

> 如果 `onConfigure` 回调成功，节点将转换为 `Inactive` 状态。
> 如果 `onConfigure` 回调结果返回失败代码(TODO 具体代码)，节点将会重新转换回 `Unconfigured` 状态。
> 如果 `onConfigure` 回调引发或产生任何其他结果代码，节点将转换为 `ErrorProcessing`。

### Transition State: `CleaningUp`

In this transition state the node's callback `onCleanup` will be called. This method is expected to clear all state and return the node to a functionally equivalent state as when first created. If the cleanup cannot be successfully achieved it will transition to `ErrorProcessing`.

> 在此转换状态下，将调用节点的回调 `onCleanup`。该方法预期会清除所有状态，并将节点返回到与创建时功能等效的状态。如果无法成功完成清理，它将转换为 `ErrorProcessing`。

> [!NOTE]
> 这样的话，初始状态就要保存在一些公共的地方，就是要有备份了

#### Valid transitions out if `CleaningUp`

- If the `onCleanup` callback succeeds the node will transition to `Unconfigured`.
- If the `onCleanup` callback raises or results in any other return code the node will transition to `ErrorProcessing`.

> 如果 `onCleanup` 回调成功，节点将转换为 `Unconfigured` 状态。
> 如果 `onCleanup` 回调引发或导致任何其他返回代码，节点将转换为 `ErrorProcessing`。

### Transition State: `Activating`

In this transition state the callback `onActivate` will be executed. This method is expected to do any final preparations to start executing. This may include acquiring resources that are only held while the node is actually active, such as access to hardware. Ideally, no preparation that requires significant time (such as lengthy hardware initialisation) should be performed in this callback.

> 在这个转换状态下，回调 `onActivate` 将被执行。此方法预期将做出任何最终准备来开始执行。这可能包括**获取仅在节点实际活动时持有的资源，例如对硬件的访问权限**。理想情况下，**不应该在此回调中执行需要大量时间(例如长时间的硬件初始化)的准备工作**。

#### Valid transitions out if `Activating`

- If the `onActivate` callback succeeds the node will transition to `Active`.
- If the `onActivate` callback raises or results in any other return code the node will transition to `ErrorProcessing`.

> 如果 `onActivate` 回调成功，节点将转换为 `Active` 状态。
> 如果 `onActivate` 回调引发或产生任何其他返回码，节点将转换为 `ErrorProcessing`。

### Transition State: Deactivating

In this transition state the callback `onDeactivate` will be executed. This method is expected to do any cleanup to start executing, and should reverse the `onActivate` changes.

> 在这个转换状态下，回调 `onDeactivate` 将被执行。此方法旨在执行任何清理工作，并且应该撤销 `onActivate` 所做的更改。

#### Valid transitions out of Deactivating

- If the `onDeactivate` callback succeeds the node will transition to `Inactive`.
- If the `onDeactivate` callback raises or results in any other return code the node will transition to `ErrorProcessing`.

> 如果 `onDeactivate` 回调成功，节点将转换为“非活动”状态。
> 如果 `onDeactivate` 回调引发或导致任何其他返回代码，节点将转换为 `ErrorProcessing`。

### Transition State: ShuttingDown

In this transition state the callback `onShutdown` will be executed. This method is expected to do any cleanup necessary before destruction. It may be entered from any Primary State except `Finalized`, the originating state will be passed to the method.

> 在这个过渡状态中，回调 `onShutdown` 将被执行。这个方法预计会在销毁之前做任何必要的清理工作。它可以从任何主状态进入，除了 `Finalized`，原始状态将被传递给该方法。

#### Valid transitions out of ShuttingDown

- If the `onShutdown` callback succeeds the node will transition to `Finalized`.
- If the `onShutdown` callback raises or results in any other return code the node will transition to `ErrorProcessing`.

> 如果 `onShutdown` 回调成功，节点将转换为 `Finalized`。
> 如果 `onShutdown` 回调引发或导致任何其他返回码，节点将转换为 `ErrorProcessing`。

### Transition State: ErrorProcessing

This transition state is where any error can be cleaned up. It is possible to enter this state from any state where user code will be executed. If error handling is successfully completed the node can return to `Unconfigured`, If a full cleanup is not possible it must fail and the node will transition to `Finalized` in preparation for destruction.

> 这个转换状态是**可以清理任何错误**的地方。可以从任何执行用户代码的状态进入这个状态。如果**错误处理成功完成，节点可以返回到 `Unconfigured` 状态**，如果不能完全清理，它必须失败，节点将转换到 `Finalized` 状态，为销毁做准备。

Transitions to `ErrorProcessing` may be caused by error return codes in callbacks as well as methods within a callback or an uncaught exception.

> 转换到 `ErrorProcessing` 可能是由回调中的错误返回码或回调中的方法或未捕获的异常引起的。

#### Valid transitions out of ErrorProcessing

- If the `onError` callback succeeds the node will transition to `Unconfigured`.
  It is expected that the `onError` will clean up all state from any previous state.

> 如果 `onError` 回调成功，节点将转换为 `Unconfigured`。
> 预计 `onError` 将清理所有之前状态的状态。

As such if entered from `Active` it must provide the cleanup of both `onDeactivate` and `onCleanup` to return success.

- If the `onShutdown` callback raises or results in any other result code the node will transition to `Finalized`.

> 如此，如果从 `Active` 状态输入，则必须提供 `onDeactivate` 和 `onCleanup` 的清理，以返回成功。
> 如果 `onShutdown` 回调引发或产生任何其他结果码，节点将转换为 `Finalized`。

### Destroy Transition

This transition will simply cause the deallocation of the node. In an object oriented environment it may just involve invoking the destructor. Otherwise it will invoke a standard deallocation method. This transition should always succeed.

> 这个转换只会**导致节点的释放。在面向对象的环境中，它可能只涉及调用析构函数。否则，它将调用标准的释放方法**。**这个转换应该总是成功的**。

### Create Transition

This transition will instantiate the node, but will not run any code beyond the constructor.

> 这个转换将实例化节点，但**不会运行构造函数之外的任何代码**。

## Management Interface

A managed node will be exposed to the ROS ecosystem by the following interface, as seen by tools that perform the managing. This interface should not be subject to the restrictions on communications imposed by the lifecycle states.

> 一个管理的节点将通过以下接口暴露给 ROS 生态系统，这将**由执行管理的工具所看到**。此接口**不应受到由生命周期状态强加的通信限制**。

> [!NOTE]
> 存在管理工具，即“manager”的概念
> 并且这个地方不收到生命周期状态的影响

It is expected that a common pattern will be to have a container class which loads a managed node implementation from a library and through a plugin architecture automatically exposes the required management interface via methods and the container is not subject to the lifecycle management. However, it is fully valid to consider any implementation which provides this interface and follows the lifecycle policies a managed node. Conversely, any object that provides these services but does not behave in the way defined in the life cycle state machine is malformed.

> **预期的一种常见模式是拥有一个容器类，它从库中加载受管理的节点实现，并通过插件架构自动暴露所需的管理接口**，容器不受生命周期管理的约束。然而，完全合法地考虑任何提供这种接口并遵循生命周期策略的实现都是受管理的节点。反之，任何提供这些服务但不按照生命周期状态机中定义的方式行事的对象都是不正确的。

> [!NOTE]
> 这里提出了一种使用方式，**定义一个容器类，从库中加载需要管理的节点**。
> 值得参考，但是不是特别清楚。

These services may also be provided via attributes and method calls (for local management) in addition to being exposed ROS messages and topics/services (for remote management). In the case of providing a ROS middleware interface, specific topics must be used, and they should be placed in a suitable namespace.

> 这些服务也可以通过属性和方法调用(用于本地管理)提供，除了通过 ROS 消息和主题/服务(用于远程管理)暴露。在提供 ROS 中间件接口的情况下，**必须使用特定的主题**，并将它们放在合适的命名空间中。

Each possible supervisory transition will be provided as a service by the name of the transition except `create`. `create` will require an extra argument for finding the node to instantiate. The service will report whether the transition was successfully completed.

> 每个可能的监督转换都将以转换的名称**提供服务**，除了“创建”。“创建”将需要一个额外的参数来查找要实例化的节点。该服务将报告转换是否成功完成。

### Lifecycle events

A topic should be provided to broadcast the new life cycle state when it changes. This topic must be latched. The topic must be named `lifecycle_state` it will carry both the end state and the transition, with result code. It will publish every time that a transition is triggered, whether successful or not.

> 每次生命周期状态发生变化时，应**提供一个主题来广播新的生命周期状态**。**此主题必须被锁定。该主题必须命名为 `lifecycle_state`**，它将携带终止状态和转换，以及结果代码。每次触发转换时，无论是否成功，都将发布。

> [!NOTE]
> 这就是 5 个默认接口之一的 pub。

## Node Management

There are several different ways in which a managed node may transition between states. Most state transitions are expected to be coordinated by an external management tool which will provide the node with it's configuration and start it. The external management tool is also expected monitor it and execute recovery behaviors in case of failures. A local management tool is also a possibility, leveraging method level interfaces. And a node could be configured to self manage, however this is discouraged as this will interfere with external logic trying to managed the node via the interface.

> 有几种不同的方式可以使管理节点在状态之间过渡。大多数**状态转换都预期由外部管理工具协调**，该工具将向节点提供其配置并启动它。**外部管理工具还应监视它并在出现故障时执行恢复行为。** 本地管理工具也是一种可能性，利用方法级接口。**节点可以配置为自我管理，但不鼓励这样做**，因为这会干扰尝试通过接口管理节点的外部逻辑。

There is one transition expected to originate locally, which is the `ERROR` transition. A managed node may also want to expose arguments to automatically configure and activate when run in an unmanaged system.

> 有一个预期从本地发起的转换，即 `ERROR` 转换。管理节点还可能希望暴露参数，以便在非管理系统中自动配置和激活。

> [!NOTE]
> 这里也是提出了，还是**要有 manager 的角色**，而不是节点自己管理自己的状态，至少两者不能共存。
> 但是 `ERROR` 的状态转换除外，这个是由节点自己出发转换的！
> 这里除了参考使用的脚本之外，还可以参开 navigation2 中的 lifecycle manager 的实现

## Extensions

This lifecycle will be required to be supported throughout the toolchain as such this design is not intended to be extended with additional states. It is expected that there will be more complicated application specific state machines. They may exist inside of any lifecycle state or at the macro level these lifecycle states are expected to be useful primitives as part of a supervisory system.

> **这个生命周期将需要在整个工具链中得到支持，因此这个设计不打算用额外的状态进行扩展。** 预计会有更复杂的特定应用状态机。它们可能存在于任何生命周期状态内，或者在宏观层面，这些生命周期状态预计可以作为监督系统的有用原语。
