---
translate by google@2023-05-26 15:18:31
layout: default
title: Managed nodes
abstract:
This article describes the concept of a node with a managed life cycle.
It aims to document some of the options for supporting manage d-life cycle nodes in ROS 2.
It has been written with consideration for the existing design of the ROS 2 C++ client library, and in particular the current design of executors.

> 本文描述了具有托管生命周期的节点的概念。
> 它旨在记录 ROS 2 中支持管理 D 寿命周期节点的一些选项。
> 它已经考虑到 ROS 2 C ++客户库库的现有设计，尤其是执行者的当前设计。

author: '[Geoffrey Biggs](https://github.com/gbiggs) [Tully Foote](https://github.com/tfoote)'
date_written: 2015-06
last_modified: 2021-02
published: true
---

{:toc}

<div class="abstract" markdown="1">
{{ page.abstract }}
</div>

Authors: {{ page.author }}
Date Written: {{ page.date_written }}
Last Modified: {% if page.last_modified %}{{ page.last_modified }}{% else %}{{ page.date_written }}{% endif %}

## Background

A managed life cycle for nodes allows greater control over the state of ROS system.

> 节点的托管生命周期可以更好地控制 ROS 系统的状态。

It will allow roslaunch to ensure that all components have been instantiated correctly before it allows any component to begin executing its behaviour.

> 它将允许 Roslaunch 确保所有组件都已正确实例化，然后才能允许任何组件开始执行其行为。

It will also allow nodes to be restarted or replaced on-line.

> 它还将允许在线重新启动或更换节点。

The most important concept of this document is that a managed node presents a known interface, executes according to a known life cycle state machine, and otherwise can be considered a black box.

> 该文档的最重要概念是，托管节点呈现已知的接口，根据已知的生命周期状态机执行，否则可以将其视为黑匣子。

This allows freedom to the node developer on how they provide the managed life cycle functionality, while also ensuring that any tools created for managing nodes can work with any compliant node.

> 这使节点开发人员如何提供托管生命周期功能的自由，同时还确保为管理节点管理创建的任何工具都可以与任何合规的节点一起使用。

## Life cycle

![The proposed node life cycle state machine](/img/node_lifecycle/life_cycle_sm.png "The proposed node life cycle state machine")

There are 4 primary states:

- `Unconfigured`
- `Inactive`
- `Active`
- `Finalized`

To transition out of a primary state requires action from an external supervisory process, with the exception of an error being triggered in the `Active` state.

> 要从主要状态过渡需要从外部监督过程中采取行动，除了在“ Active”状态下触发错误。

There are also 6 transition states which are intermediate states during a requested transition.

> 在要求的过渡期间，还有 6 个过渡状态，它们是中间状态。

- `Configuring`
- `CleaningUp`
- `ShuttingDown`
- `Activating`
- `Deactivating`
- `ErrorProcessing`

In the transitions states logic will be executed to determine if the transition is successful.

> 在过渡状态中，将执行逻辑以确定过渡是否成功。

Success or failure shall be communicated to lifecycle management software through the lifecycle management interface.

> 成功或失败应通过生命周期管理接口传达到生命周期管理软件。

There are 7 transitions exposed to a supervisory process, they are:

> 有 7 个过渡到监督过程，它们是：

- `create`
- `configure`
- `cleanup`
- `activate`
- `deactivate`
- `shutdown`
- `destroy`

The behavior of each state is as defined below.

> 每个状态的行为如下定义。

### Primary State: `Unconfigured`

This is the life cycle state the node is in immediately after being instantiated.

> 这是生命周期状态在实例化后立即进入。

This is also the state in which a node may be retuned to after an error has happened.

> 这也是在发生错误发生后可以将节点重新调整为的状态。

In this state there is expected to be no stored state.

> 在这种状态下，预计将没有存储状态。

#### Valid transition out

- The node may transition to the `Inactive` state via the `configure` transition.

> - 节点可以通过`configure'transition 过渡到'非活动状态。

- The node may transition to the `Finalized` state via the `shutdown` transition.

> - 节点可以通过`shutdown''过渡过渡到`最终确定的状态。

### Primary State: `Inactive`

This state represents a node that is not currently performing any processing.

> 该状态表示当前不执行任何处理的节点。

The main purpose of this state is to allow a node to be (re-)configured (changing configuration parameters, adding and removing topic publications/subscriptions, etc) without altering its behavior while it is running.

> 该状态的**主要目的是允许（重新）配置一个节点（更改配置参数，添加和删除主题出版物/订阅等），而无需在运行时更改其行为**。

While in this state, the node will not receive any execution time to read topics, perform processing of data, respond to functional service requests, etc.

> 在这种状态下，节点将不会收到任何执行时间来读取主题，执行数据处理，响应功能服务请求等。

In the inactive state, any data that arrives on managed topics will not be read and or processed.

> 在非活动状态下，任何到达托管主题的数据都不会被读取和或处理。

Data retention will be subject to the configured QoS policy for the topic.

> **数据保留将受到该主题配置的 QoS 策略的约束。**

Any managed service requests to a node in the inactive state will not be answered (to the caller, they will fail immediately).

> 任何托管的服务请求都不会回答非活动状态下的节点（对呼叫者，它们将立即失败）。

#### Valid transitions out of Inactive

- A node may transition to the `Finalized` state via the `shutdown` transition.
- A node may transition to the `Unconfigured` state via the `cleanup` transition.
- A node may transition to the `Active` state via the `activate` transition.

> - 节点可以通过`shutdown''过渡过渡到`最终确定的状态。
> - 节点可以通过“清理”过渡过渡到“未配置”状态。
> - 一个节点可以通过`activate''过渡到`活动'状态。

### Primary State: Active

This is the main state of the node's life cycle.

> 这是节点生命周期的主要状态。

While in this state, the node performs any processing, responds to service requests, reads and processes data, produces output, etc.

> 在此状态下，节点执行任何处理，响应服务请求，读取和处理数据，产生输出等。

If an error that cannot be handled by the node/system occurs in this state, the node will transition to `ErrorProcessing`.

> 如果在此状态下发生由节点/系统无法处理的错误，则节点将过渡到“错误处理”。

#### Valid transitions out of Active

- A node may transition to the `Inactive` state via the `deactivate` transition.
- A node may transition to the `Finalized` state via the `shutdown` transition.

> - 一个节点可以通过'停用'过渡过渡到“非活动”状态。
> - 节点可以通过`shutdown''过渡过渡到`最终确定的状态。

### Primary State: Finalized

The `Finalized` state is the state in which the node ends immediately before being destroyed.

> “最终确定”状态是节点在被破坏之前立即结束的状态。

This state is always terminal the only transition from here is to be destroyed.

> 该状态始终是终端，从这里唯一的过渡是被摧毁。

This state exists to support debugging and introspection.

> 这个状态的存在是为了支持调试和内省。

A node which has failed will remain visible to system introspection and may be potentially introspectable by debugging tools instead of directly destructing.

> 失败的节点将在系统内省可见，并且可以通过调试工具而不是直接破坏来进行内省。

If a node is being launched in a respawn loop or has known reasons for cycling it is expected that the supervisory process will have a policy to automatically destroy and recreate the node.

> 如果正在以重生循环启动节点或已知骑自行车的原因，则预计监督过程将有一项自动破坏和重新创建节点的政策。

#### Valid transitions out of Finalized

- A node may be deallocated via the `destroy` transition.

> - 可以通过“毁灭”过渡来处理一个节点。

### Transition State: `Configuring`

In this transition state the node's `onConfigure` callback will be called to allow the node to load its configuration and conduct any required setup.

> 在此过渡状态下，将调用节点的“ conconfigure”回调，以允许节点加载其配置并进行任何必需的设置。

The configuration of a node will typically involve those tasks that must be performed once during the node's life time, such as obtaining permanent memory buffers and setting up topic publications/subscriptions that do not change.

> 节点的配置通常涉及在节点寿命期间必须执行一次任务的那些任务，例如获得永久的内存缓冲区和设置不会更改的主题出版物/订阅。

The node uses this to set up any resources it must hold throughout its life (irrespective of if it is active or inactive).

> 该节点使用此设置它在整个生命中必须拥有的任何资源（无论它是活跃还是不活动）。

As examples, such resources may include topic publications and subscriptions, memory that is held continuously, and initialising configuration parameters.

> 作为示例，此类资源可能包括主题出版物和订阅，连续保存的内存以及初始化配置参数。

#### Valid transitions out of Configuring

- If the `onConfigure` callback succeeds the node will transition to `Inactive`
- If the `onConfigure` callback results in a failure code (TODO specific code) the node will transition back to `Unconfigured`.
- If the `onConfigure` callback raises or results in any other result code the node will transition to `ErrorProcessing`

> - 如果``onconfigure'回调成功了
> - 如果``onConfigure`回调都会产生故障代码（特定代码），则节点将过渡到“未配置”。
> - 如果``onConfigure`回调都会提出或结果。

### Transition State: CleaningUp

In this transition state the node's callback `onCleanup` will be called.

> 在此过渡状态下，节点的回调`oncleanup'将被调用。

This method is expected to clear all state and return the node to a functionally equivalent state as when first created.

> 预计该方法将清除所有状态，并像第一次创建时将节点返回到功能等效的状态。

If the cleanup cannot be successfully achieved it will transition to `ErrorProcessing`.

> 如果无法成功实现清理，它将过渡到“错误处理”。

#### Valid transitions out if CleaningUp

- If the `onCleanup` callback succeeds the node will transition to `Unconfigured`.
- If the `onCleanup` callback raises or results in any other return code the node will transition to `ErrorProcessing`.

> - 如果“ oncleanup”回调成功，节点将过渡到“未配置”。
> - 如果`onCleanup`回调都会提高或导致节点将过渡到`rorralProcessing'的任何其他返回代码。

### Transition State: `Activating`

In this transition state the callback `onActivate` will be executed.

> 在此过渡状态下，将执行回调`on Activate'。

This method is expected to do any final preparations to start executing.

> 预计此方法将做任何最终准备工作以开始执行。

This may include acquiring resources that are only held while the node is actually active, such as access to hardware.

> 这可能包括获取仅在节点实际活动时持有的资源，例如访问硬件。

Ideally, no preparation that requires significant time (such as lengthy hardware initialisation) should be performed in this callback.

> 理想情况下，不应在此回调中执行任何需要大量时间的准备工作（例如冗长的硬件初始化）。

#### Valid transitions out if Activating

- If the `onActivate` callback succeeds the node will transition to `Active`.

> - 如果“ noctivate”回调成功，则节点将过渡到`Active'。

- If the `onActivate` callback raises or results in any other return code the node will transition to `ErrorProcessing`.

> - 如果“ nocivate”回调会增加或导致节点将过渡到`rorralProcessing'的任何其他返回代码。

### Transition State: Deactivating

In this transition state the callback `onDeactivate` will be executed.

> 在此过渡状态下，将执行回调`onDeactivate'。

This method is expected to do any cleanup to start executing, and should reverse the `onActivate` changes.

> 预计此方法将进行任何清理以开始执行，并应逆转“激活”更改。

#### Valid transitions out of Deactivating

- If the `onDeactivate` callback succeeds the node will transition to `Inactive`.

> - 如果`dectivate'回调成功，节点将过渡到``inativive''。

- If the `onDeactivate` callback raises or results in any other return code the node will transition to `ErrorProcessing`.

> - 如果`onDeactivate`回调都会提出或导致节点将过渡到`rorralProcessing'的任何其他返回代码。

### Transition State: ShuttingDown

In this transition state the callback `onShutdown` will be executed.

> 在此过渡状态下，将执行回调`shutdown`。

This method is expected to do any cleanup necessary before destruction.

> 预计该方法将在破坏之前进行任何必要的清理。

It may be entered from any Primary State except `Finalized`, the originating state will be passed to the method.

> 它可以从任何主要状态输入，除了“最终确定”，原始状态将传递给该方法。

#### Valid transitions out of ShuttingDown

- If the `onShutdown` callback succeeds the node will transition to `Finalized`.

> - 如果``shutdown`回调成功''节点将过渡到`最终确定。

- If the `onShutdown` callback raises or results in any other return code the node will transition to `ErrorProcessing`.

> - 如果`shutdown`回调都会提高或导致节点将过渡到`rorralProcessing'的任何其他返回代码。

### Transition State: ErrorProcessing

This transition state is where any error can be cleaned up.

> 这种过渡状态是可以清理任何错误的地方。

It is possible to enter this state from any state where user code will be executed.

> 可以从将执行用户代码的任何状态输入此状态。

If error handling is successfully completed the node can return to `Unconfigured`,

> 如果成功完成错误处理，则节点可以返回到“未配置”，

If a full cleanup is not possible it must fail and the node will transition to `Finalized` in preparation for destruction.

> 如果不可能进行全面清理，则必须失败，节点将过渡到“最终确定”以准备破坏。

Transitions to `ErrorProcessing` may be caused by error return codes in callbacks as well as methods within a callback or an uncaught exception.

> 过渡到“错误处理”可能是由回调中的错误返回代码以及回调或未知异常中的方法引起的。

#### Valid transitions out of ErrorProcessing

- If the `onError` callback succeeds the node will transition to `Unconfigured`.

> - 如果`onError'回调成功，节点将过渡到“未配置”。

It is expected that the `onError` will clean up all state from any previous state.

> 预计“ Onerror”将清理任何以前的州的所有州。

As such if entered from `Active` it must provide the cleanup of both `onDeactivate` and `onCleanup` to return success.

> 因此，如果从“ Active”中输入，则必须提供对``OnDeactivate''和 onCleanup'的清理，以返回成功。

- If the `onShutdown` callback raises or results in any other result code the node will transition to `Finalized`.

> - 如果``shutdown`回调''升级或导致节点将过渡到`最终确定的任何其他结果代码。

### Destroy Transition

This transition will simply cause the deallocation of the node.
In an object oriented environment it may just involve invoking the destructor.
Otherwise it will invoke a standard deallocation method.
This transition should always succeed.

> 这种过渡将仅引起节点的交易。
> 在面向对象的环境中，它可能仅涉及调用击曲线。
> 否则，它将调用标准交易方法。
> 这种过渡应该永远成功。

### Create Transition

This transition will instantiate the node, but will not run any code beyond the constructor.

> 该过渡将实例化节点，但不会运行构造函数之外的任何代码。

## Management Interface

A managed node will be exposed to the ROS ecosystem by the following interface, as seen by tools that perform the managing.

> 通过以下接口将托管节点接触到 ROS 生态系统，如执行管理的工具所示。

This interface should not be subject to the restrictions on communications imposed by the lifecycle states.

> 该界面不应受到生命周期施加的通信的限制。

It is expected that a common pattern will be to have a container class which loads a managed node implementation from a library and through a plugin architecture automatically exposes the required management interface via methods and the container is not subject to the lifecycle management.

> 预计通用模式将是拥有一个容器类，该类别从库中加载托管节点实现，并通过插件体系结构自动通过方法来公开所需的管理接口，并且容器不受生命周期管理的约束。

However, it is fully valid to consider any implementation which provides this interface and follows the lifecycle policies a managed node.

> 但是，考虑提供此接口的任何实现并遵循托管节点的生命周期策略，这是完全有效的。

Conversely, any object that provides these services but does not behave in the way defined in the life cycle state machine is malformed.

> 相反，任何提供这些服务但不会以生命周期状态机器定义的方式畸形的对象都会畸形。

These services may also be provided via attributes and method calls (for local management) in addition to being exposed ROS messages and topics/services (for remote management).

> 除了暴露 ROS 消息和主题/服务（用于远程管理）外，还可以通过属性和方法调用（用于本地管理）提供这些服务。

In the case of providing a ROS middleware interface, specific topics must be used, and they should be placed in a suitable namespace.

> 在提供 ROS 中间件接口的情况下，必须使用特定主题，并且应将其放置在合适的名称空间中。

Each possible supervisory transition will be provided as a service by the name of the transition except `create`.

> 除“创建”之外，每个可能的监督过渡将以过渡的名称作为服务提供。

`create` will require an extra argument for finding the node to instantiate.

The service will report whether the transition was successfully completed.

> 该服务将报告过渡是否成功完成。

### Lifecycle events

A topic should be provided to broadcast the new life cycle state when it changes.

> 应该提供一个主题，以广播新的生命周期状态。

This topic must be latched.

> 这个主题必须锁定。

The topic must be named `lifecycle_state` it will carry both the end state and the transition, with result code.

> 该主题必须命名为“ LifeCycle_state”，它将带有最终状态和过渡，并带有结果代码。

It will publish every time that a transition is triggered, whether successful or not.

> 无论是否成功，它都会发布每次触发过渡时。

## Node Management

There are several different ways in which a managed node may transition between states.

> 托管节点可以通过几种不同的方式在状态之间过渡。

Most state transitions are expected to be coordinated by an external management tool which will provide the node with it's configuration and start it.

> 预计**大多数状态过渡将通过外部管理工具协调**，该工具将为节点提供其配置并启动它。

The external management tool is also expected monitor it and execute recovery behaviors in case of failures.

> 在发生故障时，还可以预期外部管理工具监控它并执行恢复行为。

A local management tool is also a possibility, leveraging method level interfaces.

> 局部管理工具也是利用方法级别接口的可能性。

And a node could be configured to self manage, however this is discouraged as this will interfere with external logic trying to managed the node via the interface.

> 并且可以将节点配置为自我管理，但是这会劝阻，因为这会干扰试图通过接口管理节点的外部逻辑。

There is one transition expected to originate locally, which is the `ERROR` transition.

> 预计将在本地起源有一个过渡，即“误差”过渡。

A managed node may also want to expose arguments to automatically configure and activate when run in an unmanaged system.

> 托管节点还可能希望公开参数以自动配置和激活在不受管理的系统中运行。

## Extensions

This lifecycle will be required to be supported throughout the toolchain as such this design is not intended to be extended with additional states.

> 将要求在整个工具链中支持此生命周期，因此该设计不打算使用其他状态扩展。

It is expected that there will be more complicated application specific state machines.

> 预计会有更复杂的应用程序机器。

They may exist inside of any lifecycle state or at the macro level these lifecycle states are expected to be useful primitives as part of a supervisory system.

> 它们可能存在于任何生命周期内或宏观层面，这些生命周期有用作为监督系统的一部分是有用的。
