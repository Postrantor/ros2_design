---
translate by baidu@2023-05-26 16:12:16
layout: default
title: Intra-process Communications in ROS 2
permalink: articles/intraprocess_communications.html
abstract: Description of the current intra-process communication mechanism in ROS 2 and of its drawbacks. Design proposal for an improved implementation. Experimental results.
> 摘要:描述了ROS2中当前的进程内通信机制及其缺点。改进实施的设计方案。实验结果。
published: true
author: '[Alberto Soragna](https://github.com/alsora) [Juan Oxoby](https://github.com/joxoby) [Dhiraj Goel](https://github.com/dgoel)'
date_written: 2020-03
last_modified: 2020-03
---

{:toc}

<div class="abstract" markdown="1">
{{ page.abstract }}
</div>

Authors: {{ page.author }}
Date Written: {{ page.date_written }}
Last Modified: {% if page.last_modified %}{{ page.last_modified }}{% else %}{{ page.date_written }}{% endif %}

## Introduction

The subscriptions and publications mechanisms in ROS 2 fall in two categories:

> ROS 2 中的订阅和发布机制分为两类:

- intra-process: messages are sent from a publisher to subscriptions via in-process memory.

> \*进程内:消息通过进程内内存从发布者发送到订阅。

- inter-process: messages are sent via the underlying ROS 2 middleware layer.

> \*进程间:消息通过底层 ROS2 中间件层发送。

The specifics of how this happens depend on the chosen middleware implementation and may involve serialization steps.

> 具体如何发生取决于所选择的中间件实现，并且可能涉及序列化步骤。

This design document presents a new implementation for the intra-process communication.

> 本设计文件为流程内通信提供了一种新的实现方式。

## Motivations for a new implementation

Even if ROS 2 supports intra-process communication, the implementation of this mechanism has still much space for improvement.

> 即使 ROS 2 支持进程内通信，该机制的实现仍有很大的改进空间。

Until ROS 2 Crystal, major performance issues and the lack of support for shared pointer messages were preventing the use of this feature in real applications.

> 在 ROS 2 Crystal 之前，主要的性能问题和对共享指针消息的缺乏支持阻碍了该功能在实际应用中的使用。

With the ROS 2 Dashing release, most of these issues have been addressed and the intra-process communication behavior has improved greatly ([see ticket](https://github.com/ros2/ros2/issues/649)).

> 随着 ROS 2 Dashing 的发布，这些问题中的大多数都得到了解决，进程内的通信行为也得到了极大的改善([参见票证](https://github.com/ros2/ros2/issues/649))。

The current implementation is based on the creation of a ring buffer for each `Publisher` and on the publication of meta-messages through the middleware layer.

> 目前的实现是基于为每个“发布者”创建一个环形缓冲区，以及通过中间件层发布元消息。

When a `Publisher` has to publish intra-process, it will pass the message to the `IntraProcessManager`.

> 当“Publisher”必须发布进程内消息时，它会将消息传递给“IntraProcessManager”。

Here the message will be stored in the **ring buffer** associated with the `Publisher`.

> 在这里，消息将存储在与“发布者”相关联的环形缓冲区中。

In order to extract a message from the `IntraProcessManager` two pieces of information are needed: the id of the `Publisher` (in order to select the correct ring buffer) and the position of the message within its ring buffer.

> 为了从“IntraProcessManager”中提取消息，需要两条信息:“发布者”的 id(以便选择正确的环形缓冲区)和消息在其环形缓冲区内的位置。

A meta-message with this information is created and sent through the ROS 2 middleware to all the `Subscription`s, which can then retrieve the original message from the `IntraProcessManager`.

> 创建包含此信息的元消息，并通过 ROS 2 中间件将其发送到所有“订阅”，然后订阅可以从“IntraProcessManager”检索原始消息。

![Current IPC Block Diagram](../img/intraprocess_communication/old_ipc.png)

Several shortcomings of the current implementation are listed below.

> 以下列出了目前实施的几个不足之处。

### Incomplete Quality of Service support

The current implementation can't be used when the QoS durability value is set to `Transient Local`.

> 当 QoS 耐久性值设置为“瞬态本地”时，无法使用当前实现。

The current implementation does not enforce the depth of the QoS history in a correct way.

> 当前的实现方式没有以正确的方式强制执行 QoS 历史的深度。

The reason is that there is a single ring buffer per `Publisher` and its size is equal to the depth of the `Publisher`'s history.

> 原因是每个“发布者”有一个单独的环形缓冲区，其大小等于“发布器”历史的深度。

> [!NOTE]
> 需要注意

A `Publisher` stores a message in the ring buffer and then it sends a meta-message to allow a `Subscription` to retrieve it.

> “发布者”将消息存储在环形缓冲区中，然后发送元消息以允许“订阅”检索该消息。

The `Subscription` correctly stores meta-messages up to the number indicated by its depth of the history, but, depending on the frequency at which messages are published and callbacks are triggered, it may happen that a meta-message processed from the `Subscription` does not correspond anymore to a valid message in the ring buffer, because it has been already overwritten.

> “订阅”正确地存储元消息，直到其历史深度所指示的数量，但是，根据消息发布和触发回调的频率，可能会发生从“订阅”处理的元消息不再对应于环形缓冲区中的有效消息，因为它已经被覆盖。

This results in the loss of the message and it is also a difference in behavior between intra and inter-process communication, since, with the latter, the message would have been received.

> 这导致消息丢失，也是进程内通信和进程间通信之间行为的差异，因为如果是进程间通信，消息就会被接收到。

Moreover, even if the use of meta-messages allows to deleagate the enforcement of other QoS settings to the RMW layer, every time a message is added to the ring buffer the `IntraProcessManager` has to compute how many `Subscription`s will need it.

> 此外，即使**元消息**的使用允许取消对 RMW 层的其他 QoS 设置的强制执行，每次将消息添加到环形缓冲区时，“IntraProcessManager”都必须计算有多少“Subscription”需要它。

> [!NOTE]
> 元消息？

This potentially breaks the advantage of having the meta-messages.

> 这可能会破坏拥有元消息的优势。

For example, the `IntraProcessManager` has to take into account that potentially all the known `Subscription`s will take the message, regardless of their reliability QoS.

> 例如，“IntraProcessManager”必须考虑到，可能所有已知的“Subscription”都会接收该消息，而不管它们的可靠性 QoS 如何。

If a `Publisher` or a `Subscription` are best-effort, they may not receive the meta-message thus preventing the `IntraProcessManager` from releasing the memory in the buffer.

> 如果“发布者”或“订阅”尽了最大努力，它们可能不会接收元消息，从而阻止“IntraProcessManager”释放缓冲区中的内存。

More details [here](https://index.ros.org/doc/ros2/Concepts/About-Quality-of-Service-Settings/).

> 更多详细信息[点击此处](https://index.ros.org/doc/ros2/Concepts/About-Quality-of-Service-Settings/)。

**TODO:** take into account also new QoS: Deadline, Liveliness and Lifespan
[reference](https://github.com/ros2/design/pull/212).

> **TODO:**同时考虑新的 QoS:截止日期、活力和寿命
> 【参考文献】(https://github.com/ros2/design/pull/212)。

### Dependent on the RMW

The current implementation of intra-process communication has to send meta-messages from the `Publisher` to the `Subscription`s.

> 进程内通信的当前实现必须将元消息从“发布者”发送到“订阅者”。

This is done using the `rmw_publish` function, the implementation of which depends on the chosen middleware.

> 这是使用“rmw_publish”函数完成的，该函数的实现取决于所选的中间件。

This results in the performance of a ROS 2 application with intra-process communication enabled being heavily dependent on the chosen RMW implementation.

> 这导致启用进程内通信的 ROS 2 应用程序的性能在很大程度上取决于所选的 RMW 实现。

Given the fact that these meta-messages have only to be received from entities within the same process, there is space for optimizing how they are transmitted by each RMW.

> 考虑到这些元消息只需要从同一进程内的实体接收，因此存在优化每个 RMW 如何传输它们的空间。

However, at the moment none of the supported RMW is actively tackling this issue.

> 然而，目前没有一个得到支持的 RMW 积极解决这个问题。

This results in that the performance of a single process ROS 2 application with intra-process communication enabled are still worst than what you could expect from a non-ROS application sharing memory between its components.

> 这导致启用进程内通信的单进程 ROS 2 应用程序的性能仍然比非 ROS 应用程序在其组件之间共享内存的性能差。

In the following some experimental evidences are quickly presented.

> 以下是一些实验证据。

#### Memory requirement

When a `Node` creates a `Publisher` or a `Subscription` to a topic `/MyTopic`, it will also create an additional one to the topic `/MyTopic/_intra`.

> 当“节点”为主题“/MyTopic”创建“发布者”或“订阅”时，它还会为主题“/MyTopic/\_itra”创建一个附加的发布者或订阅。

The second topic is the one where meta-messages travel.

> 第二个主题是元消息传播的主题。

Our [experimental results](https://github.com/irobot-ros/ros2-performance/tree/master/performances/experiments/crystal/pub_sub_memory#adding-more-nodes-x86_64) show that creating a `Publisher` or a `Subscription` has a non-negligible memory cost.

> 我们的[实验结果](https://github.com/irobot-ros/ros2-performance/tree/master/performances/experiments/crystal/pub_sub_memory#adding-more-nodes-x86_64)表明，创建“发布服务器”或“订阅”具有不可忽略的内存成本。

This is particularly true for the default RMW implementation, Fast-RTPS, where the memory requirement increases almost expontentially with the number of participants and entities.

> 对于默认的 RMW 实现 Fast RTPS 来说尤其如此，其中内存需求几乎随着参与者和实体的数量而明显增加。

#### Latency and CPU utilization

Publishing a meta-message has the same overhead as that of publishing a small inter-process message.

> 发布元消息的开销与发布小型进程间消息的开销相同。

However, comparing the publication/reception of an intra and an inter-process message, the former requires several additional operations: it has to store the message in the ring buffer, monitor the number of `Subscription`s, and extract the message.

> 然而，比较进程内消息和进程间消息的发布/接收，前者需要几个额外的操作:它必须将消息存储在环形缓冲区中，监视“订阅”的数量，并提取消息。

The result is that from the latency and CPU utilization point of view, it is convenient to use intra-process communication only when the message size is at least 5KB.

> 结果是，从延迟和 CPU 利用率的角度来看，只有当消息大小至少为 5KB 时，才可以方便地使用进程内通信。

### Problems when both inter and intra-process communication are needed

Currently, ROS 2 does not provide any API for making nodes or `Publisher` and `Subscription` to ignore each other.

> 目前，ROS 2 不提供任何 API 来创建节点，也不提供“发布者”和“订阅”来忽略彼此。

This feature would be useful when both inter and intra-process communication are needed.

> 当同时需要进程间和进程内通信时，此功能将非常有用。

The reason is that the current implementation of the ROS 2 middleware will try to deliver inter-process messages also to the nodes within the same process of the `Publisher`, even if they should have received an intra-process message.

> 原因是 ROS 2 中间件的当前实现将尝试将进程间消息也传递到“发布者”的同一进程内的节点，即使它们本应接收到进程内消息。

Note that these messages will be discarded, but they will still cause an overhead.

The DDS specification provides ways for potentially fixing this problem, i.e. with the `ignore_participant`, `ignore_publication` and `ignore_subscription`operations.

> DDS 规范提供了潜在地解决此问题的方法，即使用“ignore_participant”、“ignore_publication”和“ignore_subscription”操作。

Each of these can be used to ignore a remote participant or entity, allowing to behave as that remote participant did not exist.

> 其中的每一个都可以用来忽略远程参与者或实体，允许其表现为该远程参与者不存在。

The current intra-process communication uses meta-messages that are sent through the RMW between nodes in the same process.

> 当前进程内通信使用通过 RMW 在同一进程中的节点之间发送的元消息。

This has two consequences: first it does not allow to directly "ignore" participants in the same process, because they still have to communicate in order to send and receive meta-messages, thus requiring a more fine-grained control ignoring specific `Publisher`s and `Subscription`s.

> 这有两个后果:首先，它不允许直接“忽略”同一过程中的参与者，因为他们仍然必须进行通信才能发送和接收元消息，因此需要更细粒度的控制来忽略特定的“发布者”和“订阅”。

Moreover, the meta-messages could be delivered also to nodes in different processes if they have intra-process communication enabled.

> 此外，如果启用了进程内通信，则元消息也可以被传递到不同进程中的节点。

As before, the messages would be discarded immediately after being received, but they would still affect the performances.

> 和以前一样，消息在收到后会立即被丢弃，但它们仍然会影响性能。

The overhead caused by the additional publication of meta-messages can be potentially reduced by appending to the intra-process topic names a process specific identifier.

> 通过在进程内主题名称中添加进程特定标识符，可以潜在地减少由元消息的额外发布引起的开销。

## Proposed implementation

### Overview

The new proposal for intra-process communication addresses the issues previously mentioned.

> 关于进程内沟通的新提议解决了前面提到的问题。

It has been designed with performance in mind, so it avoids any communication through the middleware between nodes in the same process.

> 它的设计考虑到了性能，因此避免了在同一过程中通过中间件在节点之间进行任何通信。

Consider a simple scenario, consisting of `Publisher`s and `Subscription`s all in the same process and with the durability QoS set to `volatile`.

> 考虑一个简单的场景，由“发布者”和“订阅”都在同一进程中组成，持久性 QoS 设置为“易失性”。

The proposed implementation creates one buffer per `Subscription`.

> 拟议的实施方式为每个“订阅”创建一个缓冲区。

When a message is published to a topic, its `Publisher` pushes the message into the buffer of each of the `Subscription`s related to that topic and raises a notification, waking up the executor.

> **当消息发布到某个主题时，其“发布者”会将消息推送到与该主题相关的每个“订阅”的缓冲区，并发出通知，唤醒执行者。**

The executor can then pop the message from the buffer and trigger the callback of the `Subscription`.

> 然后，**执行器可以从缓冲区弹出消息，并触发“订阅”的回调。**

> [!NOTE]
> 重点！！！pub/sub/executor 之间的关系，就是这两句话

![Proposed IPC Block Diagram](../img/intraprocess_communication/new_ipc.png)

The choice of having independent buffers for each `Subscription` leads to the following advantages:

> 为每个“订阅”选择独立的缓冲区会带来以下优点:

- It is easy to support different QoS for each `Subscription`, while, at the same time, simplifying the implementation.

> - 很容易为每个“订阅”支持不同的 QoS，同时简化了实现。

- Multiple `Subscription`s can extract messages from their own buffer in parallel without blocking each other, thus providing an higher throughput.

> - **多个“订阅”可以并行地从自己的缓冲区中提取消息，而不会相互阻塞，从而提供更高的吞吐量。**

The only drawback is that the system is not reusing as much resources as possible, compared to sharing buffers between entities.

> 唯一的缺点是，与实体之间共享缓冲区相比，该系统**没有尽可能多地重用资源**。

However, from a practical point of view, the memory overhead caused by the proposed implementation with respect to the current one, will always be only a tiny delta compared to the overall memory usage of the application.

> 然而，从实用的角度来看，与应用程序的整体内存使用量相比，所提出的实现相对于当前实现所造成的内存开销始终只是一个微小的增量。

There are three possible data-types that can be stored in the buffer:

> 有三种可能的数据类型可以存储在缓冲区中:

- `MessageT`
- `shared_ptr<const MessageT>`
- `unique_ptr<MessageT>`

The choice of the buffer data-type is controlled through an additional field in the `SubscriptionOptions`.

> 缓冲区数据类型的选择是通过“SubscriptionOptions”中的一个附加字段控制的。

The default value for this option is denominated `CallbackDefault`, which corresponds to selecting the type between `shared_ptr<constMessageT>` and `unique_ptr<MessageT>` that better fits with its callback type.

This is deduced looking at the output of `AnySubscriptionCallback::use_take_shared_method()`.

> 这是通过查看`AnySubscriptionCallback::use_take_shared_method()`的输出得出的。

If the history QoS is set to `keep all`, the buffers are dynamically adjusted in size up to the maximum resource limits specified by the underlying middleware.

> 如果历史 QoS 被设置为“保持全部”，则缓冲器的大小被动态调整，直到底层中间件指定的最大资源限制。

On the other hand, if the history QoS is set to `keep last`, the buffers have a size equal to the depth of the history and they act as ring buffers (overwriting the oldest data when trying to push while its full).

> 另一方面，如果历史 QoS 被设置为“保持最后”，则缓冲器的大小等于历史的深度，并且它们充当环形缓冲器(当试图在其满时推送时覆盖最旧的数据)。

Note that in case of publishers with `keep all` and `reliable` communication, the behavior can be different from the one of inter-process communication.

In the inter-process case, the middlewares use buffers in both publisher and subscription.

> 在进程间的情况下，中间件在发布者和订阅中都使用缓冲区。

If the subscription queue is full, the publisher one would start to fill and then finally the publish call would block when that queue is full.

> 如果订阅队列已满，发布者将开始填充，然后在该队列已满时，发布调用将阻塞。

Since the intra-process communication uses a single queue on the subscription, this behavior can't be exactly emulated.

> 由于进程内通信在订阅上使用单个队列，因此无法完全模拟这种行为。

Buffers are not only used in `Subscription`s but also in each `Publisher` with a durability QoS of type `transient local`.

> 缓冲区不仅用在“订阅”中，而且用在每个“发布服务器”中，具有“瞬时本地”类型的持久性 QoS。

The data-type stored in the `Publisher` buffer is always `shared_ptr<const MessageT>`.

> 存储在“Publisher”缓冲区中的数据类型始终为“shared_ptr<const MessageT>”。

A new class derived from `rclcpp::Waitable` is defined, which is named `SubscriptionIntraProcessWaitable`.

> 定义了一个派生自“rclcpp::Waitable”的新类，该类名为“SubscriptionIntraProcessWaitable”。

An object of this type is created by each `Subscription` with intra-process communication enabled and it is used to notify the `Subscription` that a new message has been pushed into its ring buffer and that it needs to be processed.

> 这种类型的对象由启用进程内通信的每个“订阅”创建，用于通知“订阅”新消息已被推入其环形缓冲区，需要对其进行处理。

The `IntraProcessManager` class stores information about each `Publisher` and each `Subscription`, together with pointers to these structures.

> “IntraProcessManager”类存储关于每个“发布者”和每个“订阅”的信息，以及指向这些结构的指针。

This allows the system to know which entities can communicate with each other and to have access to methods for pushing data into the buffers.

> 这允许系统知道哪些实体可以相互通信，并可以访问将数据推入缓冲区的方法。

The decision whether to publish inter-process, intra-process or both is made every time the `Publisher::publish()` method is called.

> 每次调用“Publisher::publish()”方法时，都会决定是发布进程间、进程内还是同时发布两者。

For example, if the `NodeOptions::use_intra_process_comms_` is enabled and all the known `Subscription`s are in the same process, then the message is only published intra-process.

> 例如，**如果启用了`NodeOptions::use_intra_process_comms_`，并且所有已知的`Subscription`都在同一进程中，则消息仅在进程内发布。**

> [!NOTE]

This remains identical to the current implementation.

> 这与目前的实施方式保持一致。

### Creating a publisher

1. User calls `Node::create_publisher<MessageT>(...)`.
2. This boils down to `NodeTopics::create_publisher(...)`, where a `Publisher` is created through the factory.
3. Here, if intra-process communication is enabled, eventual intra-process related variables are initialized through the `Publisher::SetupIntraProcess(...)` method.
4. Then the `IntraProcessManager` is notified about the existence of the new `Publisher` through the method `IntraProcessManager::add_publisher(PublisherBase::SharedPtr publisher, PublisherOptions options)`.
5. `IntraProcessManager::add_publisher(...)` stores the `Publisher` information in an internal structure of type `PublisherInfo`.

> 1. 用户调用`Node::create_publisher<MessageT>(…)`。
> 2. 这可以归结为“NodeTopics::create_publisher(…)”，其中“发布者”是通过工厂创建的。
> 3. 这里，如果启用了进程内通信，则通过“Publisher::SetupIntraProcess(…)”方法初始化最终的进程内相关变量。
> 4. 然后通过方法“IntraProcessManager::add_Publisher(PublisherBase::SharedPtr Publisher，PublisherOptions)”通知“IntraProcessorManager'存在新的“Publisher”。
> 5. `IntraProcessManager::add_publisher(…)`将`publisher`信息存储在类型为`PublisherInfo`的内部结构中。

The structure contains information about the `Publisher`, such as its QoS and its topic name, and a weak pointer for the `Publisher` object.

> 该结构包含有关“发布者”的信息，如其 QoS 和主题名称，以及“发布者“对象的弱指针。

An `uint64_t pub_id` unique within the `rclcpp::Context` is assigned to the `Publisher`.

> “rclcpp::Context”中唯一的“uint64_t pub_id”被分配给“Publisher”。

The `IntraProcessManager` contains a `std::map<uint64_t, PublisherInfo>` object where it is possible to retrieve the `PublisherInfo` of a specific `Publisher` given its id.

> “IntraProcessManager”包含一个“std::map ＜ uint64_t，PublisherInfo ＞”对象，可以在该对象中检索给定 id 的特定“Publisher”的“PublisherInfo”。

The function returns the `pub_id`, that is stored within the `Publisher`.

> 该函数返回存储在“Publisher”中的“pub_id”。

If the `Publisher` QoS is set to `transient local`, then the `Publisher::SetupIntraProcess(...)` method will also create a ring buffer of the size specified by the depth from the QoS.

> 如果“Publisher”QoS 设置为“transient local”，则“Publisher::SetupIntraProcess(…)”方法还将创建一个大小由 QoS 深度指定的环形缓冲区。

### Creating a subscription

1. User calls `Node::create_subscription<MessageT>(...)`.
2. This boils down to `NodeTopics::create_subscription(...)`, where a `Subscription` is created through the factory.
3. Here, if intra-process communication is enabled, intra-process related variables are initialized through the `Subscription::SetupIntraProcess(...)` method. The most relevant ones being the ring buffer and the waitable object.
4. Then the `IntraProcessManager` is notified about the existence of the new `Subscription` through the method `IntraProcessManager::add_subscription(SubscriptionBase::SharedPtr subscription, SubscriptionOptions options)`.
5. `IntraProcessManager::add_subscription(...)` stores the `Subscription` information in an internal structure of type `SubscriptionInfo`.

> 1. 用户调用`Node::create_subscription<MessageT>(…)`。
> 2. 这可以归结为`NodeTopics::create_subscription(…)`，其中`subscription`是通过工厂创建的。
> 3. 这里，如果启用进程内通信，则通过`Subscription::SetupIntraProcess(…)`方法初始化进程内相关变量。最相关的是环形缓冲区和可等待对象。
> 4. 然后通过方法`IntraProcessManager::add_Subscription(SubscriptionBase::SharedPtr Subscription，SubscriptionOptions options)`通知`IntraProcessManager`存在新的`Subscription`。
> 5. `IntraProcessManager::add_subscription(…)`将`subscription`信息存储在类型为`SubscriptionInfo`的内部结构中。

The structure contains information about the `Subscription`, such as its QoS, its topic name and the type of its callback, and a weak pointer for the `Subscription` object.

> 该结构包含有关“Subscription”的信息，如其 QoS、主题名称和回调类型，以及“Subscription“对象的弱指针。

An `uint64_t sub_id` unique within the `rclcpp::Context` is assigned to the `Subscription`.

> “rclcpp::Context”中唯一的“uint64_t sub_id”被分配给“Subscription”。

The `IntraProcessManager` contains a `std::map<uint64_t, SubscriptionInfo>` object where it is possible to retrieve the `SubscriptionInfo` of a specific `Subscription` given its id.

> “IntraProcessManager”包含一个“std::map<uint64_t，SubscriptionInfo>”对象，可以在该对象中检索给定 id 的特定“Subscription”的“SubscriptionInfo”。

There is also an additional structure `std::map<uint64_t, std::pair<std::set<uint64_t>, std::set<uint64_t>>>`.

> 还有一个额外的结构`std::map<uint64_t，std::pair<std::set<uint64t>，std::set<uint64_t>>`。

The key of the map is the unique id of a `Publisher` and the value is a pair of sets of ids.

> 映射的键是“发布者”的唯一 id，值是一对 id 集。

These sets contain the ids of the `Subscription`s that can communicate with the `Publisher`.

> 这些集合包含可以与“发布者”通信的“订阅”的 ID。

We have two different sets because we want to differentiate the `Subscription`s depending on whether they request ownership of the received messages or not (note that this decision is done looking at their buffer, since the `Publisher` does not have to interact with the `Subscription` callback).

> 我们有两个不同的集合，因为我们想根据“订阅”是否请求对接收到的消息的所有权来区分它们(请注意，这个决定是根据它们的缓冲区来完成的，因为“发布者”不必与“订阅”回调交互)。

6. The `SubscriptionIntraProcessWaitable` object is added to the list of Waitable interfaces of the node through `node_interfaces::NodeWaitablesInterface::add_waitable(...)`.

> 6. “SubscriptionIntraProcessWaitable”对象通过“node_interfaces::NodeWaitablesInterface::add_Waitable(…)”添加到节点的 Waitable 接口列表中。

It is added to the same callback group used for the standard inter-process communication of that topic.

> 它被添加到用于该主题的标准进程间通信的相同回调组中。

### Publishing only intra-process

#### Publishing unique_ptr

1. User calls `Publisher::publish(std::unique_ptr<MessageT> msg)`.
2. `Publisher::publish(std::unique_ptr<MessageT> msg)` calls `IntraProcessManager::do_intra_process_publish(uint64_t pub_id, std::unique_ptr<MessageT> msg)`.
3. `IntraProcessManager::do_intra_process_publish(...)` uses the `uint64_t pub_id` to call `IntraProcessManager::get_subscription_ids_for_pub(uint64_t pub_id)`.

> 1. 用户调用`Publisher::publish(std::unique_ptr<MessageT>msg)`。
> 2. `Publisher::publish(std::unique_ptr＜MessageT>msg)`调用`IntraProcessManager::do_intra_process_publish(uint64_t pub_id，std::unique_ptr＞MessageT>sg)`。
> 3. `IntraProcessManager::do_intra_process_publish(…)`使用`uint64_t pub_id`调用`IntraProcessManager::get_subscription_ids_for_pub(uint64_t pub_id)`。

This returns the ids corresponding to `Subscription`s that have a QoS compatible for receiving the message.

> 这返回与具有与接收消息兼容的 QoS 的“订阅”相对应的 id。

These ids are divided into two sublists, according to the data-type that is stored in the buffer of each `Susbscription`: requesting ownership (`unique_ptr<MessageT>`) or accepting shared (`shared_ptr<MessageT>`, but also `MessageT` since it will copy data in any case).

> 根据存储在每个“订阅”缓冲区中的数据类型，这些 ID 被分为两个子列表:请求所有权(`unique_ptr<MessageT>`)或接受共享(`shared_ptr<MessageT>`，但也包括`MessageT'，因为它在任何情况下都会复制数据)。

4. The message is "added" to the ring buffer of all the items in the lists.

> 4. 消息被“添加”到列表中所有项目的环形缓冲区中。

The `rcl_guard_condition_t` member of `SubscriptionIntraProcessWaitable` of each `Subscription` is triggered (this wakes up `rclcpp::spin`).

> 触发每个“Subscription”的“SubscriptionIntraProcessWaitable”的“rcl_guard_condition_t”成员(这会唤醒“rclcpp::spin”)。

The way in which the `std::unique_ptr<MessageT>` message is "added" to a buffer, depends on the type of the buffer.

> 将`std::unique_ptr<MessageT>`消息“添加”到缓冲区的方式取决于缓冲区的类型。

- `BufferT = unique_ptr<MessageT>` The buffer receives a copy of `MessageT` and has ownership on it; for the last buffer, a copy is not necessary as ownership can be transferred.
- `BufferT = shared_ptr<const MessageT>` Every buffer receives a shared pointer of the same `MessageT`; no copies are required.
- `BufferT = MessageT` A copy of the message is added to every buffer.

> - `BufferT=unique_ptr<MessageT>`缓冲区接收`MessageT`的副本并拥有该副本的所有权；对于最后一个缓冲区，不需要复制，因为所有权可以转移。
> - `BufferT=shared_ptr<const MessageT>`每个缓冲区都接收同一`MessageT`的共享指针；不需要任何副本。
> - `BufferT=MessageT`消息的副本被添加到每个缓冲区。

![Sequence UML diagram](../img/intraprocess_communication/intra_process_only.png)

#### Publishing other message types

The `Publisher::publish(...)` method is overloaded to support different message types:

> 重载了`Publisher::publish(…)`方法以支持不同的消息类型:

- `unique_ptr<MessageT>`
- `MessageT &`
- `MessageT*`
- `const shared_ptr<const MessageT>`

The last two of them are actually deprecated since ROS 2 Dashing.

> 最后两个实际上是在 ROS2 Dashing 之后被弃用的。

All these methods are unchanged with respect to the current implementation: they end up creating a `unique_ptr` and calling the `Publisher::publish(std::unique_ptr<MessageT> msg)` described above.

> 所有这些方法相对于当前实现都没有变化:它们最终创建了一个“unique_ptr”并调用了上面描述的“Publisher::publish(std::unique_ptr<MessageT>msg)”。

### Receiving intra-process messages

As previously described, whenever messages are added to the ring buffer of a `Subscription`, a condition variable specific to the `Subscription` is triggered.

> 如前所述，每当消息被添加到“订阅”的环形缓冲区时，就会触发“订阅”特有的条件变量。

This condition variable has been added to the `Node` waitset so it is being monitored by the `rclcpp::spin`.

> **此条件变量已添加到“节点”等待集，因此它由“rclcpp::spin”监视。**

Remember that the `SubscriptionIntraProcessWaitable` object has access to the ring buffer and to the callback function pointer of its related `Subscription`.

> 请记住，“SubscriptionIntraProcessWaitable”对象可以访问环形缓冲区及其相关“Subscription”的回调函数指针。

1. The guard condition linked with the `SubscriptionIntraProcessWaitable` object awakes `rclcpp::spin`.
2. The `SubscriptionIntraProcessWaitable::is_ready()` condition is checked.

> 1. 与“SubscriptionIntraProcessWaitable”对象链接的保护条件唤醒“rclcpp::spin”。
> 2. 检查了“SubscriptionIntraProcessWaitable::is_ready()”条件。

This has to ensure that the ring buffer is not empty.

> 这必须确保环形缓冲区不为空。

3. The `SubscriptionIntraProcessWaitable::execute()` function is triggered.

> 3. 触发了“SubscriptionIntraProcessWaitable::execute()”函数。

Here the first message is extracted from the buffer and then the `SubscriptionIntraProcessWaitable` calls the `AnySubscriptionCallback::dispatch_intra_process(...)` method.

> 这里从缓冲区提取第一条消息，然后“SubscriptionIntraProcessWaitable”调用“AnySubscriptionCallback::dispatch_intra_process(…)”方法。

There are different implementations for this method, depending on the data-type stored in the buffer.

> 根据存储在缓冲区中的数据类型，此方法有不同的实现。

4. The `AnySubscriptionCallback::dispatch_intra_process(...)` method triggers the associated callback.

> 4. `AnySubscriptionCallback::dispatch_intra_process(…)`方法触发关联的回调。

Note that in this step, if the type of the buffer is a smart pointer one, no message copies occurr, as ownership has been already taken into account when pushing a message into the queue.

### Publishing intra and inter-process

1. User calls `Publisher::publish(std::unique_ptr<MessageT> msg)`.
2. The message is moved into a shared pointer `std::shared_ptr<MessageT> shared_msg = std::move(msg)`.
3. `Publisher::publish(std::unique_ptr<MessageT> msg)` calls `IntraProcessManager::do_intra_process_publish(uint64_t pub_id, std::shared_ptr<MessageT> shared_msg)`.

> 1. 用户调用`Publisher::publish(std::unique_ptr<MessageT>msg)`。
> 2. 消息被移动到共享指针`std::shared_ptr<MessageT> shared_msg=std::move(msg)`中。
> 3. `Publisher::publish(std::unique_ptr<MessageT>msg)`调用`IntraProcessManager::do_intra_process_publish(uint64_t pub_id，std::shared_ptr<MessageT>shared_msg)`。

The following steps are identical to steps 3, 4, and 5 applied when publishing only intra-process.

> 以下步骤与仅在进程内发布时应用的步骤 3、4 和 5 相同。

4. `IntraProcessManager::do_intra_process_publish(...)` uses the `uint64_t pub_id` to call `IntraProcessManager::get_subscription_ids_for_pub(uint64_t pub_id)`.

> 4.`IntraProcessManager::do_intra_process_publish(…)`使用`uint64_t pub_id`调用`IntraProcessManager::get_subscription_ids_for_pub(uint64_t pub_id)`。

Then it calls `IntraProcessManager::find_matching_subscriptions(PublisherInfo pub_info)`.

> 然后它调用`IntraProcessManager::find_matching_subscriptions(PublisherInfo pub_info)`。

This returns the ids corresponding to `Subscription`s that have a QoS compatible for receiving the message.

> 这返回与具有与接收消息兼容的 QoS 的“订阅”相对应的 id。

These ids are divided into two sublists, according to the data-type that is stored in the buffer of each `Susbscription`: requesting ownership (`unique_ptr<MessageT>`) or accepting shared (`shared_ptr<MessageT>`, but also `MessageT` since it will copy data in any case).

> 根据存储在每个“订阅”缓冲区中的数据类型，这些 ID 被分为两个子列表:请求所有权(`unique_ptr<MessageT>`)或接受共享(`shared_ptr<MessageT>`，但也包括`MessageT`，因为它在任何情况下都会复制数据)。

5. The message is "added" to the ring buffer of all the items in the list.

> 5. 消息被“添加”到列表中所有项目的环形缓冲区中。

The `rcl_guard_condition_t` member of `SubscriptionIntraProcessWaitable` of each `Subscription` is triggered (this wakes up `rclcpp::spin`).

> 触发每个`Subscription`的`SubscriptionIntraProcessWaitable`的`rcl_guard_condition_t`成员(这会唤醒`rclcpp::spin`)。

After the intra-process publication, the inter-process one takes place.

> **在进程内发布之后，将进行进程间发布。**

> [!NOTE]
> 这里是!!!

6. `Publisher::publish(std::unique_ptr<MessageT> msg)` calls `Publisher::do_inter_process_publish(const MessageT & inter_process_msg)`, where `MessageT inter_process_msg = *shared_msg`.

> 6. `Publisher::publish(std::unique_ptr＜MessageT>msg)`calls`Publisher:::do_inter_process_publish(const MessageT & inter_process_msg)`，其中`MessageT inter_proccess_msg = *shared_msg`。

The difference from the previous case is that here a `std::shared_ptr<const MessageT>` is being "added" to the buffers.

> 与前一种情况的不同之处在于，这里将`std::shared_ptr<const MessageT>`添加到缓冲区中。

Note that this `std::shared_ptr` has been just created from a `std::unique_ptr` and it is only used by the `IntraProcessManager` and by the RMW, while the user application has no access to it.

- `BufferT = unique_ptr<MessageT>` The buffer receives a copy of `MessageT` and has ownership on it.
- `BufferT = shared_ptr<const MessageT>` Every buffer receives a shared pointer of the same `MessageT`, so no copies are required.
- `BufferT = MessageT` A copy of the message is added to every buffer.

> -`BufferT=unique_ptr<MessageT>`缓冲区接收`MessageT`的副本并拥有该副本的所有权。 -`BufferT=shared_ptr<const MessageT>`每个缓冲区都接收同一`MessageT`的共享指针，因此不需要副本。 -`BufferT=MessageT`消息的副本被添加到每个缓冲区。

The difference with publishing a unique_ptr is that here it is not possible to save a copy.

> 与发布 unique_ptr 的不同之处在于，在此处无法保存副本。

If you move the ownership of the published message to one of the `Subscription` (so potentially saving a copy as done in the previous case), you will need to create a new copy of the message for inter-process publication.

> 如果将已发布邮件的所有权移动到“订阅”中的一个(因此可能会像前面的情况一样保存一份副本)，则需要创建一份新的邮件副本以进行流程间发布。

![Sequence UML diagram](../img/intraprocess_communication/intra_inter_process.png)

### QoS features

The proposed implementation can handle all the different QoS.

> 所提出的实现可以处理所有不同的 QoS。

- If the history is set to `keep_last`, then the depth of the history corresponds to the size of the ring buffer.

> - 如果历史设置为“keep_last”，则**历史的深度对应于环形缓冲区的大小**。

On the other hand, if the history is set to `keep_all`, the buffer becomes a standard FIFO queue with an unbounded size.

> 另一方面，如果历史设置为“keep_all”，则缓冲区将成为一个具有无限大小的标准 FIFO 队列。

- The reliability is only checked by the `IntraProcessManager` in order to understand if a `Publisher` and a `Subscription` are compatible.

> - 可靠性仅由“IntraProcessManager”检查，以了解“发布者”和“订阅”是否兼容。

The use of buffers ensures that all the messages are delivered without the need to resend them.

> 缓冲区的使用确保了所有消息的传递，而无需重新发送。

Thus, both options, `reliable` and `best-effort`, are satisfied.

> 因此，“可靠”和“尽最大努力”这两种选择都得到了满足。

- The durability QoS is used to understand if a `Publisher` and a `Subscription` are compatible.

> - 耐久性 QoS 用于了解“发布者”和“订阅”是否兼容。

How this QoS is handled is described in details in the following paragraph.

> 如何处理该 QoS 将在下面的段落中详细描述。

#### Handling Transient Local

If the `Publisher` durability is set to `transient_local` an additional buffer on the `Publisher` side is used to store the sent intra-process messages.

> 如果“Publisher”耐久性设置为“transient_local”，**则“Publisher”侧的额外缓冲区用于存储发送的进程内消息**。

Late-joiner `Subscription`s will have to extract messages from this buffer once they are added to the `IntraProcessManager`.

> 一旦消息被添加到“IntraProcessManager”中，后期加入者“Subscription”将不得不从此缓冲区提取消息。

In this case the `IntraProcessManager` has to check if the recently created `Subscription` is a late-joiner, and, if it is, it has to retrieve messages from the `Transient Local` `Publisher`s.

> 在这种情况下，“IntraProcessManager”必须检查最近创建的“Subscription”是否是延迟加入者，如果是，则必须从“Transient Local”“Publisher”检索消息。

1. Call `IntraProcessManager::find_matching_publishers(SubscriptionInfo sub_info)` that returns a list of stored `PublisherInfo` that have a QoS compatible for sending messages to this new `Subscription`.

> 1. 调用“IntraProcessManager::find_matching_publishers(SubscriptionInfo sub_info)”，返回存储的“PublisherInfo”的列表，该列表具有与向此新“Subscription”发送消息兼容的 QoS。

These will be all `Transient Local` `Publisher`s, so they have a ring buffer.

> 这些都是“瞬态本地”发布服务器，因此它们有一个环形缓冲区。

2. Copy messages from all the ring buffers found into the ring buffer of the new `Subscription`.

> 2. 将消息从找到的所有环形缓冲区复制到新的“订阅”的环形缓冲区中。

**TODO:** are there any constraints on the order in which old messages have to be retrieved? (i.e. 1 publisher at the time; all the firsts of each publisher, then all the seconds ...).

> **TODO:**对于必须检索旧消息的顺序是否有任何限制？(即一次有一个发布者；每个发布者的所有第一，然后是所有秒…)。

3. If at least 1 message was present, trigger the `rcl_guard_condition_t` member of the `SubscriptionIntraProcessWaitable` associated with the new `Subscription`.

> 3.如果至少存在 1 条消息，则触发与新的“Subscription”关联的“SubscriptionIntraProcessWaitable”的“rcl_guard_condition_t”成员。

However, this is not enough as it does not allow to handle the scenario in which a `transient local` `Publisher` has only intra-process `Subscription`s when it is created, but, eventually, a `transient local` `Subscription` in a different process joins.

> 然而，这还不够，因为它不允许处理这样一种情况，即“临时本地”“发布者”在创建时只有进程内的“订阅”，但最终在不同的进程中加入了“暂时本地”“订阅”。

Initially, published messages are not passed to the middleware, since all the `Subscription`s are in the same process.

> 最初，发布的消息不会传递给中间件，因为所有的“订阅”都在同一个过程中。

This means that the middleware is not able to store old messages for eventual late-joiners.

> 这意味着中间件不能为最终的后期加入者存储旧消息。

The solution to this issue consists in always publishing both intra and inter-process when a `Publisher` has `transient local` durability.

> 这个问题的解决方案在于，当“发布者”具有“瞬时本地”耐久性时，始终同时发布进程内和进程间。

For this reason, when `transient local` is enabled, the `do_intra_process_publish(...)` function will always process a shared pointer.

> 因此，当启用“瞬态本地”时，“do_intra_process_publish(…)”函数将始终处理共享指针。

This allows us to add the logic for storing the published messages into the buffers only in one of the two `do_intra_process_publish(...)` cases and also it allows to use buffers that have only to store shared pointers.

> 这允许我们仅在两种“do_intra_process_publish(…)”情况中的一种情况下添加用于将已发布消息存储到缓冲区中的逻辑，并且还允许使用仅用于存储共享指针的缓冲区。

### Number of message copies

In the previous sections, it has been briefly described how a message can be added to a buffer, i.e. if it is necessary to copy it or not.

> 在前面的部分中，已经简要描述了**如何将消息添加到缓冲区，即是否需要复制消息**。

Here some details about how this proposal adresses some more complex cases.

> 以下是关于这项提案如何处理一些更复杂案件的一些细节。

As previously stated, regardless of the data-type published by the user, the flow always goes towards `Publisher::publish(std::unique_ptr<MessageT> msg)`.

> 如前所述，无论用户发布的数据类型如何，流始终指向“Publisher::publish(std::unique_ptr<MessageT>msg)”。

The `std::unique_ptr<MessageT> msg` is passed to the `IntraProcessManger` that decides how to add this message to the buffers.

> “std::unique_ptr<MessageT>消息”传递给“IntraProcessManger”，由其决定如何将此消息添加到缓冲区。

The decision is taken looking at the number and the type, i.e. if they want ownership on messages or not, of the `Subscription`s.

> **决定是根据“订阅”的数量和类型做出的，即他们是否希望拥有消息的所有权。**

If all the `Subscription`s want ownership of the message, then a total of `N-1` copies of the message are required, where `N` is the number of `Subscription`s.

> 如果所有“Subscription”都希望拥有该消息的所有权，则需要该消息的总共“N-1”个副本，其中“N”是“Subscription’s”的数目。

The last one will receive ownership of the published message, thus saving a copy.

> 最后一个将获得已发布消息的所有权，从而保存一份副本。

If none of the `Subscription`s want ownership of the message, `0` copies are required.

> 如果“订阅”都不希望拥有该消息的所有权，则需要“0”份副本。

It is possible to convert the message into a `std::shared_ptr<MessageT> msg` and to add it to every buffer.

> 可以将消息转换为“std::shared_ptr<MessageT> msg”，并将其添加到每个缓冲区。

If there is 1 `Subscription` that does not want ownership while the others want it, the situation is equivalent to the case of everyone requesting ownership:`N-1` copies of the message are required.

> 如果有一个“订阅”不想要所有权，而其他人想要所有权，这种情况相当于每个人都请求所有权的情况:需要消息的“N-1”份副本。

As before the last `Subscription` will receive ownership.

> 与以前一样，最后一个“订阅”将获得所有权。

If there is more than 1 `Subscription` that do not want ownership while the others want it, a total of `M` copies of the message are required, where `M` is the number of `Subscription`s that want ownership.

> 如果有 1 个以上的“Subscription”不想要所有权，而其他人想要所有权，则总共需要消息的“M”个副本，其中“M”是想要所有权的“Subscription'的数量。

`1` copy will be shared among all the `Subscription`s that do not want ownership, while `M-1` copies are for the others.

As in the current implementation, if both inter and intra-process communication are needed, the `std::unique_ptr<MessageT> msg` will be converted into a `std::shared_ptr<MessageT> msg` and passed respectively to the `do_intra_process_publish` and `do_inter_process_publish` functions.

> 与当前实现中一样，如果**同时需要进程间和进程内通信**，则“std::unique_ptr<MessageT>msg”将被转换为“std::shared_ptr<MessageT>msg”，并分别传递给“do_intra_process_publish”和“do_inter_process_publish”函数。

> [!NOTE] > **同时需要进程间和进程内通信**

A copy of the message will be given to all the `Subscription`s requesting ownership, while the others can copy the published shared pointer.

> 消息的副本将提供给所有请求所有权的“订阅”，而其他人可以复制已发布的共享指针。

The following tables show a recap of when the proposed implementation has to create a new copy of a message.

> 下表简要介绍了何时建议的实现必须创建消息的新副本。

The notation `@` indicates a memory address where the message is stored, different memory addresses correspond to different copies of the message.

> 符号“@”表示存储消息的存储器地址，不同的存储器地址对应于消息的不同副本。

#### Publishing UniquePtr

```
| publish\<T\> | BufferT | Results |
| ----------------------- | ----------------------- | ----------------------- |
| unique_ptr\<MsgT\> @1 | unique_ptr\<MsgT\> | @1 |
| unique_ptr\<MsgT\> @1 | unique_ptr\<MsgT\> <br> unique_ptr\<MsgT\> | @1 <br> @2 |
| unique_ptr\<MsgT\> @1 | shared_ptr\<MsgT\> | @1 |
| unique_ptr\<MsgT\> @1 | shared_ptr\<MsgT\> <br> shared_ptr\<MsgT\> | @1 <br> @1 |
| unique_ptr\<MsgT\> @1 | unique_ptr\<MsgT\> <br> shared_ptr\<MsgT\> | @1 <br> @2 |
| unique_ptr\<MsgT\> @1 | unique_ptr\<MsgT\> <br> shared_ptr\<MsgT\> <br> shared_ptr\<MsgT\> | @1 <br> @2 <br> @2|
| unique_ptr\<MsgT\> @1 | unique_ptr\<MsgT\> <br> unique_ptr\<MsgT\> <br> shared_ptr\<MsgT\> <br> shared_ptr\<MsgT\> | @1 <br> @2 <br> @3 <br> @3|
```

#### Publishing SharedPtr

```
| publish\<T\> | BufferT | Results |
| ----------------------- | ----------------------- | ----------------------- |
| shared_ptr\<MsgT\> @1 | unique_ptr\<MsgT\> | @2 |
| shared_ptr\<MsgT\> @1 | unique_ptr\<MsgT\> <br> unique_ptr\<MsgT\> | @2 <br> @3 |
| shared_ptr\<MsgT\> @1 | shared_ptr\<MsgT\> | @1 |
| shared_ptr\<MsgT\> @1 | shared_ptr\<MsgT\> <br> shared_ptr\<MsgT\> | @1 <br> @1 |
| shared_ptr\<MsgT\> @1 | unique_ptr\<MsgT\> <br> shared_ptr\<MsgT\> | @2 <br> @1 |
| shared_ptr\<MsgT\> @1 | unique_ptr\<MsgT\> <br> shared_ptr\<MsgT\> <br> shared_ptr\<MsgT\> | @2 <br> @1 <br> @1|
| shared_ptr\<MsgT\> @1 | unique_ptr\<MsgT\> <br> unique_ptr\<MsgT\> <br> shared_ptr\<MsgT\> <br> shared_ptr\<MsgT\> | @2 <br> @3 <br> @1 <br> @1|
```

The possibility of setting the data-type stored in each buffer becomes helpful when dealing with more particular scenarios.

> 在处理更具体的场景时，设置存储在每个缓冲区中的数据类型的可能性变得很有帮助。

Considering a scenario with N `Subscription`s all taking a unique pointer.

> 考虑一个场景，其中 N 个“Subscription”都使用一个唯一的指针。

If the `Subscription`s don't actually take the message (e.g. they are busy and the message is being overwritten due to QoS settings) the default buffer type (`unique_ptr` since the callbacks require ownership) would result in the copy taking place anyway.

> 如果“订阅”实际上没有接收消息(例如，它们很忙，并且由于 QoS 设置，消息被覆盖)，则默认缓冲区类型(“unique_ptr”，因为回调需要所有权)无论如何都会导致复制发生。

By setting the buffer type to `shared_ptr`, no copies are needed when the `Publisher` pushes messages into the buffers.

> 通过将缓冲区类型设置为“shared_ptr”，当“Publisher”将消息推入缓冲区时，不需要任何副本。

Eventually, the `Subscription`s will copy the data only when they are ready to process it.

> 最终，只有当“Subscription”准备好处理数据时，它们才会复制数据。

On the other hand, if the published data are very small, it can be advantageous to do not use C++ smart pointers, but to directly store the data into the buffers.

> 另一方面，如果发布的数据非常小，则不使用 C++智能指针，而是直接将数据存储到缓冲区中是有利的。

In all this situations, the number of copies is always smaller or equal than the one required for the current intra-process implementation.

> 在所有这些情况下，拷贝数总是小于或等于当前进程内实现所需的拷贝数。

However, there is a particular scenario where having multiple buffers makes much more difficult saving a copy.

> 然而，有一种特殊的情况，即具有多个缓冲区会使保存副本变得更加困难。

There are two `Subscription`s, one taking a shared pointer and the other taking a unique pointer.

> 有两个“Subscription”，一个采用共享指针，另一个采用唯一指针。

With a more centralized system, if the first `Subscription` requests its shared pointer and then releases it before the second `Subscription` takes the message, it is potentially possible to optimize the system to manage this situation without requiring any copy.

> 对于更集中的系统，如果第一个“Subscription”请求其共享指针，然后在第二个“Subscriptation”接收消息之前释放它，则有可能优化系统以管理这种情况，而不需要任何副本。

On the other hand, the proposed implementation will immediately create one copy of the message for the `Subscription` requiring ownership.

> 另一方面，拟议的实施将立即为需要所有权的“订阅”创建一份电文副本。

Even in case of using a `shared_ptr` buffer as previously described, it becomes more difficult to ensure that the other `Subscription` is not using the pointer anymore.

> 即使在如前所述使用“shared_ptr”缓冲区的情况下，也更难确保另一个“Subscription”不再使用指针。

#### Where are these copies performed?

The `IntraProcessManger::do_intra_process_publish(...)` function knows whether the intra-process buffer of each `Subscription` requires ownership or not.

> `IntraProcessManger::do_intra_process_publish(…)`函数知道每个`Subscription`的进程内缓冲区是否需要所有权。

For this reason it can perform the minimum number of copies required by looking at the total number of `Subscription`s and their types.

> 因此，它可以通过查看“订阅”的总数及其类型来执行所需的最小副本数。

The buffer does not perform any copy when receiving a message, but directly stores it.

> 缓冲区在接收消息时不执行任何复制，而是直接存储消息。

When extracting a message from the buffer, the `Subscription` can require any particular data-type.

> 从缓冲区提取消息时，“订阅”可能需要任何特定的数据类型。

The intra-process buffer will perform a copy of the message whenever necessary, for example in the previously described cases where the data-type stored in the buffer is different from the callback one.

> 进程内缓冲区将在必要时执行消息的复制，例如在前面描述的缓冲区中存储的数据类型与回调数据类型不同的情况下。

## Perfomance evaluation

The implementation of the presented new intra-process communication mechanism is hosted on [GitHub here](https://github.com/alsora/rclcpp/tree/alsora/new_ipc_proposal).

> 所提出的新流程内通信机制的实现托管在[GitHub here]上(https://github.com/alsora/rclcpp/tree/alsora/new_ipc_proposal)。

This section contains experimental results obtained comparing the current intra-process communication implementation with an initial implementation of the proposed one.

> 本节包含将当前进程内通信实现与所提出的初始实现进行比较所获得的实验结果。

The tests span multiple ROS 2 applications and use-cases and have been validated on different machines.

> 这些测试涵盖了多个 ROS 2 应用程序和用例，并在不同的机器上进行了验证。

All the following experiments have been run using the ROS 2 Dashing and with `-O2` optimization enabled.

> 以下所有实验都是使用 ROS 2 Dashing 和启用“-O2”优化进行的。

```

colcon build --cmake-args  -DCMAKE_CXX_FLAGS="-O2" -DCMAKE_C_FLAGS="-O2"

> colcon内部版本--cmake参数-DCMAKE_CXX_FLAGS=“-O2”-DCMAKE_C_FLAGS=“/O2”
```

The first test has been carried out using the `intra_process_demo` package contained in the [ROS 2 demos repository](https://github.com/ros2/demos).

> 第一次测试是使用[ROS 2 演示库]中包含的“intra_process_demo”包进行的(https://github.com/ros2/demos)。

A first application, called `image_pipeline_all_in_one`, is made of 3 nodes, where the fist one publishes a `unique_ptr<Image>` message.

> 第一个应用程序名为“image_pipeline_all_in_one”，由 3 个节点组成，其中第一个节点发布“unique_ptr<image>”消息。

A second node subscribes to the topic and republishes the image after modifying it on a new topic.

> 第二个节点订阅该主题，并在将图像修改为新主题后重新发布该图像。

A third node subscribes to to this last topic.

> 第三个节点订阅了最后一个主题。

Also a variant of the application has been tested: it's `image_pipeline_with_two_image_view`, where there are 2 consumers at the end of the pipeline.

> 此外，还测试了该应用程序的一个变体:它是“image_pipeline_with_two_image_view”，其中在管道的末尾有两个使用者。

In these tests the latency is computed as the total pipeline duration, i.e. the time from when the first node publishes the image to when the last node receives it.

> 在这些测试中，延迟被计算为总管道持续时间，即从第一个节点发布图像到最后一个节点接收图像的时间。

The CPU usage and the latency have been obtained from `top` command and averaged over the experiment duration.

> CPU 使用率和延迟是从“top”命令中获得的，并在实验持续时间内取平均值。

Performance evaluation on a laptop computer with Intel i7-6600U CPU @ 2.60GHz.

> 在搭载英特尔 i7-6600U CPU 的 2.60GHz 笔记本电脑上进行性能评估。

| ROS 2 system | IPC | RMW | Latency [us] | CPU [%] | RAM [Mb] |

> |ROS 2 系统|IPC | RMW |延迟[us]| CPU[%]| RAM[Mb]|

| ------------- | ----- | ------------- | ------------ | ------- | -------- |

> |---------------|---------------|------------|--------|--------|

| image_pipeline_all_in_one | off | Fast-RTPS | 1800 | 23 | 90 |

> |image_pipeline_all_in_one|off|快速 RTPS|1800|23|90|

| image_pipeline_all_in_one | standard | Fast-RTPS | 920 | 20 | 90 |

> |image_pipeline_all_in_one|标准|快速 RTPS|920|20|90|

| image_pipeline_all_in_one | new | Fast-RTPS | 350 | 15 | 90 |

> |image_pipeline_all_in_one|新|快速 RTPS|350|15|90|

| image_pipeline_with_two_image_view | off | Fast-RTPS | 2900 | 24 | 94 |

> |image_pipeline_with_two_image_view|off|快速 RTPS|2900|24|94|

| image_pipeline_with_two_image_view | standard | Fast-RTPS | 2000 | 20 | 95 |

> |image_pipeline_with_two_image_view |标准|快速 RTPS | 2000 | 20 | 95|

| image_pipeline_with_two_image_view | new | Fast-RTPS | 1400 | 16 | 94 |

> |image_pipeline_with_two_image_view |新建|快速 RTPS | 1400 | 16 | 94|

From this simple experiment is immediately possible to see the improvement in the latency when using the proposed intra-process communication.

> 从这个简单的实验中，可以立即看到使用所提出的进程内通信时延迟的改善。

However, an even bigger improvement is present when analyzing the results from more complex applications.

> 然而，在分析更复杂应用程序的结果时，会有更大的改进。

The next results have been obtained running the iRobot benchmark application.

> 接下来的结果已经在运行 iRobot 基准测试应用程序时获得。

This allows the user to specify the topology of a ROS 2 graph that will be entirely run in a single process.

> 这允许用户指定 ROS 2 图的拓扑结构，该图将完全在单个进程中运行。

The application has been run with the topologies Sierra Nevada and Mont Blanc.

> 该应用程序已在内华达山脉和勃朗峰的拓扑结构中运行。

Sierra Nevada is a 10-node topology and it contains 10 publishers and 13 subscriptions.

> 内华达山脉是一个 10 节点的拓扑结构，包含 10 个发布者和 13 个订阅。

One topic has a message size of 10KB, while all the others have message sizes between 10 and 100 bytes.

> 一个主题的消息大小为 10KB，而所有其他主题的消息尺寸都在 10 到 100 字节之间。

Mont Blanc is a bigger 20-node topology, containing 23 publishers and 35 subscriptions.

> Mont Blanc 是一个更大的 20 节点拓扑，包含 23 个发布者和 35 个订阅。

Two topics have a message size of 250KB, three topics have message sizes between 1KB and 25KB, and the rest of the topics have message sizes smaller than 1KB.

> 两个主题的消息大小为 250KB，三个主题的邮件大小在 1KB 到 25KB 之间，其余主题的邮件尺寸小于 1KB。

A detailed description and the source code for these application and topologies can be found [here](https://github.com/irobot-ros/ros2-performance/tree/master/performances/benchmark).

> 可以在这里找到这些应用程序和拓扑的详细描述和源代码(https://github.com/irobot-ros/ros2-performance/tree/master/performances/benchmark)。

Note that, differently from the previous experiment where the ownership of the messages was moved from the publisher to the subscription, here nodes use `const std::shared_ptr<const MessageT>` messages for the callbacks.

Performance evaluation on a laptop computer with Intel i7-6600U CPU @ 2.60GHz.

> 在搭载英特尔 i7-6600U CPU 的 2.60GHz 笔记本电脑上进行性能评估。

| ROS 2 system | IPC | RMW | Latency [us] | CPU [%] | RAM [Mb] |

> |ROS 2 系统|IPC | RMW |延迟[us]| CPU[%]| RAM[Mb]|

| ------------- | ----- | ------------- | ------------ | ------- | -------- |

> |---------------|---------------|------------|--------|--------|

| Sierra Nevada | off | Fast-RTPS | 600 | 14 | 63 |

> |内华达山脉|off|Fast RTPS|600|14|63|

| Sierra Nevada | standard | Fast-RTPS | 650 | 16 | 73->79 |

> |Sierra Nevada |标准|快速 RTPS | 650 | 16 | 73->79|

| Sierra Nevada | new | Fast-RTPS | 140 | 8 | 63 |

> |内华达山脉|新|快速 RTPS|140|8|63|

| Mont Blanc | off | Fast-RTPS | 1050 | 22 | 180 |

> |勃朗峰|off|Fast RTPS|1050 | 22 | 180|

| Mont Blanc | standard | Fast-RTPS | 750 | 18 | 213->220 |

> |勃朗峰|标准|Fast RTPS|750|18|213->220|

| Mont Blanc | new | Fast-RTPS | 160 | 8 | 180 |

> |勃朗峰|新|快速 RTPS|160|8|180|

A similar behavior can be observed also running the application on resource constrained platforms.

> 在资源受限的平台上运行应用程序也可以观察到类似的行为。

The following results have been obtained on a RaspberryPi 2.

> 在 RaspberryPi 2 上获得了以下结果。

| ROS 2 system | IPC | RMW | Latency [us] | CPU [%] | RAM [Mb] |

> |ROS 2 系统|IPC | RMW |延迟[us]| CPU[%]| RAM[Mb]|

| ------------- | ----- | ------------- | ------------ | ------- | -------- |

> |---------------|---------------|------------|--------|--------|

| Sierra Nevada | off | Fast-RTPS | 800 | 18 | 47 |

> |内华达山脉|off|Fast RTPS|800|18|47|

| Sierra Nevada | standard | Fast-RTPS | 725 | 20 | 54->58 |

> |内华达山脉|标准|快速 RTPS |725|20|54->58|

| Sierra Nevada | new | Fast-RTPS | 170 | 10 | 47 |

> |内华达山脉|新增|快速 RTPS|170|10|47|

| Mont Blanc | off | Fast-RTPS | 1500 | 30 | 130 |

> |勃朗峰|off|Fast RTPS|1500 | 30 | 130|

| Mont Blanc | standard | Fast-RTPS | 950 | 26 | 154->159 |

> |勃朗峰|标准|快速 RTPS | 950 | 26 | 154->159|

| Mont Blanc | new | Fast-RTPS | 220 | 14 | 130 |

> |勃朗峰|新|快速 RTPS | 220 | 14 | 130|

For what concerns latency and CPU usage, Sierra Nevada behaves almost the same regardless if standard IPC is enabled or not.

> 在延迟和 CPU 使用方面，无论是否启用标准 IPC，Sierra Nevada 的行为几乎相同。

This is due to the fact that most of its messages are very small in size.

> 这是因为它的大多数消息都很小。

On the other hand, there are noticeable improvements in Mont Blanc, where several messages of non-negligible size are used.

> 另一方面，在勃朗峰有明显的改进，在那里使用了几个大小不可忽略的消息。

From the memory point of view, there is an almost constant increase in the utilization during the execution of the program when standard intra-process communication mechanism is used.

> 从存储器的角度来看，当使用标准进程内通信机制时，在程序执行期间，利用率几乎恒定地增加。

Since the experiments have been run for 120 seconds, there is an increase of approximately 60KB per second.

> 由于实验已经运行了 120 秒，因此每秒大约增加 60KB。

However, even considering the initial memory usage, it is possible to see how it is affected from the presence of the additional publishers and subscriptions needed for intra-process communication.

> 然而，即使考虑到初始内存使用情况，也可以看到进程内通信所需的额外发布者和订阅对内存使用的影响。

There is a difference of 10MB in Sierra Nevada and of 33MB in Mont Blanc between standard intra-process communication on and off.

> 标准流程内通信的开启和关闭之间，内华达山脉的差异为 10MB，勃朗峰的差异为 33MB。

The last experiment show how the current implementation performs in the case that both intra and inter-process communication are needed.

> 最后一个实验显示了当前实现在同时需要进程内和进程间通信的情况下的表现。

The test consists of running Sierra Nevada on RaspberryPi 2, and, in a separate desktop machine, a single node subscribing to all the available topics coming from Sierra Nevada.

> 该测试包括在 RaspberryPi 2 上运行 Sierra Nevada，以及在一台单独的台式机中，一个节点订阅来自 Sierra 内华达的所有可用主题。

This use-case is common when using tools such as `rosbag` or `rviz`.

> 这种用例在使用诸如“rosbag”或“rviz”之类的工具时很常见。

| ROS 2 system | IPC | RMW | Latency [us] | CPU [%] | RAM [Mb] |

> |ROS 2 系统|IPC | RMW |延迟[us]| CPU[%]| RAM[Mb]|

| ------------- | ----- | ------------- | ------------ | ------- | -------- |

> |---------------|---------------|------------|--------|--------|

| Sierra Nevada + debug node | off | Fast-RTPS | 800 | 22 | 50 |

> |Sierra Nevada+调试节点|关闭|快速 RTPS | 800 | 22 | 50|

| Sierra Nevada + debug node | standard | Fast-RTPS | 1100 | 35 | 60->65 |

> |Sierra Nevada+调试节点|标准|快速 RTPS | 1100 | 35 | 60->65|

| Sierra Nevada + debug node | new | Fast-RTPS | 180 | 15 | 32 |

> |Sierra Nevada+调试节点|新|快速 RTPS | 180 | 15 | 32|

These results show that if there is at least one node in a different process, with the current implementation it is better to keep intra-process communication disabled.

> 这些结果表明，如果在不同的进程中至少有一个节点，那么在当前的实现中，最好保持进程内通信禁用。

The proposed implementation does not require the ROS 2 middleware when publishing intra-process.

> 所提出的实现在发布进程内时不需要 ROS 2 中间件。

This allows to easily remove the connections between nodes in the same process when it is required to publish also inter process, potentially resulting in a very small overhead with respect to the only intra-process case.

> 这允许在需要发布进程间时轻松删除同一进程中节点之间的连接，这可能会导致相对于唯一的进程内情况而言非常小的开销。

## Open Issues

There are some open issues that are not addressed neither on the current implementation nor on the proposed one.

> 目前的执行情况和拟议的执行情况都没有解决一些悬而未决的问题。

- The proposal does not take into account the problem of having a queue with twice the size when both inter and intra-process communication are used.

> -该方案没有考虑在同时使用进程间和进程内通信时具有两倍大小的队列的问题。

A `Publisher` or a `Subscription` with a history depth of 10 will be able to store up to 20 messages without processing them (10 intra-process and 10 inter-process).

> 历史深度为 10 的“发布者”或“订阅”将能够存储多达 20 条消息，而无需处理它们(10 条进程内消息和 10 条进程间消息)。

This issue is also present in the current implementation, since each `Subscription` is doubled.

> 这一问题在目前的执行中也存在，因为每次“订阅”都要加倍。
