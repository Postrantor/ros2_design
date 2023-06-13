---
tip: translate by openai@2023-05-30 13:24:06
layout: default
title: Intra-process Communications in ROS 2
permalink: articles/intraprocess_communications.html
abstract: Description of the current intra-process communication mechanism in ROS 2 and of its drawbacks. Design proposal for an improved implementation. Experimental results.
published: true
author: '[Alberto Soragna](https://github.com/alsora) [Juan Oxoby](https://github.com/joxoby) [Dhiraj Goel](https://github.com/dgoel)'
date_written: 2020-03
last_modified: 2020-03

Authors: 
Date Written: 
Last Modified:
---
## Introduction

The subscriptions and publications mechanisms in ROS 2 fall in two categories:

> ROS 2 中的订阅和发布机制分为两类：

- **intra-process**: messages are sent from a publisher to subscriptions via in-process memory.
- **inter-process**: messages are sent via the underlying ROS 2 middleware layer.

  The specifics of how this happens depend on the chosen middleware implementation and may involve serialization steps.

> 这是如何发生的具体细节取决于所选择的中间件实现，可能涉及序列化步骤。

This design document presents a new implementation for the intra-process communication.

> 这份设计文档提出了一种新的**用于进程间通信的实现方式**。

## Motivations for a new implementation

Even if ROS 2 supports intra-process communication, the implementation of this mechanism has still much space for improvement. Until ROS 2 Crystal, major performance issues and the lack of support for shared pointer messages were preventing the use of this feature in real applications.

> 即使 ROS 2 支持进程内通信，该机制的实现仍有很大的改进空间。直到 ROS 2 Crystal，主要性能问题和缺乏对共享指针消息的支持，都阻止了在实际应用中使用这一功能。

With the ROS 2 Dashing release, most of these issues have been addressed and the intra-process communication behavior has improved greatly ([see ticket](https://github.com/ros2/ros2/issues/649)).

> 随着 ROS 2 Dashing 发布，大部分问题都已得到解决，进程内通信行为大大改善([参见门票](https://github.com/ros2/ros2/issues/649))。

The current implementation is based on the creation of a ring buffer for each `Publisher` and on the publication of meta-messages through the middleware layer. When a `Publisher` has to publish intra-process, it will pass the message to the `IntraProcessManager`. Here the message will be stored in the ring buffer associated with the `Publisher`. In order to extract a message from the `IntraProcessManager` two pieces of information are needed: the id of the `Publisher` (in order to select the correct ring buffer) and the position of the message within its ring buffer. A meta-message with this information is created and sent through the ROS 2 middleware to all the `Subscription` s, which can then retrieve the original message from the `IntraProcessManager`.

> 当前实现是基于**为每个发布者创建一个环形缓冲区**，并通过中间件层发布元消息。当发布者必须发布进程内消息时，它将消息传递给 IntraProcessManager。在这里，**消息将存储在与发布者关联的环形缓冲区中**。为了从 IntraProcessManager 中提取消息，需要两个信息：**发布者的 ID**(以选择正确的环形缓冲区)和**消息在其环形缓冲区中的位置**。将使用此信息创建元消息，并**通过 ROS 2 中间件发送到所有订阅，然后可以从 IntraProcessManager 检索原始消息。**

![Current IPC Block Diagram](../img/intraprocess_communication/old_ipc.png)

Several shortcomings of the current implementation are listed below.

> 以下列出了当前实施的几个缺点。

### Incomplete Quality of Service support

The current implementation can't be used when the QoS durability value is set to `Transient Local`.

> 当 QoS 耐久性值设置为“Transient Local”时，当前实施方案无法使用。

The current implementation does not enforce the depth of the QoS history in a correct way. The reason is that there is a single ring buffer per `Publisher` and its size is equal to the depth of the `Publisher`'s history. A `Publisher` stores a message in the ring buffer and then it sends a meta-message to allow a `Subscription` to retrieve it. The `Subscription` correctly stores meta-messages up to the number indicated by its depth of the history, but, depending on the frequency at which messages are published and callbacks are triggered, it may happen that a meta-message processed from the `Subscription` does not correspond anymore to a valid message in the ring buffer, because it has been already overwritten. This results in the loss of the message and it is also a difference in behavior between intra and inter-process communication, since, with the latter, the message would have been received.

> 当前的实现不能正确地强制 QoS 历史的深度。原因是每个“Publisher”只有一个环形缓冲区，其大小等于“Publisher”的历史深度。**“Publisher”将消息存储在环形缓冲区中，然后发送元消息以允许“Subscription”检索它。** 根据其深度的历史，“Subscription”正确地存储元消息，但是，根据发布消息和触发回调的频率，可能会发生从“Subscription”处理的元消息不再对应于环形缓冲区中的有效消息，因为它已经被覆盖。这导致消息丢失，也是进程间通信与进程内通信之间行为的差异，因为在后者中，消息将被接收。

Moreover, even if the use of meta-messages allows to deleagate the enforcement of other QoS settings to the RMW layer, every time a message is added to the ring buffer the `IntraProcessManager` has to compute how many `Subscription` s will need it. This potentially breaks the advantage of having the meta-messages. For example, the `IntraProcessManager` has to take into account that potentially all the known `Subscription` s will take the message, regardless of their reliability QoS. If a `Publisher` or a `Subscription` are best-effort, they may not receive the meta-message thus preventing the `IntraProcessManager` from releasing the memory in the buffer.

> 此外，即使使用元消息允许将其他 QoS 设置的执行委托给 RMW 层，每次将消息添加到环形缓冲区时，IntraProcessManager 都必须计算需要它的多少 Subscription。这可能破坏了拥有元消息的优势。例如，IntraProcessManager 必须考虑到可能所有已知的 Subscription 都会接收该消息，而不考虑它们的可靠性 QoS。如果发布者或订阅者是尽力而为，则可能不会收到元消息，从而阻止 IntraProcessManager 释放缓冲区中的内存。

More details [here](https://index.ros.org/doc/ros2/Concepts/About-Quality-of-Service-Settings/).

**TODO:** take into account also new QoS: Deadline, Liveliness and Lifespan [reference](https://github.com/ros2/design/pull/212).

> **待办：**也要考虑新的 QoS：Deadline，Liveliness 和 Lifespan [参考](https://github.com/ros2/design/pull/212)。

> [!NOTE]
> 实现进程间通信需要考虑 QoS 的配置，涉及 history/deadline/liveliness/lifespan。
> 和讨论“一个 pub 被多个 sub 订阅”消息是否被复制这个话题的讨论是有关系的
> 与 DDS、QoS、meta_data 以及 DDS 与 ROS2 之间的映射关系是相关的，这部分内容通过 git 在 chatgpt 中存储。

### Dependent on the RMW

The current implementation of intra-process communication has to send meta-messages from the `Publisher` to the `Subscription` s. This is done using the `rmw_publish` function, the implementation of which depends on the chosen middleware. This results in the performance of a ROS 2 application with intra-process communication enabled being heavily dependent on the chosen RMW implementation.

> 当前的**进程间通信实现需要从 `发布者` 发送元消息到 `订阅者`**。这是通过 `rmw_publish` 函数完成的，其实现取决于所选择的中间件。这导致启用进程间通信的 ROS 2 应用程序的性能严重依赖于所选择的 RMW 实现。

Given the fact that these meta-messages have only to be received from entities within the same process, there is space for optimizing how they are transmitted by each RMW. However, at the moment none of the supported RMW is actively tackling this issue. This results in that the performance of a single process ROS 2 application with intra-process communication enabled are still worst than what you could expect from a non-ROS application sharing memory between its components.

> 鉴于这些元消息只能从同一进程中的实体接收，可以优化每个 RMW 如何传输它们。然而，目前没有任何支持的 RMW 正在积极解决这个问题。这导致具有内部进程通信功能的单个进程 ROS 2 应用程序的性能仍然低于您可以从共享内存的非 ROS 应用程序中期望的性能。

In the following some experimental evidences are quickly presented.

> 以下快速介绍了一些实验证据。

#### Memory requirement

When a `Node` creates a `Publisher` or a `Subscription` to a topic `/MyTopic`, it will also create an additional one to the topic `/MyTopic/_intra`. The second topic is the one where meta-messages travel. Our [experimental results](https://github.com/irobot-ros/ros2-performance/tree/master/performances/experiments/crystal/pub_sub_memory#adding-more-nodes-x86_64) show that creating a `Publisher` or a `Subscription` has a non-negligible memory cost. This is particularly true for the default RMW implementation, Fast-RTPS, where the memory requirement increases almost expontentially with the number of participants and entities.

> 当节点创建发布者或订阅者到主题 `/MyTopic` 时，它还会**创建一个额外的主题 `/MyTopic/_intra`。第二个主题是元消息传播的主题**。我们的实验结果表明，创建发布者或订阅者具有不可忽略的内存代价。这对于默认的 RMW 实现 Fast-RTPS 来说尤其如此，其内存要求几乎指数级地增加随着参与者和实体的数量。

> [!NOTE]
> 这里提到了会隐式创建额外的主题？！

#### Latency and CPU utilization

Publishing a meta-message has the same overhead as that of publishing a small inter-process message. However, comparing the publication/reception of an intra and an inter-process message, the former requires several additional operations: it has to store the message in the ring buffer, monitor the number of `Subscription` s, and extract the message. The result is that from the latency and CPU utilization point of view, it is convenient to use intra-process communication only when the message size is at least 5KB.

> 发布元消息的开销与发布小型进程间消息相同。但是，比较发布/接收内部进程消息和外部进程消息，前者需要额外的操作：它**必须将消息存储在环形缓冲区中，监视订阅数量，并提取消息**。结果是，从延迟和 CPU 利用率的角度来看，**只有当消息大小至少为 5KB 时，才有利于使用内部进程通信**。

### Problems when both inter and intra-process communication are needed

Currently, ROS 2 does not provide any API for making nodes or `Publisher` and `Subscription` to ignore each other. This feature would be useful when both inter and intra-process communication are needed.

> 目前，ROS 2 没有提供任何 API 来使节点或发布者和订阅者互相忽略。这个功能在需要进程间和同一进程内的通信时会很有用。

The reason is that the current implementation of the ROS 2 middleware will try to deliver inter-process messages also to the nodes within the same process of the `Publisher`, even if they should have received an intra-process message. Note that these messages will be discarded, but they will still cause an overhead.

> 由于 ROS 2 中间件的当前实现会**尝试将跨进程消息也发送到发布者同一进程中的节点**，即使它们应该收到一个内部进程消息。请注意，这些消息将被丢弃，但仍会造成开销。

> [!NOTE]
> 进程间的消息也会在进程内也发一份？！
> 不太理解

The DDS specification provides ways for potentially fixing this problem, i.e. with the `ignore_participant`, `ignore_publication` and `ignore_subscription` operations. Each of these can be used to ignore a remote participant or entity, allowing to behave as that remote participant did not exist.

> DDS 规范提供了解决这个问题的方法，即使用 `ignore_participant`、`ignore_publication` 和 `ignore_subscription` 操作。每个操作都可以用来忽略远程参与者或实体，从而可以表现得就像远程参与者不存在一样。

> [!NOTE]
> 在 pub/sub_option 结构中确实存在类似的定义，`ignore_\*`

The current intra-process communication uses meta-messages that are sent through the RMW between nodes in the same process. This has two consequences: first it does not allow to directly "ignore" participants in the same process, because they still have to communicate in order to send and receive meta-messages, thus requiring a more fine-grained control ignoring specific `Publisher` s and `Subscription` s.

> **当前的进程内通信使用通过 RMW 在同一进程中的节点之间发送的元消息。** 这有两个后果：
>
> - 首先，它不允许直接“忽略”同一进程中的参与者，因为他们仍然必须进行通信以发送和接收元消息，因此需要更细粒度的控制来忽略特定的 `Publisher` 和 `Subscription`。

Moreover, the meta-messages could be delivered also to nodes in different processes if they have intra-process communication enabled. As before, the messages would be discarded immediately after being received, but they would still affect the performances. The overhead caused by the additional publication of meta-messages can be potentially reduced by appending to the intra-process topic names a process specific identifier.

> - 此外，如果启用了进程间通信，元消息也可以传送到不同进程中的节点。与以前一样，消息在接收后会立即被丢弃，但它们仍会影响性能。通过在进程特定标识符中附加进程内主题名称，可以减少由元消息发布引起的额外开销。

## Proposed implementation

### Overview

The new proposal for intra-process communication addresses the issues previously mentioned. It has been designed with performance in mind, so it avoids any communication through the middleware between nodes in the same process.

> 新的内部进程通信提案解决了先前提到的问题。它着眼于性能，因此**避免在同一进程中的节点之间通过中间件进行通信**。

Consider a simple scenario, consisting of `Publisher` s and `Subscription` s all in the same process and with the durability QoS set to `volatile`. The proposed implementation creates one buffer per `Subscription`. When a message is published to a topic, its `Publisher` pushes the message into the buffer of each of the `Subscription` s related to that topic and raises a notification, waking up the executor. The executor can then pop the message from the buffer and trigger the callback of the `Subscription`.

> 考虑一个简单的场景，由发布者和订阅者组成，都在同一进程中，可靠性 QoS 设置为 `volatile`。所提出的实现**为每个订阅创建一个缓冲区**。当消息发布到主题时，其**发布者将消息推入与该主题相关的每个订阅的缓冲区中，并引发通知，唤醒执行器**。然后，**执行器可以从缓冲区弹出消息并触发订阅的回调**。

![Proposed IPC Block Diagram](../img/intraprocess_communication/new_ipc.png)

The choice of having independent buffers for each `Subscription` leads to the following advantages:

> 每个订阅都有独立缓冲区的选择带来以下优势：

- It is easy to support different QoS for each `Subscription`, while, at the same time, simplifying the implementation.
- Multiple `Subscription` s can extract messages from their own buffer in parallel without blocking each other, thus providing an higher throughput.
- 很容易为每种 `订阅` 支持不同的 QoS，同时简化了实现。
- 多个 `订阅` 可以并行从自己的缓冲区中提取消息而不会相互阻止，从而提供了更高的吞吐量。

> [!NOTE]
> executor 的作用，从缓冲区中弹出消息并触发订阅的回调。
> 感觉这里的表述有点像是 dds 中 pub 之下又设置多个 data_writer 的意义！
> 真像！

The only drawback is that the system is not reusing as much resources as possible, compared to sharing buffers between entities. However, from a practical point of view, the memory overhead caused by the proposed implementation with respect to the current one, will always be only a tiny delta compared to the overall memory usage of the application.

> 系统的唯一缺点是，与实体之间共享缓冲区相比，它**没有尽可能多地重用资源**。但是，从实际的角度来看，与当前系统相比，所提出的实现所带来的内存开销总是**只有微小的增量，相对于应用程序的总内存使用量**而言。

There are three possible data-types that can be stored in the buffer:

> 在缓冲区中可以存储三种可能的数据类型：

- `MessageT`
- `shared_ptr<const MessageT>`
- `unique_ptr<MessageT>`

The choice of the buffer data-type is controlled through an additional field in the `SubscriptionOptions`. The default value for this option is denominated `CallbackDefault`, which corresponds to selecting the type between `shared_ptr<constMessageT>` and `unique_ptr<MessageT>` that better fits with its callback type. This is deduced looking at the output of `AnySubscriptionCallback::use_take_shared_method()`.

> 通过 `SubscriptionOptions` 中的附加字段控制缓冲数据类型的选择。此选项的默认值为 `CallbackDefault`，它对应于在 `shared_ptr<constMessageT>` 和 `unique_ptr<MessageT>` 之间选择与其回调类型最匹配的类型。这是通过查看 `AnySubscriptionCallback::use_take_shared_method()` 的输出来推断的。

If the history QoS is set to `keep all`, the buffers are dynamically adjusted in size up to the maximum resource limits specified by the underlying middleware. On the other hand, if the history QoS is set to `keep last`, the buffers have a size equal to the depth of the history and they act as ring buffers (overwriting the oldest data when trying to push while its full).

> 如果历史 QoS 设置为“保留所有”，则缓冲区的大小会根据底层中间件指定的最大资源限制动态调整。另一方面，如果历史 QoS 设置为“保留最后一个”，缓冲区的大小等于历史深度，它们充当环形缓冲区(尝试推送时覆盖最旧的数据，因为它已满)。

> [NOTE]:
> Buffers are not only used in `Subscription` s but also in each `Publisher` with a durability QoS of type `transient local`. The data-type stored in the `Publisher` buffer is always `shared_ptr<const MessageT>`.
> 缓冲区不仅用于订阅，而且还用于具有持久性 QoS 类型“瞬态本地”的每个发布者。存储在发布者缓冲区中的数据类型始终是“shared_ptr<const MessageT>”。
>
> ---
>
> @gpt-3.5
> 在 ROS2 中，`SubscriptionOptions` 提供了一种方式来控制缓冲消息的类型选择。可以使用 `subscription_options.use_take_shared_method` 参数来确定要使用的缓冲消息类型，该参数默认值为 `CallbackDefault`。此值表示 ROS2 将自动选择最适合回调函数类型的缓冲数据类型，并且可以在 `shared_ptr<constMessageT>` 和 `unique_ptr<MessageT>` 之间进行选择。
> 使用 `SubscriptionOptions` 进行配置时，可以设置 `subscription_options.use_take_shared_method` 为 true 或 false。如果设置为 `true`，则 ROS2 将使用 `shared_ptr` 缓冲消息类型，反之则使用 `unique_ptr` 类型。这种选择的方法基于 `AnySubscriptionCallback::use_take_shared_method()` 函数的输出来进行推断。
> 此外，`SubscriptionOptions` 提供了其他参数，例如 `reliable`，`keep_all` 和 `keep_last`，可以用来控制消息缓冲的可靠性和数量。这些参数可以更好地控制缓冲消息的行为，以满足不同的处理需求。

A new class derived from `rclcpp::Waitable` is defined, which is named `SubscriptionIntraProcessWaitable`. An object of this type is created by each `Subscription` with intra-process communication enabled and it is used to notify the `Subscription` that a new message has been pushed into its ring buffer and that it needs to be processed.

> 一个新的类从 `rclcpp::Waitable` 派生，被命名为 `SubscriptionIntraProcessWaitable`。这种类型的对象是由每一个具有内部进程通信功能的 `Subscription` 创建的，它用来通知 `Subscription` 一个新的消息已经被推入它的环形缓冲区，并需要处理。

The `IntraProcessManager` class stores information about each `Publisher` and each `Subscription`, together with pointers to these structures. This allows the system to know which entities can communicate with each other and to have access to methods for pushing data into the buffers.

> **`IntraProcessManager` 类存储有关每个 `Publisher` 和每个 `Subscription` 的信息，以及指向这些结构的指针。这样，系统就可以知道哪些实体可以相互通信，并且可以访问将数据推入缓冲区的方法。**

The decision whether to publish inter-process, intra-process or both is made every time the `Publisher::publish()` method is called. For example, if the `NodeOptions::use_intra_process_comms_` is enabled and all the known `Subscription` s are in the same process, then the message is only published intra-process. This remains identical to the current implementation.

> **决定是否发布跨进程、内部进程或两者都发布，每次调用 `Publisher::publish()` 方法时都会做出决定。** 例如，如果 `NodeOptions::use_intra_process_comms_` 被启用，且所有已知的 `Subscription` 都**在同一进程中，则消息仅在内部进程中发布**。这与当前实现保持一致。

> [!NOTE]
> 这就是有 manager 角色的好处！

### Creating a publisher

1. User calls `Node::create_publisher<MessageT>(...)`.
2. This boils down to `NodeTopics::create_publisher(...)`, where a `Publisher` is created through the factory.
3. Here, if intra-process communication is enabled, eventual intra-process related variables are initialized through the `Publisher::SetupIntraProcess(...)` method.
4. Then the `IntraProcessManager` is notified about the existence of the new `Publisher` through the method `IntraProcessManager::add_publisher(PublisherBase::SharedPtr publisher, PublisherOptions options)`.
5. `IntraProcessManager::add_publisher(...)` stores the `Publisher` information in an internal structure of type `PublisherInfo`.

> 1. 用户调用 `Node::create_publisher<MessageT>(...)`。
> 2. 这可以归结为 `NodeTopics::create_publisher(...)`，其中通过工厂创建一个 `Publisher`。
> 3. 如果启用了进程间通信，可以通过 `Publisher::SetupIntraProcess(...)` 方法来初始化相关的进程变量。
> 4. 然后，通过方法 `IntraProcessManager::add_publisher(PublisherBase::SharedPtr publisher，PublisherOptions options)`，`IntraProcessManager` 会被通知新的 `Publisher` 的存在。
> 5. `IntraProcessManager::add_publisher(...)` 会在一个类型为 **PublisherInfo 的内部结构中存储 Publisher 信息**。

The structure contains information about the `Publisher`, such as its QoS and its topic name, and a weak pointer for the `Publisher` object. An `uint64_t pub_id` unique within the `rclcpp::Context` is assigned to the `Publisher`. The `IntraProcessManager` contains a `std::map<uint64_t, PublisherInfo>` object where it is possible to retrieve the `PublisherInfo` of a specific `Publisher` given its id. The function returns the `pub_id`, that is stored within the `Publisher`.

> 该结构包含有关发布者的信息，如其 QoS 和主题名称，以及发布者对象的弱指针。**在 `rclcpp::Context` 中，将一个 `uint64_t pub_id` 分配给 `Publisher`。`IntraProcessManager` 包含一个 `std::map<uint64_t, PublisherInfo>` 对象，可以根据其 id 检索特定发布者的 `PublisherInfo`。** 该函数返回存储在 `Publisher` 中的 `pub_id`。

If the `Publisher` QoS is set to `transient local`, then the `Publisher::SetupIntraProcess(...)` method will also create a ring buffer of the size specified by the depth from the QoS.

> 如果 `发布者` QoS 设置为 `瞬态本地`，**那么 `Publisher::SetupIntraProcess(...)` 方法也将根据 QoS 指定的深度创建一个环形缓冲区**。

### Creating a subscription

1. User calls `Node::create_subscription<MessageT>(...)`.
2. This boils down to `NodeTopics::create_subscription(...)`, where a `Subscription` is created through the factory.
3. Here, if intra-process communication is enabled, intra-process related variables are initialized through the `Subscription::SetupIntraProcess(...)` method. The most relevant ones being the ring buffer and the waitable object.
4. Then the `IntraProcessManager` is notified about the existence of the new `Subscription` through the method `IntraProcessManager::add_subscription(SubscriptionBase::SharedPtr subscription, SubscriptionOptions options)`.
5. `IntraProcessManager::add_subscription(...)` stores the `Subscription` information in an internal structure of type `SubscriptionInfo`.

> 1. 用户调用 `Node::create_subscription<MessageT>(...)`。
> 2. 这可以归结为 `NodeTopics::create_subscription(...)`，其中通过工厂创建一个 `Subscription`。
> 3. 如果启用了进程内通信，则可以通过 `Subscription::SetupIntraProcess(...)` 方法初始化相关变量。最相关的是环形缓冲区和可等待对象。
> 4. 然后，通过方法 `IntraProcessManager::add_subscription(SubscriptionBase::SharedPtr subscription，SubscriptionOptions options)`，**`IntraProcessManager` 会被通知新的 `Subscription` 的存在**。
> 5. IntraProcessManager::add_subscription(...)将订阅信息存储在类型为 SubscriptionInfo 的内部结构中。

The structure contains information about the `Subscription`, such as its QoS, its topic name and the type of its callback, and a weak pointer for the `Subscription` object. An `uint64_t sub_id` unique within the `rclcpp::Context` is assigned to the `Subscription`. The `IntraProcessManager` contains a `std::map<uint64_t, SubscriptionInfo>` object where it is possible to retrieve the `SubscriptionInfo` of a specific `Subscription` given its id. There is also an additional structure `std::map<uint64_t, std::pair<std::set<uint64_t>, std::set<uint64_t>>>`. The key of the map is the unique id of a `Publisher` and the value is a pair of sets of ids. These sets contain the ids of the `Subscription` s that can communicate with the `Publisher`. We have two different sets because we want to differentiate the `Subscription` s depending on whether they request ownership of the received messages or not (note that this decision is done looking at their buffer, since the `Publisher` does not have to interact with the `Subscription` callback).

> 此结构包含有关订阅的信息，例如其 QoS、主题名称和回调类型，以及一个弱指针指向订阅对象。在 `rclcpp::Context` 中分配一个 `uint64_t sub_id` 来唯一标识订阅。`IntraProcessManager` 包含一个 `std::map<uint64_t, SubscriptionInfo>` 对象，可以根据其 `id` 检索特定订阅的 `SubscriptionInfo`。
> 还有一个额外的结构 `std::map<uint64_t, std::pair<std::set<uint64_t>, std::set<uint64_t>>>`。映射的键是发布者的唯一 `id`，值是一对 id 集合。这些集合包含可以与发布者通信的订阅 id。我们有两个不同的集合，因为我们**要根据接收到的消息的缓冲区来区分订阅者**(请注意，发布者不必与订阅者回调交互)。

6. The `SubscriptionIntraProcessWaitable` object is added to the list of Waitable interfaces of the node through `node_interfaces::NodeWaitablesInterface::add_waitable(...)`. It is added to the same callback group used for the standard inter-process communication of that topic.

> 6. `SubscriptionIntraProcessWaitable` 对象通过 `node_interfaces::NodeWaitablesInterface::add_waitable(...)` 添加到节点的 Waitable 接口列表中。它**添加到用于该主题的标准进程间通信的相同回调组中。**

### Publishing only intra-process

#### Publishing unique_ptr

1. User calls `Publisher::publish(std::unique_ptr<MessageT> msg)`.
2. `Publisher::publish(std::unique_ptr<MessageT> msg)` calls `IntraProcessManager::do_intra_process_publish(uint64_t pub_id, std::unique_ptr<MessageT> msg)`.
3. `IntraProcessManager::do_intra_process_publish(...)` uses the `uint64_t pub_id` to call `IntraProcessManager::get_subscription_ids_for_pub(uint64_t pub_id)`.

This returns the ids corresponding to `Subscription` s that have a QoS compatible for receiving the message. These ids are divided into two sublists, according to the data-type that is stored in the buffer of each `Susbscription`: requesting ownership (`unique_ptr<MessageT>`) or accepting shared (`shared_ptr<MessageT>`, but also `MessageT` since it will copy data in any case).

> 这会**返回与具有接收消息的兼容 QoS 的 `Subscription` 相对应的 ID**。这些 ID 被分成两个子列表，根据存储在每个 `Susbscription` 缓冲区中的数据类型：请求所有权(`unique_ptr<MessageT>`)或接受共享(`shared_ptr<MessageT>`，但也可以 `MessageT`，因为它将在任何情况下复制数据)。

4. The message is "added" to the ring buffer of all the items in the lists.

> 4. 这条消息已经添加到列表中所有项目的环形缓冲区中。

The `rcl_guard_condition_t` member of `SubscriptionIntraProcessWaitable` of each `Subscription` is triggered (this wakes up `rclcpp::spin`).

> **`Subscription` 的 `SubscriptionIntraProcessWaitable` 中的 `rcl_guard_condition_t` 成员被触发(这会唤醒 `rclcpp::spin`)。**

> [!NOTE]
> 进一步的，sub 会出发 spin，即 executor...，分发消息？

The way in which the `std::unique_ptr<MessageT>` message is "added" to a buffer, depends on the type of the buffer.

> `std::unique_ptr<MessageT>` 消息添加到缓冲区的方式取决于缓冲区的类型。

- `BufferT = unique_ptr<MessageT>` The buffer receives **a copy** of `MessageT` and has ownership on it; for the last buffer, a copy is not necessary as ownership can be transferred.
- `BufferT = shared_ptr<const MessageT>` Every buffer receives a shared pointer of the same `MessageT`; **no copies** are required.
- `BufferT = MessageT` **A copy** of the message is added to every buffer.

![Sequence UML diagram](../img/intraprocess_communication/intra_process_only.png)

#### Publishing other message types

> [!NOTE]: about message_t
> [](C:%5CUsers%5Ctrantor%5CDownloads%5CDocuments%5CChatGPT%5Cdialogue%5Cros2%5Cmessage_t_20230530.md)

The `Publisher::publish(...)` method is overloaded to support different message types:

> `Publisher::publish(...)` 方法被重载以支持不同的消息类型。

- `unique_ptr<MessageT>`
- `MessageT &`
- `MessageT*`
- `const shared_ptr<const MessageT>`

The last two of them are actually deprecated since ROS 2 Dashing. All these methods are unchanged with respect to the current implementation: they end up creating a `unique_ptr` and calling the `Publisher::publish(std::unique_ptr<MessageT> msg)` described above.

> 最后两个实际上自 ROS 2 Dashing 以来已经弃用了。所有这些方法都与当前实现保持不变：它们最终会创建一个 `unique_ptr`，并调用上述 `Publisher::publish(std::unique_ptr<MessageT> msg)`。

### Receiving intra-process messages

As previously described, whenever messages are added to the ring buffer of a `Subscription`, a condition variable specific to the `Subscription` is triggered. This condition variable has been added to the `Node` waitset so it is being monitored by the `rclcpp::spin`.

> **当消息添加到订阅的环形缓冲区时，会触发与订阅相关的条件变量。该条件变量已添加到 Node 等待集，因此由 rclcpp::spin 监视。**

> [!NOTE]: about executor

Remember that the `SubscriptionIntraProcessWaitable` object has access to the ring buffer and to the callback function pointer of its related `Subscription`.

> 记住，`SubscriptionIntraProcessWaitable` 对象可以访问它相关的 `Subscription` 的环形缓冲区和回调函数指针。

1. The guard condition linked with the `SubscriptionIntraProcessWaitable` object awakes `rclcpp::spin`.
2. The `SubscriptionIntraProcessWaitable::is_ready()` condition is checked. This has to ensure that the ring buffer is not empty.
3. The `SubscriptionIntraProcessWaitable::execute()` function is triggered. Here the first message is extracted from the buffer and then the `SubscriptionIntraProcessWaitable` calls the `AnySubscriptionCallback::dispatch_intra_process(...)` method. There are different implementations for this method, depending on the data-type stored in the buffer.
4. The `AnySubscriptionCallback::dispatch_intra_process(...)` method triggers the associated callback. Note that in this step, if the type of the buffer is a smart pointer one, no message copies occurr, as ownership has been already taken into account when pushing a message into the queue.

> 1. 当与 `SubscriptionIntraProcessWaitable` 对象相关联的守卫条件唤醒 `rclcpp::spin` 时。
> 2. 检查 `SubscriptionIntraProcessWaitable::is_ready()` 条件。这必须确保环形缓冲区不为空。
> 3. 触发 `SubscriptionIntraProcessWaitable::execute()` 函数。在此，首先从缓冲区中提取第一条消息，然后 `SubscriptionIntraProcessWaitable` 调用 `AnySubscriptionCallback::dispatch_intra_process(...)` 方法。根据缓冲区中存储的数据类型，这个方法有不同的实现。
> 4. `AnySubscriptionCallback::dispatch_intra_process(...)` 方法触发相关的回调。注意，在这一步中，如果缓冲区的类型是智能指针，则不会发生消息复制，因为在将消息推入队列时已经考虑了所有权。

> [!NOTE] > [](C:%5CUsers%5Ctrantor%5CDownloads%5CDocuments%5CChatGPT%5Cdialogue%5Cros2%5Cexecutor_spin_20230530.md)
> 需要好好看看这里面的讨论。

### Publishing intra and inter-process

1. User calls `Publisher::publish(std::unique_ptr<MessageT> msg)`.
2. The message is moved into a shared pointer `std::shared_ptr<MessageT> shared_msg = std::move(msg)`.
3. `Publisher::publish(std::unique_ptr<MessageT> msg)` calls `IntraProcessManager::do_intra_process_publish(uint64_t pub_id, std::shared_ptr<MessageT> shared_msg)`.

> 1. 用户调用 `Publisher::publish(std::unique_ptr<MessageT> msg)`。
> 2. 消息被移动到了一个共享指针 `std::shared_ptr<MessageT> shared_msg = std::move(msg)`。
> 3. `Publisher::publish(std::unique_ptr<MessageT> msg)` 调用 `IntraProcessManager::do_intra_process_publish(uint64_t pub_id, std::shared_ptr<MessageT> shared_msg)`。

The following steps are identical to steps 3, 4, and 5 applied when publishing only intra-process.

> 以下步骤与只发布内部过程时所采取的步骤 3、4 和 5 相同。

4. `IntraProcessManager::do_intra_process_publish(...)` uses the `uint64_t pub_id` to call `IntraProcessManager::get_subscription_ids_for_pub(uint64_t pub_id)`.

Then it calls `IntraProcessManager::find_matching_subscriptions(PublisherInfo pub_info)`. This returns the ids corresponding to `Subscription` s that have a QoS compatible for receiving the message. These ids are divided into two sublists, according to the data-type that is stored in the buffer of each `Susbscription`: requesting ownership (`unique_ptr<MessageT>`) or accepting shared (`shared_ptr<MessageT>`, but also `MessageT` since it will copy data in any case).

> 然后它调用 `IntraProcessManager::find_matching_subscriptions(PublisherInfo pub_info)`。这将返回与接收消息具有兼容 QoS 的 `Subscription` s 的 id。根据存储在每个 `Susbscription` 缓冲区中的数据类型，这些 id 被分为两个子列表：请求所有权(`unique_ptr<MessageT>`)或接受共享(`shared_ptr<MessageT>`，但也是 `MessageT`，因为它将在任何情况下复制数据)。

5. The message is "added" to the ring buffer of all the items in the list.

> "消息已添加到列表中所有项目的环形缓冲区中。"

The `rcl_guard_condition_t` member of `SubscriptionIntraProcessWaitable` of each `Subscription` is triggered (this wakes up `rclcpp::spin`).

> 每个订阅的 SubscriptionIntraProcessWaitable 的 rcl_guard_condition_t 成员被触发(这会唤醒 rclcpp::spin)。

After the intra-process publication, the inter-process one takes place.

> **在进程内发布之后，进程间发布就会发生。**

6. `Publisher::publish(std::unique_ptr<MessageT> msg)` calls `Publisher::do_inter_process_publish(const MessageT & inter_process_msg)`, where `MessageT inter_process_msg = *shared_msg`.

The difference from the previous case is that here a `std::shared_ptr<const MessageT>` is being "added" to the buffers. Note that this `std::shared_ptr` has been just created from a `std::unique_ptr` and it is only used by the `IntraProcessManager` and by the RMW, while the user application has no access to it.

> 上一种情况的不同之处在于，这里一个 `std::shared_ptr<const MessageT>` 被"添加"到缓冲区中。**注意，这个 `std::shared_ptr` 只是从 `std::unique_ptr` 中创建的，只能由 `IntraProcessManager` 和 RMW 使用，而用户应用程序无法访问它。**

- `BufferT = unique_ptr<MessageT>` The buffer receives a copy of `MessageT` and has ownership on it.
- `BufferT = shared_ptr<const MessageT>` Every buffer receives a shared pointer of the same `MessageT`, so no copies are required.
- `BufferT = MessageT` A copy of the message is added to every buffer.

The difference with publishing a unique_ptr is that here it is not possible to save a copy. If you move the ownership of the published message to one of the `Subscription` (so potentially saving a copy as done in the previous case), you will need to create a new copy of the message for inter-process publication.

> **发布 unique_ptr 的区别在于这里不可能保存一份副本。** 如果将发布的消息的所有权转移给其中一个“订阅”(因此可以像前面的情况一样保存副本)，则**需要为跨进程发布创建一个新的消息副本**。

![Sequence UML diagram](../img/intraprocess_communication/intra_inter_process.png)

### QoS features

The proposed implementation can handle all the different QoS.

> 所提出的实施方案能够处理所有不同的 QoS。

- If the history is set to `keep_last`, then the depth of the history corresponds to the size of the ring buffer. On the other hand, if the history is set to `keep_all`, the buffer becomes a standard FIFO queue with an unbounded size.
- The reliability is only checked by the `IntraProcessManager` in order to understand if a `Publisher` and a `Subscription` are compatible. The use of buffers ensures that all the messages are delivered without the need to resend them. Thus, both options, `reliable` and `best-effort`, are satisfied.
- The durability QoS is used to understand if a `Publisher` and a `Subscription` are compatible. How this QoS is handled is described in details in the following paragraph.
- 如果将历史记录设置为“keep_last”，则历史深度对应于环缓冲区的大小。另一方面，如果将历史记录设置为 `keep_all`，则缓冲区成为标准的 FIFO 队列，其大小无限。
- 可靠性仅由 `IntraProcessManager` 检查，以了解 `publisher` 和 `subscription` 是兼容。使用缓冲区可确保所有消息都可以传递，而无需重新发送它们。因此，满足了两个选项，“可靠”和“最佳富特”。
- 耐用性 QoS 用于理解 `Publisher` 和 `subscription` 是兼容。在以下段落中详细描述了如何处理此 QoS。

#### Handling Transient Local

If the `Publisher` durability is set to `transient_local` an additional buffer on the `Publisher` side is used to store the sent intra-process messages.

> 如果 `Publisher` 的耐久性设置为 `transient_local`，`Publisher` 端会使用额外的缓冲区来存储发送的进程内消息。

Late-joiner `Subscription` s will have to extract messages from this buffer once they are added to the `IntraProcessManager`. In this case the `IntraProcessManager` has to check if the recently created `Subscription` is a late-joiner, and, if it is, it has to retrieve messages from the `Transient Local` `Publisher` s.

> 晚加入者订阅将不得不从缓冲区中提取消息，一旦它们被添加到 IntraProcessManager 中。在这种情况下，IntraProcessManager 必须检查最近创建的订阅是否为晚加入者，如果是，它必须从 Transient Local 发布者中检索消息。

1. Call `IntraProcessManager::find_matching_publishers(SubscriptionInfo sub_info)` that returns a list of stored `PublisherInfo` that have a QoS compatible for sending messages to this new `Subscription`. These will be all `Transient Local` `Publisher` s, so they have a ring buffer.
2. Copy messages from all the ring buffers found into the ring buffer of the new `Subscription`. **TODO:** are there any constraints on the order in which old messages have to be retrieved? (i.e. 1 publisher at the time; all the firsts of each publisher, then all the seconds ...).
3. If at least 1 message was present, trigger the `rcl_guard_condition_t` member of the `SubscriptionIntraProcessWaitable` associated with the new `Subscription`.

> 1. 调用 `IntraProcessManager::find_matching_publishers(SubscriptionInfo sub_info)`，返回一组存储的 PublisherInfo，它们具有与发送消息给新 Subscription 的 QoS 兼容。这些将是所有的瞬态本地发布者，因此它们具有环形缓冲区。
> 2. 将所有找到的环形缓冲区中的消息复制到新的 `Subscription` 环形缓冲区中。**待办事项：** 检索旧消息的顺序有任何限制吗？(即，一个发布者一次; 所有发布者的第一个，然后是所有的第二个...)。
> 3. 如果至少有 1 条消息存在，触发与新的订阅相关联的 `SubscriptionIntraProcessWaitable` 的 `rcl_guard_condition_t` 成员。

However, this is not enough as it does not allow to handle the scenario in which a `transient local` `Publisher` has only intra-process `Subscription` s when it is created, but, eventually, a `transient local` `Subscription` in a different process joins. Initially, published messages are not passed to the middleware, since all the `Subscription` s are in the same process. This means that the middleware is not able to store old messages for eventual late-joiners.

> 然而，这还不够，因为它不能处理这样一种情况：**当创建时，“瞬态本地”发布者只有进程内的订阅，但最终，不同进程中的“瞬态本地”订阅者加入。** 最初，发布的消息不会传递到中间件，因为所有的订阅都在同一进程中。这意味着**中间件无法为后来加入者存储旧消息。**

The solution to this issue consists in always publishing both intra and inter-process when a `Publisher` has `transient local` durability. For this reason, when `transient local` is enabled, the `do_intra_process_publish(...)` function will always process a shared pointer. This allows us to add the logic for storing the published messages into the buffers only in one of the two `do_intra_process_publish(...)` cases and also it allows to use buffers that have only to store shared pointers.

> 解决这个问题的办法是，当发布者具有“瞬态本地”持久性时，**始终发布内部和跨进程**。因此，当启用“瞬态本地”时，`do_intra_process_publish(...)` 函数将始终处理共享指针。这样，我们可以在两个 `do_intra_process_publish(...)` 情况之一中添加将发布的消息存储到缓冲区的逻辑，并且还允许使用只存储共享指针的缓冲区。

### Number of message copies

In the previous sections, it has been briefly described how a message can be added to a buffer, i.e. if it is necessary to copy it or not. Here some details about how this proposal adresses some more complex cases. As previously stated, regardless of the data-type published by the user, the flow always goes towards `Publisher::publish(std::unique_ptr<MessageT> msg)`.

> 在前面的章节中，我们简要描述了**如何将消息添加到缓冲区，即是否需要复制它**。这里有关于如何解决更复杂情况的一些细节。正如之前所说，无论用户发布的数据类型如何，流动总是朝着 `Publisher::publish(std::unique_ptr <MessageT> msg)`。

The `std::unique_ptr<MessageT> msg` is passed to the `IntraProcessManger` that decides how to add this message to the buffers. The decision is taken looking at the number and the type, i.e. if they want ownership on messages or not, of the `Subscription` s.

> `std::unique_ptr<MessageT> msg` 被传递给 **`IntraProcessManger`，它决定如何将这条消息添加到缓冲区**。**决策是根据 `Subscription` 的数量和类型，即是否想要拥有消息，来做出的。**

If all the `Subscription` s want ownership of the message, then a total of `N-1` copies of the message are required, where `N` is the number of `Subscription` s. The last one will receive ownership of the published message, thus saving a copy.

> 如果所有订阅都想拥有消息的所有权，那么需要 N-1 份消息的副本，其中 N 是订阅的数量。最后一个将拥有已发布消息的所有权，从而**节省一份副本**。

If none of the `Subscription` s want ownership of the message, `0` copies are required. It is possible to convert the message into a `std::shared_ptr<MessageT> msg` and to add it to every buffer.

> 如果没有一个 Subscription 想要拥有这条消息，则不需要复制。可以将消息转换为 `std::shared_ptr<MessageT> msg`，然后将其添加到每个缓冲区。

If there is 1 `Subscription` that does not want ownership while the others want it, the situation is equivalent to the case of everyone requesting ownership:`N-1` copies of the message are required. As before the last `Subscription` will receive ownership.

> 如果有 1 个订阅不想拥有所有权，而其他人想要，那么情况就等同于每个人都要求拥有所有权：需要发送 N-1 份消息。和之前一样，最后一个订阅将获得所有权。

If there is more than 1 `Subscription` that do not want ownership while the others want it, a total of `M` copies of the message are required, where `M` is the number of `Subscription` s that want ownership. `1` copy will be shared among all the `Subscription` s that do not want ownership, while `M-1` copies are for the others.

> 如果有多于 1 个订阅者不想要所有权，而其他订阅者想要，则需要发送 M 份消息，其中 M 是想要所有权的订阅者的数量。将 1 份消息分享给所有不想要所有权的订阅者，而 M-1 份消息则留给其他订阅者。

As in the current implementation, if both inter and intra-process communication are needed, the `std::unique_ptr<MessageT> msg` will be converted into a `std::shared_ptr<MessageT> msg` and passed respectively to the `do_intra_process_publish` and `do_inter_process_publish` functions.

> 在目前的实现中，**如果需要进程间和进程内的通信，将 `std::unique_ptr<MessageT> msg` 转换为 `std::shared_ptr<MessageT> msg`，分别传递给 `do_intra_process_publish` 和 `do_inter_process_publish` 函数。**

A copy of the message will be given to all the `Subscription` s requesting ownership, while the others can copy the published shared pointer.

> 所有要求所有权的订阅者都会收到一份消息的副本，而其他人可以复制已发布的共享指针。

The following tables show a recap of when the proposed implementation has to create a new copy of a message. The notation `@` indicates a memory address where the message is stored, different memory addresses correspond to different copies of the message.

> 以下表格展示了建议实施时需要创建消息副本的概要。符号“@”表示存储消息的内存地址，不同的内存地址对应不同的消息副本。

#### Publishing UniquePtr

| publish\<T\>          | BufferT                                                                            | Results                    |
| ------------------- | ---------------------------------------------------------------------------------- | -------------------------- |
| unique_ptr\<MsgT\> @1 | unique_ptr\<MsgT\>                                                                   | @1                         |
| unique_ptr\<MsgT\> @1 | unique_ptr\<MsgT\> <br> unique_ptr\<MsgT\>                                             | @1 <br> @2                 |
| unique_ptr\<MsgT\> @1 | shared_ptr\<MsgT\>                                                                   | @1                         |
| unique_ptr\<MsgT\> @1 | shared_ptr\<MsgT\> <br> shared_ptr\<MsgT\>                                             | @1 <br> @1                 |
| unique_ptr\<MsgT\> @1 | unique_ptr\<MsgT\> <br> shared_ptr\<MsgT\>                                             | @1 <br> @2                 |
| unique_ptr\<MsgT\> @1 | unique_ptr\<MsgT\> <br> shared_ptr\<MsgT\> <br> shared_ptr\<MsgT\>                       | @1 <br> @2 <br> @2         |
| unique_ptr\<MsgT\> @1 | unique_ptr\<MsgT\> <br> unique_ptr\<MsgT\> <br> shared_ptr\<MsgT\> <br> shared_ptr\<MsgT\> | @1 <br> @2 <br> @3 <br> @3 |

#### Publishing SharedPtr

| publish\<T\>          | BufferT                                                                            | Results                    |
| ------------------- | ---------------------------------------------------------------------------------- | -------------------------- |
| shared_ptr\<MsgT\> @1 | unique_ptr\<MsgT\>                                                                   | @2                         |
| shared_ptr\<MsgT\> @1 | unique_ptr\<MsgT\> <br> unique_ptr\<MsgT\>                                             | @2 <br> @3                 |
| shared_ptr\<MsgT\> @1 | shared_ptr\<MsgT\>                                                                   | @1                         |
| shared_ptr\<MsgT\> @1 | shared_ptr\<MsgT\> <br> shared_ptr\<MsgT\>                                             | @1 <br> @1                 |
| shared_ptr\<MsgT\> @1 | unique_ptr\<MsgT\> <br> shared_ptr\<MsgT\>                                             | @2 <br> @1                 |
| shared_ptr\<MsgT\> @1 | unique_ptr\<MsgT\> <br> shared_ptr\<MsgT\> <br> shared_ptr\<MsgT\>                       | @2 <br> @1 <br> @1         |
| shared_ptr\<MsgT\> @1 | unique_ptr\<MsgT\> <br> unique_ptr\<MsgT\> <br> shared_ptr\<MsgT\> <br> shared_ptr\<MsgT\> | @2 <br> @3 <br> @1 <br> @1 |

The possibility of setting the data-type stored in each buffer becomes helpful when dealing with more particular scenarios.

> 在处理更多特定场景时，设置每个缓冲区中存储的数据类型的可能性变得有帮助。

Considering a scenario with N `Subscription` s all taking a unique pointer. If the `Subscription` s don't actually take the message (e.g. they are busy and the message is being overwritten due to QoS settings) the default buffer type (`unique_ptr` since the callbacks require ownership) would result in the copy taking place anyway. By setting the buffer type to `shared_ptr`, no copies are needed when the `Publisher` pushes messages into the buffers. Eventually, the `Subscription` s will copy the data only when they are ready to process it.

> 考虑一个 N 个订阅者都拥有唯一指针的场景。**如果订阅者实际上没有接收消息(例如他们正忙，而消息由于 QoS 设置被覆盖)，那么默认的缓冲区类型(因为回调函数需要所有权，所以是 unique_ptr)会导致复制发生。通过将缓冲区类型设置为 shared_ptr，发布者将消息推送到缓冲区时不需要复制。最终，订阅者只有在准备处理数据时才会复制数据。**

On the other hand, if the published data are very small, it can be advantageous to do not use C++ smart pointers, but to directly store the data into the buffers.

> 另一方面，如果**发布的数据非常小**，不使用 C++ 智能指针，而是直接将数据存储到缓冲区可能会更有利。

In all this situations, the number of copies is always smaller or equal than the one required for the current intra-process implementation.

> 在所有这些情况中，**复制的数量总是小于或等于当前进程实现所需的数量**。

However, there is a particular scenario where having multiple buffers makes much more difficult saving a copy. There are two `Subscription` s, one taking a shared pointer and the other taking a unique pointer. With a more centralized system, if the first `Subscription` requests its shared pointer and then releases it before the second `Subscription` takes the message, it is potentially possible to optimize the system to manage this situation without requiring any copy. On the other hand, the proposed implementation will immediately create one copy of the message for the `Subscription` requiring ownership. Even in case of using a `shared_ptr` buffer as previously described, it becomes more difficult to ensure that the other `Subscription` is not using the pointer anymore.

> 然而，有一种特殊情况，多个缓冲区会使保存副本变得更加困难。有两个 `Subscription`，一个接受共享指针，另一个接受唯一指针。在一个更集中的系统中，如果第一个 `Subscription` 请求其共享指针，然后在第二个 `Subscription` 获取消息之前释放它，则可能优化系统以在不需要任何副本的情况下管理此情况。另一方面，所提出的实现将立即为需要所有权的 `Subscription` 创建一个消息副本。即使使用前面描述的 `shared_ptr` 缓冲区，也很难确保其他 `Subscription` 不再使用该指针。

#### Where are these copies performed?

The `IntraProcessManger::do_intra_process_publish(...)` function knows whether the intra-process buffer of each `Subscription` requires ownership or not. For this reason it can perform the minimum number of copies required by looking at the total number of `Subscription` s and their types. The buffer does not perform any copy when receiving a message, but directly stores it.

> `IntraProcessManger::do_intra_process_publish(...)` 函数知道每个 Subscription 的内部进程缓冲区是否需要所有权。因此，它可以根据总的 Subscription 数量及其类型执行所需的最少数量的副本。在接收消息时，缓冲区不执行任何副本，而是直接存储它。

When extracting a message from the buffer, the `Subscription` can require any particular data-type. The intra-process buffer will perform a copy of the message whenever necessary, for example in the previously described cases where the data-type stored in the buffer is different from the callback one.

> 当从缓冲区提取消息时，`订阅` 可以要求任何特定的数据类型。无论何时需要，内部进程缓冲区都会对消息进行复制，例如在前面描述的情况下，缓冲区中存储的数据类型与回调类型不同。

## Perfomance evaluation

The implementation of the presented new intra-process communication mechanism is hosted on [GitHub here](https://github.com/alsora/rclcpp/tree/alsora/new_ipc_proposal).

> 实现的新的进程间通信机制可以在 [GitHub 这里](https://github.com/alsora/rclcpp/tree/alsora/new_ipc_proposal)找到。

This section contains experimental results obtained comparing the current intra-process communication implementation with an initial implementation of the proposed one. The tests span multiple ROS 2 applications and use-cases and have been validated on different machines.

> 这一部分包含了将当前进程间通信实现与所提出的初始实现进行比较而获得的实验结果。测试涵盖了多个 ROS 2 应用和用例，并在不同的机器上进行了验证。

All the following experiments have been run using the ROS 2 Dashing and with `-O2` optimization enabled.

> 所有以下实验均使用 ROS 2 Dashing，并启用了 `-O2` 优化。

```
colcon build --cmake-args  -DCMAKE_CXX_FLAGS="-O2" -DCMAKE_C_FLAGS="-O2"
```

The first test has been carried out using the `intra_process_demo` package contained in the [ROS 2 demos repository](https://github.com/ros2/demos). A first application, called `image_pipeline_all_in_one`, is made of 3 nodes, where the fist one publishes a `unique_ptr<Image>` message. A second node subscribes to the topic and republishes the image after modifying it on a new topic. A third node subscribes to to this last topic.

> 第一次测试使用 [ROS 2 demos repository](https://github.com/ros2/demos) 中包含的 `intra_process_demo` 包进行了测试。第一个应用程序，称为 `image_pipeline_all_in_one`，由 3 个节点组成，其中第一个节点发布 `unique_ptr<Image>` 消息。第二个节点订阅该主题，并在新主题上修改镜像后重新发布镜像。第三个节点订阅此最后一个主题。

Also a variant of the application has been tested: it's `image_pipeline_with_two_image_view`, where there are 2 consumers at the end of the pipeline.

> 也测试了一个应用的变体：它是 `image_pipeline_with_two_image_view`，管道的末端有两个消费者。

In these tests the latency is computed as the total pipeline duration, i.e. the time from when the first node publishes the image to when the last node receives it. The CPU usage and the latency have been obtained from `top` command and averaged over the experiment duration.

> 在这些测试中，延迟被计算为总管道持续时间，即从第一个节点发布镜像到最后一个节点接收镜像的时间。**CPU 使用率和延迟是从 `top` 命令获得的**，并在实验持续时间内平均。

Performance evaluation on a laptop computer with Intel i7-6600U CPU @ 2.60GHz.

> 性能评估：Intel i7-6600U CPU @ 2.60GHz 的笔记本电脑。

| ROS 2 system                       | IPC      | RMW       | Latency [us] | CPU [%] | RAM [Mb] |
| ---------------------------------- | -------- | --------- | ------------ | ------- | -------- |
| image_pipeline_all_in_one          | off      | Fast-RTPS | 1800         | 23      | 90       |
| image_pipeline_all_in_one          | standard | Fast-RTPS | 920          | 20      | 90       |
| image_pipeline_all_in_one          | new      | Fast-RTPS | 350          | 15      | 90       |
| image_pipeline_with_two_image_view | off      | Fast-RTPS | 2900         | 24      | 94       |
| image_pipeline_with_two_image_view | standard | Fast-RTPS | 2000         | 20      | 95       |
| image_pipeline_with_two_image_view | new      | Fast-RTPS | 1400         | 16      | 94       |

From this simple experiment is immediately possible to see the improvement in the latency when using the proposed intra-process communication. However, an even bigger improvement is present when analyzing the results from more complex applications.

> 从这个简单的实验中，可以立即看到在使用所提出的进程内通信时延迟的改善。但是，当分析更复杂应用程序的结果时，会有更大的改进。

The next results have been obtained running the iRobot benchmark application. This allows the user to specify the topology of a ROS 2 graph that will be entirely run in a single process.

> 下一个结果是通过**运行 iRobot 基准应用程序**获得的。这允许用户指定 ROS 2 图的拓扑，该图将完全在单个进程中运行。

> [!NOTE]
> **运行 iRobot 基准应用程序**，这个可以考虑一下。

The application has been run with the topologies Sierra Nevada and Mont Blanc. Sierra Nevada is a 10-node topology and it contains 10 publishers and 13 subscriptions. One topic has a message size of 10KB, while all the others have message sizes between 10 and 100 bytes.

> 应用程序已使用 Sierra Nevada 和 Mont Blanc 拓扑运行。Sierra Nevada 是一个 10 节点拓扑，它包含 10 个发布者和 13 个订阅。一个主题的消息大小为 10KB，而其他所有主题的消息大小均在 10 到 100 字节之间。

Mont Blanc is a bigger 20-node topology, containing 23 publishers and 35 subscriptions. Two topics have a message size of 250KB, three topics have message sizes between 1KB and 25KB, and the rest of the topics have message sizes smaller than 1KB.

> 蒙布朗是一个拥有 20 个节点的大型拓扑，包含 23 个发布者和 35 个订阅者。有两个主题的消息大小为 250KB，三个主题的消息大小在 1KB 和 25KB 之间，其余主题的消息大小小于 1KB。

A detailed description and the source code for these application and topologies can be found [here](https://github.com/irobot-ros/ros2-performance/tree/master/performances/benchmark).

> 详细的描述和这些应用程序和拓扑结构的源代码可以在[这里](https://github.com/irobot-ros/ros2-performance/tree/master/performances/benchmark)找到。

> [NOTE]:

```
    | ROS 2 system  | IPC      | RMW       | Latency [us] | CPU [%] | RAM [Mb] |
    | ------------- | -------- | --------- | ------------ | ------- | -------- |
    | Sierra Nevada | off      | Fast-RTPS | 600          | 14      | 63       |
    | Sierra Nevada | standard | Fast-RTPS | 650          | 16      | 73->79   |
    | Sierra Nevada | new      | Fast-RTPS | 140          | 8       | 63       |
    | Mont Blanc    | off      | Fast-RTPS | 1050         | 22      | 180      |
    | Mont Blanc    | standard | Fast-RTPS | 750          | 18      | 213->220 |
    | Mont Blanc    | new      | Fast-RTPS | 160          | 8       | 180      |
```

A similar behavior can be observed also running the application on resource constrained platforms. The following results have been obtained on a RaspberryPi 2.

> 在资源受限的平台上运行应用程序也可以观察到类似的行为。在 RaspberryPi 2 上获得了以下结果。

```
    | ROS 2 system  | IPC      | RMW       | Latency [us] | CPU [%] | RAM [Mb] |
    | ------------- | -------- | --------- | ------------ | ------- | -------- |
    | Sierra Nevada | off      | Fast-RTPS | 800          | 18      | 47       |
    | Sierra Nevada | standard | Fast-RTPS | 725          | 20      | 54->58   |
    | Sierra Nevada | new      | Fast-RTPS | 170          | 10      | 47       |
    | Mont Blanc    | off      | Fast-RTPS | 1500         | 30      | 130      |
    | Mont Blanc    | standard | Fast-RTPS | 950          | 26      | 154->159 |
    | Mont Blanc    | new      | Fast-RTPS | 220          | 14      | 130      |
```

For what concerns latency and CPU usage, Sierra Nevada behaves almost the same regardless if standard IPC is enabled or not. This is due to the fact that most of its messages are very small in size. On the other hand, there are noticeable improvements in Mont Blanc, where several messages of non-negligible size are used.

> 就延迟和 CPU 使用率而言，无论是否启用标准 IPC，Sierra Nevada 的表现几乎相同。这是因为它的大多数消息都很小。另一方面，Mont Blanc 有明显的改进，其中使用了几条不可忽视的消息。

From the memory point of view, there is an almost constant increase in the utilization during the execution of the program when standard intra-process communication mechanism is used. Since the experiments have been run for 120 seconds, there is an increase of approximately 60KB per second. However, even considering the initial memory usage, it is possible to see how it is affected from the presence of the additional publishers and subscriptions needed for intra-process communication. There is a difference of 10MB in Sierra Nevada and of 33MB in Mont Blanc between standard intra-process communication on and off.

> 从记忆点的角度来看，在使用标准的进程间通信机制时，程序执行期间的利用率几乎是持续增加的。由于实验已经运行了 120 秒，每秒增加约 60KB。但是，即使考虑到初始内存使用量，也可以看到它是如何受到进程间通信所需的额外发布者和订阅者的影响。在 Sierra Nevada 和 Mont Blanc 之间，标准进程间通信开启和关闭之间有 10MB 的差异。

The last experiment show how the current implementation performs in the case that both intra and inter-process communication are needed. The test consists of running Sierra Nevada on RaspberryPi 2, and, in a separate desktop machine, a single node subscribing to all the available topics coming from Sierra Nevada. This use-case is common when using tools such as `rosbag` or `rviz`.

> 最后一个实验表明，在需要内部和进程间通信的情况下，当前实施的性能如何。测试包括在 RaspberryPi 2 上运行 Sierra Nevada，并且在另一台台式机上，运行一个单节点，订阅来自 Sierra Nevada 的所有可用主题。当使用诸如 `rosbag` 或 `rviz` 之类的工具时，这种用例很常见。

```
    | ROS 2 system               | IPC      | RMW       | Latency [us] | CPU [%] | RAM [Mb] |
    | -------------------------- | -------- | --------- | ------------ | ------- | -------- |
    | Sierra Nevada + debug node | off      | Fast-RTPS | 800          | 22      | 50       |
    | Sierra Nevada + debug node | standard | Fast-RTPS | 1100         | 35      | 60->65   |
    | Sierra Nevada + debug node | new      | Fast-RTPS | 180          | 15      | 32       |
```

These results show that if there is at least one node in a different process, with the current implementation it is better to keep intra-process communication disabled. The proposed implementation does not require the ROS 2 middleware when publishing intra-process. This allows to easily remove the connections between nodes in the same process when it is required to publish also inter process, potentially resulting in a very small overhead with respect to the only intra-process case.

> 这些结果表明，如果至少有一个节点在不同的进程中，则使用当前实现最好保持同进程通信禁用。所提出的实现不需要 ROS 2 中间件来发布同进程。这允许在需要发布跨进程时轻松删除同一进程中节点之间的连接，从而可能产生非常小的开销，相对于仅同进程的情况。

## Open Issues

There are some open issues that are not addressed neither on the current implementation nor on the proposed one.

> 有一些没有在当前实现和拟议实现中解决的开放问题。

- The proposal does not take into account the problem of having a queue with twice the size when both inter and intra-process communication are used.

A `Publisher` or a `Subscription` with a history depth of 10 will be able to store up to 20 messages without processing them (10 intra-process and 10 inter-process). This issue is also present in the current implementation, since each `Subscription` is doubled.

> 发布者或订阅者带有 10 个历史深度可以存储多达 20 条消息而无需处理(10 个进程内和 10 个进程间)。该问题也存在于当前实现中，因为每个订阅者都被加倍。
