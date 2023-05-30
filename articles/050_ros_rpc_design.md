---
tip: translate by openai@2023-05-30 09:07:30
layout: default
title: RPC API design in ROS
permalink: articles/ros_rpc.html
abstract:
  This article is an exploration of possible design patterns for the next generation of ROS Remote Procedure Call interfaces.
  We focus here on specifying the user API and leave the implementation unspecified.
  It is expected that there are one or more RPC implementations which can be used, such as Apache Thrift, ROS RPC, or MsgPack.
author: '[Tully Foote](https://github.com/tfoote)'
date_written: 2014-06
last_modified: 2019-05
published: true

<div class="alert alert-warning" markdown="1">
  This article is out-of-date. It was written at a time before decisions were made to use DDS and RTPS as the underlying communication standards for ROS 2. It represents an idealistic understanding of what RPC and "actions" should be like in ROS. It can be considered memoranda and not necessarily the intention of how to develop the system.
</div>

Authors: {{ page.author }}
Date Written: {{ page.date_written }}
Last Modified: {% if page.last_modified %}{{ page.last_modified }}{% else %}{{ page.date_written }}{% endif %}
---

In ROS there are two types of Remote Procedure Call (RPC) primitives. ROS Services are basic request-response style RPCs, while ROS Actions additionally are preemptible and offer feedback while requests are being processed.

> 在 ROS 中有两种远程过程调用(RPC)原语。ROS 服务是基本的请求-响应式 RPC，而 ROS 动作另外可以被抢占，并在处理请求时提供反馈。

## Ideal System

It is useful to consider the ideal system to understand how it relates to the current ROS 1 system and how a new system could work. An ideal RPC system would have the qualities laid out in the following paragraphs.

> 考虑理想系统有助于了解它与当前 ROS 1 系统之间的关系以及新系统如何运作。理想的 RPC 系统应具备以下段落中所述的特性。

### Asynchronous API

An asynchronous API allows alternative threading models and is in general more flexible than a synchronous API, which can always be implemented on top of asynchronous API. Doing the reverse (building an asynchronous API on top of a synchronous API) is harder and likely less efficient.

> 异步 API 允许使用替代的线程模型，并且比同步 API 更加灵活。同步 API 可以在异步 API 之上实现，但是相反的（在同步 API 之上构建异步 API）则更困难，也可能效率更低。

### Timeouts

If a service provider hangs or otherwise does not return correctly, then a calling thread may be hung indefinitely. Having a timeout allows for recovery behavior in the case of failure conditions besides a dropped connection, allowing the user to choose to continue waiting for another timeout cycle or abort the request.

> 如果服务提供商挂起或其他方式无法正常返回，调用线程可能会被无限期挂起。设置超时允许在连接断开之外的失败条件下恢复行为，让用户可以选择继续等待另一个超时周期或中止请求。

### Preemptibility

Preemption is a desirable feature whenever there may be long-running or non-deterministically running remote procedures. Specifically, we want the ability to preempt a long-running procedure with either a timeout on synchronous requests or an explicit call to cancel on asynchronous requests. Preemptibility is a required feature for the concept of Actions to be implemented (which is one reason that Actions are built on asynchronous ROS Messages instead of synchronous ROS Services).

> 预占用是一种可取的特性，可用于长时间运行或非确定性运行的远程过程。具体而言，我们希望能够使用超时或显式取消调用来预占用长时间运行的过程。预占用是实现操作的必需特性（这也是操作基于异步 ROS 消息而不是同步 ROS 服务的一个原因）。

### Feedback

In order to effectively use preemption without a timeout, periodic or procedural feedback is usually required. Feedback can be provided via an external mechanism such as an implicitly related publish-subscribe channel. Feedback is also central to the concept of Actions in ROS.

> 为了有效地使用没有超时的抢占，通常需要定期或程序反馈。反馈可以通过外部机制提供，如隐式相关的发布订阅通道。反馈也是 ROS 中 Action 概念的核心。

### Reliable Transport

It is important that the system cannot get into an undetermined state if there is packet loss. If a request or response is never received by either side the system must be able to notice this loss, then recover and/or inform the user in some way. In ROS 1, this lack of reliability has been a problem for ROS Actions, e.g., when they are used over lossy wireless links.

> 重要的是，如果有数据包丢失，系统不能进入不确定的状态。如果任何一方都没有收到请求或响应，系统必须能够注意到这种丢失，然后恢复并/或以某种方式通知用户。在 ROS 1 中，当 ROS Actions 在有损无线链路上使用时，这种可靠性的缺失一直是一个问题。

### Logging and Introspection

When logging a ROS 1 system (e.g., using `rosbag`), recording data transmitted on topics is insufficient to capture any information about service calls. Because service calls are conceptually point to point, rather than broadcast, logging them is difficult. Still, it should be possible to efficiently record some level of detail regarding RPC interactions, such that they could be later played back in some manner (though it is not clear exactly how playback would work).

> 当记录一个 ROS 1 系统（例如，使用`rosbag`）时，仅记录发布在主题上的数据是不足以捕获有关服务调用的任何信息的。由于服务调用在概念上是点对点而不是广播，因此记录它们是困难的。尽管如此，应该有可能有效地记录有关 RPC 交互的一些细节，以便它们可以以某种方式回放（尽管尚不清楚回放究竟如何工作）。

## Proposed Approach

The features outlined above are desirable but if provided as a monolithic implementation will be much more complicated than necessary for most use cases. E.g., feedback is not always required, but in a monolithic system it would always be an exposed part of the API. We propose four levels of abstraction into which the above features can be sorted, with each higher level providing more functionality to the user.

> 以上概述的功能是可取的，但如果以单一实现的方式提供，对于大多数用例而言，将比必要的复杂得多。例如，反馈不总是必需的，但在单一系统中，它总是 API 的一个公开部分。我们提出了四个抽象层次，将上述功能分类，每个更高层次为用户提供更多功能。

![ROS RPC Higherarchy](/img/ros_rpc_design/rpc_diagram.png)

### Plain RPC

The Plain RPC API is expected to be able to be leveraged from one or more externally developed RPC libraries. We expect several libraries to meet the minimum requirements and aim to make them interchangeable.

> 我们期望有若干个库能够满足最低要求，并且能够互换。简易 RPC API 预期能够从一个或多个外部开发的 RPC 库中获得利用。

### ROS Asynchronous RPC API

The ROS Asynchronous RPC API will provide the lowest level of abstraction. It will provide a callback-based API with a timeout. It will utilize the Plain RPC API to do the communication as well as provide reliable communications either by leveraging the Plain RPC's capabilities or providing a layer on top of them with message acknowledgments.

> ROS 异步 RPC API 将提供最低级别的抽象。它将提供基于回调的 API，并带有超时功能。它将利用 Plain RPC API 进行通信，并提供可靠的通信，要么利用 Plain RPC 的功能，要么在其上提供消息确认层。

For logging/introspection purposes the RPC Server instance might publish all incoming requests and outgoing responses on topics inside the namespace of the service.

> 为了日志/内省的目的，RPC 服务器实例可能会在服务命名空间内的主题上发布所有传入请求和传出响应。

### ROS Preemptible RPC API

The ROS preemptible RPC API will extend the Asynchronous API to enable preemption of an RPC in progress using a unique identifier (UID). This UID will be provided by the initial request method.

> ROS 可抢占 RPC API 将扩展异步 API，以使用唯一标识符（UID）抢占正在进行的 RPC。此 UID 将由初始请求方法提供。

### ROS Action RPC API (Not Affecting RPC Protocol)

The feedback topic can be isolated to a separate topic, which avoids integrating the feedback into the core RPC implementation. The ROS Action RPC API will extend the preemptible RPC API to provide a feedback channel via published ROS topic. This can be built on top of the preemptible RPC API with the PubSub API thus isolating it from the RPC design.

> 反馈主题可以分离到一个单独的主题，以避免将反馈集成到核心 RPC 实现中。ROS Action RPC API 将扩展可抢占 RPC API，以通过发布的 ROS 主题提供反馈通道。这可以建立在可抢占 RPC API 的 PubSub API 之上，从而将其与 RPC 设计隔离开来。

### ROS Synchronous RPC API (Not Affecting RPC Protocol)

For each of the above Asynchronous APIs a thin wrapper can be built on top to provide a single function-based interface for ease of use. It will block until a response is returned or the timeout is reached. If wrapping a preemptible RPC, it will both timeout on the user side as well as preempt the remote side. This will just be a thin layer on top of the Asynchronous API requiring no additional features of the core RPC protocol.

> 对于上面的每个异步 API，都可以在上面构建一个薄的封装层，以提供基于单个函数的界面，以便于使用。它将阻塞，直到收到响应或达到超时时间。如果封装一个可抢占的 RPC，它将在用户端超时，同时也会抢占远程端。这只是在异步 API 之上的一个薄层，不需要 RPC 协议的其他功能。

## Technical Issues

There are some issues with the above proposed approach, which are outlined below.

> 以上提出的方法存在一些问题，如下所示。

### Visibility of Unique Identifiers

UIDs are generally necessary for asynchronous communications to pair the requests and the responses. There are possible ways to build this without a UID embedded in the data type, however it will require some level of heuristics to do data association.

> UIDs 通常对于异步通信来说是必要的，以便将请求和响应配对。虽然可以在不嵌入数据类型中的 UID 的情况下构建这种功能，但是需要一定程度的启发式算法来完成数据关联。

There are two options: (i) require the user to embed the UID into the message, or (ii) add those fields automatically at message-generation time. It is unclear how this choice affects the user's ability to introspect the messages.

> 有两个选项：（i）要求用户将 UID 嵌入消息，或（ii）在消息生成时自动添加这些字段。目前尚不清楚这种选择如何影响用户对消息的内省能力。

This also introduces issues when trying to record and potentially play back Service or Actions.

> 这也会在尝试记录和潜在播放服务或操作时引发问题。

### Action Files

Should there be a separate `.action` file type? Or should it be more like a `.srv` + `.msg` pair? This is highly influenced by the way UIDs are handled.

> 应该有一个单独的`.action`文件类型吗？还是应该像`.srv` + `.msg`配对一样？这取决于 UIDs 的处理方式。

### Logging

Recorded service calls can not be played back generically like topics because the service client will not handle it because it will not be associated with a request. There could be some test service servers which can respond with similar queries, however it is not clearly defined.

> 录制的服务调用不能像主题一样通用地播放，因为服务客户端不会处理它，因为它不会与请求相关联。可能有一些测试服务服务器可以响应类似的查询，但尚未明确定义。

Logging of RPCs is still valuable for debugging and introspection. It would be valuable to have a view of events that happened in sequence as well as their content. It should be possible to associate recorded request and response pairs, which raises the question of how to embed this association without significantly affecting the user's API.

> 日志记录 RPC 仍然对调试和内省有很大价开。有一个事件发生顺序以及它们的内容的视图将是非常有价值的。应该可以将记录的请求和响应对相关联，这就提出了如何在不显著影响用户的 API 的情况下嵌入这种关联的问题。

This is a generic issue with logging and affects potentially all logging and should be captured in a separate article. It might be possible to pad communications with debugging data.

> 这是一个关于日志记录的通用问题，可能会影响所有的日志记录，应该在另一篇文章中进行捕获。可能可以用调试数据填充通信。

The above UIDs may be only locally unique (client-specific for instance). For logging, UIDs need to be unique within the entire ROS instance to support debugging.

> 以上 UID 可能只是局部唯一的（例如特定于客户端）。为了记录，UID 需要在整个 ROS 实例中是唯一的，以支持调试。

### Collapse Preemptible and Asynchronous

These two types are very similar and limiting the variants on the API might be easier for the user. If the UID must be generated/embedded into the protocol, then it should be embedded into all calls, which will help with logging. For the non-preemptible case the implementation can simply not instantiate the state machine and preemption mechanisms.

> 这两种类型非常相似，对 API 的变量进行限制可能会更容易让用户使用。如果 UID 必须生成/嵌入到协议中，那么它应该被嵌入到所有调用中，这将有助于日志记录。对于不可抢占的情况，实现可以简单地不实例化状态机和抢占机制。

### caller_id availability

There are use cases when the concept of `caller_id` is valuable. Users of ROS Actions would sometimes use it to provide connection-based information. By default the anonymous publish-subscribe mechanism is to not provide the `caller_id`. In the current implementation the `caller_id` is actually the node name, which is ambiguous in cases like nodelets. Providing a mechanism for declaring the `caller_id` might be helpful, but would require a lot of tools to parameterize and remap in order to gain the full benefit.

> 在有些情况下，“caller_id”这一概念很有价值。ROS Actions 的用户有时会用它来提供基于连接的信息。默认情况下，匿名发布订阅机制不会提供“caller_id”。在当前的实现中，“caller_id”实际上是节点名称，在节点的情况下是模糊的。提供申明“caller_id”的机制可能会有帮助，但需要很多工具来参数化和重新映射，以获得最大的好处。

One possible solution is to write a spec where a field that matches `[CallerID caller_id]` would be automatically substituted by a publisher if embedded into a message to provide this specifically where valuable.

> 一种可能的解决方案是编写一个规格，其中匹配 `[CallerID caller_id]` 的字段将被发布者自动替换，如果嵌入到消息中，就可以特别提供有价值的信息。

### Do not bundle feedback at core level

If namespace remapping works, feedback could simply be a topic which is in the same namespace as a peer. This would enable multiple feedback topics of differing types and frequencies. Action API classes could be provided on top of the core infrastructure to bundle an RPC with a feedback.

> 如果命名空间重映射有效，反馈可以只是一个和同行在同一个命名空间的主题。这将使多种不同类型和频率的反馈主题成为可能。可以在核心基础设施之上提供动作 API 类，以将 RPC 与反馈绑定在一起。
