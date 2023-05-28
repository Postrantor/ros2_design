---
tip: translate by openai@2023-05-28 11:09:50
...
---
    layout: default
    title: RPC API design in ROS
    permalink: articles/ros_rpc.html
    abstract:
      This article is an exploration of possible design patterns for the next generation of ROS Remote Procedure Call interfaces.
      We focus here on specifying the user API and leave the implementation unspecified.
      It is expected that there are one or more RPC implementations which can be used, such as Apache Thrift, ROS RPC, or MsgPack.
    author: "[Tully Foote](https://github.com/tfoote)"
    date_written: 2014-06
    last_modified: 2019-05
    published: true
    summary: This article is out-of-date.
      It was written at a time before decisions were made to use DDS and RTPS as the underlying communication standards for ROS 2.
      It represents an idealistic understanding of what RPC and "actions" should be like in ROS.
      It can be considered memoranda and not necessarily the intention of how to develop the system.
    Authors: {{ page.author }}
    Date Written: {{ page.date_written }}
    Last Modified: {% if page.last_modified %}{{ page.last_modified }}{% else %}{{ page.date_written }}{% endif %}
---


In ROS there are two types of Remote Procedure Call (RPC) primitives.

> 在ROS中有两种类型的远程过程调用（RPC）原语。

ROS Services are basic request-response style RPCs, while ROS Actions additionally are preemptible and offer feedback while requests are being processed.

> ROS服务是基本的请求-响应式RPC，而ROS操作另外还可以支持抢占式，并且在处理请求时提供反馈。

## Ideal System


It is useful to consider the ideal system to understand how it relates to the current ROS 1 system and how a new system could work.

> 考虑理想系统有助于了解它与当前ROS 1系统之间的关系，以及新系统如何运作。

An ideal RPC system would have the qualities laid out in the following paragraphs.

> 理想的RPC系统应具备以下段落中列出的特性。

### Asynchronous API


An asynchronous API allows alternative threading models and is in general more flexible than a synchronous API, which can always be implemented on top of asynchronous API.

> 异步API允许使用替代的线程模型，比同步API更灵活，而同步API总是可以在异步API之上实现。

Doing the reverse (building an asynchronous API on top of a synchronous API) is harder and likely less efficient.

> 做反向操作（在同步API上构建异步API）更难，效率可能更低。

### Timeouts


If a service provider hangs or otherwise does not return correctly, then a calling thread may be hung indefinitely.

> 如果服务提供商挂起或以其他方式无法正确返回，那么调用线程可能会被无限期挂起。

Having a timeout allows for recovery behavior in the case of failure conditions besides a dropped connection, allowing the user to choose to continue waiting for another timeout cycle or abort the request.

> 设置超时时间可以在除了连接断开的情况下，对失败情况进行恢复操作，允许用户选择继续等待另一个超时周期，或者中止请求。

### Preemptibility


Preemption is a desirable feature whenever there may be long-running or non-deterministically running remote procedures.

> 预占用是一个可取的功能，只要可能会有长时间运行或不确定性运行的远程程序。

Specifically, we want the ability to preempt a long-running procedure with either a timeout on synchronous requests or an explicit call to cancel on asynchronous requests.

> 具体来说，我们希望能够在同步请求上设置超时或在异步请求上显式调用取消来抢占长时间运行的程序。

Preemptibility is a required feature for the concept of Actions to be implemented (which is one reason that Actions are built on asynchronous ROS Messages instead of synchronous ROS Services).

> 预占用是实现行动概念所必需的功能（这也是行动建立在异步ROS消息上而不是同步ROS服务上的一个原因）。

### Feedback


In order to effectively use preemption without a timeout, periodic or procedural feedback is usually required.

> 为了有效地使用没有超时、周期或程序反馈的抢占，通常需要定期或程序性反馈。

Feedback can be provided via an external mechanism such as an implicitly related publish-subscribe channel.

> 反馈可以通过外部机制，如隐含相关的发布订阅通道提供。

Feedback is also central to the concept of Actions in ROS.

> 反馈也是ROS中行动概念的核心部分。

### Reliable Transport


It is important that the system cannot get into an undetermined state if there is packet loss.

> 重要的是，如果有数据包丢失，系统不能进入未确定的状态。

If a request or response is never received by either side the system must be able to notice this loss, then recover and/or inform the user in some way.

> 如果一方没有收到请求或响应，系统必须能够注意到这种损失，然后恢复和/或以某种方式通知用户。

In ROS 1, this lack of reliability has been a problem for ROS Actions, e.g., when they are used over lossy wireless links.

> 在ROS 1中，这种可靠性的缺失一直是ROS Actions的问题，例如，当它们被用于有损无线链路时。

### Logging and Introspection


When logging a ROS 1 system (e.g., using `rosbag`), recording data transmitted on topics is insufficient to capture any information about service calls.

> 当记录ROS 1系统（例如使用`rosbag`）时，只记录发布到主题上的数据是不足以捕获有关服务调用的任何信息的。

Because service calls are conceptually point to point, rather than broadcast, logging them is difficult.

> 因为服务调用在概念上是点对点的，而不是广播，所以记录它们很困难。

Still, it should be possible to efficiently record some level of detail regarding RPC interactions, such that they could be later played back in some manner (though it is not clear exactly how playback would work).

> 尽管如此，应该可以有效地记录下关于RPC交互的一些细节，以便以某种方式稍后回放（尽管目前尚不清楚回放的具体工作原理）。

## Proposed Approach


The features outlined above are desirable but if provided as a monolithic implementation will be much more complicated than necessary for most use cases.

> 以上提到的功能很理想，但如果作为一个整体实现，对于大多数用例来说，要比必要的复杂得多。

E.g., feedback is not always required, but in a monolithic system it would always be an exposed part of the API.

> 例如，反馈不是总是必需的，但在一个大型系统中，它总是API的一个公开部分。

We propose four levels of abstraction into which the above features can be sorted, with each higher level providing more functionality to the user.

> 我们提出了四个抽象级别，将上述功能分类，每个较高的级别为用户提供更多的功能。

![ROS RPC Higherarchy](/img/ros_rpc_design/rpc_diagram.png)

### Plain RPC


The Plain RPC API is expected to be able to be leveraged from one or more externally developed RPC libraries.

> 预期能够从一个或多个外部开发的RPC库中利用普通RPC API。

We expect several libraries to meet the minimum requirements and aim to make them interchangeable.

> 我们期望几个库能够满足最低要求，并致力于使它们互换。

### ROS Asynchronous RPC API


The ROS Asynchronous RPC API will provide the lowest level of abstraction.

> ROS 异步 RPC API 将提供最低级别的抽象。

It will provide a callback-based API with a timeout.

> 它将提供一个基于回调的API，并具有超时功能。

It will utilize the Plain RPC API to do the communication as well as provide reliable communications either by leveraging the Plain RPC's capabilities or providing a layer on top of them with message acknowledgments.

> 它将利用普通RPC API进行通信，并通过利用普通RPC的功能或在其上提供消息确认层来提供可靠的通信。


For logging/introspection purposes the RPC Server instance might publish all incoming requests and outgoing responses on topics inside the namespace of the service.

> 为了记录/内省的目的，RPC服务器实例可能会在服务的命名空间内的主题上发布所有传入的请求和传出的响应。

### ROS Preemptible RPC API


The ROS preemptible RPC API will extend the Asynchronous API to enable preemption of an RPC in progress using a unique identifier (UID).

> ROS可抢占RPC API将扩展异步API，以使用唯一标识符（UID）抢占正在进行的RPC。

This UID will be provided by the initial request method.

> 这个UID将由初始请求方法提供。

### ROS Action RPC API (Not Affecting RPC Protocol)


The feedback topic can be isolated to a separate topic, which avoids integrating the feedback into the core RPC implementation.

> 反馈主题可以分离出来成为一个单独的主题，这样可以避免将反馈整合到核心RPC实现中。

The ROS Action RPC API will extend the preemptible RPC API to provide a feedback channel via published ROS topic.

> ROS Action RPC API 将扩展可抢占的RPC API，通过发布的ROS主题提供反馈信道。

This can be built on top of the preemptible RPC API with the PubSub API thus isolating it from the RPC design.

> 这可以基于可抢占的RPC API和PubSub API构建，从而将其与RPC设计隔离开来。

### ROS Synchronous RPC API (Not Affecting RPC Protocol)


For each of the above Asynchronous APIs a thin wrapper can be built on top to provide a single function-based interface for ease of use.

> 对于上述每个异步API，可以在上面构建一个薄的包装层，以提供基于单个函数的易用界面。

It will block until a response is returned or the timeout is reached.

> 它会一直阻塞，直到收到响应或达到超时。

If wrapping a preemptible RPC, it will both timeout on the user side as well as preempt the remote side.

> 如果包裹一个可抢占的RPC，它既会在用户端超时，也会抢占远程端。

This will just be a thin layer on top of the Asynchronous API requiring no additional features of the core RPC protocol.

> 这只是一层薄薄的封装，在异步API之上，不需要核心RPC协议的额外特性。

## Technical Issues


There are some issues with the above proposed approach, which are outlined below.

> 以上提出的方法存在一些问题，如下所述。

### Visibility of Unique Identifiers


UIDs are generally necessary for asynchronous communications to pair the requests and the responses.

> UIDs通常对于异步通信来说是必要的，以便将请求和响应配对。

There are possible ways to build this without a UID embedded in the data type, however it will require some level of heuristics to do data association.

> 有可能的方式可以在数据类型中构建这个没有UID的，但是这需要一定程度的启发式算法来完成数据关联。


There are two options: (i) require the user to embed the UID into the message, or (ii) add those fields automatically at message-generation time.

> 有两个选项：（i）要求用户将UID嵌入消息中，或（ii）在消息生成时自动添加这些字段。

It is unclear how this choice affects the user's ability to introspect the messages.

> 这个选择如何影响用户内省消息的能力尚不清楚。

This also introduces issues when trying to record and potentially play back Service or Actions.

> 这也会在尝试记录和潜在播放服务或操作时引发问题。

### Action Files


Should there be a separate `.action` file type?

> 是否应该有一种单独的`.action`文件类型？

Or should it be more like a `.srv` + `.msg` pair?

> 它应该更像一个`.srv` + `.msg`对吗？

This is highly influenced by the way UIDs are handled.

> 这受到UID处理方式的强烈影响。

### Logging


Recorded service calls can not be played back generically like topics because the service client will not handle it because it will not be associated with a request.

> 录制的服务调用不能像主题一样通用地播放，因为服务客户端不会处理它，因为它不会与请求相关联。

There could be some test service servers which can respond with similar queries, however it is not clearly defined.

> 可能有一些测试服务服务器可以响应类似的查询，但没有明确定义。


Logging of RPCs is still valuable for debugging and introspection.

> 记录RPC仍然对调试和内省有重要价值。

It would be valuable to have a view of events that happened in sequence as well as their content.

> 有一个按时间顺序展示事件及其内容的视图将会非常有价值。

It should be possible to associate recorded request and response pairs,

> 应该可以关联记录的请求和响应对。

which raises the question of how to embed this association without significantly affecting the user's API.

> .

这就引出了一个问题，即如何在不显著影响用户的API的情况下嵌入这种关联关系。


This is a generic issue with logging and affects potentially all logging and should be captured in a separate article.

> 这是一个关于日志的通用问题，可能会影响所有日志，应该在单独的文章中捕获。

It might be possible to pad communications with debugging data.

> 可能有可能在通信中添加调试数据。


The above UIDs may be only locally unique (client-specific for instance).

> 以上的UID可能只是局部唯一的（例如客户端特定的）。

For logging, UIDs need to be unique within the entire ROS instance to support debugging.

> 为了记录，UID需要在整个ROS实例中是唯一的，以支持调试。

### Collapse Preemptible and Asynchronous


These two types are very similar and limiting the variants on the API might be easier for the user.

> 这两种类型非常相似，限制API上的变体可能会更容易让用户使用。

If the UID must be generated/embedded into the protocol,

> 如果必须在协议中生成/嵌入UID，

then it should be embedded into all calls, which will help with logging.

> 那么它应该被嵌入到所有的调用中，这将有助于记录日志。

For the non-preemptible case the implementation can simply not instantiate the state machine and preemption mechanisms.

> 在不可抢占的情况下，实现可以简单地不实例化状态机和抢占机制。

### caller_id availability


There are use cases when the concept of `caller_id` is valuable.

> 有些情况下，`caller_id`这个概念是有价值的。

Users of ROS Actions would sometimes use it to provide connection-based information.

> 用户有时会使用ROS Actions来提供基于连接的信息。

By default the anonymous publish-subscribe mechanism is to not provide the `caller_id`.

> 默认情况下，匿名发布-订阅机制不提供`caller_id`。

In the current implementation the `caller_id` is actually the node name, which is ambiguous in cases like nodelets.

> 在当前实现中，`caller_id`实际上是节点名称，在像节点集这样的情况下就不清楚了。

Providing a mechanism for declaring the `caller_id` might be helpful, but would require a lot of tools to parameterize and remap in order to gain the full benefit.

> 提供一种声明`caller_id`的机制可能有帮助，但需要很多工具来参数化和重新映射，才能充分发挥作用。


One possible solution is to write a spec where a field that matches `[CallerID caller_id]` would be automatically substituted by a publisher if embedded into a message to provide this specifically where valuable.

> 一种可能的解决方案是编写一个规范，其中，如果将字段[CallerID caller_id]嵌入消息中，发布者将自动替换它，以特定地提供有价值的内容。

### Do not bundle feedback at core level


If namespace remapping works, feedback could simply be a topic which is in the same namespace as a peer.

> 如果命名空间重映射成功，反馈可以简单地是一个与同行处于同一个命名空间中的主题。

This would enable multiple feedback topics of differing types and frequencies.

> 这将使多种不同类型和频率的反馈主题成为可能。

Action API classes could be provided on top of the core infrastructure to bundle an RPC with a feedback.

> 可以在核心基础设施之上提供动作API类，以将RPC与反馈捆绑在一起。
