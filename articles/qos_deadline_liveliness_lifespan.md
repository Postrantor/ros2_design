---
tip: translate by openai@2023-05-29 08:30:16
layout: default
title: ROS QoS - Deadline, Liveliness, and Lifespan
permalink: articles/qos_deadline_liveliness_lifespan.html
abstract:
  This article makes the case for adding Deadline, Liveliness, and Lifespan QoS settings to ROS. It outlines the requirements and explores the ways it could be integrated with the existing code base.
author: '[Nick Burek](https://github.com/nburek)'
date_written: 2019-09
last_modified: 2019-09
published: true
categories: Middleware
date: February 13th 2019
Authors: {{ page.author }}
Date Written: {{ page.date_written }}
Last Modified: {% if page.last_modified %}{{ page.last_modified }}{% else %}{{ page.date_written }}{% endif %}
---
> [!NOTE]
> deadline、liveliness 对应的 pub/sub 双方
> lifespan 是针对 sub 的，超过这个限制将不再接受消息
> deadline 是对 pub/sub 双方进行通知的
> liveliness 是可以对第三方进行通知的

---

Glossary:

- DDS - Data Distribution Service
- RTPS - Real-Time Publish Subscribe
- QoS - Quality of Service
- Service Client - Also referred to as just Client, refers to an application that connects to a ROS Service to send requests and receive responses.
- Service Server - Also referred to as just Server, refers to the application that is running a ROS Service that receives requests and sends responses.

> DDS - 数据分发服务
> RTPS - 实时发布订阅
> 质量服务(QoS)
> 服务客户端-也称为客户端，是指一个连接 ROS 服务以发送请求并接收响应的应用程序。
> 服务器 - 也称为服务器，是指正在运行 ROS 服务的应用程序，它接收请求并发送响应。

## Existing ROS QoS Settings

While DDS provides many settings to enable fine-grained control over the Quality of Service (QoS) for entities, ROS only provides native support for a handful of them. ROS users are able to specify the History, Depth, Reliability, and Durability via a QoS configuration struct when they create Publishers, Subscribers, etc.

> DDS 提供了许多设置来控制服务质量(QoS)，但 ROS 只提供本机支持的少数设置。ROS 用户可以在创建发布者、订阅者等时，通过 QoS 配置结构指定历史、深度、可靠性和持久性。

This leaves a lot of QoS settings that can only be set if DDS vendor can load additional default settings via a configuration file. If a user wants to hook their code into these additional QoS settings then they would need to get a reference to the rmw implementation and program against the vendor specific API.

> **这就留下了许多 QoS 设置，只有当 DDS 供应商可以通过配置文件加载额外的默认设置时才能设置。如果用户想将他们的代码连接到这些额外的 QoS 设置，那么他们需要获取 rmw 实现的引用，并根据特定供应商的 API 编程。**

> [!NOTE]
> 这里提到了如果想要使用额外的 QoS 设置该如何操作，"need to get a reference to the rmw implementation and program against the vendor specific API"
> 意思是在 RMW 层调用底层 DDS 供应商的 API，进一步满足用户的需求？！

Without the abstraction layer provided by ROS their code becomes less portable.

> 没有 ROS 提供的抽象层，他们的代码变得不那么便携。

## New QoS Settings

As users start to build more robust applications, they are going to need more control over when data is being delivered and information about when the system is failing. To address these needs it was proposed that we start by adding support for the following new DDS QoS settings.

> 随着用户开始构建更强大的应用程序，他们需要更多的控制权来控制数据何时被传递以及系统何时出现故障的信息。为了解决这些需求，建议我们首先添加对以下新 DDS QoS 设置的支持。

### Deadline

The deadline policy establishes a contract for the amount of time allowed between messages. For Subscriptions it establishes the maximum amount of time allowed to pass between receiving messages. For Publishers it establishes the maximum amount of time allowed to pass between sending messages. Topics will support deadline tracking only up to the rmw layer. This means that a deadline will be considered missed if the rmw layer has not received a message within the specified time. In order for a Subscriber to listen to a Publisher's Topic the deadline that they request must be greater than, or equal to, the deadline set by the Publisher. A deadline time of 0 will disable the deadline tracking. The default deadline time will be 0.

> 截止期限(deadline)策略设定了在消息之间允许的时间量。对于订阅，它**设定了在接收消息之间允许的最长时间量**。对于发布者，它设定了在**发送消息之间允许的最长时间量**。
> 主题仅支持 rmw 层的截止期限跟踪。这意味着，如果在指定的时间内 rmw 层没有收到消息，则截止期限将被视为已错过。为了使订阅者能够收听发布者的主题，他们请求的截止期限必须**大于或等于发布者设定的截止期限**。
> **截止期限为 0 将禁用截止期限跟踪。默认截止期限为 0。**

> [!NOTE]
> deadline_sub 需要大于 deadline_pub

### Liveliness

The liveliness policy establishes a contract for how entities report that they are still alive. For Subscriptions it establishes the level of reporting that they require from the Publishers to which they are subscribed. For Publishers it establishes the level of reporting that they will provide to Subscribers that they are alive.

> 活力(Liveliness)策略建立了一份合同，用于规定实体如何报告他们仍然活着。对于订阅，它建立了它们所订阅的出版商所需的报告水平。对于出版商，它建立了它们将向订阅者提供的报告水平，以表明它们仍然活着。

Topics will support the following levels of liveliness:

- LIVELINESS_SYSTEM_DEFAULT - Use the ROS specified default for liveliness (which is LIVELINESS_AUTOMATIC).
- LIVELINESS_AUTOMATIC - The signal that establishes a Topic is alive comes from the ROS rmw layer.
- LIVELINESS_MANUAL_BY_TOPIC - The signal that establishes a Topic is alive is at the Topic level. Only publishing a message on the Topic or an explicit signal from the application to assert liveliness on the Topic will mark the Topic as being alive.

> - 使用 ROS 指定的默认活力(即 LIVELINESS_AUTOMATIC)来设置 LIVELINESS_SYSTEM_DEFAULT。
> - LIVELINESS_AUTOMATIC - ROS rmw 层发出建立主题的信号。
> - LIVELINESS_MANUAL_BY_TOPIC - 信号建立一个主题是活跃的是在主题级别。**只有在主题上发布消息或应用程序发出明确的信号以确认主题的活跃性**，才会将主题标记为活跃的。

In order for a Subscriber to listen to a Publisher's Topic the level of liveliness tracking they request must be equal or less verbose than the level of tracking provided by the Publisher and the time until considered not alive set by the Subscriber must be greater than the time set by the Publisher.

> 为了让订阅者听到发布者的主题，他们要求的活跃程度追踪必须与发布者提供的追踪程度相等或更少，而**订阅者设定的被认为不活跃的时间必须大于发布者设定的时间**。

> [!NOTE]
> liveliness_sub 需要大于 liveliness_pub

> [!NOTE]
> 从 cyclone dds 的角度来看，这里的 liveliness 使用的是 DDS 中 listener 机制。

### Lifespan

The lifespan policy establishes a contract for how long a message remains valid. For Subscriptions it establishes the length of time a message is considered valid, after which time it will not be received. For Publishers it establishes the length of time a message is considered valid, after which time it will be removed from the Topic history and no longer sent to Subscribers. A lifespan time of 0 will disable the lifespan tracking.

> 生命周期(lifespan)策略建立了一个协议，用来**确定消息有效期的长度**。对于订阅者，它确定了消息被认为有效的时间长度，**超过这个时间，消息将不会被接收**。对于发布者，它确定了消息被认为有效的时间长度，超过这个时间，消息将从主题历史中删除，不再发送给订阅者。生命周期时间为 0 将禁用生命周期跟踪。

The default lifespan time will be 0.

> 默认的寿命时间将为 0.

### DDS QoS Relation

These new policies are all based on the DDS QoS policies, but they do not require DDS in order for an rmw implementation to support them. More detailed information on the DDS specifics of these policies can be found below in Appendix A.

> 这些新策略都基于 DDS QoS 策略，但是它们不要求 DDS，以便 rmw 实现支持它们。有关这些策略的 DDS 特定信息，请参阅附录 A。

### ROS Changes

These are the various changes that would be needed within ROS in order to natively support Deadline and Liveliness.

> 这些是需要在 ROS 中本地支持 Deadline 和 Liveliness 所需要的各种变更。

#### Resource Status Event Handler

Both the Deadline and Liveliness policies generate events from the rmw layer of which the application will need to be informed. For Deadlines, the Subscriber receives event notifications if it doesn't receive anything within the deadline and the Publisher receives event notifications if it doesn't publish anything within the deadline. For Liveliness, Subscribers receive events when there are no longer any Publishers alive to assert the topic is alive. Services generate similar events when Clients and Servers violate the defined policies. Both of these fall under a category of "Resource Status Events".

> 两个截止日期和活跃度策略都从 rmw 层生成事件，应用程序需要了解这些事件。
> 对于截止日期，如果**订阅者在截止日期内没有收到任何东西，就会收到事件通知**，如果发布者在截止日期内没有发布任何东西，也会收到事件通知。
> 对于活跃度，当没有**发布者声明主题仍然活跃时，订阅者会收到事件**。
> 当客户端和服务器违反定义的策略时，服务也会生成类似的事件。这些都属于“资源状态事件”的范畴。

To handle these notifications, new callback functions can be provided by the user that will be called any time an event occurs for a particular Topic. It will receive as a parameter a struct value that contains the information about the event such as when the event took place and other metadata related to the event. These callback functions would be optionally provided by the user's application when it calls the create function for publishers and subscribers. The constructors and create functions will be overloaded to make this new handler optional.

> **为了处理这些通知，用户可以提供新的回调函数，每当特定主题发生事件时，就会调用这些回调函数。**
> 它将作为参数接收一个包含有关事件的信息的结构体值，例如事件发生的时间以及与事件相关的其他元数据。当应用程序调用发布者和订阅者的创建函数时，这些回调函数是可选的。构造函数和创建函数将被重载以使新的处理程序变得可选。

The status event handlers will not be called once for every status event. Instead, the event handlers will only be called if there is an status change event that has not yet been handled when the Executor that services the callbacks checks.

> **状态事件处理程序不会每次状态事件调用一次。相反，只有在服务回调的执行程序检查时发现尚未处理的状态变更事件时，才会调用事件处理程序。**

> [!NOTE]
> 可以将这些事件作为一种状态，触发自定义的回调。

#### QoS Struct

In the current version of ROS there is a single QoS struct that is used to specify the QoS policy whenever a Publisher and Subscriber are created. With these new QoS settings the supported set of QoS policies for Topics and Services diverges. Despite this, we are going to stick with a single struct for both Topics and Services instead of switching to two different struct types in order to keep the changes to a minimum and maintain as much backwards compatibility as possible in the client library interfaces.

> 在当前版本的 ROS 中，**有一个单独的 QoS 结构**，用于在创建发布者和订阅者时指定 QoS 策略。使用这些新的 QoS 设置，**主题和服务的支持 QoS 策略集有所不同**。尽管如此，我们仍然会选择一个单一的结构来处理主题和服务，而不是切换到两种不同的结构类型，以尽量减少变化，并在客户端库接口中尽可能保持向后兼容性。

The existing QoS policy struct will have new fields added to specify the desired QoS settings for Deadline, Liveliness, and Lifespan. These new fields instances will be a combination of enum and time values.

> 现有的 QoS 策略结构将添加新字段，以指定 Deadline、Liveliness 和 Lifespan 的所需 QoS 设置。这些新字段实例将是**枚举和时间值的组合**。

#### Assert Liveliness Functions

New functions will need to be added that can be used by the application to explicitly assert liveliness. One function to assert liveliness at the Node level and one to assert it at the Topic level. While liveliness will also be implicitly assumed just based on sending messages, these functions will be used by the application to explicitly declare resources are alive. These functions will need to be implemented in every layer from the rmw up through the rcl and language specific client libraries, such as rclcpp.

> 新功能需要添加，可由应用程序使用，以明确断言活力(assert liveliness)。一个函数用于在**节点级别断言活力**，另一个用于在**主题级别断言活力**。虽然基于发送消息也会隐含地假定活力，但这些函数将被应用程序用于明确声明资源是活跃的。这些函数将**需要在从 rmw 到 rcl 的每一层以及语言特定的客户端库(如 rclcpp)中实现**。

#### `rcl_wait` and `rmw_wait`

The rcl layer is currently using a WaitSet in order to be informed of events from the rmw layer, such as incoming messages. These WaitSets contain lists of several types of conditions, such as timers firing or subscriptions receiving data. In order to support new Topic Status Events, a new type will be added to the existing WaitSet and the rmw layer will set them when these events occur.

> 目前，rcl 层正在使用 WaitSet 以获取来自 rmw 层的事件，例如传入的消息。这些 WaitSet 包含许多类型的条件列表，例如定时器触发或订阅接收数据。为了支持新的主题状态事件，将在现有的 WaitSet 中添加一种新类型，并且当这些事件发生时，rmw 层将设置它们。

> [!NOTE]
> 这个 waitset 需要详细理解，这里所谓的 RMW 层，实际上应该是 DDS 层中的 waitset

#### `rcl_take_status` and `rmw_take_status`

New functions called rcl_take_status and rmw_take_status will need to be added that can directly query the status for a Topic. It will operate in a similar manner to the rcl_take and rmw_take functions that are used to retrieve messages for a subscription. It will be used by the executors when they receive a notice via the waitset mentioned above that a resource has a new status event available.

> 新功能叫做 `rcl_take_status` 和 `rmw_take_status` 将需要添加，以便可以直接查询主题的状态。它的操作方式将类似于用于检索订阅的 rcl_take 和 rmw_take 功能。当执行者通过上面提到的 waitset 收到资源有新的状态事件可用的通知时，将使用它。

> [!NOTE]
> 按照这个提示，追到代码里面可以看到 rcl 和 rmw 层中对应的 event
> 如 cyclone dds 中的 `rmw_node.cpp:rmw_take_event()`

## RMW Vendor Support

All of these new QoS policies will need to be supported in the rmw implementations. As we add new QoS policies it is likely that not all rmw vendors will provide support for every QoS policy that ROS defines. This is especially true for non-DDS based implementations that do not already have native support for these features. Because of this we need a way for the application to know if the rmw vendor that is being used supports the specified policies at the specified levels. In this case, the rmw vendor should fail the operation by returning an error code that specifies that the requested QoS policy is not supported. The explicit failure will ensure the requesting application is receiving defined behavior and not operating under unexpected conditions.

> 所有这些**新的 QoS 策略都需要在 rmw 实现中得到支持**。随着我们添加新的 QoS 策略，**不是所有的 rmw 供应商都会提供 ROS 定义的每个 QoS 策略的支持。这对于不基于 DDS 的实现尤其如此**，因为它们没有原生支持这些功能。因此，我们需要一种方法，让应用程序知道正在使用的 rmw 供应商是否支持指定的策略在指定的级别。在这种情况下，rmw 供应商应该通过返回一个指定请求的 QoS 策略不受支持的错误代码来失败操作。显式的失败将确保请求应用程序正在接收定义的行为，而不是在意外条件下操作。

In addition to an application being prevented from running with an unsupported policy, it is useful for an application to be able to query what QoS policies the rmw vendor supports. The best way to do this is to provide an API that allows the application to check if a specific policy is supported and also get all the supported settings. The design and implementation of this API is out of scope for this document and should be considered for part of the future work.

> 此外，除了防止应用程序使用不受支持的策略运行外，应用程序能够查询 rmw 供应商支持的 QoS 策略也很有用。最好的方法是提供一个 API，允许应用程序检查特定策略是否受支持，以及获取所有受支持的设置。本文档不涉及此 API 的设计和实现，应将其作为未来工作的一部分考虑。

## FAQ

- How does the Deadline policy take into account the additional overhead of ROS (such as deserialization) when determining if a deadline was missed?

> - 在确定是否错过了截止日期时，截止日期策略如何考虑 ROS(如反序列化)的额外开销？

- As a simplification it is not going to attempt to take into account any ROS overhead. A deadline will be considered missed if the rmw layer does not receive a message by the deadline and not if the user application on top of ROS does not receive it by the deadline. A new deadline policy could be added later that takes this into account.

> - 为了简化，它不会尝试考虑任何 ROS 开销。**如果 rmw 层没有在截止日期前收到消息**，则认为期限已经错过，而不是用户应用程序在 ROS 上没有收到它的截止日期。稍后可以添加一个新的期限策略，以考虑到这一点。

> [!NOTE]
> 这里明确提到，是以 rmw 层中接收到消息为准，而不是 rclcpp 或者用户应用。

---

- Why will the callback not get called for every status change event instead of potentially combining events of the same type?

> 为什么回调函数**不会为每个状态更改事件而被调用，而是有可能将同一类型的事件合并**？

- Adding this functionality would require an additional buffer that would be used to store multiple events between servicing them. Additionally, the DDS API lends itself better to only being informed of the latest change and would require a realtime response to status change events so as to not miss a single event. This is not a one way door and we could change this later to allow buffering events without breaking backwards compatibility.

> 加入这个功能需要一个额外的缓冲区来存储多个事件在服务它们之间。此外，**DDS API 更适合只被告知最新的变化，并且需要实时响应状态变化事件**，以免错过任何一个事件。这不是一个单向门，我们可以稍后更改这个设置，以允许缓冲事件而不破坏向后兼容性。

> [!NOTE] > **只被告知最新的变化**，这句话该如何更好的理解？

---

- How do these QoS policies impact Actions and Services?

> 这些 QoS 策略如何影响操作和服务？

- The initial implementation does not support Actions and Services as there are more complex subtleties to how these concepts natively support these QoS features. In the future work section below we explore some ways that Services could implement these policies.

> **初始实现不支持动作和服务**，因为这些概念本身支持这些 QoS 特性的细微差别更复杂。在下面的未来工作部分，我们探讨了服务如何实现这些策略的一些方法。

---

- How are these QoS policies affected by DDS topic instances?

> 这些 QoS 策略受 DDS 主题实例的影响有多大？

- While all of these policies can and will eventually support keyed instances, this document does not focus on the details of how as it is highly dependent on the design for ROS 2 to support keyed messages in general.

> 虽然所有这些策略都可以并且最终会支持带键的实例，但本文档并不关注 ROS 2 如何支持带键消息的细节，因为这严重依赖于设计。

## Future Work

Actions and Services were considered out of scope for the initial implementation. Here we detail how Services could potentially support these QoS policies in the future.

> 行动和服务被认为不在最初实施范围之内。在这里，我们详细说明服务未来如何支持这些 QoS 策略。

### Deadline

For Service Servers it establishes the maximum amount of time allowed to pass between receiving a request and when a response for that request is sent. For Service Clients it establishes the maximum amount of time allowed to pass between sending a request and when a response for that request is received. Services will support deadline tracking only up to the rmw layer. This means the time a request is started is marked when the request reaches the rmw layer and the time at which it is finished is when the response message reaches the rmw layer. A Service Client will **not** be prevented from making a request to a Service Server if the Server provides a deadline greater than the deadline requested by the Client.

> 为服务服务器设定了接收请求和发送响应之间允许的最长时间。对于服务客户端，它设定了发送请求和接收请求响应之间允许的最长时间。服务仅支持 rmw 层的截止日期跟踪。这意味着请求开始时间是指请求到达 rmw 层时的时间，完成时间是指响应消息到达 rmw 层时的时间。如果服务器提供的截止日期大于客户端请求的截止日期，服务客户端将**不会**被阻止向服务器发出请求。

### Liveliness

For Service Servers it establishes both the level of reporting that they will provide to Clients and also the level of reporting that they require from Clients. For Service Clients it establishes both the level of reporting that they require from Service Servers and the level of reporting that they will provide to the Server.

> 为服务服务器，它建立了它们将向客户提供的报告水平以及它们要求客户提供的报告水平。对于服务客户，它建立了它们要求服务服务器提供的报告水平以及它们将向服务器提供的报告水平。

Services will support the following levels of liveliness:

> 服务将支持以下活力水平：

- `LIVELINESS_DEFAULT` - Use the ROS specified default for liveliness (which is LIVELINESS_AUTOMATIC).
- `LIVELINESS_AUTOMATIC` - The signal that establishes a Service Server is alive comes from the ROS rmw layer.
- `LIVELINESS_MANUAL_NODE` - The signal that establishes a Service is alive is at the node level. A message on any outgoing channel on the node or an explicit signal from the application to assert liveliness on the node will mark all outgoing channels on the node as being alive.
- `LIVELINESS_MANUAL_SERVICE` - The signal that establishes a Service is alive is at the Service level. Only sending a response on the Service or an explicit signal from the application to assert liveliness on the Service will mark the Service as being alive.

> - 使用 ROS 指定的默认活力(即 LIVELINESS_AUTOMATIC)。
> - LIVELINESS_AUTOMATIC-ROS rmw 层发出的信号可以确定服务服务器是否活着。
> - LIVELINESS_MANUAL_NODE - 建立服务的信号是在节点级别上。节点上任何出站通道上的消息或应用程序发出的明确信号，都将标记节点上所有出站通道为活动状态。
> - 信号建立服务活跃的信号是在服务级别。只有在服务上发送响应或明确的应用程序信号来确认服务的活跃状态才会将服务标记为活跃状态。

Service Servers and Clients will each specify two liveliness policies, one for the liveliness policy pertaining to the Server and one pertaining to the Client. In order for a Client to connect to a Server to make a request the Client_Liveliness level requested by the Server must be greater than the level provided by the Client and the Server_Liveliness requested by the Client must be greater than the level provided by the Server.

> **服务器和客户端将各自指定两个存活策略，一个是服务器的存活策略，另一个是客户端的存活策略。** 为了使客户端连接到服务器发出请求，服务器请求的客户端存活级别必须大于客户端提供的级别，客户端请求的服务器存活级别必须大于服务器提供的级别。

### Lifespan

For Services it establishes a time at which the request is no longer valid. It would act similar to a timeout on the request. There are a lot of open questions about how you would notify Client and the Server if a request was timed out and what action should be taken when the lifespan expires.

> 对于服务，它设定了一个请求失效的时间。它的**作用类似于请求的超时**。关于如何通知客户端和服务器如果请求超时以及当生命周期结束时应采取的行动，仍有很多未解决的问题。

## Appendix A

Definitions of the QoS policies from the DDS spec.

> 定义 DDS 规范中的 QoS 策略

### Deadline

This policy is useful for cases where a Topic is expected to have each instance updated periodically. On the publishing side this setting establishes a contract that the application must meet. On the subscribing side the setting establishes a minimum requirement for the remote publishers that are expected to supply the data values.

> 这项策略对于预计每个实例定期更新的主题情况很有用。在发布方面，此设置建立了应用程序必须遵守的合同。在订阅方面，该设置为预期提供数据值的远程发布者设定了最低要求。

When the DDS Service ‘matches’ a DataWriter and a DataReader it checks whether the settings are compatible (i.e., offered deadline period<= requested deadline period) if they are not, the two entities are informed (via the listener or condition mechanism) of the incompatibility of the QoS settings and communication will not occur.

> 当 DDS 服务“匹配”一个数据写入者和一个数据读取者时，它会检查设置是否兼容(即，提供的截止期限 <=请求的截止期限)，如果不兼容，这两个实体将通过监听器或条件机制被通知 QoS 设置的不兼容性，通信将不会发生。

Assuming that the 'DataReader' and 'DataWriter' ends have compatible settings, the fulfillment of this contract is monitored by the DDS Service and the application is informed of any violations by means of the proper listener or condition.

> 假设'DataReader'和'DataWriter'端具有兼容的设置，DDS 服务将监控合同的履行情况，并通过适当的监听器或条件将任何违反情况通知应用程序。

The value offered is considered compatible with the value requested if and only if the inequality “offered deadline period <= requested deadline period” evaluates to ‘TRUE.’

> 如果且仅当不等式“提供的截止期小于等于要求的截止期”的评估结果为“TRUE”时，才认为提供的价值与要求的价值是兼容的。

The setting of the DEADLINE policy must be set consistently with that of the TIME_BASED_FILTER. For these two policies to be consistent the settings must be such that “deadline period>= minimum_separation.”

> 设定 DEADLINE 策略必须与 TIME_BASED_FILTER 一致。为了使这两个策略一致，设置必须是“deadline period>= minimum_separation”。

### Liveliness

This policy controls the mechanism and parameters used by the DDS Service to ensure that particular entities on the network are still “alive.” The liveliness can also affect the ownership of a particular instance, as determined by the OWNERSHIP QoS policy.

> 这项策略控制 DDS 服务使用的机制和参数，以确保网络上的特定实体仍然“存活”。**生存状态也会影响由 OWNERSHIP QoS 策略确定的特定实例的所有权。**

> [!NOTE]
> 应结合 liveliness 与 ownership 策略一起使用。

This policy has several settings to support both data-objects that are updated periodically as well as those that are changed sporadically. It also allows customizing for different application requirements in terms of the kinds of failures that will be detected by the liveliness mechanism.

> 这项策略设置了几项支持定期更新的数据对象以及偶尔更改的数据对象的支持。**它还允许根据不同的应用程序要求定制可以由活力机制检测到的故障类型。**

> [!NOTE]
> 应该综合使用这里的“故障类型”！

The AUTOMATIC liveliness setting is most appropriate for applications that only need to detect failures at the process-level 27 , but not application-logic failures within a process. The DDS Service takes responsibility for renewing the leases at the required rates and thus, as long as the local process where a DomainParticipant is running and the link connecting it to remote participants remains connected, the entities within the DomainParticipant will be considered alive.

> 自动活力设置最适合用于仅需要**在进程级别 27 检测故障**，而不是进程内的应用逻辑故障的应用程序。DDS 服务负责以所需速率更新租约，因此，只要运行 DomainParticipant 的本地进程和连接到远程参与者的链路保持连接，DomainParticipant 中的实体将被视为活动状态。

This requires the lowest overhead.

> 这需要最低的开销。

The MANUAL settings (`MANUAL_BY_PARTICIPANT`, `MANUAL_BY_TOPIC`), require the application on the publishing side to periodically assert the liveliness before the lease expires to indicate the corresponding Entity is still alive. The action can be explicit by calling the assert_liveliness operations, or implicit by writing some data.

> 设置为手动(MANUAL_BY_PARTICIPANT，MANUAL_BY_TOPIC)时，**发布端的应用程序在租约到期前需要定期断言相应实体仍然存活。这一动作可以通过调用 assert_liveliness 操作来显式实现，或者通过写入一些数据来隐式实现。**

> [!NOTE]
> 这里给出了另外两种 liveliness 策略的使用方式：
> 显示操作：`assert_liveliness()`
> 隐式操作：写入数据
> 手动的方式可以**控制粒度**

The two possible manual settings control the granularity at which the application must assert liveliness.

> 两种可能的手动设置**可以控制应用程序断言活动性的粒度**。

• The setting MANUAL_BY_PARTICIPANT requires only that one Entity within the publisher is asserted to be alive to deduce all other Entity objects within the same DomainParticipant are also alive.
• The setting MANUAL_BY_TOPIC requires that at least one instance within the DataWriter is asserted.

> • MANUAL_BY_PARTICIPANT 设置要求只要发布者中的一个实体被断言为活动状态，就能推断同一 DomainParticipant 中的其他实体对象也处于活动状态。
> • 设置 MANUAL_BY_TOPIC 要求 DataWriter 中至少有一个实例被断言。

### Lifespan

The purpose of this QoS is to avoid delivering “stale” data to the application.

> 这种 QoS 的目的是避免向应用程序**提供“过时”**的数据。

Each data sample written by the DataWriter has an associated ‘expiration time’ beyond which the data should not be delivered to any application. Once the sample expires, the data will be removed from the DataReader caches as well as from the transient and persistent information caches.

> 每一个由 **DataWriter** 写入的数据样本都有一个相关的“过期时间”，超过该时间，数据不应该被传送到任何应用程序。一旦样本过期，**数据将从 DataReader 缓存以及临时和持久信息缓存中移除。**

> [!NOTE]
> 这个 QOS 策略是针对 sub 而言的，是从 data_reader 中移除信息。
> 果然这里参数是对于 pub 进行设置的，应该避免对 sub 进行设置，参考：`C:\Users\trantor\Documents\Hirain\Project\doc\ros2_design\articles\qos_configurability.md:480`

The ‘expiration time’ of each sample is computed by adding the duration specified by the LIFESPAN QoS to the source timestamp. The source timestamp is either automatically computed by the DDS Service each time the DataWriter write operation is called, or else supplied by the application by means of the write_w_timestamp operation.

> 每个样本的“过期时间”是通过将 LIFESPAN QoS 指定的持续时间加到源时间戳来计算的。**源时间戳要么由 DDS 服务每次调用 DataWriter 写操作时自动计算**，要么由应用程序通过 write_w_timestamp 操作提供。

This QoS relies on the sender and receiving applications having their clocks sufficiently synchronized. If this is not the case and the DDS Service can detect it, the DataReader is allowed to use the reception timestamp instead of the source timestamp in its computation of the expiration time.

> 这种 QoS 依赖于发送者和接收应用程序具有足够同步的时钟。如果情况不是这样，并且 DDS 服务可以检测到它，则允许 DataReader 在计算到期时间时使用接收时间戳而不是源时间戳。
