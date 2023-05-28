---
tip: translate by openai@2023-05-29 08:48:52
layout: default
title: ROS 2 Quality of Service policies
permalink: articles/qos.html
abstract:
  This article describes the approach to provide QoS (Quality of Service) policies for ROS 2.
published: true
author: '[Esteve Fernandez](https://github.com/esteve)'
date_written: 2015-10
last_modified: 2019-05
Authors: {{ page.author }}
Date Written: {{ page.date_written }}
Last Modified: {% if page.last_modified %}{{ page.last_modified }}{% else %}{{ page.date_written }}{% endif %}
---

With the advent of inexpensive robots using unreliable wireless networks, developers and users need mechanisms to control how traffic is prioritized across network links.

> 随着廉价机器人使用不可靠的无线网络的出现，开发者和用户需要机制来控制如何优先处理网络链接上的流量。

## Background

ROS 1 uses TCP as the underlying transport, which is unsuitable for lossy networks such as wireless links.
With ROS 2 relying on DDS which uses UDP as its transport, we can give control over the level of reliability a node can expect and act accordingly.

> ROS 1 使用 TCP 作为底层传输，这不适用于像无线链路这样的有损网络。
> 随着 ROS 2 依赖于使用 UDP 作为传输的 DDS，我们可以控制节点可以期望的可靠性水平，并相应地采取行动。

## DDS Quality of Service policies

DDS provides fine-grained control over the Quality of Service (QoS) setting for each of the entities involved in the system.
Common entities whose behavior can be modified via QoS settings include: Topic, DataReader, DataWriter, Publisher and Subscriber.
QoS is enforced based on a Request vs Offerer Model, however Publications and Subscriptions will only match if the QoS settings are compatible.

> DDS 提供了对系统中每个实体的服务质量(QoS)设置的细粒度控制。
> 常见实体的行为可以通过 QoS 设置进行修改，包括：主题、数据读取器、数据写入器、发布者和订阅者。
> QoS 根据请求与提供者模型强制实施，但是发布和订阅只有在 QoS 设置兼容的情况下才能匹配。

## ROS 2 proposal

Given the complexity of choosing the correct QoS settings for a given scenario, it may make sense for ROS 2 to provide a set of predefined QoS profiles for common usecases (e.g. sensor data, real time, etc.), while at the same time give the flexibility to control specific features of the QoS policies for the most common entities.

> 鉴于选择适当的 QoS 设置以满足特定场景的复杂性，ROS 2 可能有必要为常见用例（例如传感器数据、实时性等）提供一组预定义的 QoS 配置，同时为最常见的实体提供控制特定 QoS 策略特征的灵活性。

## QoS profiles

A QoS profile defines a set of policies, including durability, reliability, queue depth and sample history storage.
The base QoS profile includes settings for the following policies:

> 一个 QoS 配置文件定义了一组策略，包括耐久性、可靠性、队列深度和样本历史存储。
> 基本 QoS 配置文件包括以下策略的设置：

- History.
- Keep last: only store up to N samples, configurable via the queue depth option.
- Keep all: store all samples, subject to the configured resource limits of the DDS vendor.
- Depth.
- Size of the queue: only honored if used together with "keep last".

- Reliability.
- Best effort: attempt to deliver samples, but may lose them if the network is not robust.
- Reliable: guarantee that samples are delivered, may retry multiple times.

- Durability.
- Transient local: _only applies to `DataWriter`s_.
  `DataWriter` becomes responsible of persisting samples until a corresponding `DataReader` becomes available.
- Volatile: no attempt is made to persist samples.

> 历史
> 保持最后：只存储最多 N 个样本，可通过队列深度选项进行配置。
> 保存所有：根据 DDS 供应商配置的资源限制存储所有样本。
> 深度
> 队列大小：只有与“保持最后一个”一起使用时才被承认。
> 可靠性
> 尽最大努力：尝试提供样本，但如果网络不稳定，可能会丢失样本。
> 可靠：保证样品可以送达，可以多次重试。
> 耐久性。
> 本地瞬态：仅适用于`DataWriter`s。
> 易变的：不会尝试持久化样本。

> [NOTE]:

ROS 2 will provide QoS profiles based on the following use cases:

> ROS 2 将根据以下使用案例提供 QoS 配置文件：

- Default QoS settings for publishers and subscriptions

> 默认的发布者和订阅的 QoS 设置

In order to make the transition from ROS 1 to ROS 2, exercising a similar network behavior is desirable.

> 为了实现从 ROS 1 到 ROS 2 的过渡，期望能够表现出类似的网络行为。

By default, publishers and subscriptions are reliable in ROS 2, have volatile durability, and "keep last" history.

> 默认情况下，ROS 2 中的发布者和订阅者是可靠的，具有不稳定的耐久性，并且“保留最后一个”历史记录。

- Services

In the same vein as publishers and subscriptions, services are reliable.
It is especially important for services to use volatile durability, as otherwise service servers that re-start may receive outdated requests.

> - 服务
>   在出版商和订阅的情况下，服务是可靠的。
>   服务使用易失性持久性尤为重要，否则重新启动的服务器可能会收到过时的请求。

- Sensor data

For sensor data, in most cases it's more important to receive readings in a timely fashion, rather than ensuring that all of them arrive.
That is, developers want the latest samples as soon as they are captured, at the expense of maybe losing some.
For that reason the sensor data profile uses best effort reliability and a smaller queue depth.

> - 传感器数据
>   在大多数情况下，对于传感器数据来说，更重要的是及时收到读数，而不是确保所有读数都到达。
>   那就是说，开发人员希望尽快获取最新的样本，即使可能会损失一些也在所不惜。
>   因此，传感器数据配置使用最大努力可靠性和较小的队列深度。

- Parameters

Parameters are based on services, and as such have a similar profile.
The difference is that parameters use a much larger queue depth so that requests do not get lost when, for example, the parameter client is unable to reach the parameter service server.
Profiles allow developers to focus on their applications without worrying about every QoS setting in the DDS specification.

> - 参数
>   参数是基于服务，因此具有相似的配置。
>   参数使用更大的队列深度的不同之处在于，当例如参数客户端无法访问参数服务器时，请求不会丢失。
>   设置配置文件可以让开发者不用担心 DDS 规范中的每一个 QoS 设置，而专注于他们的应用程序。

> [NOTE]:

## Integration with existing DDS deployments

Both PrismTech OpenSplice and RTI Connext support loading of QoS policies via an external XML file.

> 两个 PrismTech OpenSplice 和 RTI Connext 都支持通过外部 XML 文件加载 QoS 策略。

In environments where DDS is already deployed and also to enable more extensibility other than the offered by the ROS 2 and the predefined profiles, ROS 2 may provide loading of the QoS settings via the same mechanisms the underlying DDS implementations use.

> 在已部署 DDS 的环境中，为了提供除 ROS 2 和预定义配置文件外的更多可扩展性，ROS 2 可以通过与底层 DDS 实现相同的机制加载 QoS 设置。

However, this mechanism will not be added into the common ROS 2 API so as to keep the `rmw` layer transport agnostic and let future developers implement it for other transports (e.g. ZeroMQ, TCPROS, etc.)

> 然而，为了保持`rmw`层传输与特定实现无关，这种机制不会被添加到常见的 ROS 2 API 中，以便让未来的开发人员为其他传输（例如 ZeroMQ、TCPROS 等）实现它。

To honor the QoS settings of the system, developers can use the `rmw_qos_profile_system_default` QoS profile which delegates the responsibility of the QoS machinery to the underlying DDS vendor.

> [!NOTE]:
> **为了尊重系统的 QoS 设置，开发人员可以使用`rmw_qos_profile_system_default` QoS 配置文件，将 QoS 机制的责任委托给底层 DDS 供应商。**

This allows developers to deploy ROS 2 applications and use DDS vendor tools to configure the QoS settings.

> 这允许开发人员部署 ROS 2 应用程序，并使用 DDS 供应商工具来配置 QoS 设置。

## Integration with non DDS RMW Implementations

There is no QoS setting that affects the order messages may be received.
Instead an RMW implementation must implement the same behavior as DDS.

> 没有 QoS 设置会影响消息接收的顺序。
> 代替 RMW 实现必须实现与 DDS 相同的行为。

Once a subscriber takes a message from the RMW implementation it must not be allowed to take an earlier message from the same publisher.
If the connection is Reliable then newer messages should be kept in the History until all older messages are received.
A best effort connection should drop an older message if a subscriber has taken a newer one.

> 一旦订阅者从 RMW 实现中获取消息，就不允许再从同一发布者获取更早的消息。
> 如果连接可靠，那么新消息应保留在历史中，直到收到所有旧消息。
> 一个最佳的连接应该丢弃旧的消息，如果订阅者已经收到了新的消息。

## Open questions

How should the integration of the QoS machinery with intraprocess communication be like.

> 应该如何将 QoS 机制与进程间通信集成？

> [!NOTE]
> 这是个有价值的问题！

## References

- Gordon Hunt, [DDS - Advanced Tutorial, Using QoS to Solve Real-World Problems](http://www.omg.org/news/meetings/workshops/RT-2007/00-T5_Hunt-revised.pdf)

> - 戈登·亨特博士，[使用 QoS 解决实际问题的高级教程](http://www.omg.org/news/meetings/workshops/RT-2007/00-T5_Hunt-revised.pdf)

- [OpenDDS - QoS Policy Usages](http://www.opendds.org/qosusages.html)
- [OpenDDS - QoS Policy Compliance](http://www.opendds.org/qospolicies.html)

- Angelo Corsaro, [DDS QoS Unleashed](http://www.slideshare.net/Angelo.Corsaro/dds-qos-unleashed)

> -安杰洛·科萨罗（Angelo Corsaro），《DDS QoS 解放》（[DDS QoS Unleashed](http://www.slideshare.net/Angelo.Corsaro/dds-qos-unleashed)）

- Angelo Corsaro, [The DDS Tutorial Part II](http://www.slideshare.net/Angelo.Corsaro/the-dds-tutorial-part-ii)

> - 亚洛·科萨罗（Angelo Corsaro）的《DDS 教程第二部分》（The DDS Tutorial Part II）：http://www.slideshare.net/Angelo.Corsaro/the-dds-tutorial-part-ii

- [DDS Spec, section 2.2.3](http://www.omg.org/spec/DDS/1.4/PDF/)

- Heidi Schubert, [QoS policies for configuring reliability](https://community.rti.com/content/forum-topic/qos-policies-configuring-reliability)

> - Heidi Schubert，[配置可靠性的服务质量策略](https://community.rti.com/content/forum-topic/qos-policies-configuring-reliability)

- [RTI Connext - Comprehensive Summary of QoS Policies](https://community.rti.com/static/documentation/connext-dds/5.2.0/doc/manuals/connext_dds/RTI_ConnextDDS_CoreLibraries_QoS_Reference_Guide.pdf)
- [DDS and Delegated... Durability?](http://blogs.rti.com/2013/12/11/dds-and-delegated-durability/)
