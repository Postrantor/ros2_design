---
tip: translate by openai@2023-05-29 09:10:01
layout: default
title: Unique Network Flows
permalink: articles/unique_network_flows.html
abstract: Enable unique network flow identifiers for publishers and subscriptions in communicating nodes
author: '[Ananya Muddukrishna, Ericsson AB](https://github.com/anamud)'
date_written: 2021-05
last_modified: 2021-05
published: true
Authors: {{ page.author }}
Date Written: {{ page.date_written }}
Last Modified: {% if page.last_modified %}{{ page.last_modified }}{% else %}{{ page.date_written }}{% endif %}
---

# Unique Network Flows

For performance, ROS2 applications require careful selection of QoS for publishers and subscriptions. Although networks offer various QoS options, ROS2 publishers and subscriptions are unable to use them due to non-unique flows. As a result, ROS2 publishers and subscriptions can only hope to obtain undifferentiated QoS from networks. This ultimately degrades the performance potential of ROS2 applications and wastes networking infrastructure investments.

> 对于性能，**ROS2 应用程序需要仔细选择发布者和订阅的 QoS**。尽管网络提供了各种 QoS 选项，但由于非唯一流，ROS2 发布者和订阅者无法使用它们。因此，ROS2 发布者和订阅者只能希望从网络获得非差异化的 QoS。这最终降低了 ROS2 应用程序的性能潜力，浪费了网络基础设施投资。

We propose unique network flows for ROS2 publishers and subscriptions. Our proposal is easy to use, convenient to implement, and minimal. Plus, it respects non-DDS-middleware-friendly concerns in ROS2.

> 我们为 ROS2 发布者和订阅提供独特的网络流。我们的提议易于使用，方便实施，而且最小化。此外，它还尊重 ROS2 中不友好的非 DDS 中间件问题。

In this document, we first describe essential background concepts. After that we precisely state the problem and propose a solution to the problem. Existing solutions are compared in the end.

> 在本文档中，我们首先描述基本背景概念。然后，我们精确地陈述问题并提出解决方案。最后，对现有解决方案进行比较。

## Background

IP networking [1] is the predominant inter-networking technology used today. Ethernet, WiFi, 4G/5G telecommunication all rely on IP networking.

> IP 网络[1]是当今使用最广泛的网络技术。以太网、WiFi、4G/5G 电信都依赖于 IP 网络。

Streams of IP packets from a given source to destination are called _packet flows_ or simply _flows_. Applications can uniquely identify certain flows and explicitly specify what QoS is required from the network for those flows.

> 流量指从指定源到目的地的 IP 数据包流。应用程序可以唯一识别某些流量，并明确指定网络对这些流量所需的 QoS。

### Flow Identifers

The _5-tuple_ is a traditional unique identifier for flows. The 5-tuple consists of five parameters: source IP address, source port, destination IP address, destination port, and the transport protocol (example, TCP/UDP).

> 5 元组是流的传统唯一标识符。5 元组由五个参数组成：源 IP 地址、源端口、目的 IP 地址、目的端口和传输协议（例如 TCP/UDP）。

IPv6 specifies a _3-tuple_ for uniquely identifying flows. The IPv6 3-tuple consists of the source IP address, destination IP address, and the Flow Label. The Flow Label [2] is a 20-bit field in the IPv6 header. It is typically set by the source of the flow. The default Flow Label is zero.

> IPv6 为唯一标识流量指定了一个*3 元组*。IPv6 3 元组由源 IP 地址、目的 IP 地址和流标签组成。流标签[2]是 IPv6 头部中的 20 位字段。它通常由流的源设置。默认流标签为零。

If the 5-tuple is not sufficient, then custom 6-tuples can be created by combining the 5-tuple with the IP Options field or the IP Differentiated Services Code Point sub-field. Such custom 6-tuples are typically used as workarounds for technical difficulties.

> 如果 5 元组不够，那么可以通过将 5 元组与 IP 选项字段或 IP 不同服务代码点子字段组合来创建自定义 6 元组。这种自定义 6 元组通常用作技术困难的解决方案。

### Explicit QoS Specification

We briefly discuss two relevant explicit QoS specification methods for applications -- Differentiated Services and 5G network 5QI.

> 我们简要讨论了两种与应用相关的显式 QoS 规范方法：差分服务和 5G 网络 5QI。

- Differentiated Services (DS) [3] is a widely-used QoS architecture for IP networks. The required DS-based QoS is set by the application in the 6-bit DS Code Point (DSCP) sub-field of the 8-bit DS field in the IP packet header. For example, DSCP set to 0x2E specifies expedited forwarding as the required QoS. Expedited forwarding is typically used for real-time data such as voice and video.

> 不同服务（DS）[3]是一种广泛使用的 IP 网络的 QoS 架构。所需的基于 DS 的 QoS 由应用程序在 IP 数据包头部的 8 位 DS 字段的 6 位 DS 代码点（DSCP）子字段中设置。例如，DSCP 设置为 0x2E 指定加急转发作为所需的 QoS。加急转发通常用于实时数据，如语音和视频。

ROS2 lacks an API to specify DS-based QoS for publishers and subscriptions. The DSCP value in their flows is therefore set to 0x00. This specifies default forwarding as the required QoS from the network. However, DDS provides the Transport Priority QoS policy to specify DS-based QoS.

> ROS2 缺乏一个 API 来指定基于 DS 的 QoS，用于发布者和订阅者。因此，其流中的 DSCP 值被设置为 0x00，这指定网络所需的 QoS 为默认转发。但是，DDS 提供了传输优先级 QoS 策略来指定基于 DS 的 QoS。

A frustrating problem with DS-based QoS is that intermediate routers can reset or alter the DSCP value within flows. One workaround is to carefully configure intermediate routers such that they retain DSCP markings from incoming to outgoing flows.

> 一个令人沮丧的问题是基于 DS 的 QoS，中间路由器可以重置或更改流中的 DSCP 值。一种解决方法是仔细配置中间路由器，使其保留从传入到传出流中的 DSCP 标记。

- 5G network 5QI: The Network Exposure Function (NEF) [4] in the 5G core network provides robust and secure API for QoS specification. This API enables applications to programmatically (HTTP-JSON) specify required QoS by associating 5G QoS Identifiers (5QIs) to flow identifers, as shown in the figure next.

> 5G 网络 5QI：5G 核心网络中的网络暴露功能（NEF）[4]提供了强大且安全的 API，用于 QoS 规范。此 API 可以通过将 5G QoS 标识符（5QIs）与流标识符相关联，以编程（HTTP-JSON）的方式指定所需的 QoS，如下图所示。

![ROS2 Application 5GS Network Programmability](./ros2-app-5gs-network-programmability.png)

Twenty-six standard 5QIs are identified in the latest release-16 by 3GPP [4:Table 5.7.4-1]. We exemplify a few of them in the table below. The variation in service characteristics of the example 5QIs emphasizes the importance of careful 5QI selection.

> 二十六个标准 5QI 已在最新的 3GPP [4：表 5.7.4-1]发布中标识。我们在下表中举例了其中一些。示例 5QI 的服务特性变化强调了仔细选择 5QI 的重要性。

The 5G network also has the ability to sensibly infer 5QI QoS from DS-based QoS markings in flows.

> 5G 网络还具有从基于 DS 的 QoS 标记中明智推断 5QI QoS 的能力。

```
| 5QI         | Resource                                   | Priority | Packet Delay Budget (ms) | Packet Error Rate | Example Services                                                                                                        |
| ----------- | ------------------------------------------ | -------- | ------------------------ | ----------------- | ------------------------------------------------------------------------------------------------------------------------|
| 3           | Guaranteed bitrate (GBR)                   | 30       | 50                       | 10^-3             | Real Time Gaming; V2X messages; Electricity distribution – medium voltage; Process automation monitoring                |
| 4           | GBR                                        | 50       | 300                      | 10^-6             | Non-Conversational Video (Buffered Streaming)                                                                           |
| 7           | Non GBR (NGBR)                             | 70       | 100                      | 10^-3             | Voice Video (Live Streaming); Interactive Gaming                                                                        |
| 9 (default) | NGBR                                       | 90       | 300                      | 10^-6             | Video (Buffered Streaming); TCP-based traffic (e.g., www, e-mail, chat, ftp, p2p file sharing, progressive video, etc.) |
| 82          | Delay critical guaranteed bitrate (DC GBR) | 19       | 10                       | 10^-4             | Discrete Automation                                                                                                     |
| 85          | DC GBR                                     | 21       | 5                        | 10^-5             | Electricity distribution - high voltage; V2X messages (Remote Driving)                                                  |
```

## Problem

All publishers and subscriptions in communicating nodes have the same flow identifers (5-tuple or 3-tuple). This disables explicit network QoS differentiation for publishers and subscriptions in communicating nodes. In other words, publishers and subscriptions in communicating nodes can only be assigned the same network QoS.

> 所有在通信节点上的发布者和订阅者都具有相同的流标识（5 元组或 3 元组）。这使得在通信节点上的发布者和订阅者不能显式地进行网络 QoS 分类。**换句话说，在通信节点上的发布者和订阅者只能被分配相同的网络 QoS。**

We believe the problem occurs by design. For performance reasons, RMW implementations are likely to associate IP address, port, and transport protocol to nodes and not to individual publishers/subscriptions. This is true for all the tier-1 RMW implementations today (Foxy Fitzroy at the time of writing). None of the tier-1 RMW implementations set the IPv6 flow label to differentiate flows of publisher/subscriptions.

> 我们相信这个问题是由设计引起的。出于性能原因，**RMW 实现很可能将 IP 地址、端口和传输协议关联到节点，而不是个别发布者/订阅者。目前所有一级 RMW 实现都是如此（写作时为 Foxy Fitzroy）。没有一级 RMW 实现设置 IPv6 流标签来区分发布者/订阅者的流量。**

### Example

We use the following example to highlight the problem.

> 我们使用以下例子来突出问题。

Consider a distributed robotics application with communicating nodes N1 and N2. N1 is active on device D1 and N2 on device D2. D1 and D2 are connected by an IP network, say a 5G network.

> 考虑一个分布式机器人应用程序，其中有通信节点 N1 和 N2。N1 在设备 D1 上活动，N2 在设备 D2 上活动。D1 和 D2 通过 IP 网络连接，比如 5G 网络。

N1 contains publishers P1 and P2. P1 publishes video data whereas P2 publishes battery status data.

> N1 包含出版商 P1 和 P2。P1 出版视频数据，而 P2 出版电池状态数据。

N2 contains subscriptions S1 and S2. S1 receives video from P1 and performs real-time object detection. S2 receives battery data from P2 and performs non-real-time battery management.

> N2 包含订阅 S1 和 S2。S1 从 P1 接收视频并执行实时物体检测。S2 从 P2 接收电池数据并执行非实时电池管理。

The link P1-S1 requires low-latency QoS from the 5G network, say a maximum delay of 5ms. P2-S2 requires default latency QoS i.e., 300ms. Then, by construction, since the flow identifiers of P1-S1 and P2-S2 links are the same, they cannot be assigned the required QoS by the network. Both P1-S1 and P2-S2 can either be assigned QoS with 5ms delay or with 300ms delay. The former case represents a waste of network resources, the latter case degrades performance.

> 链路 P1-S1 需要 5G 网络提供低延迟的 QoS，比如最大延迟 5ms。P2-S2 需要默认的延迟 QoS，即 300ms。
> **因此，由于 P1-S1 和 P2-S2 链路的流标识相同，网络无法为它们分配所需的 QoS。** P1-S1 和 P2-S2 可以分配 5ms 延迟的 QoS 或 300ms 延迟的 QoS。前者会浪费网络资源，后者会降低性能。

## Proposed Solution

Our proposal to solve the problem is to make the flows of publishers and subscriptions in communicating nodes unique.

> 我们提出的解决问题的建议是使发布者和订阅者在交流节点中的流量唯一。

### Definitions

- _Network flow_: A tuple of networking resources selected by a RMW implementation for transmission of messages from a publisher to a subscription. The networking resources considered are:
- transport protocol UDP or TCP (publisher and subscription)
- transport port (publisher and subscription)
- internet protocol IPv4 or IPv6 (publisher and subscription)
- internet address (publisher and subscription)
- DSCP (publisher only)
- Flow label (publisher only)

> - _网络流_：RMW 实现从发布者到订阅者传输消息时选择的网络资源元组。考虑的网络资源包括：
> - 传输协议 UDP 或 TCP（发布者和订阅者）
> - 运输端口（发布者和订阅者）
> - 网络协议 IPv4 或 IPv6（发布者和订阅者）
> - 网络地址（出版商和订阅）
> - 只限发布者的 DSCP
> - 流标签（仅限发布者）

Network flows are defined for UDP/TCP and IP-based RMW implementations only. It is not a limiting definition since these are the majority protocols used today. The definition can be later extended to include relevant non-IP networks such as deterministic ethernet.

> 网络流仅针对 UDP/TCP 和基于 IP 的 RMW 实现进行定义。这不是一个限制性定义，因为这些是当今使用最广泛的协议。该定义可以稍后扩展到包括确定性以太网等相关的非 IP 网络。

- _Network Flow Endpoint (NFE)_: The portion of a network flow specific to the publisher or the subscription. In other words, each network flow has two NFEs; one for the publisher and the other for the subscription.
- _Dissimilar NFE pair_: Two NFEs are dissimilar if one or more of their networking resources are different.
- _Unique NFE_: A NFE is unique if it is dissimilar to all other NFEs.
- _Strongly and weakly unique network flow_: A network flow is strongly unique if both publisher and subscription NFEs are unique. A weakly unique network flow either has a publisher NFE or a subscriber NFE that is unique.

> 网络流端点（NFE）：特定于发布者或订阅的网络流的一部分。换句话说，每个网络流都有两个 NFE；一个是发布者，另一个是订阅者。
> 两个 NFE 不相似，如果它们的一个或多个网络资源不同。
> _唯一的 NFE_：如果 NFE 与所有其他 NFE 不同，则它是唯一的。
> 强唯一网络流和弱唯一网络流：如果发布者和订阅者 NFE 都是唯一的，则网络流是强唯一的。弱唯一网络流要么有一个唯一的发布者 NFE，要么有一个唯一的订阅者 NFE。

### Unique Network Flow Endpoints

We construct a publisher/subscription creation-time option called `require_unique_network_flow_endpoints` as a candidate structure to enable unique identification of network flows. This option takes a value from the enumeration shown below.

> 我们构建了一个发布/订阅创建时选项，名为`require_unique_network_flow_endpoints`，作为一种使网络流能够唯一标识的候选结构。该选项的值来自下面的枚举。

```cpp
enum unique_network_flow_endpoints_requirement_t {
  UNIQUE_NETWORK_FLOW_ENDPOINTS_NOT_REQUIRED = 0,
  UNIQUE_NETWORK_FLOW_ENDPOINTS_STRICTLY_REQUIRED,
  UNIQUE_NETWORK_FLOW_ENDPOINTS_OPTIONALLY_REQUIRED,
  UNIQUE_NETWORK_FLOW_ENDPOINTS_SYSTEM_DEFAULT
}
```

Upon receiving the publisher/subscription creation request, the RMW implementation assigns unique NFEs according to the `require_unique_network_flow_endpoints` option value.

> 收到出版商/订阅创建请求后，RMW 实现根据“require_unique_network_flow_endpoints”选项值分配唯一的 NFE。

The default value of the option is `UNIQUE_NETWORK_FLOW_ENDPOINTS_NOT_REQUIRED` which indicates to the RMW implementation that unique NFEs are not required.

> 默认选项的值为`UNIQUE_NETWORK_FLOW_ENDPOINTS_NOT_REQUIRED`，这表明 RMW 实现不需要唯一的 NFEs。

The value `UNIQUE_NETWORK_FLOW_ENDPOINTS_STRICTLY_REQUIRED` indicates to the RMW implementation that unique NFEs are strictly required. If not feasible, the RMW implementation must flag an error and not create the associated publisher/subscription.

> `UNIQUE_NETWORK_FLOW_ENDPOINTS_STRICTLY_REQUIRED`的值表明 RMW 实现必须严格要求唯一的 NFEs。如果不可行，RMW 实现必须标记一个错误，而不能创建相关的发布者/订阅者。

The value `UNIQUE_NETWORK_FLOW_ENDPOINTS_OPTIONALLY_REQUIRED` enables the RMW implementation to create unique NFEs if feasible. If not feasible, it can continue to create the associated publisher/subscription and not flag an error.

> `UNIQUE_NETWORK_FLOW_ENDPOINTS_OPTIONALLY_REQUIRED`的值使 RMW 实现能够在可行的情况下创建唯一的 NFE。如果不可行，它可以继续创建相关的发布者/订阅，而不会标记错误。

The value `UNIQUE_NETWORK_FLOW_ENDPOINTS_SYSTEM_DEFAULT` delegates the decision to create unique NFEs fully to the RMW implementation. The RMW implementation can decide internally on its own or be coerced through side-loading mechanisms to create unique NFEs for the associated publisher/subscription.

> `UNIQUE_NETWORK_FLOW_ENDPOINTS_SYSTEM_DEFAULT`的值完全将决定创建唯一的 NFE 的权力委托给 RMW 实现。RMW 实现可以自行内部决定，或者通过旁路机制来强制为相关的发布者/订阅创建唯一的 NFE。

The example C++ snippet below shows a node creating two subscriptions and a publisher. Subscription `sub_x_` strictly requires unique NFEs whereas `sub_y_` is indifferent. Publisher `pub_z_` optionally requires a unique NFE.

> 下面的 C++代码片段示例展示了**一个节点创建两个订阅和一个发布者**。订阅'sub*x*'严格要求唯一的 NFE，而'sub*y*'则漠不关心。发布者'pub*z*'可选择性要求唯一的 NFE。

```cpp
    // Unique network flow endpoints strictly required
    auto options_x = rclcpp::SubscriptionOptions();
    options_x.require_unique_network_flow_endpoints = UNIQUE_NETWORK_FLOW_ENDPOINTS_STRICTLY_REQUIRED;

    sub_x_ = this->create_subscription<std_msgs::msg::String>(
      "topic_x", 10, std::bind(
        &MyNode::topic_x_callback, this,
        _1), options_x);

    // Unique network flow endpoints not required, relying on default
    auto options_y = rclcpp::SubscriptionOptions();

    sub_y_ = this->create_subscription<std_msgs::msg::String>(
      "topic_y", 10, std::bind(
        &MyNode::topic_y_callback, this,
        _1), options_y);

    // Unique network flow endpoints optionally required
    auto options_z = rclcpp::PublisherOptions();
    options_z.require_unique_network_flow_endpoints = RMW_UNIQUE_NETWORK_FLOW_ENDPOINTS_OPTIONALLY_REQUIRED;

    pub_z_ = this->create_publisher<std_msgs::msg::String>("topic_z", 10, options_z);
```

A weakly unique network flow is created by making either the publisher or the subscription NFE unique. This is sufficient to enable network QoS differentiation for the common case where nodes have a single publisher/subscription per topic. RMW implementations can therefore decide to support either the publisher or subscription option to enable unique NFEs.

> 当使出版者或订阅者 NFE 独特时，就会创建一个弱唯一的网络流。这足以为每个主题只有一个发布者/订阅者的常见情况提供网络 QoS 分化。因此，RMW 实现可以决定支持发布者或订阅者选项来启用唯一的 NFE。

A strongly unique network flow is created by making both publisher and subscription NFEs unique. This enables network QoS differentiation in the rather odd case of a node having multiple publishers/subscriptions for the same topic. RMW implementations can decide not to support such odd cases i.e., not support both publisher and subscription option for unique NFEs.

> 强唯一的网络流量是通过使发布者和订阅 NFEs 都是唯一的来创建的。这使得在节点拥有多个发布者/订阅者的同一个主题的比较奇怪的情况下，网络 QoS 可以得到差异化。RMW 实现可以决定不支持这种奇怪的情况，即不支持唯一 NFEs 的发布者和订阅者的选项。

We list few candidate alternatives next for RMW implementations to implement the `require_unique_network_flow_endpoint` option.

> 我们接下来列出几个候选替代方案，以实施`require_unique_network_flow_endpoint`选项的 RMW 实现。

- A simple option that works for both IPv6 and IPv4 is to select a unique transport port (UDP/TCP) and accordingly update the port field in the transport protocol header.
- If the node is communicating using IPv6, then the RMW implementation can write a unique value (such as a suitable derivative of the RTPS entity ID) in the Flow Label field.
- If the node is communicating via IPv4, then the RMW implementation can write a unique value in the DSCP field. This option should only be considered as a last resort since re-purposing the DSCP as an identifier is prone to misinterpretation, is limited to 64 entries, and requires careful configuration of intermediate routers.

> 一个适用于 IPv6 和 IPv4 的简单选项是选择一个唯一的传输端口（UDP/TCP），并相应地更新传输协议头中的端口字段。
> 如果节点使用 IPv6 进行通信，那么 RMW 实现可以在流标签字段中写入一个唯一的值（比如 RTPS 实体 ID 的合适衍生值）。
> 如果节点通过 IPv4 进行通信，那么 RMW 实现可以在 DSCP 字段中写入唯一值。应该将此选项作为最后的手段，因为将 DSCP 重新用作标识符容易被误解，仅限于 64 个条目，并需要对中间路由器进行精确配置。

Both DDS and non-DDS RMW implementations can trivially set fields in IP or transport protocol headers using native socket API on all ROS2 platforms (Linux, Windows, MacOS).

> 两种 DDS 和非 DDS RMW 实现都可以在所有 ROS2 平台（Linux、Windows、MacOS）上使用本地套接字 API 轻松设置 IP 或传输协议头中的字段。

### Get Network Flow Endpoints

To enable applications to discern NFEs created by the RMW implementation, we propose a simple getter interface. Continuing from the previous code snippet,

> 为了使应用程序能够分辨出 RMW 实现创建的 NFE，我们提出了一个简单的 Getter 接口。继续从前面的代码片段，

```cpp
// Get network flows
auto a_nfe_x = sub_x_->get_network_flow_endpoints();
auto a_nfe_y = sub_y_->get_network_flow_endpoints();

// Ensure sub_x_ has unique network flow endpoints
if (a_nfe_x.size() > 0 && a_nfe_y.size() > 0) {
  for (auto nfe_x : a_nfe_x) {
    for (auto nfe_y  : a_nfe_y) {
      if (nfe_x == nfe_y) {
        std::runtime_error("Network flow endpoints are not unique!");
      }
    }
  }
```

The proposed method `get_network_flow_endpoints()` requires RMW implementations to return NFEs of the associated publisher/subscription. The data structure of the NFE is specified concretely by the `rmw` layer.

> 所提出的方法`get_network_flow_endpoints()`需要 RMW 实现返回相关发布者/订阅的 NFE。NFE 的数据结构由`rmw`层具体指定。

To reiterate, the NFEs returned by `get_network_flow_endpoints()` for a publisher represents the NFEs created on the publisher-side only (local). It does not contain information about NFEs of matched subscriptions. Similarly, the NFEs returned by `get_network_flow_endpoints()` for a subscription are localized with no information about matched publishers.

> 重申一下，`get_network_flow_endpoints()`返回的 NFEs 仅代表发布者端（本地）创建的 NFEs。它不包含匹配订阅的 NFEs 的信息。同样，`get_network_flow_endpoints()`返回的 NFEs 只是本地化的，没有匹配发布者的信息。

DDS-based RMW implementations can obtain required information using the DDS-standard `Locator_t` structure when `get_network_flow_endpoints()` is called.

> DDS 的 RMW 实现可以在调用 `get_network_flow_endpoints()` 时使用 DDS 标准的 `Locator_t` 结构获取所需的信息。

### Advantages

Our proposal has the following advantages:

> 我们的提案具有以下优势：

- Easy to use: Application developers are only required to decide if unique flow identifiers for publishers/subscriptions are necessary.
- Light-weight implementation: Both non-DDS and DDS RMW can implement the required support conveniently using native socket API with negligible impact on performance.
- RMW-agnostic: No particular network is preferred, respecting ROS2 design preferences.
- Minimal change: The application layer is responsible for automating network QoS configuration. This represents minimal disruption to the ROS framework.

> 使用方便：应用程序开发人员只需要决定是否需要为发布者/订阅者设置唯一的流标识符。
> 轻量级实现：非 DDS 和 DDS RMW 都可以使用本地套接字 API 方便地实现所需的支持，对性能影响可以忽略不计。
> 不偏好任何特定的网络，尊重 ROS2 设计偏好。
> 应用层负责自动化网络 QoS 配置。这对 ROS 框架的干扰最小。

### Limitations

- If the RMW implementation decides to create a NFE using the IPv4 DSCP field, then only up to 64 (2^6) publishers and subscriptions can be uniquely identified assuming all other resources in the NFE remain constant. In addition, network administration processes should be notified that the DSCP field is re-purposed as an identifier to prevent misinterpretation and erasure.

> 如果 RMW 实施决定使用 IPv4 DSCP 字段创建 NFE，那么假设其他资源保持不变，最多只能唯一标识 64（2^6）个发布者和订阅者。此外，应通知网络管理过程，DSCP 字段被重新用作标识符，以防止误解和抹去。

## Alternative Solutions

We list a few alternative solutions to the problem that are limited and dissatisfactory.

> 我们列出了几种有限且令人不满意的问题替代解决方案。

1. Dedicated nodes: Publishers and subscriptions that require special network QoS can be isolated to dedicated nodes. Such isolation indeed makes their 5-tuple flow identifier unique. However, this breaks the functionality-based node architecture of the application and degrades performance since nodes are heavy-weight structures. In the worst case, a dedicated node per publisher or subscription is required.

> 1. 专用节点：需要特殊网络 QoS 的发布者和订阅者可以被隔离到专用节点。这种隔离确实使得它们的 5 元组流标识符变得独特。但是，这会破坏应用程序的基于功能的节点架构，并降低性能，因为节点是重量级结构。在最坏的情况下，每个发布者或订阅者需要一个专用节点。

2. Custom 6-tuple using side-loaded DDS Transport Priority QoS policies: Conceptually, a custom 6-tuple can be constructed by side-loading unique values into the Transport Priority QoS policy of the DDS RMW implementation. In practice, however, this is difficult to implement for several reasons. First, it expects DDS RMW side-loading competence from application programmers which is inconvenient. Second, re-purposing DSCP values as unique identifiers is limited to 64 identifiers and requires careful network administration as mentioned before. Third, side-loading support varies across DDS RMW implementations. To the best of our knowledge, none of the tier-1 DDS implementations for ROS2 today (Foxy) support side-loading Transport Priority QoS policies for _select few_ publishers and subscriptions in a node due to lack of fine-grained interfaces. A glaring limitation is that this alternative ignores non-DDS RMW.

> 使用侧载 DDS 传输优先级 QoS 策略构建自定义 6 元组：从概念上讲，可以通过在 DDS RMW 实现的传输优先级 QoS 策略中侧载独特的值来构建自定义 6 元组。但是从实践上讲，由于几点原因，这很难实现。首先，它期望应用程序程序员具备 DDS RMW 侧载能力，这很不方便。其次，将 DSCP 值重新用作唯一标识符仅限于 64 个标识符，并且需要仔细进行网络管理，如前所述。第三，DDS RMW 实现的侧载支持各不相同。据我们所知，由于缺乏细粒度接口，ROS2 今天的第一级 DDS 实现（Foxy）不支持在节点上为*选择的少数*发布者和订阅者侧载传输优先级 QoS 策略。一个明显的局限性是，这种替代方案忽略了非 DDS RMW。

3. DS-based QoS using side-loaded DDS Transport Priority QoS policies: This gets ahead of the problem by directly specifying the required DS-based QoS through side-loaded Transport Priority QoS policies. However, this suffers from similar impracticalities as the previous alternative. It ignores non-DDS RMW, expects DS competence from programmers, and is not supported by tier-1 RMW implementations.

> 这种方法通过侧载的传输优先级 QoS 策略直接指定所需的基于 DS 的 QoS，从而避免了前面的问题。但是，它也有类似的不切实际之处，忽略了非 DDS RMW，期望程序员具备 DS 能力，而且一级 RMW 实现不支持它。

## References

[1] [Internet Protocol (IETF RFC-791)](https://tools.ietf.org/html/rfc791)
[2] [IPv6 Flow Label Specification (IETF RFC-6437)](https://tools.ietf.org/html/rfc6437)
[3] [Differentiated Services (IETF RFC-2474)](https://tools.ietf.org/html/rfc2474)
[4] [5G System Architecture Specification (3GPP TS 23.501)](https://portal.3gpp.org/desktopmodules/Specifications/SpecificationDetails.aspx?specificationId=3144)
