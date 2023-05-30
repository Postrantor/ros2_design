---
translate by baidu@2023-05-30 23:31:20
layout: default
title: Node to Participant mapping
permalink: articles/Node_to_Participant_mapping.html
abstract: This article analyzes the performance implications of enforcing a one-to-one mapping between ROS Nodes and DDS Participants, and proposes alternative implementation approaches.
author: '[Ivan Paunovic](https://github.com/ivanpauno)'
date_written: 2020-06
last_modified: 2020-07
published: true
categories: Middleware

Authors: {{ page.author }}
Date Written: {{ page.date_written }}
Last Modified: {% if page.last_modified %}{{ page.last_modified }}{% else %}{{ page.date_written }}{% endif %}
---

## Background

### Node

In ROS, a Node is an entity used to group other entities. For example: Publishers, Subscriptions, Servers, Clients. Nodes ease organization and code reuse, as they can be composed in different ways.

> 在 ROS 中，节点是用于对其他实体进行分组的实体。例如：发布服务器、订阅、服务器、客户端。节点简化了组织和代码重用，因为它们可以以不同的方式组合。

### Domain Participant

A Participant is a type of DDS entity. Participant also group other entities, like Publishers, Subscribers, Data Writters, Data Readers, etc. Creating more Participants adds overhead to an application:

> 参与者是一种 DDS 实体。**参与者还将其他实体分组**，如发布者、订阅服务器、数据写入器、数据读取器等。创建更多参与者会增加应用程序的开销：

- Each Participant participates in discovery. Creating more than one Participant usually increases CPU usage and network IO load.
- Each Participant keeps track of other DDS entities. Using more than one within a single process may result in data duplication.
- Each Participant may create multiple threads for event handling, discovery, etc. The number of threads created per Participant depends on the DDS vendor (e.g.: [RTI Connext](https://community.rti.com/best-practices/create-few-domainParticipants-possible)).

- 每个参与者都参与发现。创建多个参与者通常会增加 CPU 使用率和网络 IO 负载。
- 每个参与者都跟踪其他 DDS 实体。在一个过程中使用多个过程可能会导致数据重复。
- 每个参与者**可能会创建多个线程用于事件处理，发现等。每个参与者创建的线程数取决于 DDS 供应商**(例如：[rti connext](https：//community.rti.rti.com/best-practices/create-Few-domainpartipant-Prables))。

For those reasons, a Participant is a heavyweight entity.

> 出于这些原因，参与者是一个重量级实体。

### Context

In ROS, a Context is the non-global state of an init-shutdown cycle. It also encapsulates shared state between Nodes and other entities. In most applications, there is only one ROS Context in a process.

> 在 ROS 中，**上下文是初始化关闭周期的非全局状态**。它还**封装了节点和其他实体之间的共享状态**。在大多数应用程序中，一个过程中只有一个 ROS 上下文。

## Behavior pre-Foxy

There is a one-to-one mapping between Nodes and DDS Participants. This simplified the original implementation, as DDS Participants provide many features equivalent to the ones of ROS Nodes. The drawback of this approach is the overhead that comes with creating many Participants. Furthermore, the maximum number of Domain Participants is rather small. For example, in [RTI Connext](https://community.rti.com/kb/what-maximum-number-Participants-domain) it is limited to 120 Participants per Domain.

> **节点和 DDS 参与者之间存在一对一映射**。这简化了最初的实现，因为 DDS 参与者提供了许多与 ROS 节点等效的功能。这种方法的**缺点是创建许多参与者所带来的开销**。此外，领域参与者的最大数量相当少。例如，在[RTI 连接文本]中(https://community.rti.com/kb/what-maximum-number-Participants-domain)每个域限制为120个参与者。

## Proposed approach

The goal of this proposal is to improve overall performance by avoiding the creation of one Participant per Node. API changes will be avoided, if possible.

> 该方案的目标是通过**避免每个节点创建一个参与者来提高整体性能**。如果可能，将避免更改 API。

### Mapping of DDS Participant to a ROS entity

There are two alternatives, besides the one-to-one Node to Participant mapping used pre-Foxy:

> 除了 Foxy 之前使用的一对一节点到参与者映射之外，还有两种替代方案：

- Using one Participant per process.
- Using one Participant per Context.

The second approach is much more flexible, allowing more than one Participant in a single application for those that need it e.g. domain bridge applications. Thus, a one-to-one Participant to Context mapping was chosen.

> 第二种方法要灵活得多，允许在单个应用程序中有多个参与者来满足需要，例如域桥应用程序。因此，**选择了一对一的参与者到上下文映射**。

When multiple Nodes are running in a single process, there are different options for grouping them by - ranging from a separate context for each Node, over grouping a few Nodes in the same context, to using a single context for all Nodes. For most applications, only one Context is created.

> 当多个节点在一个进程中运行时，有不同的选项可以通过以下方式对它们进行分组：从每个节点使用单独的上下文，在同一上下文中对几个节点进行分组，到对**所有节点使用单个上下文**。对于大多数应用程序，只创建一个上下文。

### Discovery information

If a one-to-one Node to Participant mapping is not used, extra discovery information is needed to be able to match other entities to a Node e.g. Publishers, Subscriptions, etc. Several approaches can be used to share this information. The proposed approach uses a topic for it. Each Participant publishes a message with all the information needed to match an entity to a Node. The message structure is the following:

> 如果不使用一对一的节点到参与者映射，则需要**额外的发现信息才能将其他实体与节点相匹配**，例如发布者、订阅等。可以使用多种方法来共享这些信息。所提出的方法使用了一个主题。每个参与者发布一条消息，其中包含将实体与节点匹配所需的所有信息。消息结构如下：

- ParticipantInfo
  - gid
  - NodeInfo
    - Node namespace
    - Node name
    - Reader gid
    - writed gid

When one entity is updated (e.g.: a Publisher is created or destroyed), a new message is sent.

> 当更新一个实体(例如：创建或销毁发布服务器)时，会发送一条新消息。

Identification of Clients and Servers happens according to the ROS conventions for their topic names (see [ Topic and Service name mapping to DDS](140_topic_and_service_name_mapping.md)).

> 客户端和服务器的标识是根据其主题名称的 ROS 约定进行的(请参见[topic and Service name mapping to DDS](140_topic_and_Service_name_mapping.md))。

This topic is considered an implementation detail, and not all `rmw` implementations have to use it. Thus, all the necessary logic has to be in the rmw implementation itself or in an upstream package. Implementing this logic in `rcl` would make it part of the API, and not an implementation detail.

> 这个主题被认为是一个实现细节，并不是所有的“rmw”实现都必须使用它。因此，所有必要的逻辑都必须在 rmw 实现本身或上游包中。在“rcl”中实现此逻辑将使其成为 API 的一部分，而不是实现细节。

To avoid code repetition, a common implementation of this logic is provided by the [rmw_dds_common](https://github.com/ros2/rmw_dds_common/) package.

> 为了避免代码重复，该逻辑的通用实现由[rmw_dds_common]提供(https://github.com/ros2/rmw_dds_common/)包装。

#### Details of the ROS discovery topic

- topic name: `ros_discovery_info`
- Writer qos:
  - durability: transient local
  - history: keep last
  - history depth: 1
  - reliability: reliable
- Reader qos:
  - durability: transient local
  - history: keep all
  - history depth: 1
  - reliability: reliable

## Other implications

#### Security

Previously, each Node could have different security artifacts. That was possible because each Node was mapped to one Participant. The new approach allows to specify different security artifacts for each process. For more details, see [ROS 2 Security Enclaves](182_ros2_security_enclaves.md).

> 以前，每个节点可能有不同的安全工件。这是可能的，因为每个节点都映射到一个参与者。**新方法允许为每个进程指定不同的安全工件**。有关更多详细信息，请参阅[ROS 2 Security Enclaves](182_ros2_Security_enviders.md)。

#### Ignore local publications option

There is an `ignore_local_publications` option that can be set when [creating a Subscription](https://github.com/ros2/rmw/blob/2250b3eee645d90f9e9d6c96d71ce3aada9944f3/rmw/include/rmw/rmw.h#L517). That option avoids receiving messages from Publishers within the same Node. This wasn't implemented in all the rmw implementations (e.g.: [FastRTPS](https://github.com/ros2/rmw_fastrtps/blob/099f9eed9a0f581447405fbd877c6d3b15f1f26e/rmw_fastrtps_cpp/src/rmw_Subscription.cpp#L118)).

> 在[创建订阅]时可以设置“ignore_local_publications”选项。该选项可避免从同一节点内的发布服务器接收消息。这并没有在所有的rmw实现中实现(例如：[FastRTPS])。

After this change, implementing this feature will be less direct. Some extra logic needs to be added in order to identify from which Node a Publisher was created.

> 在这个变化之后，实现这个特性将不那么直接。需要添加一些额外的逻辑，以确定发布服务器是从哪个节点创建的。

## Alternative implementations

### Using keyed topics

Keyed topics could be used, with the Participant gid as the key. That would allow the Reader side of the topic to use a keep last, depth 1 history.

> 可以使用带关键字的主题，并将 Participant gid 作为关键字。这将允许主题的读者侧使用保留最后一个深度 1 历史记录。

### Using Participant/Writer/Reader userData QoS

Instead of using a custom topic to share the extra ROS specific discovery information, a combination of Participant/Reader/Writer `userData` could be used:

> 可以使用参与者/读者/作者`userData`的组合，而不是使用自定义主题来共享额外的 ROS 特定发现信息：

- Participant `userData`: list of Nodes created by the Participant (updated when Node created destroyed).
- Reader user `userData`: Node name/namespace.
- Writer user `userData`: Node name/namespace.

That information would be enough to satisfy ROS required graph API. This mechanism would also preclude having a topic that has to be read and written by all Nodes, which is better from a security perspective.

> 这些信息足以满足 ROS 要求的 API 图形。这种机制还将排除必须由所有节点读取和写入的主题，这从安全角度来看更好。

This alternative wasn't implemented because of lack of support in some of the currently supported DDS vendors.

> 由于目前一些受支持的 DDS 供应商缺乏支持，因此没有实施此替代方案。

## Further work

This optimization has been applied to `rmw_cyclonedds`, `rmw_fastrtps_cpp` and `rmw_fastrtps_dynamic_cpp`. Other DDS based `rmw` implementations, like `rmw_connext_cpp` could use the same approach.

> 此优化已应用于“rmw_cyclonedds”、“rmw_fastrtps_cpp”和“rmw_fastrtps_dynamic_cpp”。其他基于 DDS 的“rmw”实现，如“rmw_connext_cpp”，也可以使用相同的方法。
