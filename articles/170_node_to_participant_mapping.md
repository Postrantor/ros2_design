---
tip: translate by openai@2023-05-29 08:13:41
...
---
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


In ROS, a Node is an entity used to group other entities.

> 在ROS中，节点是用来组织其他实体的实体。

For example: Publishers, Subscriptions, Servers, Clients.

> 例如：出版商、订阅、服务器、客户端。

Nodes ease organization and code reuse, as they can be composed in different ways.

> .

节点可以简化组织和代码重用，因为它们可以以不同的方式组合。

### Domain Participant


A Participant is a type of DDS entity.

> 参与者是DDS实体的一种类型。

Participant also group other entities, like Publishers, Subscribers, Data Writters, Data Readers, etc.

> 参与者还可以分组其他实体，如发布者、订阅者、数据写入者、数据读取者等。

Creating more Participants adds overhead to an application:

> 创建更多参与者会给应用程序增加额外的开销。


- Each Participant participates in discovery.

> 每位参与者都参与发现。

  Creating more than one Participant usually increases CPU usage and network IO load.

> 创建多个参与者通常会增加CPU使用率和网络IO负载。

- Each Participant keeps track of other DDS entities.

> 每个参与者都跟踪其他DDS实体。

  Using more than one within a single process may result in data duplication.

> 使用一个过程中的多个可能会导致数据重复。

- Each Participant may create multiple threads for event handling, discovery, etc.

> 每位參與者可以創建多個線程來處理事件、發現等等。

  The number of threads created per Participant depends on the DDS vendor (e.g.: [RTI Connext](https://community.rti.com/best-practices/create-few-domainParticipants-possible)).

> 参与者每次创建的线程数取决于DDS供应商（例如：[RTI Connext](https://community.rti.com/best-practices/create-few-domainParticipants-possible)）。


For those reasons, a Participant is a heavyweight entity.

> 因此，参与者是一个重量级实体。


> [NOTE]:
Many DDS vendors, however, do not perform this kind of optimization (e.g.: `RTI Connext` and `Fast-RTPS`), and actually recommend creating just one Participant per process.

> 许多DDS供应商但是不执行这种优化（例如：RTI Connext和Fast-RTPS），实际上建议每个进程只创建一个参与者。

### Context


In ROS, a Context is the non-global state of an init-shutdown cycle.

> 在ROS中，上下文是初始化-关闭周期的非全局状态。

It also encapsulates shared state between Nodes and other entities.

> 它还封装了节点和其他实体之间的共享状态。

In most applications, there is only one ROS Context in a process.

> 在大多数应用中，一个进程中只有一个ROS上下文。

## Behavior pre-Foxy


There is a one-to-one mapping between Nodes and DDS Participants.

> 在节点和DDS参与者之间存在一对一映射。

This simplified the original implementation, as DDS Participants provide many features equivalent to the ones of ROS Nodes.

> 这简化了原始实现，因为DDS参与者提供了与ROS节点相当的许多功能。

The drawback of this approach is the overhead that comes with creating many Participants.

> 这种方法的缺点是创建许多参与者会带来额外的开销。

Furthermore, the maximum number of Domain Participants is rather small.

> 此外，域参与者的最大数量相当少。

For example, in [RTI Connext](https://community.rti.com/kb/what-maximum-number-Participants-domain) it is limited to 120 Participants per Domain.

> 例如，在RTI Connext中，每个域的参与者数量最多限制为120个。

## Proposed approach


The goal of this proposal is to improve overall performance by avoiding the creation of one Participant per Node.

> 本提案的目的是通过避免在每个节点上创建一个参与者来提高整体性能。

API changes will be avoided, if possible.

> 如果可能，将避免API的更改。

### Mapping of DDS Participant to a ROS entity


There are two alternatives, besides the one-to-one Node to Participant mapping used pre-Foxy:

> 有两种选择，除了在Foxy之前使用的一对一节点到参与者映射之外：

- Using one Participant per process.

> 使用一个参与者每个过程。

- Using one Participant per Context.

> 使用每个上下文中的一个参与者。


The second approach is much more flexible, allowing more than one Participant in a single application for those that need it e.g. domain bridge applications.

> 第二种方法更加灵活，允许一个应用中有多个参与者，对于那些需要的，例如域桥应用程序。

Thus, a one-to-one Participant to Context mapping was chosen.

> 因此，选择了一对一的参与者与上下文的映射。


When multiple Nodes are running in a single process, there are different options for grouping them by - ranging from a separate context for each Node, over grouping a few Nodes in the same context, to using a single context for all Nodes.

> 当多个节点在同一个进程中运行时，可以通过从每个节点的独立上下文到将几个节点组合在同一上下文中，再到使用单一上下文来组织所有节点，来对它们进行分组。

For most applications, only one Context is created.

> 大多数应用程序只创建一个上下文。

### Discovery information


If a one-to-one Node to Participant mapping is not used, extra discovery information is needed to be able to match other entities to a Node e.g. Publishers, Subscriptions, etc.

> 如果不使用一对一的节点到参与者映射，则需要额外的发现信息才能将其他实体与节点匹配，例如发布者、订阅等。

Several approaches can be used to share this information.

> 几种方法可以用来共享这些信息。

The proposed approach uses a topic for it.

> 提出的方法使用一个主题来实现它。

Each Participant publishes a message with all the information needed to match an entity to a Node.

> 每位參與者都會發布一條消息，其中包含所有必要的信息，以將实体與節點匹配。

The message structure is the following:

> 消息结构如下：


* ParticipantInfo

> * 参与者信息

  * gid

> * gid：全局唯一标识符

  * NodeInfo

> * 节点信息
    * Node namespace
    * Node name
    * Reader gid
    * writed gid


When one entity is updated (e.g.: a Publisher is created or destroyed), a new message is sent.

> 当一个实体被更新时（例如：创建或销毁一个发布者），将会发送一条新消息。


Identification of Clients and Servers happens according to the ROS conventions for their topic names (see [	

> 客户端和服务器的识别根据ROS的主题名称约定进行（参见[

Topic and Service name mapping to DDS](140_topic_and_service_name_mapping.md)).

> 主题和服务名称映射到DDS


This topic is considered an implementation detail, and not all `rmw` implementations have to use it.

> 这个主题被认为是一个实现细节，并不是所有的`rmw`实现都必须使用它。

Thus, all the necessary logic has to be in the rmw implementation itself or in an upstream package.

> 因此，所有必要的逻辑都必须在rmw实现本身或上游包中。

Implementing this logic in `rcl` would make it part of the API, and not an implementation detail.

> 在`rcl`中实现这种逻辑将使其成为API的一部分，而不是实现细节。


To avoid code repetition, a common implementation of this logic is provided by the [rmw_dds_common](https://github.com/ros2/rmw_dds_common/) package.

> 为了避免代码重复，[rmw_dds_common](https://github.com/ros2/rmw_dds_common/) 包提供了这种逻辑的常见实现。

#### Details of the ROS discovery topic


- topic name: `ros_discovery_info`

> 主题名称：ros_discovery_info

- Writer qos:

> 写作质量：

  - durability: transient local

> 耐久性：暂时的本地

  - history: keep last

> - 历史：保留最后一次

  - history depth: 1

> 历史深度：1

  - reliability: reliable

> 可靠性：可靠的

- Reader qos:

> 阅读者服务质量：

  - durability: transient local

> 耐久性：短暂的本地

  - history: keep all

> - 历史：保存全部

  - history depth: 1

> 历史深度：1

  - reliability: reliable

> 可靠性：可靠的

## Other implications

#### Security


Previously, each Node could have different security artifacts.

> 以前，每个节点可以有不同的安全工件。

That was possible because each Node was mapped to one Participant.

> .

因为每个节点都映射到一个参与者，所以这是可能的。

The new approach allows to specify different security artifacts for each process.

> 新方法允许为每个流程指定不同的安全工件。

For more details, see [ROS 2 Security Enclaves](182_ros2_security_enclaves.md).

> 更多细节，请参见[ROS 2 安全飞地](182_ros2_security_enclaves.md)。

#### Ignore local publications option


There is an `ignore_local_publications` option that can be set when [creating a Subscription](https://github.com/ros2/rmw/blob/2250b3eee645d90f9e9d6c96d71ce3aada9944f3/rmw/include/rmw/rmw.h#L517).

> 有一个可以在[创建订阅](https://github.com/ros2/rmw/blob/2250b3eee645d90f9e9d6c96d71ce3aada9944f3/rmw/include/rmw/rmw.h#L517)时设置的`ignore_local_publications`选项。

That option avoids receiving messages from Publishers within the same Node.

> 那个选项可以避免接收同一节点内出版者发送的消息。

This wasn't implemented in all the rmw implementations (e.g.: [FastRTPS](https://github.com/ros2/rmw_fastrtps/blob/099f9eed9a0f581447405fbd877c6d3b15f1f26e/rmw_fastrtps_cpp/src/rmw_Subscription.cpp#L118)).

> 这并没有被实现在所有的rmw实现中（例如：[FastRTPS](https://github.com/ros2/rmw_fastrtps/blob/099f9eed9a0f581447405fbd877c6d3b15f1f26e/rmw_fastrtps_cpp/src/rmw_Subscription.cpp#L118))。


After this change, implementing this feature will be less direct.

> 经过这次变更，实现这个功能将不太直接。

Some extra logic needs to be added in order to identify from which Node a Publisher was created.

> 需要添加一些额外的逻辑来识别发布者是从哪个节点创建的。

## Alternative implementations

### Using keyed topics


Keyed topics could be used, with the Participant gid as the key.

> 可以使用带有参与者gid作为键的主题。

That would allow the Reader side of the topic to use a keep last, depth 1 history.

> 这将允许话题的读者端使用最后一个，深度为1的历史记录。

### Using Participant/Writer/Reader userData QoS


Instead of using a custom topic to share the extra ROS specific discovery information, a combination of Participant/Reader/Writer `userData` could be used:

> 取代使用自定义主题来共享额外的ROS特定发现信息，可以使用参与者/读者/写者的`userData`组合：


- Participant `userData`: list of Nodes created by the Participant (updated when Node created destroyed).

> 参与者`userData`：参与者创建的节点列表（在节点创建和销毁时更新）。

- Reader user `userData`: Node name/namespace.

> 读取用户`userData`：节点名称/命名空间。

- Writer user `userData`: Node name/namespace.

> 写入用户`userData`：节点名称/命名空间。


That information would be enough to satisfy ROS required graph API.

> 那些信息足以满足ROS所需的图形API。

This mechanism would also preclude having a topic that has to be read and written by all Nodes, which is better from a security perspective.

> .

这种机制也可以避免所有节点都必须读取和写入的主题，从安全性的角度来看更好。


This alternative wasn't implemented because of lack of support in some of the currently supported DDS vendors.

> 这个替代方案没有实施，是因为当前支持的DDS供应商中有些支持不足。

## Further work


This optimization has been applied to `rmw_cyclonedds`, `rmw_fastrtps_cpp` and `rmw_fastrtps_dynamic_cpp`.

> 这种优化已经应用于`rmw_cyclonedds`、`rmw_fastrtps_cpp`和`rmw_fastrtps_dynamic_cpp`。

Other DDS based `rmw` implementations, like `rmw_connext_cpp` could use the same approach.

> 其他基于DDS的`rmw`实现，如`rmw_connext_cpp`可以使用相同的方法。
