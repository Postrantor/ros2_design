---
tip: translate by openai@2023-05-30 13:29:04
layout: default
title: Topological Discovery and Communication Negotiation
abstract:
  This article lays out the logical components and possibilities within a discovery and transport negotiation system. This article was written to try and understand the different possibilities for how the middleware could be implemented.
  本文列出了发现和运输谈判系统中的逻辑组成部分和可能性。本文的编写是为了尝试了解如何实现中间件的不同可能性。
published: true
author: '[William Woodall](https://github.com/wjwwood)'
date_written: 2014-06
last_modified: 2019-03

  For context, this article explores the ideal and theoretical aspects of discovery and negotiation. It does not aim to answer all implementation questions or suggest implementation strategies. It simply tries to capture the concepts in the design space and identify trade-offs and relationships between design elements.
  对于上下文，本文探讨了发现和谈判的理想和理论方面。它并不是要回答所有实施问题或建议实施策略。它只是试图捕获设计空间中的概念，并确定设计元素之间的权衡和关系。

Authors: 
Date Written: 
Last Modified:
---
> [!NOTE]
> 这里主要谈到的是 graph 的概念，实际上 ros2 中有 rqt_graph 的工具，可以查看当前 node 组成的 graph 之间的连接关系
> 进一步的，这里就涉及到 node 生命周期的概念
> 尤其是 `## Node Life Cycles` 这一节的内容

---

## Problem Space

ROS systems tend to be implemented as a **computational graph**, where there are graph **node**'s connected by **topic**'s and **service**'s. The graph which models the computational nodes with their topics and services can be different from the graph which represents the physical groupings and connections which implement the behavior modeled in the topics and services graph.

> **ROS 系统往往被实现为计算图**，其中有图节点由主题和服务连接。模拟计算节点及其主题和服务的图可能与表示实现主题和服务图中模型行为的物理分组和连接的图不同。

For the purposes of this document, the graph which defines the computational nodes and how they are connected using topics and services will be referred to as the **computational graph**. The graph which models processes around nodes and physical layers between nodes will be referred to as the **data layer graph**. A **node** is any addressable participant in the **computational graph**, it does not imply how nodes are organized into system processes, i.e. a node is neither necessarily a single process, nor does it necessarily share a process with other nodes.

> 为了本文件的目的，用主题和服务连接计算节点的图表被称为**计算图**。模拟节点和节点之间物理层的过程的图表被称为**数据层图**。**节点**是**计算图**中可寻址的参与者，它不意味着节点如何组织到系统进程中，即**节点既不一定是单个进程，也不一定与其他节点共享进程**。

To demonstrate the difference between the **computational graph** and the **data layer graph**, consider the following:

> 为了证明计算图和数据层图之间的区别，请考虑以下内容：

- A computational graph has nodes N1, N2, N3, and N4.
- N1 is on machine M1 and in process P1.
- N2 is on machine M1 and in process P1.
- N3 is on machine M1 and in process P2.
- N4 is on machine M2 and in process P3.
- N1, N2, N3, and N4 are all publishing and subscribing to topic T1.
- N1 is publishing to topic T2, while N2, N3, and N4 are subscribing to topic T2.

For the purposes of this example, assume these computational nodes can optionally share processes and be on different machines. How this is accomplished is not relevant to this document, but might be covered in another document.

> 对于这个例子，假设这些计算节点可以选择共享进程，并且可以在不同的机器上运行。如何实现这一点不在本文档中涉及，但可能在另一个文档中有所涉及。

The above describes how the **computational graph** is organized from a conceptual point of view. The existence of a node, its host machine and host process are products of the graph's execution, which would be done by the user executing each in turn, or by a process management system. The topic publications and subscriptions are defined at runtime by the user code in each of the nodes, though for some use cases the topics might also have to be captured externally and statically.

> 以上描述了从概念上看计算图的组织方式。节点的存在、其所在的机器和进程都是图的执行产物，这可以由用户一步步执行，或者由进程管理系统来完成。主题发布和订阅是由用户在每个节点中的代码定义的，尽管在某些用例中，主题也可能需要在外部静态捕获。

The above constraints do not at all describe the method by which messages over topics are delivered to various nodes of the graph, this is known as the **data layer graph**. It is the responsibility of some entity, or entities, to use this **computational graph** (and optionally the information about process and machine layout) to decide how the **data layer graph** should be implemented and then execute that implementation.

> 上述约束并不描述消息如何通过主题发送到图形的各个节点，这被称为**数据层图**。某个实体或实体有责任使用这个**计算图**(可选择性地使用关于过程和机器布局的信息)来决定**数据层图**应该如何实现，然后执行该实现。

In the current ROS system, each node reports its address and configuration to a master process which coordinates the graph. The master is responsible for maintaining the graph state and notifying nodes of relevant changes to the graph, e.g. a new publisher of a topic which this node subscribes to was created. The nodes will then initiate connections to other nodes where appropriate, so in this sense the nodes are somewhat autonomous. One could argue that ROS should have multiple masters rather than sharing one master amongst multiple machines, or that there should be no master and all of the nodes should be completely autonomous.The point here is that there is no one-size fits all solution, therefore this paper will try to identify ways that these discovery and negotiation steps can be abstracted such that many different implementations may be supported.

> 在当前的 ROS 系统中，**每个节点都会向一个主进程报告其地址和配置，该主进程负责协调图形。主进程负责维护图形状态，并通知节点有关图形的相关变化，例如，此节点订阅的新发布者已创建**。然后，节点将在适当的情况下与其他节点建立连接，因此，从某种意义上讲，节点有一定的自主性。可以说，ROS 应该有多个主而不是在多台机器之间共享一个主，或者不应该有主，而所有节点都应该完全自主。重点是没有一种通用的解决方案，因此本文将尝试确定这些发现和协商步骤的抽象方式，以支持多种不同的实现。

> [!NOTE]
> 存在 manager 的角色？
> 最直接相关的就是 “ROS2 Lifecycle Manager”。

## Use Cases

For the proposed solution to this design space, this paper will build up the interfaces in layers, such that for different use cases the process can be gracefully degraded. Therefore, this paper will list the use cases in order of complexity.

> 对于这个设计空间的提议解决方案，本文将以分层的方式构建接口，以便为不同的用例提供优雅的降级过程。因此，本文将按复杂程度列出用例。

### Statically Configured Nodes

The most basic use case is where each node is given a list of declarative instructions on how to connect to the data layer. These instructions would likely come in the form of url's, but would be of the notion "Connect to Topic `<topic>` of type `<msg_type>` via `<protocol>://<address>:<port>/` using `<transport>` and `<serialization>`". A more concrete example might be "Connect to Topic `/foo` of type `pkg/Foo` via `udpm://225.82.79.83:11311/` using `<zmq_pgm>` and `<protobuf>`".

> 最基本的用例是，每个节点都被赋予了一个声明性指令列表，用于如何连接到数据层。这些指令可能以 url 的形式出现，但其概念是“使用“< 传输 >”和“< 序列化 >”连接到类型为“<msg_type>”的主题“<topic>” via `<protocol>://<address>:<port>/`。一个更具体的例子可能是“连接到类型为 `pkg/Foo` 的主题 `/foo` via `udpm://225.82.79.83:11311/` 使用 `<zmq_pgm>` 和 `<protobuf>`”。

The basic building block that this requires is that nodes provide an API which allows something to instruct the node to establish some connection ("connect_to") and to map publishers and subscribers to a given connection ("map_to"). The program parsing the declarative instructions would just iterate over the instructions, calling this "connect_to" function for each url and then the "map_to" function on each publisher and subscriber.

> 这需要的基本构建块是，节点提供一个 API，允许某些东西指示节点建立一些连接(“connect_to”)，并将发布者和订阅者映射到给定的连接(“map_to”)。解析声明性指令的程序只需要遍历这些指令，为每个 url 调用“connect_to”函数，然后为每个发布者和订阅者调用“map_to”函数。

> [!NOTE]
> 这里提到了一个有意思的想法：**指示节点建立一些连接(“connect_to”)**

It should be noted that at this point, this "map_to" call should fail if the node is asked to do something which it has not previously set itself up to do, e.g. a node is asked to map a topic and subscriber to a connection for which it has not created a Subscriber instance. This constraint implies the need for a life cycle where any publishers and subscribers are instantiated by the node in one step and then connections are made in another step.

> 需要注意的是，在这一点上，如果节点被要求做一些以前没有准备好的事情，比如要求节点将主题和订阅者映射到尚未创建订阅者实例的连接，此时此"map_to"调用将失败。**这一约束意味着需要一个生命周期，其中任何发布者和订阅者都是由节点在一个步骤中实例化的，然后在另一个步骤中建立连接。**

> [!NOTE]
> 为什么需要声明周期！

Because the decisions about how to connect to the data layer are static, it also removes the possibility for dynamically creating publishers and subscribers on the fly because no entity will be watching for new publishers/subscribers and dynamically determining and executing data layer connections.

> 由于关于如何连接到数据层的决定是静态的，它也消除了动态创建发布者和订阅者的可能性，因为没有实体会监视新的发布者/订阅者，并动态确定和执行数据层连接。

Another point is that when nodes lose connection with each other on the data layer (temporary loss of network), the node implementation which calls "connect_to" would be responsible for issuing a new "connect_to" after the connection dies. This implies there should be away to introspect the node by polling it or by getting notifications about the state of the underlying connections which were created.

> 另一个重要的一点是，当节点在数据层上失去连接(网络暂时丢失)时，调用“connect_to”的节点实现将负责在连接断开后发出新的“connect_to”。这意味着应该有一种方法可以通过轮询或获取有关创建的底层连接状态的通知来内省节点。

### Statically Configured Graph

The next more complicated system is one where each node starts and waits for an outside process to tell it how to make its connections to the data layer. In this scenario a central authority has the static configuration of all node addresses and knows how they should be connected to each other in the data layer. In order to execute this, the central authority needs to be able to call the previously described "connect_to" and "map_to" functions remotely.

> 下一个更复杂的系统是，**每个节点都启动并等待外部进程告知它如何与数据层连接。在这种情况下，中央权威机构具有所有节点地址的静态配置，并知道它们应如何在数据层中相互连接。为了执行这一点，中央权威机构需要能够远程调用先前描述的“connect_to”和“map_to”函数。**

This adds the necessity for a node to provide an RPC interface for the "connect_to" and "map_to" functions.

> 这就增加了节点提供 RPC 接口以支持"connect_to"和"map_to"功能的必要性。

Like the previous use case, ideally someone should be monitoring the state of the connections, reactivating them if necessary. If the central authority has this responsibility then it should have some way of introspecting the state of these connections. This leads to a need for introspection of graph participants be available externally as well.

> 像之前的用例一样，理想情况下应该有人监视连接的状态，如果有必要，重新激活它们。如果中央当局负责此职责，则应该有某种方式来检查这些连接的状态。这就需要图形参与者的内省信息也可以外部访问。

The main evolution of required functionality for this system over the previous one is that graph participant's interfaces must be remotely accessible. The ability to remotely access the "connect_to" and "map_to" functions is a requirement, where as the ability to access the node's state remotely is only required when an external process is monitoring the connections that were established.

> 与之前的系统相比，此系统所需功能的主要演变是图参与者的接口必须远程可访问。要求具有远程访问“connect_to”和“map_to”功能的能力，而当外部进程监视所建立的连接时，才需要远程访问节点状态的能力。

### A Fork in the Features

This is were the system features matrix forks. There are two glaring limitations of the previous use cases:

> 这就是系统特性矩阵的分叉点。之前用例的两个明显的局限性是：

- Nodes have no general way to notify the rest of the graph about events, events like:

  - node life cycle state changed
  - heartbeat
  - connection_established/connection_lost
  - mapping_established/mapping_lost
  - etc...
- The list of nodes and their addresses, publishers, subscribers, etc. are statically maintained, which prevents:

  - dynamically computing the data layer connections
  - dynamically adding nodes
  - dynamically adding publishers and subscribers

The first limitation is about having the ability to introspect the changes in the **data layer graph** so that the system can react dynamically to things like dropped connections. The second limitation is about having the ability to dynamically discover and manipulate the layout of the **computational graph** which might in turn change the **data layer graph**.

> 第一个限制是关于有能力反省**数据层图**的变化，以便系统可以动态地对掉线等事件做出反应。第二个限制是关于能够动态发现和操纵**计算图**的布局，这可能会改变**数据层图**。

First this paper will look at how the system can be dynamically configured.

> 首先，本文将探讨如何动态配置系统。

### Dynamically Configured Graph with Static Discovery

One of the issues with the "Statically Configured Graph" system described above is that the topics and/or services each node provides or uses must be statically defined, either as part of the configuration for each node, or as part of the centralized authority's configuration. This does not allow for dynamically defined topic subscriptions and publications nor dynamically provided services.

> 一个关于上面描述的“静态配置图”系统的问题是，每个节点提供或使用的主题和/或服务必须静态定义，作为每个节点的配置的一部分，或作为中央权威机构的配置的一部分。这不允许动态定义的主题订阅和发布以及动态提供的服务。

In order to enable these type of flexible or dynamic node configurations, each node must provided an externally accessible function for getting the configuration of itself. This would allow a central authority to periodically check each node for new topic subscriptions or publications and/or new service providers and dynamically change the **data layer graph** layout to reflect these changes.

> 为了实现这种灵活或动态的节点配置，**每个节点必须提供一个可外部访问的函数来获取自身的配置**。这将允许中央权威机构定期检查每个节点的新主题订阅或发布以及/或新的服务提供商，并动态更改**数据层图**布局以反映这些变化。

This also allows for a system where the list of graph participants and their location is known, but the data layer graph is unknown and can be determined at runtime. This would allow a small additional flexibility over a completely statically configured system.

> 这也允许建立一个系统，其中图表参与者的列表及其位置是已知的，但数据层图是未知的，可以在运行时确定。这将使完全静态配置的系统具有一定的额外灵活性。

### Statically Configured Graph with Data Layer Events

The other issue with the "Statically Configured Graph" system above is that it cannot easily monitor the state of each of the graph participants and their connections because it would require some form of polling or point to point event systems. In order to better facilitate use cases where graph state would be maintained in a decentralized manner, point to point events should be avoided (at least conceptually), and instead participants in the graph should be able to send messages (events) to the "graph" notifying the rest of the graph participants of changes to their own state, and anything which wants to monitor the state of the graph should be able to maintain a consistent state of the graph by listening to messages sent to the graph by its participants. This assumes that nodes are the authority of their state in the graph.

> 其他问题是，上述“静态配置图”系统**无法轻松监视图中各个参与者及其连接的状态，因为它需要某种形式的轮询或点对点事件系统**。为了更好地实现图状态在分散管理的情况下保持的用例，**应该尽量避免点对点事件(至少在概念上)，而是图中的参与者应该能够向“图”发送消息(事件)，通知其他图参与者自身状态的变化**，以及任何想要监视图状态的东西都应该能够通过监听图中参与者发送给图的消息来维持图的一致状态。这假设节点是图中状态的权威。

This begins to outline the need for a graph interface, which allows a user to maintain the state of the graph, get asynchronous notifications of graph changes, and send events to the graph. What the graph interface looks like is discussed in a later section.

> 这开始勾勒出需要一个图形界面的需求，它允许用户维护图形的状态，获取图形变化的异步通知，并向图形发送事件。图形界面的外观将在后面的部分进行讨论。

With a graph interface anyone, either a graph participant or an observer, can monitor the state of the graph and potentially react to changes in the graph. This allows for scenarios like a long running central authority which can setup the graph initially, and restore any connections when necessary at runtime.

> 有了图形界面，任何人，无论是图形参与者还是观察者，都可以监视图形的状态，并可能对图形的变化做出反应。这允许长期运行的中央权威机构最初设置图形，并在运行时恢复任何连接。

> [!NOTE]
>
> [rqt_graph](C:%5CUsers%5Ctrantor%5CDocuments%5CHirain%5CProject%5Crolling%5Cros-visualization%5Crqt_graph)

Along with data layer events, comes the notion of liveliness. When a connection is terminated for some reason an event should be sent to the graph, but often the reason for a disconnect will be that one end of the connection has dropped off the graph unexpectedly and therefore an event is unlikely to reach the graph. For this reason, it makes sense to include liveliness into the system when data layer events are added. Liveliness is not required, but could be added to any system which has a notion of the graph interface and is able to send and receive messages to the graph, these messages would be some form of heartbeat.

> 随着数据层事件的出现，活力(liveliness)的概念也随之而来。当连接因某种原因中断时，应该向图表发送一个事件，但是通常断开连接的原因是一端意外地从图表中断开，因此事件不太可能到达图表。因此，在添加数据层事件时，包含活力是有意义的。活力不是必需的，但可以添加到任何具有图形界面概念并能够向图表发送和接收消息的系统中，这些消息将是某种形式的心跳。

### Dynamically Configured Graph with Data Layer Events and Static Discovery

This system simply adds the data layer events (connection established/lost, heartbeat, etc...) on top of the "Dynamically Configured Graph with Static Discovery". This system would be able to take a static set of nodes, with addresses, and dynamically detect their configuration, determine an appropriate data layer graph, and execute it. It would also be able to adapt to a change in the configuration of the node and adapt to data layer events, like temporarily lost connections, or connection state introspection.

> 这个系统只是在“动态配置图与静态发现”之上添加数据层事件(连接建立/丢失，心跳等)。**这个系统能够拿到一组静态节点，带有地址，并动态检测它们的配置**，确定一个合适的数据层图，并执行它。它也能够适应节点配置的变化，并适应像临时失去连接或连接状态内省等数据层事件。

### Dynamically Configured Graph with Dynamic Discovery

The obvious next step is a system where the participants of the graph and their locations (addresses) are discovered at runtime. This information along with the ability for nodes to dynamically configure their topics and services, allows for a lot of the tools which exist currently in ROS. Being able to add a node to the graph in an ad-hoc manner and then dynamically create publishers and subscribers, allows for dynamic introspection of the graph as well as dynamic development of the graph.

> 显而易见的下一步是建立一个系统，在运行时发现图的参与者及其位置(地址)。这些信息以及节点动态配置主题和服务的能力，可以让 ROS 中现有的大量工具得以利用。可以**以临时方式将节点添加到图中，然后动态创建发布者和订阅者，这样可以动态地观察图，也可以动态地开发图**。

Implementation of this system requires the notion of the graph interface, so that on node creation and termination, the node can send messages to the graph, notifying the rest of the graph of their participation in the graph or their leaving of the graph.

> 实现这个系统需要图形界面的概念，这样，在节点创建和终止时，节点可以向图形发送消息，通知图形中其他节点参与图形或离开图形。

This system does not have the data layer events described in previous systems, though it is likely that once a system is capable of dynamic discovery and dynamic configuration, then the data layer events will likely also be present.

> 这个系统没有在以前的系统中描述的数据层事件，尽管一旦系统具有动态发现和动态配置的能力，那么数据层事件很可能也会出现。

### Dynamically Configured Graph with Data Layer Events and Dynamic Discovery

This is the most fully featured system covered in this paper, as it combines dynamic configuration of nodes (topics and services), dynamic discovery of nodes, and data layer events.

> 这是本文中最完整的系统，因为它结合了节点的动态配置(主题和服务)、节点的动态发现以及数据层事件。

This system is capable of supporting dynamic insertion and removal of nodes in the graph. Each of those nodes can dynamically change their configurations at will. One or more entities can monitor the state of the nodes and their **computational graph** layout, determine part of or a whole **data layer graph** layout, and execute the **data layer graph** layout. Further more these entities can get event driven notifications of changes to the nodes in the graph, changes to their computational graph connections amongst each other, or changes to their data layer connections.

> **这个系统能够支持动态地插入和移除图中的节点。** 这些节点可以随意地动态改变它们的配置。一个或多个实体可以监控节点的状态和它们的计算图布局，确定数据层图布局的一部分或整体，并执行数据层图布局。此外，这些实体可以获得关于图中节点的变化、它们之间计算图连接的变化以及数据层连接的变化的事件驱动通知。

All of these capabilities together allows for complex systems which are capable of dynamic behavior.

> 所有这些功能加在一起，可以实现复杂的系统，具有动态行为能力。

### Summary of Use Cases

Below is a table summarizing the above mentioned use cases and what interfaces/features each of them need to be implemented. All of the systems in the table below require the basic local node API with the "connect_to" and "map_to" functions as well as basic connection introspection.

> 以下是汇总上述用例及其需要实现的接口/功能的表格。下表中的所有系统都需要具有"connect_to"和"map_to"函数以及基本连接内省功能的基本本地节点 API。

| System Name                                                    | Remote Node API | Node Configuration API | Data Layer Events | Dynamic Discovery | Requires Graph API |
| -------------------------------------------------------------- | --------------- | ---------------------- | ----------------- | ----------------- | ------------------ |
| Statically Configured Nodes                                    | .               | .                      | .                 | .                 | .                  |
| Statically Configured Graph                                    | &#x2713;              | .                      | .                 | .                 | .                  |
| Dynamically Configured Graph with Static Discovery             | &#x2713;              | &#x2713;                     | .                 | .                 | .                  |
| Statically Configured Graph with Events                        | &#x2713;              | .                      | &#x2713;                | .                 | &#x2713;                 |
| Dynamically Configured Graph with Events and Static Discovery  | &#x2713;              | &#x2713;                     | &#x2713;                | .                 | &#x2713;                 |
| Dynamically Configured Graph with Dynamic Discovery            | &#x2713;              | &#x2713;                     | .                 | &#x2713;                | &#x2713;                 |
| Dynamically Configured Graph with Events and Dynamic Discovery | &#x2713;              | &#x2713;                     | &#x2713;                | &#x2713;                | &#x2713;                 |

## **Node Life Cycles**

Something that wasn't covered in the previous section of use cases was verifiability of the system. There are different levels of the system that may need to be verified, depending on the use case. In the most flexible and dynamic system, one could imagine that the user develops the system using all of the freedom which the dynamic system gives him/her at development time, but also wanting to ensure that his/her system is up and connected the way he expects. In a simpler system, like the "Statically Configured Graph" system, one could imagine that a centralized authority would want all nodes up and connected before trying to use the system. In both of these cases, the system needs each participant in the graph to report something about their state.

> **在先前的使用案例部分没有涵盖的是系统的可验证性。** 根据使用案例，可能需要验证不同级别的系统。在最灵活和动态的系统中，可以想象用户在开发时利用动态系统提供的所有自由权，但也希望确保他/她的系统正确连接并运行。在更简单的系统中，比如“静态配置图”(Statically Configured Graph)系统，可以想象中央权威(centralized authority)在尝试使用系统之前，希望所有节点都处于连接状态。在这两种情况下，**系统都需要图中的每个参与者报告其状态**。

One option is to provide a Life Cycle to nodes in the graph. Each node would have a set series of states in which it could be, something like starting, establishing configuration, establishing connections, running, error, and stopping. The exact states are not as important as the ability for the system to introspect the life cycle state of all of the nodes.

> **一个选项是为图中的节点提供一个生命周期。** 每个节点都有一系列状态，如启动，建立配置，建立连接，运行，错误和停止。确切的状态不如系统可以内省所有节点的生命周期状态重要。

> [!NOTE]
> 这里就给出了 lifecycle 的出发点？！

Combining the life cycle information of the nodes with the state of graph could allow for various levels of system verifiability. It would allow something to ask the question, "Is the system up and running?". Currently in ROS, a user would run a launch file and hope that all of their nodes start without crashing and then make the correct connections, and it is difficult to verify if the thing the user designed in the launch file is what the user got at launch time.

> **将节点的生命周期信息与图的状态结合起来，可以允许各种级别的系统可验证性。** 它将允许某人问：“系统正在运行吗？” **目前在 ROS 中**，用户将运行启动文件，并希望所有节点都能在不崩溃的情况下启动，并建立正确的连接，**很难验证用户在启动文件中设计的内容是否与启动时所得到的内容相同**。

> [!NOTE]
> 这里想说的应该是：用户设计好了图，即 node 之间的连接关系，但是启动的过程中不能保证与设计的图是一样的
> 这里应用 lifecycle 的设计，可以保证节点之间建立的关系、依赖、顺序等等。

Life cycle information could be presented to the user as an API, which the user should call at various points in the code to signify transitioning to different states. If users are fine with design their code in a structured component like construct, the system could provide life cycle reporting automatically. Life cycle reporting should not be required by default, otherwise it might discourage, or make it more difficult, to debug and experiment using a scripting language.

> **生命周期信息可以作为 API 呈现给用户，用户在代码中的不同点应该调用它来表示转换到不同的状态。** 如果用户愿意以结构化组件的形式设计他们的代码，系统可以自动提供生命周期报告。**默认情况下不应该要求生命周期报告**，否则可能会挫伤人们，或者使用脚本语言调试和实验变得更加困难。

Life cycle state information could be provided by nodes in any above described use case except the "Statically Configured Nodes" system. For systems with a graph interface, the life cycle state changes can be sent as a graph message, otherwise a simple remote procedure call interface could be provided.

> **生命周期状态信息**可以在上述任何使用案例中**由节点提供**，除了“静态配置节点”系统。对于具有图形界面的系统，可以**将生命周期状态变化作为图形消息发送**，否则可以提供简单的远程过程调用接口。

## Proposed Interfaces

In this section of the paper, some more details about interfaces which were eluded to in the use cases above are provided.

> 在本节中，提供了一些关于上述用例中暗示的接口的更多细节。

### Node Interface

The most basic interface used above is the Node API. It was briefly described as having "connect_to" and "map_to" functions along with basic connection introspection.

> 最基本的接口是 Node API，它简要描述为具有“connect_to”和“map_to”功能以及基本的连接内省。

The "connect_to" function is necessary in order to execute the data layer connections. This function might cause the node to connect to a remote TCP/IP server, setup a local TCP/IP server, join a UDP Multicast Group, set a shared memory block, or something else.

> "connect_to" 函数是执行数据层连接所必需的。这个函数可能会导致节点连接到远程 TCP/IP 服务器、设置本地 TCP/IP 服务器、加入 UDP 多播组、设置共享内存块或其他操作。

In order to better support non point-to-point communication transport types like UDP Multicast, the system needs to be able to differentiate between transport and topic, which is why the "map_to" function is described as a separate function. The topic information could be included in the "connect_to" call, but that would make some use cases more difficult. For example, if a system wanted to send multiple types of topics over a single connection, possibly a single TCP or UDP connection, then it would not really make sense to call the "connect_to" function for a topic whose connection is already established. Instead the proposed set of functions would prescribe that the "connect_to" function would be called once, returning a handle to the connection, and that handle would be reused in multiple calls to "map_to".

> 为了更好地支持非点对点通信传输类型，如 UDP 多播，系统需要能够区分传输和主题，这就是为什么“map_to”函数被描述为一个单独的函数。主题信息可以包含在“connect_to”调用中，但这会使某些用例更加困难。例如，如果一个系统想要在单个连接(可能是单个 TCP 或 UDP 连接)上发送多种类型的主题，那么就不太合理调用“connect_to”函数来连接已经建立的主题。相反，拟议的函数集合提出，只调用一次“connect_to”函数，返回一个连接句柄，并在多次调用“map_to”时重用该句柄。

In addition to the "connect_to" and "map_to" functions, the basic node interface should provide introspection of the created connections. This allows the entity which created the connections to keep tabs on the state of those connections. This could be implemented using a handle for each connection, returned by "connect_to", allowing the user to poll the state of the connection, but an asynchronous interface could also be implemented, allowing for local events.

> 除了"connect_to"和"map_to"功能之外，基本的节点界面应该**提供对创建的连接的内省。这允许创建连接的实体保持对这些连接状态的跟踪**。这可以通过由"connect_to"返回的每个连接的句柄来实现，**允许用户轮询连接的状态，但也可以实现异步接口，允许本地事件**。

This API should always be provided locally, but in most systems described above it should also be exposed over RPC to allow for remote control and management of the node.

> 这个 **API 应该总是在本地提供**，但是在上面描述的大多数系统中，它**也应该通过 RPC 公开，以允许远程控制和管理节点**。

### Node Configuration Interface

The node configuration interface allows a user of the interface to get the configuration of the node. This paper will not try to exhaustively list the contents of the node configuration, but it should at least contain some way to uniquely identify the node, probably a tuple of machine id, process id, and node id. The configuration should probably also include publications, subscriptions, and services consumed or provided by the node. In order for a negotiation algorithm to come up with "connect_to" and "map_to" instructions for each node, the configuration for each node will probably need to contain the supported communication paradigms, serialization wire formats, and transports. The negotiator can then know what connections need to be made and what connections are possible, and then it can use some algorithm to decide how to wire nodes together.

> **节点配置界面允许界面的用户获取节点的配置。** 本文不会尝试详尽地列出节点配置的内容，但至少应包含一些唯一标识节点的方式，可能是机器 ID、进程 ID 和节点 ID 的元组。配置也可能包括节点发布的、订阅的和消费或提供的服务。为了让协商算法为每个节点生成“connect_to”和“map_to”指令，每个节点的配置可能需要包含支持的通信范式、序列化线路格式和传输协议。协商者可以知道需要建立哪些连接以及哪些连接是可能的，然后可以使用一些算法来决定如何将节点连接在一起。

> [!NOTE]
> 利用 beatles 中现有的 manifest 文件实现配置

The format for the configurations should probably be some extensible format rather than a statically defined data structure. Though this might not scale well to small computers or embedded devices. Obvious candidates would be something like JSON or XML, though a non-nestable solution like INI file might be sufficient for the use case while simultaneously being much easier to parse on embedded computers.

> 格式应该是一种可扩展的格式，而不是一个静态定义的数据结构。虽然这可能不适用于小型计算机或嵌入式设备。明显的候选者可能是类似 JSON 或 XML 的东西，尽管一个不可嵌套的解决方案，比如 INI 文件可能足够满足使用情况，同时也更容易在嵌入式计算机上解析。

### Graph Interface

Several of the above systems described the need for a graph interface which would allow the graph participants to send and receive messages to the graph. This graph transport abstraction is the most basic interface required for the graph. On top of these send and receive graph message functions other functionality can be built. This design allows the graph interface to be layered.

> 几个上述系统描述了需要一个图形界面，允许图形参与者发送和接收消息到图形。这个图形传输抽象是图形所需最基本的接口。在这些发送和接收图形消息功能之上可以构建其他功能。这种设计允许图形界面被分层。

The first layer of the graph interface is the send and receive layer, which basically allows users to pass messages to and receive messages from the graph in an abstract sense. The method by which these messages are delivered is not something that the users of the interface should be concerned with. Systems can provide implementations of the graph transport to match their needs. This layer of the interface enables nodes to dynamically join the graph, dynamically provide configuration data, send data layer events, send heartbeat messages, send life cycle state changes, and potentially provide other information. In the case of a broker system each message a node sends to the graph will go directly to the master and the master will redistribute the messages to the appropriate nodes as it sees fit. In the case of a completely distributed, master-less system each message sent by a node could be broadcast to every other node and each node could filter the incoming messages as it sees fit.

> 第一层图形界面是发送和接收层，基本上允许用户以抽象意义上的方式将消息发送到图形并接收来自图形的消息。用户不必关心这些消息是如何传递的。系统可以提供图形传输的实现以满足其需求。该界面层使节点能够动态加入图形、动态提供配置数据、发送数据层事件、发送心跳消息、发送生命周期状态变化，并可能提供其他信息。对于经纪人系统，节点发送到图形的每条消息都将直接发送到主节点，主节点将根据自己的需要将消息重新分发到适当的节点。在完全分布式、无主的系统中，节点发送的每条消息都可以广播到每个其他节点，每个节点都可以根据自己的需要过滤传入的消息。

The nodes which will be coordinating with each other will have to agree on a graph transport implementation a priori. Because there is no opportunity to negotiate the graph transport implementation, there must exist a simple "lingua franca" and the system wide graph transport implementation serves as that unifying language. It would still be possible to write programs which could serve to transparently bridge networks which used different graph transports.

> **两个协调一起工作的节点必须提前就图形传输实现达成一致。由于没有机会来协商图形传输实现，必须存在一种简单的“共同语言”，而系统范围内的图形传输实现可以作为这种统一的语言。仍然可以编写程序，用于透明地桥接使用不同图形传输的网络。**

> [!NOTE]
> 不太理解？！

On top of the first layer an asynchronous graph event system could be created. Users of this API would instantiate a system which monitors the graph using the receive message function described above and would notify the user when events which the user is interested in occurs. For instance, the user could ask the system to notify them anytime a new publisher for the topic 'foo' is created. This could be used by the implementation of a negotiation system, master or master-less, or it could be used by a diagnostics system or even user land code to build more robust systems.

> 在第一层之上，可以创建一个异步图事件系统。此 API 的用户将实例化一个系统，该系统使用上面描述的接收消息功能监视图形，并在用户感兴趣的事件发生时通知用户。例如，用户可以要求系统在创建主题“foo”的新发布者时通知他们。这可以用于实现协商系统，主或无主，或者可以用于诊断系统或用户端代码来构建更强大的系统。

With the graph event interface another interface which maintains the state of the graph and allows for queries on the graph can be built. A user would use this interface by instantiating an object which would maintain the graph state by monitoring the graph using the graph event system and the node configuration interface. This interface might be used by a master system or by a visualization tool. A master system would use this interface in the master process to maintain the graph state, so that questions like "what nodes are publishing to topic 'foo'" can be answered efficiently. A master-less system would have each node in the graph use this interface to maintain a graph state so that all nodes have a notion of the graph topology and could make distributed decisions about how to connect to their peers. A graph visualization tool would also likely use this interface in order to visualize the current state of the graph.

> 通过使用图形事件接口，可以**构建另一个接口来维护图形的状态，并允许对图形进行查询**。用户可以通过实例化一个对象来使用这个接口，该对象通过使用图形事件系统和节点配置接口来监视图形状态。这个接口可能会被主系统或可视化工具使用。主系统会在主进程中使用这个接口来维护图形状态，以便**可以快速回答像“哪些节点正在发布到主题“foo”的问题”**。**无主系统会让图形中的每个节点都使用这个接口来维护图形状态，以便所有节点都有图形拓扑的概念，并可以做出分布式决策，关于如何连接到他们的同行**。图形可视化工具也可能会使用这个接口来可视化图形的当前状态。

## Communication Negotiation

The previous paragraphs have not discussed the negotiation of the communications at all. It has been described how nodes may provide their configurations dynamically, or the configurations might be captured statically and it has been described how nodes can be instructed to establish connections on the data layer either internally or externally, but not much has been said about determining the appropriate **data layer graph** layout based on the node's configurations and the machine/network topology. It is left to the implementor of the the negotiation system to use the configurations of the nodes and potentially other information to design and execute a **data layer graph**. This method of negotiation implies that when determining the **data layer graph**'s layout, all of the required information can be retrieved from the node, i.e. a node should be able to answer "what transports do you support?" through the node configuration API.

> 上一段**没有讨论通信协商的问题**。它描述了**节点如何动态提供配置，或者如何静态捕获配置**，以及如何指示节点在数据层上建立内部或外部连接，但是没有太多关于基于节点的配置和机器/网络拓扑确定合适的**数据层图**布局的内容。这就留给协商系统的实现者使用节点的配置以及其他可能的信息来设计和执行**数据层图**。这种协商方法意味着，在确定**数据层图**的布局时，所有所需的信息都可以从节点中检索，即节点应该能够通过节点配置 API 回答“你支持什么传输？”

What the above set of use cases does do is try to ensure that most conceivable negotiation systems could be implemented on top of these interfaces. To illustrate, this paper will describe some theoretical systems.

> 以上一组用例的目的是确保大多数可以想象的谈判系统可以在这些接口之上实现。为了说明，本文将描述一些理论系统。

### Client-Server Master System with Point to Point TCP Data Only

This system is very similar to the existing ROS system in that each node on startup contacts a centralized master. The node reports its existence to the master and notifies the master any time a new publisher or subscriber is created in the node. The master notifies nodes when a publisher exists for their subscribers and the nodes initiate a TCP connection to the publishing node. There is no point at which some more sophisticated **data layer graph** layout is chosen.

> 这个系统与现有的 ROS 系统非常相似，因为**每个启动的节点都会联系一个中央主机。节点向主机报告它的存在**，并在节点中创建新的发布者或订阅者时通知主机。当发布者存在于他们的订阅者时，主机会通知节点，节点会启动一个到发布节点的 TCP 连接。没有某种更复杂的**数据层图**布局被选择的时刻。

In this system the graph transport implementation is a connection to the master for each node. The state of the graph is maintained in the master process only, and each node calls its own interface in order to execute the data layer connections between nodes. All events and configurations for each node are sent to the master and relayed to the correct nodes by the master.

> 在这个系统中，图形传输实现是每个节点与主节点的连接。**图的状态仅在主进程中维护，每个节点调用自己的接口来执行节点间的数据层连接。所有事件和配置都发送到主节点，然后由主节点转发到正确的节点。**

> [!NOTE]
> 这里主进程对于维护图来说就是 master 了，不仅仅是 manager?!

### Distributed System with Intelligent Multicast

This theoretical system has each node maintain the graph's state and make connections to different UDP multicast groups based on the topics on which it is publishing or subscribing. But the TTL for the multicast is set to 0, so multicast datagrams do not leave the local machine. Each node, however, is intelligent enough to know that it needs to start a TCP server when a node on another machine is subscribing to a topic on which it is publishing. And the node on the other machine is intelligent enough to open a TCP connection to the publishing node rather than joining the multicast group for that topic on its local machine.

> 这个理论系统让每个节点维护图形的状态，并根据其发布或订阅的主题连接到不同的 UDP 多播组。但是 TTL 设置为 0，所以多播数据报不会离开本地机器。但是，每个节点都足够聪明，知道当另一台机器上的节点订阅其发布的主题时，它需要启动一个 TCP 服务器。而另一台机器上的节点也足够聪明，而不是在本地机器上加入该主题的多播组，而是打开一个 TCP 连接到发布节点。

In this system the graph transport system might be a udp multicast group, with a higher TTL level or which bridges machines with the help of udp bridges. The state of the graph is maintained in each node, and each node uses the configuration checksum in the heartbeat message from other nodes to know when it needs to get updated configurations from a node using its node configuration interface. The nodes each have negotiation logic which allows them to make cooperative networking decisions and use their own node interface to initiate connections and topic-connection mappings.

> 在这个系统中，图形传输系统可能是一个 UDP 多播组，具有更高的 TTL 级别或者使用 UDP 桥接机器。图形的状态在每个节点中保持，每个节点使用其他节点发送的心跳消息中的配置校验和来知道何时需要从使用其节点配置接口的节点获取更新的配置。节点上每个都有协商逻辑，允许它们做出协作网络决策，并使用自己的节点接口来启动连接和主题连接映射。

## Open Questions

There exist still some open questions surrounding Discovery and Negotiation.

> 仍然有一些关于发现和谈判的问题悬而未决。

### User Hints to Data Layer Implementation

One use case not addressed above is how to allow user code to hint or constrain the generation of the **data layer graph** layout. Ideally, a user could indicate that data on a certain topic is or is not suitable for unreliable transportation, but as it stands there is no direct way for a node to effect change on the data layer.

> 一个未在上面提及的用例是如何允许用户代码提示或约束数据层图形布局。理想情况下，**用户可以指出某个主题的数据是否适合不可靠的运输**，但是现在没有直接的方式让节点对数据层产生影响。

One option might be to allow certain hints or constraints to be added to the node configuration, which the negotiation system could use to make more ideal decisions when generating the **data layer graph** layout. However, this implies that the **data layer graph** is determined at runtime and not static.

> 一个选项可能是允许在节点配置中添加某些提示或约束，这样谈判系统在生成**数据层图**布局时可以做出更理想的决策。但是，这意味着**数据层图**是在运行时确定的，而不是静态的。

Another option is not allow constraints at all, because there will always be the scenario where an end-users wants to reuse a node in a manner for which it was not originally designed and constraints which are not overridable would make the node less reusable.

> 另一个选择是完全不允许约束，因为总会有一种情况，即最终用户希望以与原始设计不同的方式重用节点，而不可覆盖的约束会使节点的可重用性降低。

### Graph Communication

What RPC system should nodes use to expose their API's?

> **节点使用什么 RPC 系统来公开它们的 API？**

What system should be used for serialization or graph messages?

> 何种系统应该用于序列化或图表消息？

What transport should be used for graph messages?

> 何种转换工具应该用于图形消息？

One option is just pick one of the available systems that are used by the data layer. Another option is pick a simple set which must always be available.

> 一种选择是选择数据层所使用的可用系统之一。另一种选择是选择一个必须始终可用的简单集合。

### Topic Tools

How could runtime topic remapping and/or aliasing work?

> 怎样才能实现运行时主题重映射和/或别名？

Currently ROS can remap any "ROS Name" at runtime, would it be possible to do this during runtime as well?

> 目前 ROS 可以在运行时重新映射任何"ROS Name"，是否也可以在运行时进行？

One option might be to allow some level of setting of configurations through the Node Configuration API described above.

> 一个选项可能是允许通过上面描述的节点配置 API 设置一些配置。
