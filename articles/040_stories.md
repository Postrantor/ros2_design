---
tip: translate by openai@2023-05-30 22:36:19
layout: default
title: Stories driving the development
permalink: articles/stories.html
abstract:
  This article captures some stories which drive the direction of features in ROS.
published: true
author: '[Dirk Thomas](https://github.com/dirk-thomas)'
date_written: 2016-12
last_modified: 2016-12
categories: Overview
Authors: {{ page.author }}
Date Written: {{ page.date_written }}
Last Modified: {% if page.last_modified %}{{ page.last_modified }}{% else %}{{ page.date_written }}{% endif %}
---
The article enumerates a few stories which sketch what will be possible with ROS in the future. The list is by no means exhaustive.

> 文章列举了一些故事，勾勒出 ROS 在未来将有可能实现的功能。这份清单并不是全面的。

## Push ROS into the hardware nodes

A couple of design decisions in ROS 1 make it difficult to integrate hardware devices (e.g. a sensor or actuator) natively into the ROS graph. The master is a central entity in the ROS graph and needs to be started before any of the nodes. Also the communication between the nodes and the master is done using XML-RPC which poses a significant dependency when being implemented on small resource constraint systems / micro controllers due to its recursive unbounded nature. Instead commonly a driver is being used which uses a custom protocol to communicate between the device and the computer and exposes a ROS interface on the computer.

> 一些 ROS 1 的设计决策使得很难将硬件设备（例如传感器或执行器）本机集成到 ROS 图中。主节点是 ROS 图中的中央实体，在任何节点启动之前都需要先启动。另外，节点和主节点之间的通信是使用 XML-RPC 完成的，由于其递归无界的性质，当在资源受限的系统/微控制器上实施时会带来重大的依赖性。相反，通常使用驱动程序，该驱动程序使用自定义协议在设备和计算机之间进行通信，并在计算机上公开 ROS 接口。

In the future it should be possible to implement the ROS protocol directly on the devices embedded system. A ROS-enabled device would then be able to discover other nodes automatically and expose a ROS interface (composed of publisher, services and parameters). The adoption of an industry standard like DDS as well as the decentralized nature of the middleware are important pieces to enable this.

> 在未来，可以直接在嵌入式系统上实施 ROS 协议。然后，ROS 启用的设备就可以自动发现其他节点，并提供 ROS 接口（由发布者、服务和参数组成）。采用像 DDS 这样的行业标准以及中间件的去中心化特性是实现这一目标的重要组成部分。

The advantage of this approach is twofold:

> 这种方法的优势有两方面：

- The integration effort to build a new system is being reduced since no custom drivers are required anymore.
- By implementing a specific ROS-based interface the devices of specific class can easily be substituted without the need to spend time on integrating the software / hardware from e.g. a different vendor.

## Delay decision on process layout to deploy time

In ROS 1 nodes commonly use the `Node` API and implement their own `main` function. Only a few packages leverage the `Nodelet` API and compile their components into shared libraries instead. The developer has to choose between these two different APIs and converting from using one to the other requires some non trivial effort.

> 在 ROS 1 中，节点通常使用“Node”API 并实现自己的“main”函数。只有少数几个包利用“Nodelet”API，将它们的组件编译成共享库。开发人员必须在这两种不同的 API 之间进行选择，并且从使用一种转换到另一种需要一些非常规努力。

In ROS 2 the recommended way of writing nodes will be similar to nodelets. This will enable the user to decide at deploy time how a set of nodes should be started. On the one hand each node could be started in a separate process to ease debugging of them individually. On the other multiple nodes can be aggregate in a single process to benefit from the performance advantages possible by in-process message passing.

> 在 ROS 2 中，推荐的编写节点的方式将类似于 nodelet。这将使用户能够在部署时决定如何启动一组节点。一方面，每个节点可以在单独的进程中启动，以便单独调试它们。另一方面，多个节点可以聚合在一个进程中，以获得通过进程内消息传递可能的性能优势。

## Deterministic launch

In ROS 1 the launch system is only able to start a set of processes. It doesn't provide any more feedback beyond the information if a process has finished. For complex systems this is often not sufficient. It is quite common that developers write their processes in a way which either waits a fixed amount of time or waits for a custom flag which signals that "everything" is ready before starting to process data. Also when "something" goes wrong during startup the attentive developer will notice and manually restart the launch process. Obviously in use cases where the software is being used on a product this is not feasible either.

> 在 ROS 1 中，发射系统仅能启动一组进程。除了进程完成的信息之外，它不提供任何其他反馈。对于复杂的系统，这通常是不够的。开发人员经常以一种方式编写他们的进程，要么等待一定的时间，要么等待一个自定义标志，表示“一切”准备就绪，然后才开始处理数据。此外，当启动过程中“出现问题”时，细心的开发人员会注意到并手动重新启动发射过程。显然，在软件用于产品的场景中，这也是不可行的。

The goal is to enable the launch system to ensure a deterministic result. In order to achieve this the launch system needs to be able to introspect the started processes (which are usually ROS nodes) and ensure that they have been started successfully, finished initializing, and have established all require communication channels with other entities. This will require a minimalistic life cycle as well as the ability to introspect the state from the launch system. It would even be possible to compare the started ROS graph with a "known good" state to ensure the system has been started according to the expectations.

> 目标是使发射系统能够确保确定性的结果。为了实现这一点，发射系统需要能够内省已经启动的进程（通常是 ROS 节点），并确保它们已经成功启动，完成初始化，并与其他实体建立所有必要的通信通道。这将需要极简的生命周期以及从发射系统中内省状态的能力。甚至可以将启动的 ROS 图与“已知的良好”状态进行比较，以确保系统按照预期启动。

## Introspection, Orchestration, and beyond

In complex systems the observation of the system and its dynamic configuration becomes more important. In ROS 1 nodes do not have any specific state and only a few components (like the nodelet manager) provide an interface to get information or even manipulate the running system.

> 在复杂的系统中，观察系统及其动态配置变得更加重要。在 ROS 1 中，节点没有任何特定的状态，只有少数组件（如节点管理器）提供了获取信息甚至操纵正在运行的系统的接口。

Once a ROS system is using the above features ([Nodelet-style nodes](#delay-decision-on-process-layout-to-deploy-time) and an [accessible life cycle](#delay-decision-on-process-layout-to-deploy-time)) the abilities for introspection as well as orchestration can be leverages to build more complicated systems. The following a only a few example scenario enabled by the comprehensive introspection and debugging capabilities:

> 一旦 ROS 系统使用上述功能（[Nodelet 样式节点]（#delay-decision-on-process-layout-to-deploy-time）和[可访问的生命周期]（#delay-decision-on-process-layout-to-deploy-time）），就可以利用内省和编排的能力构建更复杂的系统。 以下仅是一些受全面内省和调试能力支持的示例场景：

- The state of each node can be monitored and based on the information specific actions can be triggered. E.g. certain error conditions can be signaled to the user, or in case fall back behaviors are available they can be selected to provide a degraded continuation of the system.
- The resource usage of the system could be monitored at runtime. Based on the available information the system can be dynamically reconfigured e.g. by enabling / disabling specific nodes, altering any kind of parameter (e.g. frame rate, or any other threshold).
- In case of a single process containing multiple nodes crashing the system should decide to not only restart these nodes but also separate them into individual processes to isolate the problem in future cases.
- If the system load on a single computer exceeds a certain threshold an orchestration entity can trigger the following steps: pause all nodes through the life cycle interface, shutdown a specific node, spawning the node on a separate machine and passing the same configuration / parameters, and once all communication channels have been established resume all nodes.
