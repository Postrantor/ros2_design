---
tip: translate by openai@2023-05-28 11:08:20
...
---
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


The article enumerates a few stories which sketch what will be possible with ROS in the future.

> 这篇文章列举了一些故事，描绘了ROS在未来可能实现的功能。

The list is by no means exhaustive.

> 这份列表绝不是详尽的。

## Push ROS into the hardware nodes


A couple of design decisions in ROS 1 make it difficult to integrate hardware devices (e.g. a sensor or actuator) natively into the ROS graph.

> 在ROS 1中的一些设计决策使得很难将硬件设备（例如传感器或执行器）本地集成到ROS图中。

The master is a central entity in the ROS graph and needs to be started before any of the nodes.

> 主节点是ROS图中的中心实体，在启动任何节点之前都需要先启动它。

Also the communication between the nodes and the master is done using XML-RPC which poses a significant dependency when being implemented on small resource constraint systems / micro controllers due to its recursive unbounded nature.

> 也使用XML-RPC在节点和主节点之间进行通信，当在资源受限的系统/微控制器上实施时，由于其递归无界的性质，会带来重大的依赖性。

Instead commonly a driver is being used which uses a custom protocol to communicate between the device and the computer and exposes a ROS interface on the computer.

> 通常使用一个驱动程序来实现，它使用自定义协议在设备和计算机之间进行通信，并在计算机上暴露ROS接口。


In the future it should be possible to implement the ROS protocol directly on the devices embedded system.

> 在未来，可以直接在嵌入式系统设备上实现ROS协议。

A ROS-enabled device would then be able to discover other nodes automatically and expose a ROS interface (composed of publisher, services and parameters).

> 一个支持ROS的设备可以自动发现其他节点，并暴露一个ROS接口（由发布者、服务和参数组成）。

The adoption of an industry standard like DDS as well as the decentralized nature of the middleware are important pieces to enable this.

> 采用像DDS这样的行业标准以及中间件的去中心化特性是实现这一目标的重要要素。


The advantage of this approach is twofold:

> 这种方法的优势有两个：


- The integration effort to build a new system is being reduced since no custom drivers are required anymore.

> 集成建立新系统的努力正在减少，因为不再需要自定义驱动程序了。

- By implementing a specific ROS-based interface the devices of specific class can easily be substituted without the need to spend time on integrating the software / hardware from e.g. a different vendor.

> 通过实施基于ROS的特定接口，可以轻松地替换特定类别的设备，而无需花费时间将来自不同供应商的软件/硬件集成。

## Delay decision on process layout to deploy time


In ROS 1 nodes commonly use the `Node` API and implement their own `main` function.

> 在ROS 1中，节点通常使用“Node” API并实现自己的“main”函数。

Only a few packages leverage the `Nodelet` API and compile their components into shared libraries instead.

> 只有少数几个包利用`Nodelet` API，将其组件编译成共享库。

The developer has to choose between these two different APIs and converting from using one to the other requires some non trivial effort.

> 开发者必须在这两个不同的API之间做选择，从一个转换到另一个需要一些不同寻常的努力。


In ROS 2 the recommended way of writing nodes will be similar to nodelets.

> 在ROS 2中，推荐的编写节点的方式将类似于nodelets。

This will enable the user to decide at deploy time how a set of nodes should be started.

> 这将使用户能够在部署时决定如何启动一组节点。

On the one hand each node could be started in a separate process to ease debugging of them individually.

> 一方面，每个节点都可以在单独的进程中启动，以方便对它们进行单独调试。

On the other multiple nodes can be aggregate in a single process to benefit from the performance advantages possible by in-process message passing.

> 在其他节点上，可以在单个进程中聚合多个节点，以获取通过进程内消息传递可能带来的性能优势。

## Deterministic launch


In ROS 1 the launch system is only able to start a set of processes.

> 在ROS 1中，发射系统只能启动一组进程。

It doesn't provide any more feedback beyond the information if a process has finished.

> 它不会提供任何除了进程完成的信息之外的反馈。

For complex systems this is often not sufficient.

> 对于复杂的系统，这通常是不够的。

It is quite common that developers write their processes in a way which either waits a fixed amount of time or waits for a custom flag which signals that "everything" is ready before starting to process data.

> 开发人员通常会以一种方式编写他们的过程，要么等待一段固定的时间，要么等待一个自定义标志，以表示“一切”就绪后才开始处理数据。

Also when "something" goes wrong during startup the attentive developer will notice and manually restart the launch process.

> 如果启动过程中出现"某些"问题，细心的开发人员会注意到并手动重新启动启动过程。

Obviously in use cases where the software is being used on a product this is not feasible either.

> 显然，在使用软件在产品上的用例中，这也是不可行的。


The goal is to enable the launch system to ensure a deterministic result.

> 目标是使发射系统能够确保一个确定的结果。

In order to achieve this the launch system needs to be able to introspect the started processes (which are usually ROS nodes) and ensure that they have been started successfully, finished initializing, and have established all require communication channels with other entities.

> 为了实现这一目标，发射系统需要能够内省已启动的进程（通常是ROS节点），并确保它们已成功启动，完成初始化，并与其他实体建立所有必要的通信通道。

This will require a minimalistic life cycle as well as the ability to introspect the state from the launch system.

> 这需要一个极简的生命周期，以及从启动系统中自省状态的能力。

It would even be possible to compare the started ROS graph with a "known good" state to ensure the system has been started according to the expectations.

> 可以甚至把启动的ROS图与“已知良好”状态进行比较，以确保系统按预期启动。

## Introspection, Orchestration, and beyond


In complex systems the observation of the system and its dynamic configuration becomes more important.

> 在复杂系统中，观察系统及其动态配置变得更加重要。

In ROS 1 nodes do not have any specific state and only a few components (like the nodelet manager) provide an interface to get information or even manipulate the running system.

> 在ROS 1中，节点没有任何特定的状态，只有少数组件（如节点管理器）提供了获取信息或甚至操作运行系统的接口。


Once a ROS system is using the above features ([Nodelet-style nodes](#delay-decision-on-process-layout-to-deploy-time) and an [accessible life cycle](#delay-decision-on-process-layout-to-deploy-time)) the abilities for introspection as well as orchestration can be leverages to build more complicated systems.

> 一旦ROS系统使用了上述功能（Nodelet样式的节点和可访问的生命周期），就可以利用内省和编排的能力来构建更复杂的系统。

The following a only a few example scenario enabled by the comprehensive introspection and debugging capabilities:

> 以下只是全面内省和调试能力所能启用的一些示例场景：


- The state of each node can be monitored and based on the information specific actions can be triggered.

> 每个节点的状态都可以监控，并根据信息触发特定的操作。

  E.g. certain error conditions can be signaled to the user, or in case fall back behaviors are available they can be selected to provide a degraded continuation of the system.

> 例如，可以向用户发出某些错误条件的信号，或者如果可用的回退行为，可以选择它们以提供系统的降级继续。


- The resource usage of the system could be monitored at runtime.

> 系统的资源使用情况可以在运行时进行监控。

  Based on the available information the system can be dynamically reconfigured e.g. by enabling / disabling specific nodes, altering any kind of parameter (e.g. frame rate, or any other threshold).

> 根据可用信息，系统可以通过启用/禁用特定节点，改变任何参数（例如帧率或其他任何阈值）来动态重新配置。


- In case of a single process containing multiple nodes crashing the system should decide to not only restart these nodes but also separate them into individual processes to isolate the problem in future cases.

> 如果一个包含多个节点的单个进程崩溃，系统应该不仅重启这些节点，而且将它们分离成单独的进程，以便在今后的情况下隔离问题。


- If the system load on a single computer exceeds a certain threshold an orchestration entity can trigger the following steps: pause all nodes through the life cycle interface, shutdown a specific node, spawning the node on a separate machine and passing the same configuration / parameters, and once all communication channels have been established resume all nodes.

> 如果单台计算机的系统负载超过一定阈值，编排实体可以触发以下步骤：通过生命周期接口暂停所有节点，关闭特定节点，在另一台机器上启动节点并传递相同的配置/参数，一旦所有通信通道建立完毕，恢复所有节点。
