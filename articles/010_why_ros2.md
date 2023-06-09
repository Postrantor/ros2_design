---
tip: translate by openai@2023-05-30 09:15:15
layout: default
title: Why ROS 2?
permalink: articles/why_ros2.html
abstract:
  This article captures the reasons for making breaking changes to the ROS API, hence the 2.0.
published: true
author: Brian Gerkey
date_written: 2014-06
last_modified: 2022-05
categories: Overview
Authors: {{ page.author }}
Date Written: {{ page.date_written }}
Last Modified: {% if page.last_modified %}{{ page.last_modified }}{% else %}{{ page.date_written }}{% endif %}
---
We started work on ROS in November 2007. A lot has happened since then and we believe that it is now time to build the next generation ROS platform. In this article we will explain why.

> 我们于 2007 年 11 月开始使用 ROS。从那以后发生了很多事情，我们相信现在是时候构建下一代 ROS 平台了。在本文中，我们将解释原因。

In May 2022, [Robot Operating System 2: Design, architecture, and uses in the wild](https://www.science.org/doi/10.1126/scirobotics.abm6074) was published in Science Robotics describing ROS 2's motivations and design (among other things). This is a good reference as well. If you use ROS 2 in your work please cite:

> 在 2022 年 5 月，《机器人操作系统 2：设计、架构及野外应用》发表在《科学机器人学》上，描述了 ROS 2 的动机和设计（以及其他内容）。这也是一个很好的参考资料。如果您在工作中使用 ROS 2，请引用：

```
S. Macenski, T. Foote, B. Gerkey, C. Lalancette, W. Woodall, “Robot Operating System 2: Design, architecture, and uses in the wild,” Science Robotics vol. 7, May 2022.

@article{
    doi:10.1126/scirobotics.abm6074,
    author = {Steven Macenski  and Tully Foote  and Brian Gerkey  and Chris Lalancette  and William Woodall },
    title = {Robot Operating System 2: Design, architecture, and uses in the wild},
    journal = {Science Robotics},
    volume = {7},
    number = {66},
    pages = {eabm6074},
    year = {2022},
    doi = {10.1126/scirobotics.abm6074},
    URL = {https://www.science.org/doi/abs/10.1126/scirobotics.abm6074}
}
```

## How we got here

ROS began life as the development environment for the Willow Garage PR2 robot. Our primary goal was to provide the software tools that users would need to undertake novel research and development projects with the PR2. At the same time, we knew that the PR2 would not be the only, or even the most important, robot in the world, and we wanted ROS to be useful on other robots. So we put a lot of effort into defining levels of abstraction (usually through message interfaces) that would allow much of the software to be reused elsewhere.

> ROS 最初是为 Willow Garage PR2 机器人开发的环境。我们的主要目标是为 PR2 用户提供必要的软件工具，以进行新的研究和开发项目。同时，我们知道 PR2 不是世界上唯一的也不是最重要的机器人，因此我们希望 ROS 在其他机器人上也能有用。因此，我们花了很多精力来定义抽象层次（通常通过消息接口），以便在其他地方重复使用大部分软件。

Still, we were guided by the PR2 use case, the salient characteristics of which included:

> 然而，我们仍然受到 PR2 用例的指导，其中的显著特征包括：

- a single robot;
- workstation-class computational resources on board;
- no real-time requirements (or, any real-time requirements would be met in a special-purpose manner);
- excellent network connectivity (either wired or close-proximity high-bandwidth wireless);
- applications in research, mostly academia; and
- maximum flexibility, with nothing prescribed or proscribed (e.g., "we don't wrap your main()").

It is fair to say that ROS satisfied the PR2 use case, but also overshot by becoming useful on a surprisingly wide [variety of robots](http://wiki.ros.org/Robots). Today we see ROS used not only on the PR2 and robots that are similar to the PR2, but also on wheeled robots of all sizes, legged humanoids, industrial arms, outdoor ground vehicles (including self-driving cars), aerial vehicles, surface vehicles, and more.

> 可以说 ROS 满足了 PR2 的使用案例，但也超出了预期，在出人意料的各种机器人上也很有用（参见 [http://wiki.ros.org/Robots](http://wiki.ros.org/Robots)）。今天，我们不仅看到 ROS 用于 PR2 和类似 PR2 的机器人，还用于各种大小的轮式机器人、腿式人形机器人、工业机械臂、户外地面车辆（包括自动驾驶汽车）、航空器、水面车辆等等。

In addition, we are seeing ROS adoption in domains beyond the mostly academic research community that was our initial focus. ROS-based products are coming to market, including manufacturing robots, agricultural robots, commercial cleaning robots, and others. Government agencies are also looking more closely at ROS for use in their fielded systems; e.g., NASA is expected to be running ROS on the Robonaut 2 that is deployed to the International Space Station.

> 此外，我们看到 ROS 在我们最初关注的主要学术研究社区之外的领域得到应用。基于 ROS 的产品正在上市，包括制造机器人、农业机器人、商业清洁机器人等。政府机构也正在更加密切地关注 ROS，用于他们部署的系统；例如，预计美国宇航局将在部署到国际空间站的 Robonaut 2 上运行 ROS。

With all these new uses of ROS, the platform is being stretched in unexpected ways. While it is holding up well, we believe that we can better meet the needs of a now-broader ROS community by tackling their new use cases head-on.

> 随着 ROS 的新用途的出现，这个平台正在以意想不到的方式被拉伸。虽然它表现良好，但我们相信，通过直面新的用例，我们可以更好地满足现在更广泛的 ROS 社区的需求。

## New use cases

Of specific interest to us for the ongoing and future growth of the ROS community are the following use cases, which we did not have in mind at the beginning of the project:

> 对我们来说，特别有趣的是，为了 ROS 社区的持续和未来的发展，我们在项目开始时没有考虑到的以下用例：

- Teams of multiple robots: while it is possible to build multi-robot systems using ROS today, there is no standard approach, and they are all somewhat of a hack on top of the single-master structure of ROS.
- Small embedded platforms: we want small computers, including "bare-metal" micro controllers, to be first-class participants in the ROS environment, instead of being segregated from ROS by a device driver.
- Real-time systems: we want to support real-time control directly in ROS, including inter-process and inter-machine communication (assuming appropriate operating system and/or hardware support).
- Non-ideal networks: we want ROS to behave as well as is possible when network connectivity degrades due to loss and/or delay, from poor-quality WiFi to ground-to-space communication links.
- Production environments: while it is vital that ROS continue to be the platform of choice in the research lab, we want to ensure that ROS-based lab prototypes can evolve into ROS-based products suitable for use in real-world applications.
- Prescribed patterns for building and structuring systems: while we will maintain the underlying flexibility that is the hallmark of ROS, we want to provide clear patterns and supporting tools for features such as life cycle management and static configurations for deployment.
- 多个机器人的团队：虽然今天可以使用 ROS 构建多机器人系统，但没有标准的方法，并且它们在 ROS 的单主结构之上都有些骇客。
- 小型嵌入式平台：我们希望小型计算机（包括“裸机”微控制器）成为 ROS 环境中的一流参与者，而不是由设备驱动程序隔离 ROS。
- 实时系统：我们希望直接支持 ROS 中的实时控制，包括过程间和机间通信（假设适当的操作系统和/或硬件支持）。
- 非理想网络：当网络连通性因损失和/或延迟而导致的网络连通性降低时，我们希望 ROS 的行为和可能，从质量不高的 wifi 到地面通信链接。
- 生产环境：虽然 ROS 继续成为研究实验室的首选平台至关重要，但我们希望确保基于 ROS 的实验室原型可以演变为基于 ROS 的产品，适用于用于现实世界中应用程序。
- 建筑和结构系统的规定模式：虽然我们将维护 ROS 的标志的基本灵活性，但我们希望为诸如生命周期管理和部署静态配置等功能提供清晰的模式和支持工具。

## New technologies

At the core of ROS is an anonymous publish-subscribe middleware system that is built almost entirely from scratch. Starting in 2007, we built our own systems for discovery, message definition, serialization, and transport. The intervening seven years have seen the development, improvement, and/or widespread adoption of several new technologies that are relevant to ROS in all of those areas, such as:

> 在 ROS 的核心是一个匿名的发布-订阅中间件系统，几乎完全是从零开始构建的。从 2007 年开始，我们为发现、消息定义、序列化和传输建立了自己的系统。在此期间的七年中，开发、改进和/或广泛采用了几种与 ROS 在所有这些领域相关的新技术，例如：

- Zeroconf;
- Protocol Buffers;
- ZeroMQ (and the other MQs);
- Redis;
- WebSockets; and
- DDS (Data Distribution Service).

It is now possible to build a ROS-like middleware system using off-the-shelf open source libraries. We can benefit tremendously from this approach in many ways, including:

> 现在可以使用现成的开源库来构建一个类似 ROS 的中间件系统。我们可以从这种方法中获得巨大的好处，包括：

- we maintain less code, especially non-robotics-specific code;
- we can take advantage of features in those libraries that are beyond the scope of what we would build ourselves;
- we can benefit from ongoing improvements that are made by others to those libraries; and
- we can point to existing production systems that already rely on those libraries when people ask us whether ROS is "ready for prime time".

## API changes

A further reason to build ROS 2 is to take advantage of the opportunity to improve our user-facing APIs. A great deal of the ROS code that exists today is compatible with the client libraries as far back as the 0.4 "Mango Tango" release from February 2009. That's great from the point of view of stability, but it also implies that we're still living with API decisions that were made several years ago, some of which we know now to be not the best.

> ROS 2 的另一个建设原因是利用这个机会来改善我们面向用户的 API。如今现有的 ROS 代码与客户端库的兼容性可追溯到 2009 年 2 月发布的 0.4 “Mango Tango”版本。从稳定性的角度来看，这很棒，但是这也意味着我们仍然在使用几年前做出的一些 API 决策，其中有些我们现在知道不是最好的。

So, with ROS 2, we will design new APIs, incorporating to the best of our ability the collective experience of the community with the first-generation APIs. As a result, while the key concepts (distributed processing, anonymous publish/subscribe messaging, RPC with feedback (i.e., actions), language neutrality, system introspectability, etc.) will remain the same, you should not expect ROS 2 to be API-compatible with existing ROS code.

> 随着 ROS 2 的出现，我们将设计新的 API，尽可能地结合社区对第一代 API 的经验。因此，尽管关键概念（分布式处理、匿名发布/订阅消息、带反馈的 RPC（即动作）、语言中立性、系统内省性等）保持不变，但您不应期望 ROS 2 与现有 ROS 代码兼容。

But fear not: there will be mechanisms in place to allow ROS 2 code to coexist with existing ROS code. At the very least, there will be translation relays that will support run-time interactions between the two systems. And it is possible that there will be library shims that will allow existing ROS code to compile/run against ROS 2 libraries, with behavior that is qualitatively similar to what is seen today.

> 不用担心：将会有机制使 ROS 2 代码能够与现有的 ROS 代码共存。至少，将会有翻译中继，来支持两个系统之间的运行时交互。有可能会有库 shim，使现有的 ROS 代码能够编译/运行在 ROS 2 库上，行为与今天看到的相似。

## Why not just enhance ROS 1

In principle, the changes described above could be integrated into the existing core ROS code. E.g., new transport technologies could be added to `roscpp` and `rospy`. We considered this option and concluded that, given the intrusive nature of the changes that would be required to achieve the benefits that we are seeking, there is too much risk associated with changing the current ROS system that is relied upon by so many people. We want ROS 1 as it exists today to keep working and be unaffected by the development of ROS 2. So ROS 2 will be built as a parallel set of packages that can be installed alongside and interoperate with ROS 1 (e.g., through message bridges).

> 原则上，上述更改可以集成到现有的核心 ROS 代码中。例如，可以向 `roscpp` 和 `rospy` 添加新的传输技术。我们考虑了这个选择，并得出结论，考虑到为了实现我们正在寻求的好处所需的侵入性变化，与改变当前许多人依赖的 ROS 系统相关的风险太大。我们希望 ROS 1 保持原样工作，不受 ROS 2 开发的影响。因此，ROS 2 将作为一组可以沿着 ROS 1 安装并相互协作（例如，通过消息桥）的并行软件包来构建。
