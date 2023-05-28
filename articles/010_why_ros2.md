---
tip: translate by openai@2023-05-28 10:58:22
    layout: default
    title: Why ROS 2?
    permalink: articles/why_ros2.html
    abstract: This article captures the reasons for making breaking changes to the ROS API, hence the 2.0.
    published: true
    author: Brian Gerkey
    date_written: 2014-06
    last_modified: 2022-05
    categories: Overview
    Authors: {{ page.author }}
    Date Written: {{ page.date_written }}
    Last Modified: {% if page.last_modified %}{{ page.last_modified }}{% else %}{{ page.date_written }}{% endif %}
---

We started work on ROS in November 2007. A lot has happened since then and we believe that it is now time to build the next generation ROS platform. In this article we will explain why. In May 2022, [Robot Operating System 2: Design, architecture, and uses in the wild](https://www.science.org/doi/10.1126/scirobotics.abm6074) was published in Science Robotics describing ROS 2's motivations and design (among other things). This is a good reference as well. If you use ROS 2 in your work please cite:

> 我们在 2007 年 11 月开始使用 ROS。自那以后发生了很多事情，我们相信现在是时候建立下一代 ROS 平台了。在本文中，我们将解释为什么。在 2022 年 5 月，《机器人操作系统 2：设计、架构及其野外应用》（Science Robotics 上发表）描述了 ROS 2 的动机和设计（以及其他内容）。这也是一个很好的参考资料。如果您在工作中使用 ROS 2，请引用：

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

ROS began life as the development environment for the Willow Garage PR2 robot.

> ROS 最初是为 Willow Garage PR2 机器人开发环境而生。

Our primary goal was to provide the software tools that users would need to undertake novel research and development projects with the PR2.

> 我们的主要目标是为 PR2 提供用户所需的软件工具，以便进行新的研发项目。

At the same time, we knew that the PR2 would not be the only, or even the most important, robot in the world, and we wanted ROS to be useful on other robots.

> 同时，我们知道 PR2 不会是世界上唯一的，甚至不是最重要的机器人，我们希望 ROS 也能在其他机器人上有用。

So we put a lot of effort into defining levels of abstraction (usually through message interfaces) that would allow much of the software to be reused elsewhere.

> 我们花了很多努力来确定抽象层次（通常通过消息接口），以便让软件的大部分可以在其他地方重复使用。

Still, we were guided by the PR2 use case, the salient characteristics of which included:

> 仍然，我们受 PR2 的用例指导，其中最突出的特征包括：

- a single robot;
- workstation-class computational resources on board;
- no real-time requirements (or, any real-time requirements would be met in a special-purpose manner);
- excellent network connectivity (either wired or close-proximity high-bandwidth wireless);
- applications in research, mostly academia; and
- maximum flexibility, with nothing prescribed or proscribed (e.g., "we don't wrap your main()").

> 一台机器人
> 载有工作站级计算资源。
> 没有实时要求（或者，任何实时要求将以特殊方式得到满足）。
> 优质的网络连接（有线或近距离高带宽无线）。
> 研究，主要是学术领域中的应用。
> 最大的灵活性，没有任何规定或禁止（例如，“我们不会包装你的 main（）”）。

It is fair to say that ROS satisfied the PR2 use case, but also overshot by becoming useful on a surprisingly wide [variety of robots](http://wiki.ros.org/Robots).

> 可以说 ROS 满足了 PR2 的用例，而且还出乎意料地在各种各样的机器人上也有用（http://wiki.ros.org/Robots）。

Today we see ROS used not only on the PR2 and robots that are similar to the PR2, but also on wheeled robots of all sizes, legged humanoids, industrial arms, outdoor ground vehicles (including self-driving cars), aerial vehicles, surface vehicles, and more.

> 今天，我们不仅在 PR2 和类似 PR2 的机器人上使用 ROS，而且还在各种轮式机器人、双足人形机器人、工业机械臂、户外地面车辆（包括自动驾驶车）、航空器、水面车辆等上使用 ROS。

In addition, we are seeing ROS adoption in domains beyond the mostly academic research community that was our initial focus.

> 此外，我们正在看到 ROS 在我们最初关注的主要是学术研究社区以外的领域得到采用。

ROS-based products are coming to market, including manufacturing robots, agricultural robots, commercial cleaning robots, and others.

> ROS 技术的产品正在推向市场，包括制造机器人、农业机器人、商业清洁机器人等等。

Government agencies are also looking more closely at ROS for use in their fielded systems; e.g., NASA is expected to be running ROS on the Robonaut 2 that is deployed to the International Space Station.

> 政府机构也正在更加密切地关注 ROS 在其部署系统中的使用；例如，预计美国宇航局将在部署到国际空间站的 Robonaut 2 上运行 ROS。

With all these new uses of ROS, the platform is being stretched in unexpected ways.

> 随着 ROS 的所有新用途，这个平台正在以意想不到的方式被拉伸。

While it is holding up well, we believe that we can better meet the needs of a now-broader ROS community by tackling their new use cases head-on.

> 我们相信，通过直面新的使用案例，我们可以更好地满足现在更广泛的 ROS 社区的需求，尽管它现在运行良好。

## New use cases

Of specific interest to us for the ongoing and future growth of the ROS community are the following use cases, which we did not have in mind at the beginning of the project:

> 我们对 ROS 社区的持续和未来发展特别感兴趣的是以下用例，这些用例在项目开始时我们没有考虑到：

- Teams of multiple robots: while it is possible to build multi-robot systems using ROS today, there is no standard approach, and they are all somewhat of a hack on top of the single-master structure of ROS.

> 组合多个机器人：尽管今天可以使用 ROS 构建多机器人系统，但没有标准的方法，它们都是基于 ROS 单主机结构的某种技术把戏。

- Small embedded platforms: we want small computers, including "bare-metal" micro controllers, to be first-class participants in the ROS environment, instead of being segregated from ROS by a device driver.

> 我们希望小型计算机，包括“裸机”微控制器，成为 ROS 环境中的一流参与者，而不是被设备驱动程序隔离出 ROS。

- Real-time systems: we want to support real-time control directly in ROS, including inter-process and inter-machine communication (assuming appropriate operating system and/or hardware support).

> 我们**希望直接在 ROS 中支持实时控制，包括进程间和机器间通信**（假设有适当的操作系统和/或硬件支持）。

> [!NOTE]
> 这里提到的实时，考虑了进程间和机器间。

- Non-ideal networks: we want ROS to behave as well as is possible when network connectivity degrades due to loss and/or delay, from poor-quality WiFi to ground-to-space communication links.

> 我们希望 ROS 在网络连接由于损失和/或延迟而降级时，从劣质 WiFi 到地面到空间通信链路，能尽可能地表现得更好。

- Production environments: while it is vital that ROS continue to be the platform of choice in the research lab, we want to ensure that ROS-based lab prototypes can evolve into ROS-based products suitable for use in real-world applications.

> - 生产环境：虽然 ROS 继续成为研究实验室的平台选择至关重要，但我们希望确保基于 ROS 的实验室原型能够演变成适用于真实应用环境的基于 ROS 的产品。

- Prescribed patterns for building and structuring systems: while we will maintain the underlying flexibility that is the hallmark of ROS, we want to provide clear patterns and supporting tools for features such as life cycle management and static configurations for deployment.

> - 为构建和结构化系统提供规定的模式：虽然我们将保持 ROS 的标志性灵活性，但我们希望为诸如生命周期管理和静态部署配置等功能提供清晰的模式和支持工具。

## New technologies

At the core of ROS is an anonymous publish-subscribe middleware system that is built almost entirely from scratch.

> 在 ROS 的核心是一个几乎完全从头开始构建的匿名发布-订阅中间件系统。

Starting in 2007, we built our own systems for discovery, message definition, serialization, and transport.

> 2007 年开始，我们建立了自己的发现、消息定义、序列化和传输系统。

The intervening seven years have seen the development, improvement, and/or widespread adoption of several new technologies that are relevant to ROS in all of those areas, such as:

> 过去七年间，出现了许多与 ROS 相关的新技术，它们在这些领域中得到了发展、改进和/或广泛采用，比如：

- Zeroconf;
- Protocol Buffers;
- ZeroMQ (and the other MQs);
- Redis;
- WebSockets; and
- DDS (Data Distribution Service).

It is now possible to build a ROS-like middleware system using off-the-shelf open source libraries.

> 现在可以使用现成的开源库来构建类似 ROS 的中间件系统。

We can benefit tremendously from this approach in many ways, including:

> 我们可以从这种方法中获益良多，包括：

- we maintain less code, especially non-robotics-specific code;
- we can take advantage of features in those libraries that are beyond the scope of what we would build ourselves;
- we can benefit from ongoing improvements that are made by others to those libraries; and
- we can point to existing production systems that already rely on those libraries when people ask us whether ROS is "ready for prime time".

> 我们维护的代码更少，特别是非机器人特定代码。
> 我们可以利用这些库中超出我们自己可以构建的范围的功能。
> 我们可以从别人对这些库的不断改进中受益；
> 我们可以指出现有的生产系统已经依赖于这些库，当人们问我们 ROS 是否“准备就绪”时。

## API changes

A further reason to build ROS 2 is to take advantage of the opportunity to improve our user-facing APIs.

> 构建 ROS 2 的另一个原因是利用这个机会来改进我们面向用户的 API。

A great deal of the ROS code that exists today is compatible with the client libraries as far back as the 0.4 "Mango Tango" release from February 2009.

> 很多现存的 ROS 代码与自 2009 年 2 月发布的 0.4 版“Mango Tango”客户端库兼容。

That's great from the point of view of stability, but it also implies that we're still living with API decisions that were made several years ago, some of which we know now to be not the best.

> 那从稳定性的角度来说是很棒的，但它也意味着我们仍然在使用几年前做出的 API 决定，其中有些我们现在知道不是最好的。

So, with ROS 2, we will design new APIs, incorporating to the best of our ability the collective experience of the community with the first-generation APIs.

> 所以，通过 ROS 2，我们将设计新的 API，尽可能充分利用社区对第一代 API 的经验。

As a result, while the key concepts (distributed processing, anonymous publish/subscribe messaging, RPC with feedback (i.e., actions), language neutrality, system introspectability, etc.) will remain the same, you should not expect ROS 2 to be API-compatible with existing ROS code.

> 结果，尽管关键概念（分布式处理、匿名发布/订阅消息、具有反馈（即动作）的 RPC、语言中立性、系统可内省性等）将保持不变，但不应期望 ROS 2 与现有 ROS 代码具有 API 兼容性。

But fear not: there will be mechanisms in place to allow ROS 2 code to coexist with existing ROS code.

> 不用担心：将会有机制让 ROS 2 代码与现有的 ROS 代码共存。

At the very least, there will be translation relays that will support run-time interactions between the two systems.

> 至少，会有翻译中继来支持运行时两个系统之间的交互。

And it is possible that there will be library shims that will allow existing ROS code to compile/run against ROS 2 libraries, with behavior that is qualitatively similar to what is seen today.

> 可能会有库 shims，可以让现有的 ROS 代码编译/运行在 ROS 2 库上，其行为与今天看到的相似。

## Why not just enhance ROS 1

In principle, the changes described above could be integrated into the existing core ROS code.

> 原则上，上述更改可以整合到现有的核心 ROS 代码中。

E.g., new transport technologies could be added to `roscpp` and `rospy`.

> 例如，可以向`roscpp`和`rospy`中添加新的运输技术。

We considered this option and concluded that, given the intrusive nature of the changes that would be required to achieve the benefits that we are seeking, there is too much risk associated with changing the current ROS system that is relied upon by so many people.

> 我们考虑了这个选项，并得出结论，鉴于实现我们正在寻求的好处所需的侵入性变化太大，改变目前被很多人依赖的 ROS 系统所带来的风险太大。

We want ROS 1 as it exists today to keep working and be unaffected by the development of ROS 2.

> 我们希望 ROS 1 保持现状，不受 ROS 2 的开发影响。

So ROS 2 will be built as a parallel set of packages that can be installed alongside and interoperate with ROS 1 (e.g., through message bridges).

> 所以 ROS 2 将作为一组可以与 ROS 1（例如，通过消息桥）一起安装和互操作的并行软件包而构建。
