---
tip: translate by openai@2023-05-28 11:05:25
    layout: default
    title: ROS on DDS
    permalink: articles/ros_on_dds.html
    abstract:
      This article makes the case for using DDS as the middleware for ROS, outlining the pros and cons of this approach, as well as considering the impact to the user experience and code API that using DDS would have.
      The results of the "ros_dds" prototype are also summarized and used in the exploration of the issue.
    author: '[William Woodall](https://github.com/wjwwood)'
    date_written: 2014-06
    last_modified: 2019-07
    published: true
    categories: Middleware
    summary: This article was written at a time before decisions were made to use DDS and RTPS as the underlying communication standards for ROS 2. For details on how ROS 2 has been implemented, see the [Core Documentation](http://docs.ros2.org/)
    Authors: {{ page.author }}
    Date Written: {{ page.date_written }}
    Last Modified: {% if page.last_modified %}{{ page.last_modified }}{% else %}{{ page.date_written }}{% endif %}

---

Terminology:

> 术语

    - [Data Distribution Service (DDS)](http://en.wikipedia.org/wiki/Data_Distribution_Service)
    - [Real-Time Publish Subscribe (RTPS)](https://en.wikipedia.org/wiki/Real-Time_Publish-Subscribe_(RTPS)_Protocol)
    - The [Object Management Group (OMG)](http://www.omg.org/)
    - OMG [Interface Description Language (IDL)](http://www.omg.org/gettingstarted/omg_idl.htm) | [Formal description](http://www.omg.org/cgi-bin/doc?formal/2014-03-01)

## Why Consider DDS

When exploring options for the next generation communication system of ROS, the initial options were to either improve the ROS 1 transport or build a new middleware using component libraries such as [ZeroMQ](http://zeromq.org/), Protocol Buffers, and zeroconf (Bonjour/Avahi).

> 当探索 ROS 下一代通信系统的选项时，最初的选择是改进 ROS 1 传输或使用诸如[ZeroMQ](http://zeromq.org/)、Protocol Buffers 和 zeroconf（Bonjour/Avahi）等组件库构建新的中间件。

However, in addition to those options, both of which involved us building a middleware from parts or scratch, other end-to-end middlewares were considered.

> 然而，除了这些选择，我们都考虑了从零部件或从头开始构建中间件的选择外，还考虑了其他端到端中间件。

During our research, one middleware that stood out was DDS.

> 在我们的研究中，最突出的中间件是 DDS。

### An End-to-End Middleware

The benefit of using an end-to-end middleware, like DDS, is that there is much less code to maintain and the behavior and exact specifications of the middleware have already been distilled into documentation.

> 使用端到端中间件（如 DDS）的好处是，需要维护的代码大大减少，而且中间件的行为和精确规格已经被提炼成文档。

In addition to system-level documentation, DDS also has recommended use cases and a software API.

> 除了系统级文档，DDS 还提供建议的使用案例和软件 API。

With this concrete specification, third parties can review, audit, and implement the middleware with varying degrees of interoperability.

> 随着这个具体的规范，第三方可以审查、审计和实施中间件，实现不同程度的互操作性。

This is something that ROS has never had, besides a few basic descriptions in a wiki and a reference implementation.

> 这是 ROS 从未拥有过的东西，除了维基上的几个基本描述和参考实现之外。

Additionally, this type of specification would need to be created anyway if a new middleware were to be built from existing libraries.

> 此外，如果要从现有库构建新的中间件，就需要创建这种类型的规范。

The drawback of using an end-to-end middleware is that ROS must work within that existing design.

> 使用端到端中间件的缺点是 ROS 必须在现有设计中运行。

If the design did not target a relevant use case or is not flexible, it might be necessary to work around the design.

> 如果设计没有针对相关的使用案例或灵活性不够，可能需要围绕设计进行工作。

On some level, adopting an end-to-end middleware includes adopting the philosophy and culture of that middleware, which should not be taken lightly.

> 采用端到端中间件，从某种程度上来说，就是采用该中间件的理念和文化，这一点不容忽视。

## What is DDS

DDS provides a publish-subscribe transport which is very similar to ROS's publish-subscribe transport.

> DDS 提供的发布-订阅传输与 ROS 的发布-订阅传输非常相似。

DDS uses the "Interface Description Language (IDL)" as defined by the [Object Management Group (OMG)](http://www.omg.org/) for message definition and serialization.

> DDS 使用由[Object Management Group（OMG）]（http://www.omg.org/）定义的“接口描述语言（IDL）”来定义消息和序列化。

DDS has a request-response style transport, which would be like ROS's service system, in beta 2 as of June 2016 (called [DDS-RPC](http://www.omg.org/spec/DDS-RPC/)).

> DDS 拥有一种请求-响应式传输，类似于 ROS 的服务系统，截至 2016 年 6 月，已在 beta 2 版本中实现（称为[DDS-RPC](http://www.omg.org/spec/DDS-RPC/)）。

The default discovery system provided by DDS, which is required to use DDS's publish-subscribe transport, is a distributed discovery system.

> DDS 所提供的默认发现系统，它是使用 DDS 发布订阅传输所必需的，是一个分布式发现系统。

This allows any two DDS programs to communicate without the need for a tool like the ROS master.

> 这允许任何两个 DDS 程序在不需要像 ROS 主控程序这样的工具的情况下进行通信。

This makes the system more fault tolerant and flexible.

> 这使得系统更加容错和灵活。

It is not required to use the dynamic discovery mechanism, however, as multiple DDS vendors provide options for static discovery.

> 不需要使用动态发现机制，但是，由于多个 DDS 供应商提供了静态发现的选项。

### Where did DDS come from

DDS got its start as a group of companies which had similar middleware frameworks and became a standard when common customers wanted to get better interoperability between the vendors.

> DDS 最初是一群具有相似中间件框架的公司组成的，当普通客户想要提高供应商之间的互操作性时，它便成为了一种标准。

The DDS standard was created by the Object Management Group, which are the same people that brought us UML, CORBA, SysML, and other generic software related standards.

> DDS 标准由对象管理组(Object Management Group)创建，这些人也给我们带来了 UML、CORBA、SysML 和其他通用软件相关标准。

Now, depending on your perspective, this may be a positive endorsement or a negative endorsement.

> 现在，根据您的观点，这可能是一个积极的认可或消极的认可。

On the one hand you have a standards committee which is perennial and clearly has a huge influence on the software engineering community, but on the other hand you have a slow moving body which is slow to adapt to changes and therefore arguably doesn't always keep up with the latest trends in software engineering.

> 一方面，你有一个标准委员会，它是永久性的，显然对软件工程界有巨大的影响力，但另一方面，你有一个缓慢的机构，缓慢地适应变化，因此可以说不总是跟上软件工程领域的最新趋势。

DDS was originally several similar middlewares which eventually became so close to one another that writing a standard to unify them made sense.

> DDS 最初是几个类似的中间件，最终它们彼此非常接近，因此写一个标准来统一它们是有意义的。

So in this way, even though the DDS specification has been written by a committee, it has evolved to its current form by reacting to the needs of its users.

> 因此，即使 DDS 规范是由一个委员会编写的，但它已经通过回应用户的需求而发展到现在的形式。

This type of organic evolution of the specification before it was ratified helps to alleviate the concern that the system was designed in a vacuum and that it does not perform well in real environments.

> 这种规范在被批准前的有机演变有助于减轻人们的担忧，即系统是在真空中设计的，在真实环境中表现不佳。

There are some examples of committees coming up with well intentioned and well described specifications that nobody wants to use or that don't meet the needs of the community they serve, but this does not appear to be the case for DDS.

> 有一些例子，委员会提出了出色的意图和描述的规范，但没有人愿意使用或不能满足所服务社区的需求，但这不适用于 DDS。

There is also a concern that DDS is a static specification which was defined and is used in "legacy" systems, but has not kept current.

> 有人担心 DDS 是一个静态的规范，它是在“传统”系统中定义和使用的，但没有保持最新。

This kind of stereotype comes from horror stories about things like UML and CORBA, which are also products of OMG.

> 这种刻板印象来自关于 UML 和 CORBA 等 OMG 的产品的恐怖故事。

On the contrary, DDS seems to have an active and organic specification, which in the recent past has added, or is adding, more specifications for things like websockets, security over SSL, extensible types, request and response transport, and a new, more modern C++11 style API specification for the core API to replace the existing C++ interface.

> 反之，DDS 似乎有一个活跃而有机的规范，最近添加了，或正在添加，更多的规范，比如 Websockets，SSL 安全，可扩展类型，请求和响应传输，以及一个新的，更现代的 C++11 样式的 API 规范，用于替换现有的 C++接口。

This type of evolution in the standard body for DDS is an encouraging thing to observe, and even though the body is relatively slow, as compared to software engineering technology trends, it is evolving to meet demands of its users.

> 这种在标准 DDS 身体上的演化是一件令人鼓舞的事情，尽管与软件工程技术趋势相比，它的发展速度相对较慢，但它正在演变以满足其用户的需求。

### Technical Credibility

DDS has an extensive list of varied installations which are typically mission critical.

> DDS 拥有一份广泛的、多样化的安装清单，通常是任务关键性的。

DDS has been used in:

> DDS 已被用于：

- battleships

> - 战舰

- large utility installations like dams

> 大型公用设施，如大坝

- financial systems

> - 财务系统

- space systems

> 空间系统

- flight systems

> 飞行系统

- train switchboard systems

> - 训练开关板系统

and many other equally important and varied scenarios.

> 和许多其他同样重要和多样化的场景。

These successful use cases lend credibility to DDS's design being both reliable and flexible.

> 这些成功的用例证明了 DDS 的设计既可靠又灵活。

Not only has DDS met the needs of these use cases, but after talking with users of DDS (in this case government and NASA employees who are also users of ROS), they have all praised its reliability and flexibility.

> .

不仅 DDS 满足了这些用例的需求，而且在与 DDS 用户（在这种情况下是政府和 NASA 的员工也是 ROS 的用户）交谈后，他们都赞扬了它的可靠性和灵活性。

Those same users will note that the flexibility of DDS comes at the cost of complexity.

> 那些相同的用户会注意到，DDS 的灵活性是以复杂性为代价的。

The complexity of the API and configuration of DDS is something that ROS would need to address.

> API 和 DDS 的配置复杂性是 ROS 需要解决的问题。

The DDS wire specification ([DDSI-RTPS](http://www.omg.org/spec/DDSI-RTPS/)) is extremely flexible, allowing it to be used for reliable, high level systems integration as well as real-time applications on embedded devices.

> DDS 线缆规范（[DDSI-RTPS]（http://www.omg.org/spec/DDSI-RTPS/））非常灵活，可用于可靠的高级系统集成以及嵌入式设备上的实时应用程序。

Several of the DDS vendors have special implementations of DDS for embedded systems which boast specs related to library size and memory footprint on the scale of tens or hundreds of kilobytes.

> 许多 DDS 供应商都有特殊的嵌入式系统的 DDS 实现，它们的规格与库大小和内存占用量有关，可以达到数十或数百 KB 的规模。

Since DDS is implemented, by default, on UDP, it does not depend on a reliable transport or hardware for communication.

> 由于 DDS 默认实现在 UDP 上，因此它不依赖可靠的传输或硬件进行通信。

This means that DDS has to reinvent the reliability wheel (basically TCP plus or minus some features), but in exchange DDS gains portability and control over the behavior.

> 这意味着 DDS 必须重新发明可靠性轮（基本上是 TCP 加上或减去一些特性），但作为交换，DDS 获得了可移植性和对行为的控制。

Control over several parameters of reliability, what DDS calls Quality of Service (QoS), gives maximum flexibility in controlling the behavior of communication.

> 控制多个可靠性参数（DDS 称之为服务质量（QoS）），可以最大限度地控制通信的行为。

For example, if you are concerned about latency, like for soft real-time, you can basically tune DDS to be just a UDP blaster.

> 例如，如果您担心延迟，就像软实时一样，您基本上可以调整 DDS 来成为 UDP 发射器。

In another scenario you might need something that behaves like TCP, but needs to be more tolerant to long dropouts, and with DDS all of these things can be controlled by changing the QoS parameters.

> 在另一种情况下，您可能需要一些行为类似 TCP 的东西，但需要对长时间断开更加宽容，而使用 DDS，所有这些都可以通过更改 QoS 参数来控制。

Though the default implementation of DDS is over UDP, and only requires that level of functionality from the transport, OMG also added support for DDS over TCP in version 1.2 of their specification.

> 尽管 DDS 的默认实现是基于 UDP 的，并且只需要运输层提供这种功能，但 OMG 在他们 1.2 版本的规范中也增加了对 DDS over TCP 的支持。

Only looking briefly, two of the vendors (RTI and ADLINK Technologies) both support DDS over TCP.

> 只简单地看一眼，两家供应商（RTI 和 ADLINK Technologies）都支持通过 TCP 的 DDS。

From RTI's website ([http://community.rti.com/kb/xml-qos-example-using-rti-connext-dds-tcp-transport](http://community.rti.com/kb/xml-qos-example-using-rti-connext-dds-tcp-transport)):

> 从 RTI 的网站([http://community.rti.com/kb/xml-qos-example-using-rti-connext-dds-tcp-transport](http://community.rti.com/kb/xml-qos-example-using-rti-connext-dds-tcp-transport))：

> By default, RTI Connext DDS uses the UDPv4 and Shared Memory transport to communicate with other DDS applications.
> In some circumstances, the TCP protocol might be needed for discovery and data exchange.
> For more information on the RTI TCP Transport, please refer to the section in the RTI Core Libraries and Utilities User Manual titled "RTI TCP Transport".

From ADLINK's website, they support TCP as of OpenSplice v6.4:

> 从 ADLINK 的网站上看，从 OpenSplice v6.4 开始就支持 TCP 了。

[https://www.adlinktech.com/en/data-distribution-service.aspx](https://www.adlinktech.com/en/data-distribution-service.aspx)

> [https://www.adlinktech.com/zh-cn/data-distribution-service.aspx](https://www.adlinktech.com/zh-cn/data-distribution-service.aspx) 数据分发服务

### Vendors and Licensing

The OMG defined the DDS specification with several companies which are now the main DDS vendors.

> OMG 定义了 DDS 规范，有几家公司现在是主要的 DDS 供应商。

Popular DDS vendors include:

> 流行的 DDS 供应商包括：

- RTI

> - 通用请求信息（RTI）

- ADLINK Technologies

> ADLINK 技术

- Twin Oaks Software

> 双橡软件

Amongst these vendors is an array of reference implementations with different strategies and licenses.

> 在这些供应商中，有各种不同策略和许可证的参考实现。

The OMG maintains an active [list](http://dds-directory.omg.org/vendor/list.htm) of DDS vendors.

> OMG 维护一个活跃的 DDS 供应商[列表](http://dds-directory.omg.org/vendor/list.htm)。

In addition to vendors providing implementations of the DDS specification's API, there are software vendors which provide an implementation with more direct access to the DDS wire protocol, RTPS.

> 除了提供 DDS 规范 API 实现的供应商外，还有软件供应商提供更直接访问 DDS 线路协议 RTPS 的实现。

For example:

> 例如：

- eProsima

> - eProsima（伊普罗西玛）

These RTPS-centric implementations are also of interest because they can be smaller in scope and still provide the needed functionality for implementing the necessary ROS capabilities on top.

> 这些以 RTPS 为中心的实现也很有趣，因为它们的范围可以更小，但仍然可以提供实现所需 ROS 功能所需的功能。

> [RTI]:
> By "compatible with the ROS community's needs," we mean that, though it is not an [OSI-approved license](https://opensource.org/licenses), research has shown it to be adequately permissive to allow ROS to keep a BSD style license and for anyone in the ROS community to redistribute it in source or binary form.

> 通过“与 ROS 社区需求兼容”，我们的意思是，尽管它不是 OSI 认可的许可证，但研究表明它足够宽松，允许 ROS 保持 BSD 样式许可证，并允许 ROS 社区的任何人以源代码或二进制形式重新分发它。

> [RTI]:
> Like the other vendors this license is available for the core set of functionality, basically the basic DDS API, whereas other parts of their product like development and introspection tools are proprietary.

> 像其他供应商一样，此许可证可用于核心功能集，基本上是基本的 DDS API，而他们产品的其他部分，如开发和内省工具是专有的。

> [RTI]:

ADLINK's DDS implementation, OpenSplice, is licensed under the LGPL, which is the same license used by many popular open source libraries, like glibc, ZeroMQ, and Qt.

> ADLINK 的 DDS 实现 OpenSplice 使用 LGPL 许可证，这与许多流行的开源库（如 glibc、ZeroMQ 和 Qt）使用的许可证相同。

It is available on [Github](https://github.com):

> 它可以在[Github](https://github.com)上找到。

[https://github.com/ADLINK-IST/opensplice](https://github.com/ADLINK-IST/opensplice)

> [https://github.com/ADLINK-IST/opensplice](https://github.com/ADLINK-IST/opensplice)（简体中文）

ADLINK's implementation comes with a basic, functioning build system and was fairly easy to package.

> ADLINK 的实施带有一个基本的、可运行的构建系统，打包相对容易。

OpenSplice appears to be the number two DDS implementation in use, but that is hard to tell for sure.

> OpenSplice 似乎是第二大 DDS 实施方案，但很难确定。

TwinOaks's CoreDX DDS implementation is proprietary only, but apparently they specialize in minimal implementations which are able to run on embedded devices and even bare metal.

> 双橡树的 CoreDX DDS 实现是专有的，但显然它们专门从事能够在嵌入式设备甚至裸机上运行的最小实现。

eProsima's FastRTPS implementation is available on GitHub and is LGPL licensed:

> eProsima 的 FastRTPS 实现可在 GitHub 上获得，并受 LGPL 许可：

[https://github.com/eProsima/Fast-RTPS](https://github.com/eProsima/Fast-RTPS)

> [https://github.com/eProsima/Fast-RTPS](https://github.com/eProsima/Fast-RTPS) 简体中文版

eProsima Fast RTPS is a relatively new, lightweight, and open source implementation of RTPS.

> eProsima Fast RTPS 是一个相对较新的、轻量级的、开源的 RTPS 实现。

It allows direct access to the RTPS protocol settings and features, which is not always possible with other DDS implementations.

> 它允许直接访问 RTPS 协议设置和功能，这在其他 DDS 实现中并不总是可能的。

eProsima's implementation also includes a minimum DDS API, IDL support, and automatic code generation and they are open to working with the ROS community to meet their needs.

> eProsima 的实现还包括最小的 DDS API、IDL 支持和自动代码生成，他们也愿意与 ROS 社区合作，满足他们的需求。

Given the relatively strong LGPL option and the encouraging but custom license from RTI, it seems that depending on and even distributing DDS as a dependency should be straightforward.

> 根据相对强大的 LGPL 选项和 RTI 的鼓励但自定义许可，似乎依赖和甚至分发 DDS 应该是很简单的。

One of the goals of this proposal would be to make ROS 2 DDS vendor agnostic.

> 一个本提案的目标是使 ROS 2 DDS 供应商无关性。

So, just as an example, if the default implementation is Connext, but someone wants to use one of the LGPL options like OpenSplice or FastRTPS, they simply need to recompile the ROS source code with some options flipped and they can use the implementation of their choice.

> 如果默认实现是 Connext，但有人想使用像 OpenSplice 或 FastRTPS 之类的 LGPL 选项，他们只需要用一些选项重新编译 ROS 源代码，就可以使用他们选择的实现。

This is made possible because of the fact that DDS defines an API in its specification.

> 这是因为 DDS 在其规范中定义了一个 API 而成为可能的。

Research has shown that making code which is vendor agnostic is possible if not a little painful since the APIs of the different vendors is almost identical, but there are minor differences like return types (pointer versus shared_ptr like thing) and header file organization.

> 研究表明，如果不是有点痛苦，就可以制作供应商中立的代码，因为不同供应商的 API 几乎是相同的，但是有一些小的差异，比如返回类型（指针与 shared_ptr 之类的）和头文件组织。

### Ethos and Community

DDS comes out of a set of companies which are decades old, was laid out by the OMG which is an old-school software engineering organization, and is used largely by government and military users.

> DDS 来自一组已有数十年历史的公司，由 OMG（一个老式的软件工程组织）制定，主要被政府和军事用户使用。

So it comes as no surprise that the community for DDS looks very different from the ROS community and that of similar modern software projects like ZeroMQ.

> 所以，DDS 社区与 ROS 社区以及像 ZeroMQ 这样的现代软件项目的社区有很大的不同，这并不奇怪。

Though RTI has a respectable on-line presence, the questions asked by community members are almost always answered by an employee of RTI and though technically open source, neither RTI nor OpenSplice has spent time to provide packages for Ubuntu or Homebrew or any other modern package manager.

> 尽管 RTI 在线上有可观的存在，但社区成员提出的问题几乎总是由 RTI 的员工回答，尽管在技术上是开源的，但 RTI 和 OpenSplice 都没有花时间提供 Ubuntu 或 Homebrew 或任何其他现代包管理器的软件包。

They do not have extensive user-contributed wikis or an active Github repository.

> 他们没有大量的用户贡献的 wiki 或一个活跃的 Github 仓库。

This staunch difference in ethos between the communities is one of the most concerning issues with depending on DDS.

> 这种两个社区之间的坚定的理念差异是依赖 DDS 最令人担忧的问题之一。

Unlike options like keeping TCPROS or using ZeroMQ, there isn't the feeling that there is a large community to fall back on with DDS.

> 与保持 TCPROS 或使用 ZeroMQ 等选项不同，DDS 没有大量可以依靠的社区的感觉。

However, the DDS vendors have been very responsive to our inquiries during our research and it is hard to say if that will continue when it is the ROS community which brings the questions.

> 然而，在我们的研究期间，DDS 供应商对我们的询问非常反应迅速，很难说当 ROS 社区提出问题时会不会继续如此。

Even though this is something which should be taken into consideration when making a decision about using DDS, it should not disproportionately outweigh the technical pros and cons of the DDS proposal.

> 即使这是在考虑使用 DDS 时应该考虑的事情，也不应该过分地压制 DDS 提案的技术优缺点。

## ROS Built on DDS

The goal is to make DDS an implementation detail of ROS 2.

> 目标是让 DDS 成为 ROS 2 的实现细节。

This means that all DDS specific APIs and message definitions would need to be hidden.

> 这意味着所有 DDS 特定的 API 和消息定义都需要被隐藏起来。

DDS provides discovery, message definition, message serialization, and publish-subscribe transport.

> DDS 提供发现、消息定义、消息序列化和发布订阅传输。

Therefore, DDS would provide discovery, publish-subscribe transport, and at least the underlying message serialization for ROS.

> 因此，DDS 将提供发现，发布-订阅传输以及 ROS 的基础消息序列化。

ROS 2 would provide a ROS 1 like interface on top of DDS which hides much of the complexity of DDS for the majority of ROS users, but then separately provides access to the underlying DDS implementation for users that have extreme use cases or need to integrate with other, existing DDS systems.

> ROS 2 将在 DDS 之上提供一个类似于 ROS 1 的界面，为大多数 ROS 用户隐藏 DDS 的复杂性，但对于拥有极端用例或需要与其他现有 DDS 系统集成的用户，则可以单独访问底层 DDS 实现。

![DDS and ROS API Layout](/img/ros_on_dds/api_levels.png)

_DDS and ROS API Layout_

> _DDS 和 ROS API 布局_

Accessing the DDS implementation would require depending on an additional package which is not normally used.

> 访问 DDS 实现需要依赖于一个通常不使用的额外的包。

In this way you can tell if a package has tied itself to a particular DDS vendor by just looking at the package dependencies.

> 通过查看包的依赖性，可以这样确定一个包是否已经与某个 DDS 供应商绑定在一起。

The goal of the ROS API, which is on top of DDS, should be to meet all the common needs for the ROS community, because once a user taps into the underlying DDS system, they will lose portability between DDS vendors.

> ROS API 的目标在 DDS 之上，应该满足 ROS 社区的所有常见需求，因为一旦用户接入底层的 DDS 系统，他们就会失去在 DDS 供应商之间的可移植性。

Portability among DDS vendors is not intended to encourage people to frequently choose different vendors, but rather to enable power users to select the DDS implementation that meets their specific requirements, as well as to future-proof ROS against changes in the DDS vendor options.

> 可移植性在 DDS 供应商之间并不是为了鼓励人们经常更换供应商，而是为了使高级用户能够选择满足其特定要求的 DDS 实现，以及使 ROS 免受 DDS 供应商选择变化的影响。

There will be one recommended and best-supported default DDS implementation for ROS.

> 将有一个针对 ROS 的推荐且最佳支持的默认 DDS 实现。

### Discovery

DDS would completely replace the ROS master based discovery system.

> DDS 将完全取代基于 ROS 主节点的发现系统。

ROS would need to tap into the DDS API to get information like a list of all nodes, a list of all topics, and how they are connected.

> ROS 需要调用 DDS API 来获取信息，比如所有节点的列表、所有主题的列表以及它们之间的连接情况。

Accessing this information would be hidden behind a ROS defined API, preventing the users from having to call into DDS directly.

> 访问这些信息将被 ROS 定义的 API 所隐藏，从而防止用户直接调用 DDS。

The advantage of the DDS discovery system is that, by default, it is completely distributed, so there is no central point of failure which is required for parts of the system to communicate with each other.

> DDS 发现系统的优势在于，默认情况下，它是完全分布式的，因此不需要中央点来实现系统各部分之间的通信。

DDS also allows for user defined meta data in their discovery system, which will enable ROS to piggyback higher level concepts onto publish-subscribe.

> .

DDS 允许在其发现系统中使用用户定义的元数据，这将使 ROS 能够在发布-订阅上搭载更高级的概念。

### Publish-Subscribe Transport

The DDSI-RTPS (DDS-Interoperability Real Time Publish Subscribe) protocol would replace ROS's TCPROS and UDPROS wire protocols for publish/subscribe.

> DDSI-RTPS（DDS 互操作性实时发布订阅）协议将取代 ROS 的 TCPROS 和 UDPROS 线路协议进行发布/订阅。

The DDS API provides a few more actors to the typical publish-subscribe pattern of ROS 1.

> DDS API 为 ROS 1 的典型发布订阅模式增加了更多演员。

In ROS the concept of a node is most clearly paralleled to a graph participant in DDS.

> 在 ROS 中，节点的概念最清楚地与 DDS 中的图形参与者相对应。

A graph participant can have zero to many topics, which are very similar to the concept of topics in ROS, but are represented as separate code objects in DDS, and is neither a subscriber nor a publisher.

> 一个图形参与者可以有零到多个主题，它们与 ROS 中的主题概念非常相似，但在 DDS 中表示为单独的代码对象，既不是订阅者也不是发布者。

Then, from a DDS topic, DDS subscribers and publishers can be created, but again these are used to represent the subscriber and publisher concepts in DDS, and not to directly read data from or write data to the topic.

> 然后，可以创建 DDS 主题的订阅者和发布者，但是这些仅用于表示 DDS 中的订阅者和发布者概念，而不是直接从主题中读取或写入数据。

DDS has, in addition to the topics, subscribers, and publishers, the concept of DataReaders and DataWriters which are created with a subscriber or publisher and then specialized to a particular message type before being used to read and write data for a topic.

> DDS 除了主题、订阅者和发布者之外，还有 DataReader 和 DataWriter 的概念，这些对象是通过订阅者或发布者创建的，然后针对特定的消息类型进行专门化，然后用于读取和写入主题数据。

These additional layers of abstraction allow DDS to have a high level of configuration, because you can set QoS settings at each level of the publish-subscribe stack, providing the highest granularity of configuration possible.

> 这些额外的抽象层使得 DDS 具有很高的配置级别，因为您可以在发布-订阅堆栈的每个级别设置 QoS 设置，提供最高细度的配置。

Most of these levels of abstractions are not necessary to meet the current needs of ROS.

> 大多数这些抽象层次并不是满足 ROS 当前需求所必需的。

Therefore, packaging common workflows under the simpler ROS-like interface (Node, Publisher, and Subscriber) will be one way ROS 2 can hide the complexity of DDS, while exposing some of its features.

> 因此，将常见工作流程封装在简单的 ROS 类似接口（节点、发布者和订阅者）之下，将是 ROS 2 隐藏 DDS 复杂性的一种方式，同时暴露其一些功能。

### Efficient Transport Alternatives

In ROS 1 there was never a standard shared-memory transport because it is negligibly faster than localhost TCP loop-back connections.

> 在 ROS 1 中从来没有标准的共享内存传输，因为与本地主机 TCP 回环连接的速度相比几乎可以忽略不计。

It is possible to get non-trivial performance improvements from carefully doing zero-copy style shared-memory between processes, but anytime a task required faster than localhost TCP in ROS 1, nodelets were used.

> 可以通过仔细地在进程之间进行零拷贝式共享内存来获得非平凡的性能改进，但是在 ROS 1 中，一旦任务需要比本地主机 TCP 更快，就会使用节点。

Nodelets allow publishers and subscribers to share data by passing around `boost::shared_ptr`s to messages.

> 节点允许发布者和订阅者通过传递`boost::shared_ptr`来共享数据。

This intraprocess communication is almost certainly faster than any interprocess communication options and is orthogonal to the discussion of the network publish-subscribe implementation.

> 这种内部进程通信几乎肯定比任何进程间通信选项要快，并且与网络发布订阅实现的讨论无关。

In the context of DDS, most vendors will optimize message traffic (even between processes) using shared-memory in a transparent way, only using the wire protocol and UDP sockets when leaving the localhost.

> 在 DDS 的背景下，大多数供应商会透明地使用共享内存来优化消息流量（甚至是进程间的流量），只有当离开本地主机时才使用线路协议和 UDP 套接字。

This provides a considerable performance increase for DDS, whereas it did not for ROS 1, because the localhost networking optimization happens at the call to `send`.

> 这为 DDS 提供了相当大的性能提升，但对 ROS 1 来说却没有，因为本地主机网络优化发生在调用`send`时。

For ROS 1 the process was: serialize the message into one large buffer, call TCP's `send` on the buffer once.

> 对于 ROS 1，过程是：将消息序列化为一个大缓冲区，调用 TCP 的`send`函数一次。

For DDS the process would be more like: serialize the message, break the message into potentially many UDP packets, call UDP's `send` many times.

> 对于 DDS 来说，过程会更像是：序列化消息，将消息分解成可能的多个 UDP 数据包，多次调用 UDP 的`send`。

In this way sending many UDP datagrams does not benefit from the same speed up as one large TCP `send`.

> 在这种方式下，发送许多 UDP 数据报并不能像发送一个大的 TCP'send'一样获得同样的加速效果。

Therefore, many DDS vendors will short circuit this process for localhost messages and use a blackboard style shared-memory mechanism to communicate efficiently between processes.

> 因此，许多 DDS 供应商会简化本地主机消息的过程，并使用黑板式共享内存机制来高效地在进程之间进行通信。

However, not all DDS vendors are the same in this respect, so ROS would not rely on this "intelligent" behavior for efficient **intra**process communication.

> 然而，在这方面，并不是所有的 DDS 供应商都一样，因此 ROS 不会依赖这种“智能”行为来实现高效的**intra**进程通信。

Additionally, if the ROS message format is kept, which is discussed in the next section, it would not be possible to prevent a conversion to the DDS message type for intraprocess topics.

> 另外，如果保持 ROS 消息格式，即在下一节中讨论的内容，那么无法阻止将 intraprocess topics 转换为 DDS 消息类型。

Therefore a custom intraprocess communication system would need to be developed for ROS which would never serialize nor convert messages, but instead would pass pointers (to shared in-process memory) between publishers and subscribers using DDS topics.

> 因此，需要为 ROS 开发一种自定义的内部进程通信系统，它不会序列化或转换消息，而是使用 DDS 主题在发布者和订阅者之间传递指针（指向共享的进程内存）。

This same intraprocess communication mechanism would be needed for a custom middleware built on ZeroMQ, for example.

> 这个同样的内部进程通信机制也适用于基于 ZeroMQ 构建的自定义中间件，例如。

The point to take away here is that efficient **intra**process communication will be addressed regardless of the network/interprocess implementation of the middleware.

> 这里要指出的重点是，不管中间件的网络/进程间实现如何，都将解决高效的进程内通信问题。

### Messages

There is a great deal of value in the current ROS message definitions.

> 当前 ROS 消息定义中有很大的价值。

The format is simple, and the messages themselves have evolved over years of use by the robotics community.

> 格式很简单，而且这些消息本身已经经过机器人社区多年的使用而不断发展。

Much of the semantic contents of current ROS code is driven by the structure and contents of these messages, so preserving the format and in-memory representation of the messages has a great deal of value.

> 许多当前 ROS 代码的语义内容受这些消息的结构和内容的驱动，因此保留消息的格式和内存表示具有很大的价值。

In order to meet this goal, and in order to make DDS an implementation detail, ROS 2 should preserve the ROS 1 like message definitions and in-memory representation.

> 为了实现这个目标，为了让 DDS 成为一个实现细节，ROS 2 应该保留 ROS 1 类似的消息定义和内存表示。

Therefore, the ROS 1 `.msg` files would continue to be used and the `.msg` files would be converted into `.idl` files so that they could be used with the DDS transport.

> 因此，ROS 1 `.msg` 文件将继续使用，`.msg` 文件将被转换为`.idl`文件，以便它们可以与 DDS 传输一起使用。

Language specific files would be generated for both the `.msg` files and the `.idl` files as well as conversion functions for converting between ROS and DDS in-memory instances.

> 为`.msg`文件和`.idl`文件生成特定语言的文件，以及用于在 ROS 和 DDS 内存实例之间进行转换的转换函数。

The ROS 2 API would work exclusively with the `.msg` style message objects in memory and would convert them to `.idl` objects before publishing.

> ROS 2 API 只能够与`.msg`样式的消息对象在内存中工作，并且在发布之前将它们转换为`.idl`对象。

![Message Generation Diagram](/img/ros_on_dds/message_generation.png "Message Generation Diagram")

At first, the idea of converting a message field-by-field into another object type for each call to publish seems like a huge performance problem, but experimentation has shown that the cost of this copy is insignificant when compared to the cost of serialization.

> 首先，把消息字段一个一个转换成另一种对象类型每次发布似乎是一个巨大的性能问题，但实验表明，与序列化成本相比，这种复制的成本微不足道。

This ratio between the cost of converting types and the cost of serialization, which was found to be at least one order of magnitude, holds true with every serialization library that we tried, except [Cap'n Proto](http://kentonv.github.io/capnproto/) which doesn't have a serialization step.

> 这个在转换类型和序列化成本之间的比率至少是一个数量级，对于我们尝试的每一个序列化库都是成立的，除了[Cap'n Proto](http://kentonv.github.io/capnproto/)，它没有序列化步骤。

Therefore, if a field-by-field copy will not work for your use case, neither will serializing and transporting over the network, at which point you will have to utilize an intraprocess or zero-copy interprocess communication.

> 因此，如果字段复制不能满足您的使用情况，序列化和网络传输也不会工作，此时您将不得不使用进程内或零复制进程间通信。

The intraprocess communication in ROS would not use the DDS in-memory representation so this field-by-field copy would not be used unless the data is going to the wire.

> ROS 中的内部进程通信不会使用 DDS 内存表示，因此除非数据要发送到线上，否则不会使用字段逐字拷贝。

Because this conversion is only invoked in conjunction with a more expensive serialization step, the field-by-field copy seems to be a reasonable trade-off for the portability and abstraction provided by preserving the ROS `.msg` files and in-memory representation.

> 由于这种转换仅与更昂贵的序列化步骤相结合，因此字段逐个复制似乎是为了保留 ROS` .msg`文件和内存表示而提供的可移植性和抽象性的合理折衷方案。

This does not preclude the option to improve the `.msg` file format with things like default values and optional fields.

> 这并不排除改进`.msg`文件格式的选择，比如默认值和可选字段。

But this is a different trade-off which can be decided later.

> 但这是一个不同的折衷方案，可以稍后决定。

### Services and Actions

DDS currently does not have a ratified or implemented standard for request-response style RPC which could be used to implement the concept of services in ROS.

> DDS 目前没有一个被批准或实施的标准，用于实现 ROS 中服务概念的请求-响应式 RPC。

There is currently an RPC specification being considered for ratification in the OMG DDS working group, and several of the DDS vendors have a draft implementation of the RPC API.

> 目前，OMG DDS 工作组正在考虑批准一项 RPC 规范，而且有几家 DDS 供应商已经起草了 RPC API 的草案实现。

It is not clear, however, whether this standard will work for actions, but it could at least support non-preemptable version of ROS services.

> 这个标准是否适用于行动尚不清楚，但它至少可以支持 ROS 服务的不可抢占版本。

ROS 2 could either implement services and actions on top of publish-subscribe (this is more feasible in DDS because of their reliable publish-subscribe QoS setting) or it could use the DDS RPC specification once it is finished for services and then build actions on top, again like it is in ROS 1.

> ROS 2 可以在发布-订阅之上实现服务和操作（由于其可靠的发布-订阅 QoS 设置，这在 DDS 中更容易实现），或者可以在服务上使用 DDS RPC 规范，然后再建立操作，就像 ROS 1 中一样。

Either way actions will be a first class citizen in the ROS 2 API and it may be the case that services just become a degenerate case of actions.

> 不管怎样，动作将成为 ROS 2 API 中的一等公民，服务可能只是动作的一种退化情况。

### Language Support

DDS vendors typically provide at least C, C++, and Java implementations since APIs for those languages are explicitly defined by the DDS specification.

> DDS 供应商通常提供至少 C、C++ 和 Java 实现，因为 DDS 规范明确定义了这些语言的 API。

There are not any well established versions of DDS for Python that research has uncovered.

> 没有研究发现为 Python 建立的 DDS 版本。

Therefore, one goal of the ROS 2 system will be to provide a first-class, feature complete C API.

> 因此，ROS 2 系统的一个目标将是提供一流的、功能完整的 C API。

This will allow bindings for other languages to be made more easily and to enable more consistent behavior between client libraries, since they will use the same implementation.

> 这将使其他语言的绑定更容易实现，并使客户端库之间的行为更加一致，因为它们将使用相同的实现。

Languages like Python, Ruby, and Lisp can wrap the C API in a thin, language idiomatic implementation.

> 诸如 Python、Ruby 和 Lisp 这样的语言可以用一种薄薄的、符合语言习惯的实现来包装 C API。

The actual implementation of ROS can either be in C, using the C DDS API, or in C++ using the DDS C++ API and then wrapping the C++ implementation in a C API for other languages.

> 实际的 ROS 实现可以使用 C 语言和 C DDS API，或者使用 DDS C++ API，然后用 C API 将 C++实现包装起来，以便其他语言使用。

Implementing in C++ and wrapping in C is a common pattern, for example [ZeroMQ](http://zeromq.org/) does exactly this.

> 在 C++中实现并用 C 封装是一种常见模式，例如[ZeroMQ](http://zeromq.org/)就是这样做的。

The author of [ZeroMQ](http://zeromq.org/), however, did not do this in his new library, [nanomsg](http://nanomsg.org/), citing increased complexity and the bloat of the C++ stdlib as a dependency.

> 作者在新的库[nanomsg](http://nanomsg.org/)中没有使用[ZeroMQ](http://zeromq.org/)，因为增加了复杂性，而且依赖于 C++ stdlib 的臃肿。

Since the C implementation of DDS is typically pure C, it would be possible to have a pure C implementation for the ROS C API all the way down through the DDS implementation.

> 由于 DDS 的 C 实现通常是纯 C 的，因此可以在 ROS C API 中从 DDS 实现一直到纯 C 实现。

However, writing the entire system in C might not be the first goal, and in the interest of getting a minimal viable product working, the implementation might be in C++ and wrapped in C to begin with and later the C++ can be replaced with C if it seems necessary.

> 然而，将整个系统完全用 C 编写可能不是首要目标，为了让最小可行产品可以正常工作，实现可能以 C++开始，并用 C 包装，然后如果有必要，可以用 C 替换 C++。

### DDS as a Dependency

One of the goals of ROS 2 is to reuse as much code as possible ("do not reinvent the wheel") but also minimize the number of dependencies to improve portability and to keep the build dependency list lean.

> ROS 2 的目标之一是尽可能多地重用代码（“不要重新发明轮子”），同时最大限度地减少依赖关系，以提高可移植性，并保持构建依赖关系列表的精简。

These two goals are sometimes at odds, since it is often the choice between implementing something internally or relying on an outside source (dependency) for the implementation.

> 这两个目标有时会相互矛盾，因为通常要在在内部实施某事或依赖外部来源（依赖）来实施之间做出选择。

This is a point where the DDS implementations shine, because two of the three DDS vendors under evaluation build on Linux, OS X, Windows, and other more exotic systems with no external dependencies.

> 这是 DDS 实现的一个亮点，因为在评估中的三个 DDS 供应商中，有两个建立在 Linux、OS X、Windows 和其他更多异国系统上，而且没有外部依赖关系。

The C implementation relies only on the system libraries, the C++ implementations only rely on a C++03 compiler, and the Java implementation only needs a JVM and the Java standard library.

> C 实现仅依赖于系统库，C++ 实现仅依赖于 C++03 编译器，而 Java 实现仅需要 JVM 和 Java 标准库。

Bundled as a binary (during prototyping) on both Ubuntu and OS X, the C, C++, Java, and C# implementations of OpenSplice (LGPL) is less than three megabytes in size and has no other dependencies.

> 在 Ubuntu 和 OS X 上，将 OpenSplice（LGPL）的 C、C++、Java 和 C#实现作为二进制文件（在原型设计期间）打包时，大小不到三兆字节，且没有其他依赖性。

As far as dependencies go, this makes DDS very attractive because it significantly simplifies the build and run dependencies for ROS.

> 就依赖性而言，DDS 非常有吸引力，因为它显著简化了 ROS 的构建和运行依赖关系。

Additionally, since the goal is to make DDS an implementation detail, it can probably be removed as a transitive run dependency, meaning that it will not even need to be installed on a deployed system.

> 此外，由于目标是使 DDS 成为实现细节，它可能被作为传递运行依赖性删除，这意味着它甚至不需要在部署的系统上安装。

## The ROS on DDS Prototype

Following the research into the feasibility of ROS on DDS, several questions were left, including but not limited to:

> 在研究 ROS 在 DDS 上的可行性之后，留下了一些问题，包括但不限于：

- Can the ROS 1 API and behavior be implemented on top of DDS?

> - DDS 能否实现 ROS 1 API 和行为？

- Is it practical to generate IDL messages from ROS MSG messages and use them with DDS?

> 是从 ROS MSG 消息生成 IDL 消息并使用 DDS 来实现吗？

- How hard is it to package (as a dependency) DDS implementations?

> 多少难度来打包（作为一个依赖）DDS 实现？

- Does the DDS API specification actually make DDS vendor portability a reality?

> .

这个 DDS API 规范真的使得 DDS 供应商可移植性成为现实吗？

- How difficult is it to configure DDS?

> 多么难配置 DDS？

In order to answer some of these questions a prototype and several experiments were created in this repository:

> 为了回答这些问题，本存储库中创建了一个原型和几个实验：

[https://github.com/osrf/ros_dds](https://github.com/osrf/ros_dds)

> [https://github.com/osrf/ros_dds](https://github.com/osrf/ros_dds)：ROS DDS（可扩展可靠性数据发布/订阅）

More questions and some of the results were captured as issues:

> 更多的问题和一些结果被捕获为问题：

[https://github.com/osrf/ros_dds/issues?labels=task&page=1&state=closed](https://github.com/osrf/ros_dds/issues?labels=task&page=1&state=closed)

> [https://github.com/osrf/ros_dds/issues?labels=任务&page=1&state=关闭](https://github.com/osrf/ros_dds/issues?labels=任务&page=1&state=关闭)

The major piece of work in this repository is in the `prototype` folder and is a ROS 1 like implementation of the Node, Publisher, and Subscriber API using DDS:

> 在这个存储库中最重要的工作位于`prototype`文件夹中，它是一个使用 DDS 实现的类似 ROS 1 的节点、发布者和订阅者 API：

[https://github.com/osrf/ros_dds/tree/master/prototype](https://github.com/osrf/ros_dds/tree/master/prototype)

> [https://github.com/osrf/ros_dds/tree/master/prototype](https://github.com/osrf/ros_dds/tree/master/prototype)：原型

Specifically this prototype includes these packages:

> 具体来说，这个原型包括这些包：

- Generation of DDS IDLs from `.msg` files: [https://github.com/osrf/ros_dds/tree/master/prototype/src/genidl](https://github.com/osrf/ros_dds/tree/master/prototype/src/genidl)

> 生成 DDS IDLs 从`.msg`文件：[https://github.com/osrf/ros_dds/tree/master/prototype/src/genidl](https://github.com/osrf/ros_dds/tree/master/prototype/src/genidl)

- Generation of DDS specific C++ code for each generated IDL file: [https://github.com/osrf/ros_dds/tree/master/prototype/src/genidlcpp](https://github.com/osrf/ros_dds/tree/master/prototype/src/genidlcpp)

> 生成每个生成的 IDL 文件的 DDS 特定的 C++代码：[https://github.com/osrf/ros_dds/tree/master/prototype/src/genidlcpp](https://github.com/osrf/ros_dds/tree/master/prototype/src/genidlcpp)

- Minimal ROS Client Library for C++ (rclcpp): [https://github.com/osrf/ros_dds/tree/master/prototype/src/rclcpp](https://github.com/osrf/ros_dds/tree/master/prototype/src/rclcpp)

> 简化的 C++ ROS 客户端库（rclcpp）：[https://github.com/osrf/ros_dds/tree/master/prototype/src/rclcpp](https://github.com/osrf/ros_dds/tree/master/prototype/src/rclcpp)

- Talker and listener for pub-sub and service calls: [https://github.com/osrf/ros_dds/tree/master/prototype/src/rclcpp_examples](https://github.com/osrf/ros_dds/tree/master/prototype/src/rclcpp_examples)

> 示例代码：发布/订阅和服务调用的发言者和收听者：[https://github.com/osrf/ros_dds/tree/master/prototype/src/rclcpp_examples](https://github.com/osrf/ros_dds/tree/master/prototype/src/rclcpp_examples)

- A branch of `ros_tutorials` in which `turtlesim` has been modified to build against the `rclcpp` library: [https://github.com/ros/ros_tutorials/tree/ros_dds/turtlesim](https://github.com/ros/ros_tutorials/tree/ros_dds/turtlesim).

> 一个修改过的`ros_tutorials`分支，其中`turtlesim`已经被修改为基于`rclcpp`库构建：[https://github.com/ros/ros_tutorials/tree/ros_dds/turtlesim](https://github.com/ros/ros_tutorials/tree/ros_dds/turtlesim)。

This branch of `turtlesim` is not feature-complete (e.g., services and parameters are not supported), but the basics work, and it demonstrates that the changes required to transition from ROS 1 `roscpp` to the prototype of ROS 2 `rclcpp` are not dramatic.

> 这个`turtlesim`分支还不是完整功能的（例如，不支持服务和参数），但基本功能可以正常使用，这表明从 ROS 1 `roscpp`迁移到 ROS 2 `rclcpp`的原型所需的改变并不大。

This is a rapid prototype which was used to answer questions, so it is not representative of the final product or polished at all.

> 这是一个快速原型，用来回答问题，所以它不代表最终产品，也没有经过抛光处理。

Work on certain features was stopped cold once key questions had been answered.

> 一旦关键的问题得到解答，对某些特征的工作就被立刻停止了。

The examples in the `rclcpp_example` package showed that it was possible to implement the basic ROS like API on top of DDS and get familiar behavior.

> 例子在`rclcpp_example`包中表明，在 DDS 之上实现基本的 ROS 类似的 API，并获得熟悉的行为是可能的。

This is by no means a complete implementation and doesn't cover all of the features, but instead it was for educational purposes and addressed most of the doubts which were held with respect to using DDS.

> 这绝不是一个完整的实现，也不涵盖所有功能，而是出于教育目的，解决了大多数关于使用 DDS 的疑虑。

Generation of IDL files proved to have some sticking points, but could ultimately be addressed, and implementing basic things like services proved to be tractable problems.

> 生成 IDL 文件被证明有一些棘手的问题，但最终可以解决，实现像服务这样的基本东西也被证明是可行的问题。

In addition to the above basic pieces, a pull request was drafted which managed to completely hide the DDS symbols from any publicly installed headers for `rclcpp` and `std_msgs`:

> 此外，经过起草的拉取请求，成功地将`rclcpp`和`std_msgs`中的 DDS 符号完全隐藏在任何公开安装的头文件中：

[https://github.com/osrf/ros_dds/pull/17](https://github.com/osrf/ros_dds/pull/17)

> [https://github.com/osrf/ros_dds/pull/17](https://github.com/osrf/ros_dds/pull/17) 拉取请求：将 ROS DDS 支持更新到 CycloneDDS 0.5.0

This pull request was ultimately not merged because it was a major refactoring of the structure of the code and other progress had been made in the meantime.

> 这个拉取请求最终没有被合并，因为它是对代码结构的重大重构，而此时又取得了其他进展。

However, it served its purpose in that it showed that the DDS implementation could be hidden, though there is room for discussion on how to actually achieve that goal.

> 然而，它起到了其目的，即证明 DDS 实施可以被隐藏，尽管关于如何实现这一目标仍有讨论的空间。

## Conclusion

After working with DDS and having a healthy amount of skepticism about the ethos, community, and licensing, it is hard to come up with any real technical criticisms.

> 在使用 DDS 并对其精神、社区和许可有健康的怀疑之后，很难想出任何真正的技术批评。

While it is true that the community surrounding DDS is very different from the ROS community or the ZeroMQ community, it appears that DDS is just solid technology on which ROS could safely depend.

> 毫无疑问，DDS 的社区与 ROS 社区或 ZeroMQ 社区大不相同，但 DDS 似乎是一种稳定可靠的技术，ROS 可以安全地依赖它。

There are still many questions about exactly how ROS would utilize DDS, but they all seem like engineering exercises at this point and not potential deal breakers for ROS.

> 还有许多关于 ROS 如何利用 DDS 的具体问题，但目前看来它们都只是工程练习，而不是 ROS 的潜在交易破坏者。
