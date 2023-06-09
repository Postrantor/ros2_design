---
tip: translate by openai@2023-05-30 09:12:14
layout: default
title: ROS on DDS
permalink: articles/ros_on_dds.html
abstract:
  This article makes the case for using DDS as the middleware for ROS, outlining the pros and cons of this approach, as well as considering the impact to the user experience and code API that using DDS would have. The results of the "ros_dds" prototype are also summarized and used in the exploration of the issue.
  本文为使用DDS作为ROS的中间件，概述了这种方法的优缺点，并考虑了对用户体验和使用DDS使用的代码API的影响。“ ROS_DDS”原型的结果也被总结并用于探索该问题。
author: '[William Woodall](https://github.com/wjwwood)'
date_written: 2014-06
last_modified: 2019-07
published: true
categories: Middleware

<div class="alert alert-warning" markdown="1">
  This article was written at a time before decisions were made to use DDS and RTPS as the underlying communication standards for ROS 2. For details on how ROS 2 has been implemented, see the [Core Documentation](http://docs.ros2.org/)
  本文是在决定使用DDS和RTP作为ROS2的基础通信标准之前写的。有关ROS 2的实施方式，请参见[核心文档]
</div>

Authors: {{ page.author }}
Date Written: {{ page.date_written }}
Last Modified: {% if page.last_modified %}{{ page.last_modified }}{% else %}{{ page.date_written }}{% endif %}
---
Terminology:

- [Data Distribution Service (DDS)](http://en.wikipedia.org/wiki/Data_Distribution_Service)
- [Real-Time Publish Subscribe (RTPS)](https://en.wikipedia.org/wiki/Real-Time_Publish-Subscribe_(RTPS)_Protocol)
- The [Object Management Group (OMG)](http://www.omg.org/)
- OMG [Interface Description Language (IDL)](http://www.omg.org/gettingstarted/omg_idl.htm) | [Formal description](http://www.omg.org/cgi-bin/doc?formal/2014-03-01)

## Why Consider DDS

When exploring options for the next generation communication system of ROS, the initial options were to either improve the ROS 1 transport or build a new middleware using component libraries such as [ZeroMQ](http://zeromq.org/), Protocol Buffers, and zeroconf (Bonjour/Avahi). However, in addition to those options, both of which involved us building a middleware from parts or scratch, other end-to-end middlewares were considered. During our research, one middleware that stood out was DDS.

> 当探索 ROS 下一代通信系统的选项时，最初的选择是改进 ROS 1 传输或使用诸如 [ZeroMQ](http://zeromq.org/)、**Protocol Buffers** 和 zeroconf(Bonjour/Avahi)之类的组件库构建新的中间件。但是，除了这些选项，这两者都涉及我们从零开始构建中间件外，还考虑了其他端到端中间件。在我们的研究中，有一个中间件引起了我们的注意，那就是 DDS。

### An End-to-End Middleware

The benefit of using an end-to-end middleware, like DDS, is that there is much less code to maintain and the behavior and exact specifications of the middleware have already been distilled into documentation. In addition to system-level documentation, DDS also has recommended use cases and a software API. With this concrete specification, third parties can review, audit, and implement the middleware with varying degrees of interoperability. This is something that ROS has never had, besides a few basic descriptions in a wiki and a reference implementation. Additionally, this type of specification would need to be created anyway if a new middleware were to be built from existing libraries.

> 使用端到端中间件(如 DDS)的好处是，需要维护的代码要少得多，中间件的行为和精确规范已经被提炼到文档中。除了系统级文档之外，DDS 还提供了建议的用例和软件 API。有了这个具体的规范，第三方可以审阅、审计和实施中间件，具有不同程度的互操作性。这是 ROS 从未拥有过的，除了维基百科上的几个基本描述和参考实现之外。此外，如果要从现有库构建新的中间件，也需要创建这种类型的规范。

The drawback of using an end-to-end middleware is that ROS must work within that existing design. If the design did not target a relevant use case or is not flexible, it might be necessary to work around the design. On some level, adopting an end-to-end middleware includes adopting the philosophy and culture of that middleware, which should not be taken lightly.

> **使用端到端中间件的缺点是 ROS 必须符合该现有设计。** 如果设计不针对相关的用例或不灵活，可能需要绕过设计。从某种程度上讲，采用端到端中间件就意味着采用该中间件的理念和文化，这是不容忽视的。

## What is DDS

DDS provides a publish-subscribe transport which is very similar to ROS's publish-subscribe transport. DDS uses the "Interface Description Language (IDL)" as defined by the [Object Management Group (OMG)](http://www.omg.org/) for message definition and serialization. DDS has a request-response style transport, which would be like ROS's service system, in beta 2 as of June 2016 (called [DDS-RPC](http://www.omg.org/spec/DDS-RPC/)).

> DDS 提供了一种发布-订阅传输，它与 ROS 的发布-订阅传输非常相似。DDS 使用由 [Object Management Group(OMG)](http://www.omg.org/)定义的“接口描述语言(IDL)”来定义消息和序列化。**DDS 有一种请求-响应式传输，就像 ROS 的服务系统**，在 2016 年 6 月的 beta 2 版本(称为 [DDS-RPC](http://www.omg.org/spec/DDS-RPC/))。

The default discovery system provided by DDS, which is required to use DDS's publish-subscribe transport, is a distributed discovery system. This allows any two DDS programs to communicate without the need for a tool like the ROS master. This makes the system more fault tolerant and flexible. It is not required to use the dynamic discovery mechanism, however, as multiple DDS vendors provide options for static discovery.

> DDS 提供的默认发现系统，它是使用 DDS 发布/订阅传输所必需的，是一个分布式发现系统。这使得任何两个 DDS 程序都可以在不需要像 ROS 主节点这样的工具的情况下进行通信。这使得系统更具容错性和灵活性。不需要使用动态发现机制，但是多个 DDS 供应商提供了静态发现的选项。

> [!NOTE]
> ref: `C:\Users\trantor\Downloads\Documents\ChatGPT\dialogue\dds\end_to_end_20230530.md`

### Where did DDS come from

DDS got its start as a group of companies which had similar middleware frameworks and became a standard when common customers wanted to get better interoperability between the vendors. The DDS standard was created by the Object Management Group, which are the same people that brought us UML, CORBA, SysML, and other generic software related standards. Now, depending on your perspective, this may be a positive endorsement or a negative endorsement. On the one hand you have a standards committee which is perennial and clearly has a huge influence on the software engineering community, but on the other hand you have a slow moving body which is slow to adapt to changes and therefore arguably doesn't always keep up with the latest trends in software engineering.

> DDS 始于一组具有相似中间件框架的公司，当客户想要获得更好的供应商间互操作性时，它就成为了一种标准。DDS 标准是由对象管理组创建的，他们也是为我们带来 UML、CORBA、SysML 和其他通用软件相关标准的人。现在，根据您的观点，这可能是一个积极的认可或消极的认可。一方面，您有一个永恒的标准委员会，显然对软件工程社区有巨大的影响力，但另一方面，您有**一个缓慢移动的机构，缓慢适应变化，因此可以说不能总是跟上软件工程的最新趋势**。

DDS was originally several similar middlewares which eventually became so close to one another that writing a standard to unify them made sense. So in this way, even though the DDS specification has been written by a committee, it has evolved to its current form by reacting to the needs of its users. This type of organic evolution of the specification before it was ratified helps to alleviate the concern that the system was designed in a vacuum and that it does not perform well in real environments. There are some examples of committees coming up with well intentioned and well described specifications that nobody wants to use or that don't meet the needs of the community they serve, but this does not appear to be the case for DDS.

> DDS 最初是几个类似的中间件，最终变得如此接近，以至于编写一个标准来统一它们是有意义的。因此，尽管 DDS 规范是由一个委员会编写的，但它已经通过响应用户的需求而演变到现在的形式。在规范被批准之前，这种有机演变有助于减轻人们的担忧，即该系统是在真空环境中设计的，并且在真实环境中表现不佳。有一些关于委员会提出出色的规范，但没有人想使用或不能满足它们服务的社区需求的例子，但这并不适用于 DDS。

There is also a concern that DDS is a static specification which was defined and is used in "legacy" systems, but has not kept current. This kind of stereotype comes from horror stories about things like UML and CORBA, which are also products of OMG. On the contrary, DDS seems to have an active and organic specification, which in the recent past has added, or is adding, more specifications for things like websockets, security over SSL, extensible types, request and response transport, and a new, more modern C++11 style API specification for the core API to replace the existing C++ interface. This type of evolution in the standard body for DDS is an encouraging thing to observe, and even though the body is relatively slow, as compared to software engineering technology trends, it is evolving to meet demands of its users.

> 有人担心 DDS 是一种静态规范，它定义并用于“传统”系统，但没有保持最新。这种刻板印象来自有关 UML 和 CORBA 等产品的恐怖故事，这些产品也是 OMG 的产品。相反，DDS 似乎有一个活跃和有机的规范，最近新增了或正在增加更多的规范，如 websockets，SSL 安全，可扩展类型，请求和响应传输，以及一个新的，更现代的 C ++ 11 风格的 API 规范，用于替换现有的 C ++ 接口。这种 DDS 标准机构的演变是令人鼓舞的，尽管与软件工程技术趋势相比，该机构的发展速度相对较慢，但它正在不断发展，以满足用户的需求。

### Technical Credibility

DDS has an extensive list of varied installations which are typically mission critical. DDS has been used in:

> DDS 拥有多样化的安装列表，通常是关键任务。DDS 已被用于：

- battleships
- large utility installations like dams
- financial systems
- space systems
- flight systems
- train switchboard systems

and many other equally important and varied scenarios. These successful use cases lend credibility to DDS's design being both reliable and flexible.

> 还有许多其他同样重要且多样化的场景。这些成功的用例证明了 DDS 的设计既可靠又灵活。

Not only has DDS met the needs of these use cases, but after talking with users of DDS (in this case government and NASA employees who are also users of ROS), they have all praised its reliability and flexibility. Those same users will note that the flexibility of DDS comes at the cost of complexity. The complexity of the API and configuration of DDS is something that ROS would need to address.

> 不仅 DDS 满足了这些用例的需求，而且在与 DDS 用户(在这种情况下是政府和 NASA 雇员，也是 ROS 用户)交谈后，他们都赞扬了它的可靠性和灵活性。同样的用户会注意到，**DDS 的灵活性是以复杂性为代价的**。 DDS 的 API 和配置的复杂性是 ROS 需要解决的问题。

The DDS wire specification ([DDSI-RTPS](http://www.omg.org/spec/DDSI-RTPS/)) is extremely flexible, allowing it to be used for reliable, high level systems integration as well as real-time applications on embedded devices. Several of the DDS vendors have special implementations of DDS for embedded systems which boast specs related to library size and memory footprint on the scale of tens or hundreds of kilobytes. Since DDS is implemented, by default, on UDP, it does not depend on a reliable transport or hardware for communication. This means that DDS has to reinvent the reliability wheel (basically TCP plus or minus some features), but in exchange DDS gains portability and control over the behavior. Control over several parameters of reliability, what DDS calls Quality of Service (QoS), gives maximum flexibility in controlling the behavior of communication. For example, if you are concerned about latency, like for soft real-time, you can basically tune DDS to be just a UDP blaster. In another scenario you might need something that behaves like TCP, but needs to be more tolerant to long dropouts, and with DDS all of these things can be controlled by changing the QoS parameters.

> DDS 线路规范([DDSI-RTPS](http://www.omg.org/spec/DDSI-RTPS/))非常灵活，可用于可靠的高级系统集成以及嵌入式设备上的实时应用程序。 DDS 的几个供应商都有专门为嵌入式系统设计的 DDS 实现，其规格与库大小和内存占用量相关，达到数十或数百 KB 的规模。由于 **DDS 默认采用 UDP 实现**，因此不依赖于可靠的传输或硬件进行通信。**这意味着 DDS 必须重新发明可靠性的轮子(基本上是加上或减去一些功能的 TCP)，但作为交换，DDS 获得了可移植性和对行为的控制**。对可靠性的几个参数的控制，即 DDS 所称的服务质量(QoS)，可以最大限度地控制通信行为。例如，如果您关注延迟，如软实时，则可以通过更改 QoS 参数将 DDS 调整为仅 UDP 发射器。在另一种情况下，您可能需要行为类似于 TCP 的东西，但需要对长时间的断开更加宽容，而使用 DDS，所有这些都可以通过更改 QoS 参数来控制。

Though the default implementation of DDS is over UDP, and only requires that level of functionality from the transport, OMG also added support for DDS over TCP in version 1.2 of their specification. Only looking briefly, two of the vendors (RTI and ADLINK Technologies) both support DDS over TCP.

> 虽然 DDS 的默认实现是基于 UDP 的，而且**只需要传输层提供这个功能，OMG 在 1.2 版本中也增加了基于 TCP 的 DDS 支持**。简单看一下，有两家供应商(RTI 和 ADLINK Technologies)都支持基于 TCP 的 DDS。

From RTI's website ([http://community.rti.com/kb/xml-qos-example-using-rti-connext-dds-tcp-transport](http://community.rti.com/kb/xml-qos-example-using-rti-connext-dds-tcp-transport)):

> 从 RTI 的网站([http://community.rti.com/kb/xml-qos-example-using-rti-connext-dds-tcp-transport](http://community.rti.com/kb/xml-qos-example-using-rti-connext-dds-tcp-transport))：

> By default, RTI Connext DDS uses the UDPv4 and Shared Memory transport to communicate with other DDS applications. In some circumstances, the TCP protocol might be needed for discovery and data exchange. For more information on the RTI TCP Transport, please refer to the section in the RTI Core Libraries and Utilities User Manual titled "RTI TCP Transport".

> 默认情况下，RTI 连接 DDS 使用 UDPV4 和共享内存传输与其他 DDS 应用程序进行通信。在某些情况下，发现和数据交换可能需要进行 TCP 协议。有关 RTI TCP 传输的更多信息，请参阅 RTI 核心库中的部分和公用事业用户手册，名为“ RTI TCP Transport”。

From ADLINK's website, they support TCP as of OpenSplice v6.4:

> 从 ADLINK 的网站上看，从 OpenSplice v6.4 开始，他们支持 TCP。

[https://www.adlinktech.com/en/data-distribution-service.aspx](https://www.adlinktech.com/en/data-distribution-service.aspx)

### Vendors and Licensing

The OMG defined the DDS specification with several companies which are now the main DDS vendors. Popular DDS vendors include:

> OMG 定义了 DDS 规范，其中几家公司现在是主要的 DDS 供应商。流行的 DDS 供应商包括：

- RTI
- ADLINK Technologies
- Twin Oaks Software

Amongst these vendors is an array of reference implementations with different strategies and licenses. The OMG maintains an active [list](http://dds-directory.omg.org/vendor/list.htm) of DDS vendors.

> 在这些供应商中，有各种不同策略和许可证的参考实现。 OMG 维护着一份活跃的 DDS 供应商[列表](http://dds-directory.omg.org/vendor/list.htm)。

In addition to vendors providing implementations of the DDS specification's API, there are software vendors which provide an implementation with more direct access to the DDS wire protocol, RTPS. For example:

> 此外，除了供应商提供 DDS 规范 API 的实现外，还有软件供应商提供更直接访问 DDS 线路协议 RTPS 的实现。例如：

- eProsima

These RTPS-centric implementations are also of interest because they can be smaller in scope and still provide the needed functionality for implementing the necessary ROS capabilities on top.

> 这些以 RTPS 为中心的实现也很有趣，因为它们的范围可以更小，但仍然能够为实现所需的 ROS 功能提供所需的功能。

> [RTI]:

ADLINK's DDS implementation, OpenSplice, is licensed under the LGPL, which is the same license used by many popular open source libraries, like glibc, ZeroMQ, and Qt. It is available on [Github](https://github.com):

> ADLINK 的 DDS 实现 OpenSplice 采用 LGPL 许可，这是许多流行开源库(如 glibc、ZeroMQ 和 Qt)使用的相同许可。它可以在 [Github](https://github.com) 上找到。

[https://github.com/ADLINK-IST/opensplice](https://github.com/ADLINK-IST/opensplice)

ADLINK's implementation comes with a basic, functioning build system and was fairly easy to package. OpenSplice appears to be the number two DDS implementation in use, but that is hard to tell for sure.

> ADLINK 的实施提供了一个基本的、可运行的构建系统，打包也相当容易。OpenSplice 似乎是目前使用最多的第二种 DDS 实现，但准确的说很难说。

TwinOaks's CoreDX DDS implementation is proprietary only, but apparently they specialize in minimal implementations which are able to run on embedded devices and even bare metal.

> TwinOaks 的 CoreDX DDS 实现是专有的，但显然它们专注于可以在嵌入式设备甚至裸机上运行的最小实现。

eProsima's FastRTPS implementation is available on GitHub and is LGPL licensed:

> eProsima 的 FastRTPS 实现可在 GitHub 上获得，拥有 LGPL 许可：

[https://github.com/eProsima/Fast-RTPS](https://github.com/eProsima/Fast-RTPS)

eProsima Fast RTPS is a relatively new, lightweight, and open source implementation of RTPS. It allows direct access to the RTPS protocol settings and features, which is not always possible with other DDS implementations. eProsima's implementation also includes a minimum DDS API, IDL support, and automatic code generation and they are open to working with the ROS community to meet their needs.

> eProsima Fast RTPS 是一种相对较新的、轻量级的、开源的 RTPS 实现。它允许直接访问 RTPS 协议设置和功能，而其他 DDS 实现并不总是可能的。eProsima 的实现还包括最小的 DDS API、IDL 支持以及自动代码生成，他们也愿意与 ROS 社区合作，以满足他们的需求。

Given the relatively strong LGPL option and the encouraging but custom license from RTI, it seems that depending on and even distributing DDS as a dependency should be straightforward. One of the goals of this proposal would be to make ROS 2 DDS vendor agnostic. So, just as an example, if the default implementation is Connext, but someone wants to use one of the LGPL options like OpenSplice or FastRTPS, they simply need to recompile the ROS source code with some options flipped and they can use the implementation of their choice.

> 根据相对强大的 LGPL 选项和 RTI 的鼓励但定制的许可证，似乎依赖和甚至分发 DDS 应该是直接的。这项提案的目标之一是使 ROS 2 DDS 供应商无关。因此，例如，如果默认实现是 Connext，但有人想使用 LGPL 选项之一，如 OpenSplice 或 FastRTPS，他们只需要使用一些选项重新编译 ROS 源代码，就可以使用他们选择的实现。

This is made possible because of the fact that DDS defines an API in its specification. Research has shown that making code which is vendor agnostic is possible if not a little painful since the APIs of the different vendors is almost identical, but there are minor differences like return types (pointer versus shared_ptr like thing) and header file organization.

> 这是因为 DDS 在其规范中定义了一个 API，所以可能实现。研究表明，如果不是有点痛苦，就可以制作与供应商无关的代码，因为不同供应商的 API 几乎相同，但有一些小差异，比如返回类型(指针与 shared_ptr 类似的东西)和头文件组织。

### Ethos and Community

DDS comes out of a set of companies which are decades old, was laid out by the OMG which is an old-school software engineering organization, and is used largely by government and military users. So it comes as no surprise that the community for DDS looks very different from the ROS community and that of similar modern software projects like ZeroMQ. Though RTI has a respectable on-line presence, the questions asked by community members are almost always answered by an employee of RTI and though technically open source, neither RTI nor OpenSplice has spent time to provide packages for Ubuntu or Homebrew or any other modern package manager. They do not have extensive user-contributed wikis or an active Github repository.

> DDS 来自一组已有几十年历史的公司，由 OMG 提出，这是一个老式的软件工程组织，主要被政府和军事用户使用。因此，DDS 社区看起来与 ROS 社区和像 ZeroMQ 这样的现代软件项目非常不同，这也就不足为奇了。尽管 RTI 在线存在受到尊重，但社区成员提出的问题几乎总是由 RTI 的员工回答，尽管在技术上是开源的，但 RTI 和 OpenSplice 都没有花时间为 Ubuntu 或 Homebrew 或任何其他现代软件包管理器提供包。他们没有大量用户贡献的维基百科或活跃的 Github 存储库。

This staunch difference in ethos between the communities is one of the most concerning issues with depending on DDS. Unlike options like keeping TCPROS or using ZeroMQ, there isn't the feeling that there is a large community to fall back on with DDS. However, the DDS vendors have been very responsive to our inquiries during our research and it is hard to say if that will continue when it is the ROS community which brings the questions.

> 这种社区之间的坚定差异是依赖 DDS 最令人担忧的问题之一。与保持 TCPROS 或使用 ZeroMQ 等选项不同，我们没有感觉可以依靠 DDS 的大社区。然而，在我们的研究期间，DDS 供应商对我们的询问非常反应积极，很难说当 ROS 社区提出问题时会不会继续如此。

Even though this is something which should be taken into consideration when making a decision about using DDS, it should not disproportionately outweigh the technical pros and cons of the DDS proposal.

> 即使这是在做出关于使用 DDS 的决定时应该考虑的事情，它也不应该过分地超出 DDS 提案的技术利弊。

## ROS Built on DDS

The goal is to make DDS an implementation detail of ROS 2. This means that all DDS specific APIs and message definitions would need to be hidden. DDS provides discovery, message definition, message serialization, and publish-subscribe transport. Therefore, DDS would provide discovery, publish-subscribe transport, and at least the underlying message serialization for ROS. ROS 2 would provide a ROS 1 like interface on top of DDS which hides much of the complexity of DDS for the majority of ROS users, but then separately provides access to the underlying DDS implementation for users that have extreme use cases or need to integrate with other, existing DDS systems.

> 目标是使 DDS 成为 ROS 2 的实现细节。这意味着**所有 DDS 特定的 API 和消息定义都需要被隐藏。DDS 提供发现、消息定义、消息序列化和发布订阅传输**。因此，DDS 将为 ROS 提供发现、发布订阅传输，以及至少底层的消息序列化。ROS 2 将在 DDS 之上提供一个类似于 ROS 1 的接口，为大多数 ROS 用户隐藏 DDS 的复杂性，但**另外为那些拥有极端用例或需要与其他现有 DDS 系统集成的用户提供对底层 DDS 实现的访问**。

![DDS and ROS API Layout](/img/ros_on_dds/api_levels.png)

_DDS and ROS API Layout_

Accessing the DDS implementation would require depending on an additional package which is not normally used. In this way you can tell if a package has tied itself to a particular DDS vendor by just looking at the package dependencies. The goal of the ROS API, which is on top of DDS, should be to meet all the common needs for the ROS community, because once a user taps into the underlying DDS system, they will lose portability between DDS vendors. Portability among DDS vendors is not intended to encourage people to frequently choose different vendors, but rather to enable power users to select the DDS implementation that meets their specific requirements, as well as to future-proof ROS against changes in the DDS vendor options. There will be one recommended and best-supported default DDS implementation for ROS.

> 访问 DDS 实现需要依赖额外的包，这些包通常不会使用。通过查看包依赖关系，可以确定一个包是否与某个 DDS 供应商绑定。ROS API(位于 DDS 之上)的目标应该是满足 ROS 社区的所有常见需求，因为一旦用户访问底层的 DDS 系统，他们将失去 DDS 供应商之间的可移植性。DDS 供应商之间的可移植性旨在鼓励人们不断选择不同的供应商，而是为了使 ROS 用户能够选择满足其特定要求的 DDS 实现，以及使 ROS 免受 DDS 供应商选项变化的影响。ROS 将有一个推荐的、最佳支持的默认 DDS 实现。

### Discovery

DDS would completely replace the ROS master based discovery system. ROS would need to tap into the DDS API to get information like a list of all nodes, a list of all topics, and how they are connected. Accessing this information would be hidden behind a ROS defined API, preventing the users from having to call into DDS directly.

> **DDS 将完全取代基于 ROS 主控系统的发现系统。** ROS 需要访问 DDS API 以获取信息，如所有节点的列表、所有主题的列表以及它们之间的连接方式。访问这些信息将被 ROS 定义的 API 所隐藏，从而防止用户直接调用 DDS。

The advantage of the DDS discovery system is that, by default, it is completely distributed, so there is no central point of failure which is required for parts of the system to communicate with each other. DDS also allows for user defined **meta data** in their discovery system, which will enable ROS to piggyback higher level concepts onto publish-subscribe.

> **优势之一是 DDS 发现系统默认是完全分布式的，因此不需要中央点来使系统的各个部分相互通信。** DDS 还允许用户在发现系统中定义**元数据**，这将使 ROS 能够在发布-订阅上搭载更高级概念。

> [!NOTE]
> ref: [meta data](C:%5CUsers%5Ctrantor%5CDownloads%5CDocuments%5CChatGPT%5Cdialogue%5Cdds%5Cmeta_data_20230530.md)

### Publish-Subscribe Transport

The DDSI-RTPS (DDS-Interoperability Real Time Publish Subscribe) protocol would replace ROS's TCPROS and UDPROS wire protocols for publish/subscribe. The DDS API provides a few more actors to the typical publish-subscribe pattern of ROS 1. In ROS the concept of a node is most clearly paralleled to a graph participant in DDS. A graph participant can have zero to many topics, which are very similar to the concept of topics in ROS, but are represented as separate code objects in DDS, and is neither a subscriber nor a publisher. Then, from a DDS topic, DDS subscribers and publishers can be created, but again these are used to represent the subscriber and publisher concepts in DDS, and not to directly read data from or write data to the topic. DDS has, in addition to the topics, subscribers, and publishers, the concept of DataReaders and DataWriters which are created with a subscriber or publisher and then specialized to a particular message type before being used to read and write data for a topic. These additional layers of abstraction allow DDS to have a high level of configuration, because you can set QoS settings at each level of the publish-subscribe stack, providing the highest granularity of configuration possible. Most of these levels of abstractions are not necessary to meet the current needs of ROS. Therefore, packaging common workflows under the simpler ROS-like interface (Node, Publisher, and Subscriber) will be one way ROS 2 can hide the complexity of DDS, while exposing some of its features.

> DDSI-RTPS(DDS 互操作性实时发布订阅)协议将取代 ROS 的 TCPROS 和 UDPROS 线路协议进行发布/订阅。DDS API 为 ROS 1 中典型的发布-订阅模式增加了一些演员。**在 ROS 中，节点的概念与 DDS 中的图表参与者(graph participant)最为接近。** 图表参与者(graph participant)可以拥有零到多个主题，这些**主题与 ROS 中的主题概念非常相似**，但在 DDS 中表示为单独的代码对象，既不是订阅者也不是发布者。然后，**从 DDS 主题中可以创建 DDS 订阅者和发布者，但这些也仅用于表示 DDS 中的订阅者和发布者概念，而不是直接从主题中读取或写入数据**。除了主题、订阅者和发布者之外，DDS 还拥有数据读取器和数据写入器的概念，这些数据读取器和数据写入器是使用订阅者或发布者创建的，然后针对特定的消息类型进行专门化，然后才用于读取和写入主题的数据。这些附加的抽象层允许 DDS 具有很高的配置级别，因为您可以在发布-订阅堆栈的每个级别设置 QoS 设置，从而提供最高粒度的配置可能性。ROS 当前的需求大多不需要这些抽象层。因此，在更简单的 ROS 样式界面(节点、发布者和订阅者)下封装常见的工作流程将是 ROS 2 隐藏 DDS 复杂性的一种方式，同时也能够暴露其一些功能。

> [!NOTE]
> ref: [Topic](C:%5CUsers%5Ctrantor%5CDownloads%5CDocuments%5CChatGPT%5Cdialogue%5Cdds%5Ctopic_20230530.md)
> ref: [participant](C:%5CUsers%5Ctrantor%5CDownloads%5CDocuments%5CChatGPT%5Cdialogue%5Cdds%5Cparticipant_20230530.md)
> ref: [dds_ros](C:%5CUsers%5Ctrantor%5CDownloads%5CDocuments%5CChatGPT%5Cdialogue%5Cdds%5Cconcept_ros_dds_20230530.md)
> ref: [meta_data](C:%5CUsers%5Ctrantor%5CDownloads%5CDocuments%5CChatGPT%5Cdialogue%5Cdds%5Cmeta_data_20230530.md)

### Efficient Transport Alternatives

In ROS 1 there was never a standard shared-memory transport because it is negligibly faster than localhost TCP loop-back connections. It is possible to get non-trivial performance improvements from carefully doing zero-copy style shared-memory between processes, but anytime a task required faster than localhost TCP in ROS 1, nodelets were used. Nodelets allow publishers and subscribers to share data by passing around `boost::shared_ptr` s to messages. This intraprocess communication is almost certainly faster than any interprocess communication options and is orthogonal to the discussion of the network publish-subscribe implementation.

> 在 ROS 1 中，从未有标准的共享内存传输，因为它比本地主机 TCP 回路连接的速度要快得太少。通过仔细地进行零复制式共享内存，可以获得非平凡的性能改进，但是，如果 ROS 1 中的任务需要比本地主机 TCP 更快的速度，就会使用节点。**节点允许发布者和订阅者通过传递 `boost::shared_ptr` 来共享数据。** 此进程间通信几乎肯定比任何进程间通信选项都要快，并且与网络发布订阅实现无关。

> [!NOTE]
> 通过 shared_ptr 实现进程间共享内存通信

In the context of DDS, most vendors will optimize message traffic (even between processes) using shared-memory in a transparent way, only using the wire protocol and UDP sockets when leaving the localhost. This provides a considerable performance increase for DDS, whereas it did not for ROS 1, because the localhost networking optimization happens at the call to `send`. For ROS 1 the process was: serialize the message into one large buffer, call TCP's `send` on the buffer once. For DDS the process would be more like: serialize the message, break the message into potentially many UDP packets, call UDP's `send` many times. In this way sending many UDP datagrams does not benefit from the same speed up as one large TCP `send`. Therefore, many DDS vendors will short circuit this process for localhost messages and use a blackboard style shared-memory mechanism to communicate efficiently between processes.

> 在 DDS 的背景下，大多数供应商会**透明地使用共享内存来优化消息传输(甚至是进程之间)，只有在离开本地主机时才使用线路协议和 UDP 套接字**。这为 DDS 提供了相当大的性能提升，而对 ROS 1 却没有帮助，因为本地主机网络优化发生在调用 `send` 时。对于 ROS 1，过程是：将消息序列化为一个大缓冲区，一次调用 TCP 的 `send`。对于 **DDS，过程更像是：序列化消息，将消息分解为多个 UDP 数据包，多次调用 UDP 的 `send`。因此，发送许多 UDP 数据报不会从同一个大的 TCP `send` 中获得同样的加速。因此，许多 DDS 供应商会为本地主机消息简化此过程，并使用黑板式共享内存机制在进程之间进行高效通信。**

However, not all DDS vendors are the same in this respect, so ROS would not rely on this "intelligent" behavior for efficient **intra**process communication. Additionally, if the ROS message format is kept, which is discussed in the next section, it would not be possible to prevent a conversion to the DDS message type for intraprocess topics. Therefore a custom intraprocess communication system would need to be developed for ROS which would never serialize nor convert messages, but instead would pass pointers (to shared in-process memory) between publishers and subscribers using DDS topics. This same intraprocess communication mechanism would be needed for a custom middleware built on ZeroMQ, for example.

> 然而，在这方面，**并非所有 DDS 供应商都是相同的，因此 ROS 不会依赖于这种“智能”行为来实现高效的进程间通信**。此外，如果保留 ROS 消息格式(在下一节中将进行讨论)，则**无法防止将进程间主题转换为 DDS 消息类型。因此，ROS 需要开发一种自定义的进程间通信系统**，该系统既不会序列化也不会转换消息，而是使用 DDS 主题在发布者和订阅者之间传递指向共享进程内存的指针。对于基于 ZeroMQ 的自定义中间件也需要使用相同的进程间通信机制。

The point to take away here is that efficient **intra**process communication will be addressed regardless of the network/interprocess implementation of the middleware.

> 这里要提取的要点是，无论中间件的网络/进程间实现如何，都将解决有效的进程内通信。

### Messages

There is a great deal of value in the current ROS message definitions. The format is simple, and the messages themselves have evolved over years of use by the robotics community. Much of the semantic contents of current ROS code is driven by the structure and contents of these messages, so preserving the format and in-memory representation of the messages has a great deal of value. In order to meet this goal, and in order to make DDS an implementation detail, ROS 2 should preserve the ROS 1 like message definitions and in-memory representation.

> 在当前的 ROS 消息定义中有很大的价值。格式很简单，消息本身也经过机器人社区多年的使用而演变。当前 ROS 代码的大部分语义内容是由这些消息的结构和内容驱动的，因此**保留消息的格式和内存表示有很大的价值**。为了实现这一目标，并且为了使 DDS 成为实现细节，ROS 2 应该保留 ROS 1 类似的消息定义和内存表示。

Therefore, the ROS 1 `.msg` files would continue to be used and the `.msg` files would be converted into `.idl` files so that they could be used with the DDS transport. Language specific files would be generated for both the `.msg` files and the `.idl` files as well as conversion functions for converting between ROS and DDS in-memory instances. The ROS 2 API would work exclusively with the `.msg` style message objects in memory and would convert them to `.idl` objects before publishing.

> 因此，ROS 1 的 `.msg` 文件将继续使用，**`.msg` 文件将被转换成 `.idl` 文件**，以便它们可以与 DDS 传输一起使用。将为 `.msg` 文件和 `.idl` 文件生成特定于语言的文件，以及用于在 ROS 和 DDS 内存实例之间转换的转换函数。 ROS 2 API 将专门使用 `.msg` 样式的消息对象在内存中工作，并**在发布前将它们转换为 `.idl` 对象**。

![Message Generation Diagram](/img/ros_on_dds/message_generation.png "Message Generation Diagram")

At first, the idea of converting a message field-by-field into another object type for each call to publish seems like a huge performance problem, but experimentation has shown that the cost of this copy is insignificant when compared to the cost of serialization. This ratio between the cost of converting types and the cost of serialization, which was found to be at least one order of magnitude, holds true with every serialization library that we tried, except [Cap'n Proto](http://kentonv.github.io/capnproto/) which doesn't have a serialization step. Therefore, if a field-by-field copy will not work for your use case, neither will serializing and transporting over the network, at which point you will have to utilize an intraprocess or zero-copy interprocess communication. The intraprocess communication in ROS would not use the DDS in-memory representation so this field-by-field copy would not be used unless the data is going to the wire. Because this conversion is only invoked in conjunction with a more expensive serialization step, the field-by-field copy seems to be a reasonable trade-off for the portability and abstraction provided by preserving the ROS `.msg` files and in-memory representation.

> 首先，把每个发布调用中的消息字段逐个转换成另一种对象类型的想法似乎是一个巨大的性能问题，但实验表明，**与序列化的成本相比，这种复制的成本是微不足道的**。我们尝试的每个序列化库都发现，**类型转换和序列化成本之间的比率至少是一个数量级**，除了 [Cap'n Proto](http://kentonv.github.io/capnproto/)，它没有序列化步骤。因此，如果字段逐个复制不适用于您的用例，那么也不会序列化并通过网络传输，此时您将不得不使用进程内或零复制进程间通信。**ROS 中的进程内通信不会使用 DDS 内存表示**，因此除非数据要发送到线路，否则不会使用字段逐个复制。由于此转换仅在更昂贵的序列化步骤中调用，因此字段逐个复制似乎是一个合理的折衷，以保留 ROS 的 `.msg` 文件和内存表示，从而提供可移植性和抽象性。

This does not preclude the option to improve the `.msg` file format with things like default values and optional fields. But this is a different trade-off which can be decided later.

> 这并不排除改进 `.msg` 文件格式的选项，比如默认值和可选字段等。但这是一个不同的折衷方案，可以稍后再做决定。

### Services and Actions

DDS currently does not have a ratified or implemented standard for request-response style RPC which could be used to implement the concept of services in ROS. There is currently an RPC specification being considered for ratification in the OMG DDS working group, and several of the DDS vendors have a draft implementation of the RPC API. It is not clear, however, whether this standard will work for actions, but it could at least support non-preemptable version of ROS services. ROS 2 could either implement services and actions on top of publish-subscribe (this is more feasible in DDS because of their reliable publish-subscribe QoS setting) or it could use the DDS RPC specification once it is finished for services and then build actions on top, again like it is in ROS 1. Either way actions will be a first class citizen in the ROS 2 API and it may be the case that services just become a degenerate case of actions.

> **DDS 目前没有通过批准或实施的用于实现 ROS 服务概念的请求-响应式 RPC 标准。** 目前 OMG DDS 工作组正在考虑批准一项 RPC 规范，DDS 供应商中有几家已经实施了 RPC API 的草案。但目前尚不清楚该标准是否适用于动作，但至少可以支持 ROS 1 中的非抢占式版本的 ROS 服务。ROS 2 可以在发布-订阅(由于它们可靠的发布-订阅 QoS 设置，在 DDS 中更容易实现)上实现服务和动作，也可以在服务上使用完成的 DDS RPC 规范，然后在上面构建动作，就像 ROS 1 一样。无论如何，动作将成为 ROS 2 API 中的一等公民，服务可能只是动作的一种特例。

### Language Support

DDS vendors typically provide at least C, C++, and Java implementations since APIs for those languages are explicitly defined by the DDS specification. There are not any well established versions of DDS for Python that research has uncovered. Therefore, one goal of the ROS 2 system will be to provide a first-class, feature complete C API. This will allow bindings for other languages to be made more easily and to enable more consistent behavior between client libraries, since they will use the same implementation. Languages like Python, Ruby, and Lisp can wrap the C API in a thin, language idiomatic implementation.

> DDS 供应商通常提供至少 C、C++ 和 Java 实现，因为这些语言的 API 由 DDS 规范明确定义。**研究发现没有任何 Python 的成熟版本的 DDS。因此，ROS 2 系统的一个目标将是提供一个一流的、功能完整的 C API。这将允许更容易地为其他语言创建绑定，并且可以在客户端库之间实现更一致的行为，因为它们将使用相同的实现。** 像 Python、Ruby 和 Lisp 这样的语言可以在薄的语言本地实现中包装 C API。

The actual implementation of ROS can either be in C, using the C DDS API, or in C++ using the DDS C++ API and then wrapping the C++ implementation in a C API for other languages. Implementing in C++ and wrapping in C is a common pattern, for example [ZeroMQ](http://zeromq.org/) does exactly this. The author of [ZeroMQ](http://zeromq.org/), however, did not do this in his new library, [nanomsg](http://nanomsg.org/), citing increased complexity and the bloat of the C++ stdlib as a dependency. Since the C implementation of DDS is typically pure C, it would be possible to have a pure C implementation for the ROS C API all the way down through the DDS implementation. However, writing the entire system in C might not be the first goal, and in the interest of getting a minimal viable product working, the implementation might be in C++ and wrapped in C to begin with and later the C++ can be replaced with C if it seems necessary.

> 实际的 ROS 实现可以使用 C 语言和 C DDS API，或者使用 DDS C++ API 并将 C++ 实现包装到 C API 中供其他语言使用。使用 C++ 进行包装并使用 C 是一种常见的模式，例如 [ZeroMQ](http://zeromq.org/) 就是这样做的。[ZeroMQ](http://zeromq.org/) 的作者在他的新库 [nanomsg](http://nanomsg.org/) 中没有这样做，他提到了增加的复杂性以及 C++ stdlib 的膨胀作为依赖。由于 DDS 的 C 实现通常是纯 C，因此可以有一个纯 C 实现，用于 ROS C API 以及 DDS 实现。但是，仅使用 C 编写整个系统可能不是第一个目标，为了获得最小可行产品，实现可能是用 C++ 实现并用 C 包装，然后稍后可以将 C++ 替换为 C，如果有必要的话。

### DDS as a Dependency

One of the goals of ROS 2 is to reuse as much code as possible ("do not reinvent the wheel") but also minimize the number of dependencies to improve portability and to keep the build dependency list lean. These two goals are sometimes at odds, since it is often the choice between implementing something internally or relying on an outside source (dependency) for the implementation.

> **一个 ROS 2 的目标是尽可能多地重用代码(“不要重复造轮子”)，同时最大限度地减少依赖关系，以提高可移植性，并保持构建依赖关系列表的精简。这两个目标有时会产生冲突，因为通常需要在在内部实现某些内容或依赖外部源(依赖项)来实现。**

This is a point where the DDS implementations shine, because two of the three DDS vendors under evaluation build on Linux, OS X, Windows, and other more exotic systems with no external dependencies. The C implementation relies only on the system libraries, the C++ implementations only rely on a C++03 compiler, and the Java implementation only needs a JVM and the Java standard library. Bundled as a binary (during prototyping) on both Ubuntu and OS X, the C, C++, Java, and C# implementations of OpenSplice (LGPL) is less than three megabytes in size and has no other dependencies. As far as dependencies go, this makes DDS very attractive because it significantly simplifies the build and run dependencies for ROS. Additionally, since the goal is to make DDS an implementation detail, it can probably be removed as a transitive run dependency, meaning that it will not even need to be installed on a deployed system.

> 这是 DDS 实现的一个亮点，因为在评估的三个 DDS 供应商中，有两个在 Linux、OS X、Windows 和其他更多异国系统上构建，没有外部依赖关系。C 语言实现只依赖于系统库，C++ 实现只依赖于 C++03 编译器，而 Java 实现只需要 JVM 和 Java 标准库。作为 Ubuntu 和 OS X 上的二进制文件(在原型期)，OpenSplice(LGPL)的 C、C++、Java 和 C＃实现大小不到 3MB，没有其他依赖关系。就依赖性而言，这使得 DDS 非常有吸引力，因为它大大简化了 ROS 的构建和运行依赖关系。此外，由于目标是使 DDS 成为实现细节，它可能被移除作为传递运行依赖关系，这意味着它甚至不需要安装在部署的系统上。

## The ROS on DDS Prototype

Following the research into the feasibility of ROS on DDS, several questions were left, including but not limited to:

> 随着对 ROS 在 DDS 上可行性的研究，留下了几个问题，包括但不限于：

- Can the ROS 1 API and behavior be implemented on top of DDS?
- Is it practical to generate IDL messages from ROS MSG messages and use them with DDS?
- How hard is it to package (as a dependency) DDS implementations?
- Does the DDS API specification actually make DDS vendor portability a reality?
- How difficult is it to configure DDS?

In order to answer some of these questions a prototype and several experiments were created in this repository:

> 为了回答其中一些问题，我们在这个仓库中创建了一个原型和几个实验：

[https://github.com/osrf/ros_dds](https://github.com/osrf/ros_dds)

More questions and some of the results were captured as issues:

> 更多的问题和一些结果被捕获为问题：

[https://github.com/osrf/ros_dds/issues?labels=task&page=1&state=closed](https://github.com/osrf/ros_dds/issues?labels=task&page=1&state=closed)

The major piece of work in this repository is in the `prototype` folder and is a ROS 1 like implementation of the Node, Publisher, and Subscriber API using DDS:

> 本存储库中的主要工作位于 `prototype` 文件夹中，是使用 DDS 实现的类似 ROS 1 的节点、发布者和订阅者 API：

[https://github.com/osrf/ros_dds/tree/master/prototype](https://github.com/osrf/ros_dds/tree/master/prototype)

Specifically this prototype includes these packages:

> 具体来说，这个原型包括这些包：

- Generation of DDS IDLs from `.msg` files: [https://github.com/osrf/ros_dds/tree/master/prototype/src/genidl](https://github.com/osrf/ros_dds/tree/master/prototype/src/genidl)
- Generation of DDS specific C++ code for each generated IDL file: [https://github.com/osrf/ros_dds/tree/master/prototype/src/genidlcpp](https://github.com/osrf/ros_dds/tree/master/prototype/src/genidlcpp)
- Minimal ROS Client Library for C++ (rclcpp): [https://github.com/osrf/ros_dds/tree/master/prototype/src/rclcpp](https://github.com/osrf/ros_dds/tree/master/prototype/src/rclcpp)
- Talker and listener for pub-sub and service calls: [https://github.com/osrf/ros_dds/tree/master/prototype/src/rclcpp_examples](https://github.com/osrf/ros_dds/tree/master/prototype/src/rclcpp_examples)
- A branch of `ros_tutorials` in which `turtlesim` has been modified to build against the `rclcpp` library: [https://github.com/ros/ros_tutorials/tree/ros_dds/turtlesim](https://github.com/ros/ros_tutorials/tree/ros_dds/turtlesim).

  This branch of `turtlesim` is not feature-complete (e.g., services and parameters are not supported), but the basics work, and it demonstrates that the changes required to transition from ROS 1 `roscpp` to the prototype of ROS 2 `rclcpp` are not dramatic.

> 这个 `turtlesim` 的分支还不是完整的(例如不支持服务和参数)，但基本功能可以正常工作，而且它表明，从 ROS 1 `roscpp` 转换到 ROS 2 `rclcpp` 的原型所需的变化并不太大。

This is a rapid prototype which was used to answer questions, so it is not representative of the final product or polished at all. Work on certain features was stopped cold once key questions had been answered.

> 这是一个快速原型，用于回答问题，所以它不代表最终产品，也没有经过任何润饰。一旦关键问题被回答，对某些功能的工作就被冷却了。

The examples in the `rclcpp_example` package showed that it was possible to implement the basic ROS like API on top of DDS and get familiar behavior. This is by no means a complete implementation and doesn't cover all of the features, but instead it was for educational purposes and addressed most of the doubts which were held with respect to using DDS.

> 例子在 `rclcpp_example` 包中表明，可以在 DDS 之上实现基本的 ROS 类似的 API，并获得熟悉的行为。这绝不是一个完整的实现，不涵盖所有功能，而是用于教育目的，解决了关于使用 DDS 的所有疑问。

Generation of IDL files proved to have some sticking points, but could ultimately be addressed, and implementing basic things like services proved to be tractable problems.

> 生成 IDL 文件虽然存在一些棘手的问题，但最终可以解决，而实现像服务这样的基本事物也被证明是可行的问题。

In addition to the above basic pieces, a pull request was drafted which managed to completely hide the DDS symbols from any publicly installed headers for `rclcpp` and `std_msgs`:

> 除了上述基本部件外，还起草了一个拉取请求，该请求设法完全隐藏了 `rclcpp` 和 `std_msgs` 的公开安装头文件中的 DDS 符号：

[https://github.com/osrf/ros_dds/pull/17](https://github.com/osrf/ros_dds/pull/17)

> [https://github.com/osrf/ros_dds/pull/17](https://github.com/osrf/ros_dds/pull/17) 拉取请求：添加对 DDS 的支持

This pull request was ultimately not merged because it was a major refactoring of the structure of the code and other progress had been made in the meantime. However, it served its purpose in that it showed that the DDS implementation could be hidden, though there is room for discussion on how to actually achieve that goal.

> 这次拉取请求最终没有被合并，因为它是对代码结构的重大重构，而此时其他进展已经取得。但是，它起到了其目的，即表明 DDS 实现可以被隐藏，尽管如何实现这一目标仍有商议的余地。

## Conclusion

After working with DDS and having a healthy amount of skepticism about the ethos, community, and licensing, it is hard to come up with any real technical criticisms. While it is true that the community surrounding DDS is very different from the ROS community or the ZeroMQ community, it appears that DDS is just solid technology on which ROS could safely depend. There are still many questions about exactly how ROS would utilize DDS, but they all seem like engineering exercises at this point and not potential deal breakers for ROS.

> 在使用 DDS 并对其信条、社区和许可证持有健康的怀疑态度之后，很难想出任何实质性的技术批评。虽然 DDS 周围的社区与 ROS 社区或 ZeroMQ 社区明显不同，但似乎 DDS 只是 ROS 可以安全依赖的坚实技术。仍然有许多关于 ROS 如何利用 DDS 的问题，但它们似乎都只是工程实践，而不是 ROS 的潜在交易破坏者。
