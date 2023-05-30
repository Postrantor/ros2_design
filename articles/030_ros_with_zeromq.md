---
tip: translate by openai@2023-05-30 09:18:24
layout: default
title: ROS on ZeroMQ and Friends
html_title: ZeroMQ and Friends
permalink: articles/ros_with_zeromq.html
abstract:
  This article makes the case for using ZeroMQ and other libraries to implement a new, modern middleware for ROS.
  This article also covers the results of the ZeroMQ based prototype made by OSRF.
published: true
author: '[William Woodall](https://github.com/wjwwood)'
date_written: 2014-06
last_modified: 2019-05

Authors: {{ page.author }}
Date Written: {{ page.date_written }}
Last Modified: {% if page.last_modified %}{{ page.last_modified }}{% else %}{{ page.date_written }}{% endif %}
---

While this article covers proposals and related experiments for building a new middleware specifically around ZeroMQ, it also generally captures the idea of building a new middleware out of a few component libraries. This strategy of composing existing libraries into a middleware is in contrast to wrapping up an existing end-to-end middleware which provides most if not all of the middleware needs for ROS out of the box.

> 这篇文章既涵盖了建立基于 ZeroMQ 的新中间件的提议和相关实验，也概括了由几个组件库构建新中间件的想法。这种将现有库组合成中间件的策略与包装现有的端到端中间件相反，后者可以为 ROS 提供大部分甚至全部中间件需求。

## Building a Prototype Middleware from Scratch

In order to meet the needs of ROS, a middleware needs to provide a few key services. First it needs to provide a way for parts of the system to discover each other and make connections dynamically at run time. Then the middleware needs to provide one or more transport paradigms for moving information between parts of the system, and for ROS specifically the publish-subscribe pattern is required at a minimum. Additional communication patterns (such as request-response) can be implemented on top of publish-subscribe. Finally, the middleware should provide a means of defining messages and then preparing them for transport, i.e. serialization. However, if the middleware lacks a serialization mechanism, this can be provided by an external component. Since ROS 1 was designed, there have been several new libraries in these component fields to gain popularity.

> 为了满足 ROS 的需求，中间件需要提供几项关键服务。首先，它需要提供一种方法，让系统的各个部分在运行时相互发现并建立动态连接。然后，中间件需要提供一种或多种传输范式，用于在系统的各个部分之间传递信息，对于 ROS 来说，至少需要实现发布-订阅模式。可以在发布-订阅模式之上实现其他的通信模式（如请求-响应）。最后，中间件应该提供定义消息并准备它们进行传输（即序列化）的方法。但是，如果中间件缺乏序列化机制，可以由外部组件提供。自从 ROS 1 被设计以来，这些组件领域出现了几个新的库来获得普及。

### Discovery

For discovery the first solution that was investigated was [Zeroconf](http://en.wikipedia.org/wiki/Zero_configuration_networking) with Avahi/Bonjour. Some simple experiments were conducted which used [pybonjour](https://code.google.com/p/pybonjour/) to try out using the Zeroconf system for discovery. The core technology here is `mDNSresponder`, which is provided by Apple as free software, and is used by both Bonjour (OS X and Windows) and Avahi (Linux, specifically avahi-compat).

> 为了发现第一个解决方案，我们研究了[Zeroconf](http://en.wikipedia.org/wiki/Zero_configuration_networking)，使用 Avahi/Bonjour。我们进行了一些简单的实验，使用[pybonjour](https://code.google.com/p/pybonjour/)来尝试使用 Zeroconf 系统进行发现。这里的核心技术是`mDNSresponder`，由苹果免费提供，被 Bonjour（OS X 和 Windows）和 Avahi（Linux，特别是 avahi-compat）所使用。

These Zeroconf implementations, however, proved to not be so reliable with respect to keeping a consistent graph between machines. Adding and removing more than about twenty items at a time from subprocesses typically resulted in inconsistent state on at least one of the computers on the network. One particularly bad case was the experiment of removing items from Zeroconf, where in several "nodes" were registered on machine A and then after a few seconds shutdown cleanly. The observed behavior on remote machines B and C was that the Zeroconf browser would show all "nodes" as registered, but then after being shutdown only some would be removed from the list, resulting in "zombie nodes". Worse still is that the list of "zombie nodes" were different on B and C. This problem was only observed between machines using avahi as a compatibility layer, which lead into a closer look into avahi and its viability as a core dependency. This closer look at avahi raised some concerns about the quality of the implementation with respect to the [Multicast DNS](http://en.wikipedia.org/wiki/Multicast_DNS) and [DNS Service Discovery (DNS-SD)](http://en.wikipedia.org/wiki/Zero_configuration_networking#Service_discovery) technology.

> 这些 Zeroconf 实现，然而，在保持机器之间一致的图表方面证明并不那么可靠。从子进程中添加和删除超过 20 个项目通常会导致网络上至少一台计算机的不一致状态。一个特别糟糕的情况是从 Zeroconf 中删除项目的实验，在几个“节点”在机器 A 上注册，然后在几秒钟内干净地关闭。在远程机器 B 和 C 上观察到的行为是，Zeroconf 浏览器会显示所有“节点”都已注册，但是关闭后只有一些会从列表中删除，导致“僵尸节点”。更糟糕的是，B 和 C 上的“僵尸节点”列表是不同的。这个问题只在使用 avahi 作为兼容层的机器之间观察到，这导致了对 avahi 及其作为核心依赖项的可行性的更仔细的观察。这种对 avahi 的仔细观察引发了一些关于[多播 DNS](http://en.wikipedia.org/wiki/Multicast_DNS)和[DNS 服务发现（DNS-SD）](http://en.wikipedia.org/wiki/Zero_configuration_networking#Service_discovery)技术实现质量的担忧。

Furthermore, DNS-SD seems to prefer the trade-off of light networking load for eventual consistency. This works reasonably well for something like service name look up, but it did not work well for quickly and reliably discovering the proto-ROS graph in the experiments. This led to the development of a custom discovery system which is implemented in a few languages as part of the prototype here:

> 此外，DNS-SD 似乎偏向于以轻负荷网络为代价来获得最终的一致性。这对于服务名称查找之类的东西工作相当不错，但是在实验中，它无法快速可靠地发现 proto-ROS 图。因此，开发了一个定制的发现系统，它在这里的原型中用几种语言实现：

[https://bitbucket.org/osrf/disc_zmq/src](https://bitbucket.org/osrf/disc_zmq/src) OSRF 的发现 ZMQ 源

The custom discovery system used multicast UDP packets to post notifications like "Node started", "Advertise a publisher", and "Advertise a subscription", along with any meta data required to act, like for publishers, an address to connect to using ZeroMQ. The details of this simple discovery system can be found at the above URL.

> 自定义发现系统使用多播 UDP 数据包发布类似“节点已启动”、“发布者广告”和“订阅广告”的通知，以及执行所需的元数据，如发布者的连接地址使用 ZeroMQ。这个简单的发现系统的详细信息可以在上面的 URL 中找到。

This system, though simple, was quite effective and was sufficient to prove that implementing such a custom discovery system, even in multiple languages, is a tractable problem.

> 这个系统虽然简单，但是非常有效，足以证明实施这样一个定制发现系统，即使是多种语言，也是一个可行的问题。

### Data Transport

For transporting bytes between processes, a popular library is [ZeroMQ](http://zeromq.org/), but there are also libraries like [nanomsg](http://nanomsg.org/) and [RabbitMQ](http://www.rabbitmq.com/). In all of those cases the goal of the library is to allow you to establish connections, explicitly, to other participants and then send strings or bytes according to some communication pattern. ZeroMQ is an LGPL licensed library which has recently become very popular, is written in C++ with a C API, and has bindings to many languages. nanomsg is a new MIT licensed library which was created by one of the original authors of ZeroMQ, is written in C with a C API, but is far less mature than ZeroMQ. RabbitMQ is a broker that implements several messaging protocols, mainly AMQP, but also provides gateways for ZeroMQ, STOMP and MQTT. By being a broker, it meets some of the discovery needs as well as the transport needs for ROS. Although ZeroMQ is usually used in brokerless deployments, it can also be used in conjunction with RabbitMQ to provide persitence and durability of messages. RabbitMQ is licensed under the Mozilla Public License. All of these libraries could probably be used to replace the ROSTCP transport, but for the purposes of this article we will use ZeroMQ in a brokerless deployment.

> 为了在进程之间传输字节，一个流行的库是[ZeroMQ](http://zeromq.org/)，但也有像[nanomsg](http://nanomsg.org/)和[RabbitMQ](http://www.rabbitmq.com/)的库。在所有这些情况下，库的目标是允许您明确地建立到其他参与者的连接，然后根据某种通信模式发送字符串或字节。ZeroMQ 是一个以 LGPL 许可的库，最近变得非常流行，用 C++ 编写，有 C API，并且有绑定到许多语言。 nanomsg 是由 ZeroMQ 的原始作者之一创建的新的 MIT 许可库，用 C 编写，但比 ZeroMQ 要不成熟得多。 RabbitMQ 是一个实现多种消息协议的代理，主要是 AMQP，但也提供 ZeroMQ，STOMP 和 MQTT 的网关。通过作为代理，它满足了 ROS 的一些发现需求以及传输需求。虽然 ZeroMQ 通常用于无代理部署，但也可以与 RabbitMQ 结合使用，以提供消息的持久性和可靠性。 RabbitMQ 根据 Mozilla 公共许可证授权。所有这些库都可以用来替换 ROSTCP 传输，但是在本文中，我们将在无代理部署中使用 ZeroMQ。

In this prototype:

> 在这个原型中：

[https://bitbucket.org/osrf/disc_zmq/src](https://bitbucket.org/osrf/disc_zmq/src)

ZeroMQ was used as the transport, which conveniently has bindings in C, C++, and Python.

> ZeroMQ 被用作传输，它有 C、C++和 Python 的绑定，很方便。

After making discoveries using the above described simple discovery system, connections were made using ZeroMQ's `ZMQ_PUB` and `ZMQ_SUB` sockets. This worked quite well, allowing for communication between processes in an efficient and simple way. However, in order to get more advanced features, like for instance latching, ZeroMQ takes the convention approach, meaning that it must be implemented by users with a well known pattern. This is a good approach which keeps ZeroMQ lean and simple, but does mean more code which must be implemented and maintained for the prototype.

> 经过使用上述简单的发现系统进行发现后，使用 ZeroMQ 的“ZMQ_PUB”和“ZMQ_SUB”套接字进行了连接。这种方法非常有效，可以以高效且简单的方式在进程之间进行通信。但是，为了获得更多高级功能，比如锁存，ZeroMQ 采用了约定方法，这意味着用户必须使用众所周知的模式来实现它。这是一种很好的方法，可以让 ZeroMQ 保持精简和简单，但是这意味着必须为原型实现和维护更多的代码。

Additionally, ZeroMQ, in particular, relies on reliable transports like TCP or [PGM (Pragmatic General Multicast)](http://en.wikipedia.org/wiki/Pragmatic_General_Multicast), so it makes it unsuitable for soft real-time scenarios.

> 此外，ZeroMQ 特别依赖可靠的传输协议，如 TCP 或 PGM（Pragmatic General Multicast），因此不适合软实时场景。

### Message Serialization

In ROS 1, messages are defined in `.msg` files and code is generated at build time for each of the supported languages. ROS 1 generated code can instantiate and then later serialize the data in a message as a mechanism for exchanging information. Since ROS was created, several popular libraries which take care of this responsibility have come about. Google's [Protocol Buffers (Protobuf)](https://code.google.com/p/protobuf/), [MessagePack](http://msgpack.org/), [BSON](http://bsonspec.org/), and [Cap'n Proto](http://kentonv.github.io/capnproto/) are all examples of serialization libraries which have come to popularity since ROS was originally written. An entire article could be devoted to the pros and cons of different message definition formats, serialization libraries, and their wire formats, but for the purposes of this prototype we worked with either plain strings or Protobuf.

> 在 ROS 1 中，消息定义在`.msg`文件中，并且为每种支持的语言在构建时生成代码。ROS 1 生成的代码可以实例化，然后再序列化消息中的数据，以作为交换信息的机制。自 ROS 创建以来，已经出现了几个处理这项职责的流行库。Google 的[Protocol Buffers（Protobuf）](https://code.google.com/p/protobuf/)，[MessagePack](http://msgpack.org/)，[BSON](http://bsonspec.org/)和[Cap'n Proto](http://kentonv.github.io/capnproto/)都是自 ROS 最初编写以来流行起来的序列化库的例子。可以花一篇文章来讨论不同消息定义格式，序列化库及其线路格式的优缺点，但是为了本次原型的目的，我们使用普通字符串或 Protobuf。

## Conclusions

After implementing the custom middleware prototype, some points worth noting were made. First, there isn't any existing discovery systems which address the needs of the middleware which are not attached to other middlewares. Implementing a custom discovery system is a possible but time consuming.

> 实施自定义中间件原型后，有一些值得注意的要点。首先，没有任何现有的发现系统可以满足中间件的需求，而不附加到其他中间件上。实施自定义发现系统是可能的，但是需要花费大量的时间。

Second, there is a good deal of software that needs to exist in order to integrate discovery with transport and serialization. For example, the way in which connections are established, whether using point to point or multicast, is a piece of code which lives between the transport and discovery systems. Another example is the efficient intra-process communications: ZeroMQ provides an INPROC socket, but the interface to that socket is bytes, so you cannot use that without constructing a system where you pass around pointers through INPROC rather than serialized data. At the point where you are passing around pointers rather than serialized data you have to start to duplicate behavior between the intraprocess and interprocess communications which are abstracted at the ROS API level. One more piece of software which is needed is the type-safety system which works between the transport and the message serialization system. Needless to say, even with these component libraries solving a lot of the problems with implementing a middleware like ROS's, there still exists quite a few glue pieces which are needed to finish the implementation.

> 第二，需要有许多软件来将发现与传输和序列化集成在一起。例如，使用**点对点或多播建立连接**的方式，是一段位于传输和发现系统之间的代码。另一个例子是高效的进程间通信：ZeroMQ 提供了一个 INPROC 套接字，但是该套接字的接口是字节，所以您无法使用它而不构建一个系统，在该系统中，您通过 INPROC 而不是序列化数据传递指针。当您在传递指针而不是序列化数据时，您必须开始在进程间和进程间通信之间复制 ROS API 级别抽象的行为。还需要一个软件，它在传输和消息序列化系统之间工作，以提供类型安全性。毋庸置疑，即使使用这些组件库解决了实现 ROS 类中间件的许多问题，仍然需要许多胶水来完成实现。

Even though it would be a lot of work to implement a middleware using component libraries like ZeroMQ and Protobuf, the result would likely be a finely tuned and well understood piece of software. This path would most likely give the most control over the middleware to the ROS community.

> 即使使用像 ZeroMQ 和 Protobuf 这样的组件库来实现中间件会有很多工作，但结果可能是一款精细调整和深入了解的软件。这条路径很可能会给 ROS 社区带来最多的对中间件的控制权。

In exchange for the creative control over the middleware comes the responsibility to document its behavior and design to the point that it can be verified and reproduced. This is a non-trivial task which ROS 1 did not do very well because it had a relatively good pair of reference implementations. Many users that wish to put ROS into mission critical situations and into commercial products have lamented that ROS lacks this sort of governing design document which allows them to certify and audit the system. It would be of paramount importance that this new middleware be well defined, which is not a trivial task and almost certainly rivals the engineering cost of the initial implementation.

> 为了拥有中间件的创造性控制，就有责任将其行为和设计文档化，以便可以验证和复制。这是一项不容易的任务，ROS 1 并没有做得很好，因为它有一对相对较好的参考实现。许多希望将 ROS 应用于关键任务和商业产品的用户抱怨 ROS 缺乏这种支配设计文档，可以让他们对系统进行认证和审计。这个新的中间件必须得到很好的定义，这是一项不容易的任务，几乎肯定超过了最初实施的工程成本。
