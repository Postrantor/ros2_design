---
tip: translate by openai@2023-05-28 11:07:05
...
---
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


While this article covers proposals and related experiments for building a new middleware specifically around ZeroMQ, it also generally captures the idea of building a new middleware out of a few component libraries.

> 这篇文章既涵盖了基于ZeroMQ构建新的中间件的提案和相关实验，也概括性地捕捉了由几个组件库构建新的中间件的想法。

This strategy of composing existing libraries into a middleware is in contrast to wrapping up an existing end-to-end middleware which provides most if not all of the middleware needs for ROS out of the box.

> 这种将现有库组合成中间件的策略与将现有的端到端中间件包装起来形成针对ROS的一站式中间件需求形成鲜明对比。

## Building a Prototype Middleware from Scratch


In order to meet the needs of ROS, a middleware needs to provide a few key services.

> 为了满足ROS的需求，中间件需要提供几项关键服务。

First it needs to provide a way for parts of the system to discover each other and make connections dynamically at run time.

> 首先，它需要提供一种方式，使系统的各个部分能够在运行时动态地发现彼此并建立连接。

Then the middleware needs to provide one or more transport paradigms for moving information between parts of the system, and for ROS specifically the publish-subscribe pattern is required at a minimum.

> 那么中间件需要提供一种或多种传输范式来在系统的不同部分之间传输信息，尤其是对于ROS，至少需要发布-订阅模式。

Additional communication patterns (such as request-response) can be implemented on top of publish-subscribe.

> 除了发布订阅模式之外，还可以实现其他的通信模式（如请求-响应）。

Finally, the middleware should provide a means of defining messages and then preparing them for transport, i.e. serialization.

> 最后，中间件应提供定义消息并准备进行传输（即序列化）的方法。

However, if the middleware lacks a serialization mechanism, this can be provided by an external component.

> 如果中间件没有序列化机制，可以由外部组件提供。

Since ROS 1 was designed, there have been several new libraries in these component fields to gain popularity.

> 自ROS 1被设计以来，这些组件领域中出现了几个新的库，受到了广泛的欢迎。

### Discovery


For discovery the first solution that was investigated was [Zeroconf](http://en.wikipedia.org/wiki/Zero_configuration_networking) with Avahi/Bonjour.

> 对于发现，首先调查的解决方案是[Zeroconf](http://en.wikipedia.org/wiki/Zero_configuration_networking) 和 Avahi/Bonjour。

Some simple experiments were conducted which used [pybonjour](https://code.google.com/p/pybonjour/) to try out using the Zeroconf system for discovery.

> 一些简单的实验被进行，它们使用[pybonjour](https://code.google.com/p/pybonjour/)来尝试使用Zeroconf系统进行发现。

The core technology here is `mDNSresponder`, which is provided by Apple as free software, and is used by both Bonjour (OS X and Windows) and Avahi (Linux, specifically avahi-compat).

> 核心技术是由苹果免费提供的mDNSresponder，它被Bonjour（OS X和Windows）和Avahi（Linux，特别是avahi-compat）所使用。


These Zeroconf implementations, however, proved to not be so reliable with respect to keeping a consistent graph between machines.

> 这些Zeroconf实现但是在保持机器之间的一致图表方面并不那么可靠。

Adding and removing more than about twenty items at a time from subprocesses typically resulted in inconsistent state on at least one of the computers on the network.

> 添加和删除超过20个项目的子进程通常会导致网络上至少一台电脑的状态不一致。

One particularly bad case was the experiment of removing items from Zeroconf, where in several "nodes" were registered on machine A and then after a few seconds shutdown cleanly.

> 一个特别糟糕的情况是从Zeroconf中移除项目的实验，其中几个“节点”在机器A上注册，然后几秒钟后干净地关闭。

The observed behavior on remote machines B and C was that the Zeroconf browser would show all "nodes" as registered, but then after being shutdown only some would be removed from the list, resulting in "zombie nodes".

> 观察到远程机器B和C的行为是，Zeroconf浏览器会显示所有“节点”已注册，但是关机后只有一些会从列表中移除，导致“僵尸节点”。

Worse still is that the list of "zombie nodes" were different on B and C.

> 情况更糟的是，B和C上的“僵尸节点”列表不同。

This problem was only observed between machines using avahi as a compatibility layer, which lead into a closer look into avahi and its viability as a core dependency.

> 这个问题只在使用Avahi作为兼容层的机器之间观察到，这导致我们更加仔细地研究了Avahi及其作为核心依赖项的可行性。

This closer look at avahi raised some concerns about the quality of the implementation with respect to the [Multicast DNS](http://en.wikipedia.org/wiki/Multicast_DNS) and [DNS Service Discovery (DNS-SD)](http://en.wikipedia.org/wiki/Zero_configuration_networking#Service_discovery) technology.

> 这次对Avahi的仔细检查引发了一些关于[多播DNS](http://en.wikipedia.org/wiki/Multicast_DNS)和[DNS服务发现（DNS-SD）](http://en.wikipedia.org/wiki/Zero_configuration_networking#Service_discovery)技术实现质量的担忧。


Furthermore, DNS-SD seems to prefer the trade-off of light networking load for eventual consistency.

> 此外，DNS-SD似乎更偏向于以轻负荷网络为代价以达到最终的一致性。

This works reasonably well for something like service name look up, but it did not work well for quickly and reliably discovering the proto-ROS graph in the experiments.

> 这对于像服务名称查找这样的事情来说效果还可以，但是在实验中快速可靠地发现原始ROS图表却表现不佳。

This led to the development of a custom discovery system which is implemented in a few languages as part of the prototype here:

> 这导致了一个定制发现系统的开发，它作为原型的一部分用几种语言实现：


[https://bitbucket.org/osrf/disc_zmq/src](https://bitbucket.org/osrf/disc_zmq/src)

> [https://bitbucket.org/osrf/disc_zmq/src](https://bitbucket.org/osrf/disc_zmq/src) 是 OSRF 的分布式 ZeroMQ 库的源代码。


The custom discovery system used multicast UDP packets to post notifications like "Node started", "Advertise a publisher", and "Advertise a subscription", along with any meta data required to act, like for publishers, an address to connect to using ZeroMQ.

> 系统使用多播UDP数据包发布通知，例如“节点启动”、“发布发布者”和“发布订阅”，以及执行所需的任何元数据，例如对于发布者，使用ZeroMQ连接的地址。

The details of this simple discovery system can be found at the above URL.

> 该简单发现系统的详细信息可以在上面的URL中找到。


This system, though simple, was quite effective and was sufficient to prove that implementing such a custom discovery system, even in multiple languages, is a tractable problem.

> 这个系统虽然简单，但是非常有效，足以证明实施这样的自定义发现系统，即使是多种语言，也是一个可行的问题。

### Data Transport


For transporting bytes between processes, a popular library is [ZeroMQ](http://zeromq.org/), but there are also libraries like [nanomsg](http://nanomsg.org/) and [RabbitMQ](http://www.rabbitmq.com/).

> 一个流行的库用于在进程之间传输字节是[ZeroMQ](http://zeromq.org/)，但也有[nanomsg](http://nanomsg.org/)和[RabbitMQ](http://www.rabbitmq.com/)等库。

In all of those cases the goal of the library is to allow you to establish connections, explicitly, to other participants and then send strings or bytes according to some communication pattern.

> 在所有这些情况下，库的目的是允许您明确地与其他参与者建立连接，然后根据某种通信模式发送字符串或字节。

ZeroMQ is an LGPL licensed library which has recently become very popular, is written in C++ with a C API, and has bindings to many languages.

> ZeroMQ是一个LGPL许可的库，最近变得非常流行，用C++编写，有C语言的API，并且有多种语言的绑定。

nanomsg is a new MIT licensed library which was created by one of the original authors of ZeroMQ, is written in C with a C API, but is far less mature than ZeroMQ.

> nanomsg是一个新的MIT许可的库，由ZeroMQ的原始作者之一创建，用C编写，具有C API，但不及ZeroMQ成熟。

RabbitMQ is a broker that implements several messaging protocols, mainly AMQP, but also provides gateways for ZeroMQ, STOMP and MQTT.

> RabbitMQ是一个实现了多种消息协议的中间件，主要是AMQP，同时还提供ZeroMQ、STOMP和MQTT的网关。

By being a broker, it meets some of the discovery needs as well as the transport needs for ROS.

> 作为一个经纪人，它满足了ROS的一些发现需求以及运输需求。

Although ZeroMQ is usually used in brokerless deployments, it can also be used in conjunction with RabbitMQ to provide persitence and durability of messages.

> 雖然ZeroMQ通常用於無代理部署，但它也可以與RabbitMQ結合使用，以提供消息的持久性和可靠性。

RabbitMQ is licensed under the Mozilla Public License.

> RabbitMQ按照Mozilla公共许可证授权。

All of these libraries could probably be used to replace the ROSTCP transport, but for the purposes of this article we will use ZeroMQ in a brokerless deployment.

> 所有这些库可能都可以用来取代ROSTCP传输，但是为了本文的目的，我们将在无代理部署中使用ZeroMQ。


In this prototype:

> 在这个原型中：


[https://bitbucket.org/osrf/disc_zmq/src](https://bitbucket.org/osrf/disc_zmq/src)

> [https://bitbucket.org/osrf/disc_zmq/src](https://bitbucket.org/osrf/disc_zmq/src)：OSRF Disc ZMQ 源代码


ZeroMQ was used as the transport, which conveniently has bindings in C, C++, and Python.

> ZeroMQ被用作传输，它有C、C++和Python的绑定，方便易用。

After making discoveries using the above described simple discovery system, connections were made using ZeroMQ's `ZMQ_PUB` and `ZMQ_SUB` sockets.

> 在使用上述简单发现系统发现之后，使用ZeroMQ的`ZMQ_PUB`和`ZMQ_SUB`套接字建立连接。

This worked quite well, allowing for communication between processes in an efficient and simple way.

> 这很有效，可以在进程之间进行高效简单的通信。

However, in order to get more advanced features, like for instance latching, ZeroMQ takes the convention approach, meaning that it must be implemented by users with a well known pattern.

> 然而，为了获得更高级的功能，比如闭锁，ZeroMQ采用了约定的方法，这意味着用户必须使用一种众所周知的模式来实现它。

This is a good approach which keeps ZeroMQ lean and simple, but does mean more code which must be implemented and maintained for the prototype.

> 这是一种很好的方法，可以让ZeroMQ保持简洁，但是意味着为原型实现和维护的代码更多。


Additionally, ZeroMQ, in particular, relies on reliable transports like TCP or [PGM (Pragmatic General Multicast)](http://en.wikipedia.org/wiki/Pragmatic_General_Multicast), so it makes it unsuitable for soft real-time scenarios.

> 另外，ZeroMQ特别依赖可靠的传输协议，比如TCP或PGM（实用通用多播），因此它不适合软实时场景。

### Message Serialization


In ROS 1, messages are defined in `.msg` files and code is generated at build time for each of the supported languages.

> 在ROS 1中，消息定义在`.msg`文件中，为每种支持的语言在构建时生成代码。

ROS 1 generated code can instantiate and then later serialize the data in a message as a mechanism for exchanging information.

> ROS 1 生成的代码可以实例化，然后稍后将消息中的数据序列化，作为交换信息的机制。

Since ROS was created, several popular libraries which take care of this responsibility have come about.

> 自从ROS创建以来，已经出现了几个负责处理这项责任的流行库。

Google's [Protocol Buffers (Protobuf)](https://code.google.com/p/protobuf/), [MessagePack](http://msgpack.org/), [BSON](http://bsonspec.org/), and [Cap'n Proto](http://kentonv.github.io/capnproto/) are all examples of serialization libraries which have come to popularity since ROS was originally written.

> 谷歌的协议缓冲区（Protobuf）、MessagePack、BSON和Cap'n Proto都是自ROS最初编写以来流行起来的序列化库的例子。

An entire article could be devoted to the pros and cons of different message definition formats, serialization libraries, and their wire formats, but for the purposes of this prototype we worked with either plain strings or Protobuf.

> 一篇文章可以用来讨论不同消息定义格式、序列化库及其线路格式的利弊，但为了这个原型，我们使用普通字符串或Protobuf。

## Conclusions


After implementing the custom middleware prototype, some points worth noting were made.

> 在实施自定义中间件原型后，提出了一些值得注意的要点。

First, there isn't any existing discovery systems which address the needs of the middleware which are not attached to other middlewares.

> 首先，没有任何现有的发现系统可以满足不和其他中间件相连的中间件的需求。

Implementing a custom discovery system is a possible but time consuming.

> 实现一个自定义发现系统是可能的，但是耗时。


Second, there is a good deal of software that needs to exist in order to integrate discovery with transport and serialization.

> 第二，为了将发现与传输和序列化集成在一起，需要存在大量的软件。

For example, the way in which connections are established, whether using point to point or multicast, is a piece of code which lives between the transport and discovery systems.

> 例如，建立连接的方式，无论是使用点对点还是多播，都是位于传输和发现系统之间的一段代码。

Another example is the efficient intra-process communications: ZeroMQ provides an INPROC socket, but the interface to that socket is bytes, so you cannot use that without constructing a system where you pass around pointers through INPROC rather than serialized data.

> 另一个例子是高效的进程内通信：ZeroMQ提供INPROC套接字，但是连接到该套接字的接口是字节，因此您无法使用它而不构建一个系统，在该系统中您通过INPROC传递指针而不是序列化数据。

At the point where you are passing around pointers rather than serialized data you have to start to duplicate behavior between the intraprocess and interprocess communications which are abstracted at the ROS API level.

> 当你使用指针而不是序列化数据传递时，你必须开始在ROS API级别抽象的进程内通信和进程间通信之间复制行为。

One more piece of software which is needed is the type-safety system which works between the transport and the message serialization system.

> 需要的另一个软件是在传输和消息序列化系统之间工作的类型安全系统。

Needless to say, even with these component libraries solving a lot of the problems with implementing a middleware like ROS's, there still exists quite a few glue pieces which are needed to finish the implementation.

> 毋庸置疑，即使有了这些组件库来解决实施像ROS这样的中间件的许多问题，仍然需要一些胶水来完成实施。


Even though it would be a lot of work to implement a middleware using component libraries like ZeroMQ and Protobuf, the result would likely be a finely tuned and well understood piece of software.

> 即使使用像ZeroMQ和Protobuf这样的组件库来实现中间件会是很多工作，但最终的结果可能是一款经过精心调整和深入理解的软件。

This path would most likely give the most control over the middleware to the ROS community.

> 这条路径很可能会给ROS社区带来最大的中间件控制权。


In exchange for the creative control over the middleware comes the responsibility to document its behavior and design to the point that it can be verified and reproduced.

> 交换中间件的创造性控制权就是要负责文档化它的行为和设计，以便可以进行验证和复制。

This is a non-trivial task which ROS 1 did not do very well because it had a relatively good pair of reference implementations.

> 这是一项非常复杂的任务，ROS 1 做得不太好，因为它有一对相对较好的参考实现。

Many users that wish to put ROS into mission critical situations and into commercial products have lamented that ROS lacks this sort of governing design document which allows them to certify and audit the system.

> 许多希望将ROS应用于关键任务和商业产品的用户抱怨缺乏可以让他们认证和审计系统的支配性设计文档。

It would be of paramount importance that this new middleware be well defined, which is not a trivial task and almost certainly rivals the engineering cost of the initial implementation.

> 这个新的中间件的定义非常重要，这是一项不容易的任务，几乎可以肯定，它的工程成本超过最初实施的成本。
