---
tip: translate by openai@2023-05-30 08:57:32
layout: default
title: ROS 2 middleware interface
permalink: articles/ros_middleware_interface.html
abstract:
  This article describes the rationale for using an abstract middleware interface between ROS and a specific middleware implementation.
  It will outline the targeted use cases as well as their requirements and constraints.
  Based on that the developed middleware interface is explained.
author: '[Dirk Thomas](https://github.com/dirk-thomas)'
date_written: 2014-08
last_modified: 2017-09
published: true
categories: Middleware

Authors: {{ page.author }}
Date Written: {{ page.date_written }}
Last Modified: {% if page.last_modified %}{{ page.last_modified }}{% else %}{{ page.date_written }}{% endif %}
---

## The _middleware interface_

### Why does ROS 2 have a _middleware interface_

The ROS client library defines an API which exposes communication concepts like publish / subscribe to users.

> ROS 客户端库定义了一个 API，它向用户公开了像发布/订阅这样的通信概念。

In ROS 1 the implementation of these communication concepts was built on custom protocols (e.g., [TCPROS](http://wiki.ros.org/ROS/TCPROS)).

> 在 ROS 1 中，这些通信概念的实现是基于自定义协议(例如[TCPROS](http://wiki.ros.org/ROS/TCPROS))构建的。

For ROS 2 the decision has been made to build it on top of an existing middleware solution (namely [DDS](http://en.wikipedia.org/wiki/Data_Distribution_Service)). The major advantage of this approach is that ROS 2 can leverage an existing and well developed implementation of that standard.

> 对于 ROS 2，我们已经决定在现有的中间件解决方案(即[DDS](http://en.wikipedia.org/wiki/Data_Distribution_Service))之上构建它。这种方法的主要优点是 ROS 2 可以利用现有和完善的标准实现。

ROS could build on top of one specific implementation of DDS. But there are numerous different implementations available and each has its own pros and cons in terms of supported platforms, programming languages, performance characteristics, memory footprint, dependencies and licensing.

> ROS 可以建立在 DDS 的特定实现之上。但是有许多不同的实现可供选择，每个实现都有自己的优缺点，包括支持的平台、编程语言、性能特征、内存占用、依赖关系和许可证。

Therefore ROS aims to support multiple DDS implementations despite the fact that each of them differ slightly in their exact API. In order to abstract from the specifics of these APIs, an abstract interface is being introduced which can be implemented for different DDS implementations. This _middleware interface_ defines the API between the ROS client library and any specific implementation.

> 因此，尽管每个 DDS 实现都在其确切的 API 上略有不同，ROS 旨在支持多个 DDS 实现。为了抽象出这些 API 的特定之处，引入了一个抽象接口，可以为不同的 DDS 实现实现。这个*中间件接口*定义了 ROS 客户端库和任何特定实现之间的 API。

Each implementation of the interface will usually be a thin adapter which maps the generic _middleware interface_ to the specific API of the middleware implementation. In the following the common separation of the adapter and the actual middleware implementation will be omitted.

> 每个接口的实现通常都是一个薄适配器，将通用的中间件接口映射到中间件实现的特定 API。在接下来的内容中，将省略适配器和实际中间件实现的常见分离。

    +-----------------------------------------------+
    |                   user land                   |
    +-----------------------------------------------+
    |              ROS client library               |
    +-----------------------------------------------+
    |             middleware interface              |
    +-----------------------------------------------+
    | DDS adapter 1 | DDS adapter 2 | DDS adapter 3 |
    +---------------+---------------+---------------+
    |    DDS impl 1 |    DDS impl 2 |    DDS impl 3 |
    +---------------+---------------+---------------+

## Why should the _middleware interface_ be agnostic to DDS

The ROS client library should not expose any DDS implementation specifics to the user. This is primarily to hide the intrinsic complexity of the DDS specification and API.

> ROS 客户端库不应向用户暴露任何 DDS 实现细节。这主要是为了隐藏 DDS 规范和 API 的内在复杂性。

While ROS 2 only aims to support DDS based middleware implementations it can strive to keep the _middleware interface_ free of DDS specific concepts to enable implementations of the interface using a different middleware. It would also be feasible to implement the interface by tying together several unrelated libraries providing the necessary functions of discovery, serialization and publish / subscribe.

> 虽然 ROS 2 只旨在支持基于 DDS 的中间件实现，但它可以努力让中间件接口中不包含 DDS 特定的概念，以便使用不同的中间件实现该接口。也可以通过将提供发现、序列化和发布/订阅所必需功能的几个不相关的库结合起来来实现该接口。

    +-----------------------------------+
    |             user land             |   no middleware implementation specific code
    +-----------------------------------+
    |        ROS client library         |   above the interface
    +-----------------------------------+
    |       middleware interface        |   ---
    +-----------------------------------+
    | mw impl 1 | mw impl 2 | mw impl 3 |
    +-----------+-----------+-----------+

## How does the information flow through the _middleware interface_

One goal of the _middleware interface_ is to not expose any DDS specific code to the user land code. Therefore the ROS client library "above" the _middleware interface_ needs to only operate on ROS data structures. ROS 2 will continue to use ROS message files to define the structure of these data objects and derive the data structures for each supported programming language from them.

> 一个中间件接口的目标是不向用户层代码暴露任何 DDS 特定的代码。因此，位于中间件接口之上的 ROS 客户端库只能操作 ROS 数据结构。ROS 2 将继续使用 ROS 消息文件来定义这些数据对象的结构，并从中派生每种支持的编程语言的数据结构。

The middleware implementation "below" the _middleware interface_ must convert the ROS data objects provided from the client library into its own custom data format before passing it to the DDS implementation. In reverse custom data objects coming from the DDS implementation must be converted into ROS data objects before being returned to the ROS client library.

> 实现"下面"中间件接口的中间件必须将 ROS 数据对象从客户端库转换为自己的自定义数据格式，然后再将其传递给 DDS 实现。反之，来自 DDS 实现的自定义数据对象必须转换为 ROS 数据对象，然后才能返回到 ROS 客户端库。

The definition for the middleware specific data types can be derived from the information specified in the ROS message files. A defined mapping between the primitive data types of ROS message and middleware specific data types ensures that a bidirectional conversion is possible. The functionality to convert between ROS types and the implementation specific types or API is encapsulated in the `type support` (see below for different kind of type supports).

> 定义中间件特定数据类型可以从 ROS 消息文件中指定的信息中推导出来。定义 ROS 消息的原始数据类型和中间件特定数据类型之间的映射，确保可以实现双向转换。将 ROS 类型和实现特定类型或 API 之间进行转换的功能封装在`type support`中(见下文关于不同类型支持)。

    +----------------------+
    |      user land       |   1) create a ROS message
    +----------------------+      v
    |  ROS client library  |   2) publish the ROS message
    +----------------------+      v
    | middleware interface |      v
    +----------------------+      v
    |      mw impl N       |   3) convert the ROS message into a DDS sample and publish the DDS sample
    +----------------------+

<!--- separate code blocks -->

    +----------------------+
    |      user land       |   3) use the ROS message
    +----------------------+      ^
    |  ROS client library  |   2) callback passing a ROS message
    +----------------------+      ^
    | middleware interface |      ^
    +----------------------+      ^
    |      mw impl N       |   1) convert the DDS sample into a ROS message and invoke subscriber callback
    +----------------------+

Depending on the middleware implementation the extra conversion can be avoided by implementing serialization functions directly from ROS messages as well as deserialization functions into ROS messages.

> 根据中间件的实现，可以通过直接从 ROS 消息实现序列化函数以及反序列化函数到 ROS 消息来避免额外的转换。

## Considered use cases

The following use cases have been considered when designing the middleware interface:

> 在设计中间件接口时，已经考虑了以下用例：

### Single middleware implementation

ROS applications are not built in a monolithic way but distributed across several packages. Even with a _middleware interface_ in place the decision of which middleware implementation to use will affect significant parts of the code.

> ROS 应用程序不是以单一的方式构建，而是分布在几个软件包中。即使有中间件接口存在，选择哪个中间件实现也会影响代码的重要部分。

For example, a package defining a ROS message will need to provide the mapping to and from the middleware specific data type. Naively each package defining ROS messages might contain custom (usually generated) code for the specific middleware implementation.

> 例如，定义 ROS 消息的包需要提供与中间件特定数据类型的映射。天真地说，每个定义 ROS 消息的包可能都包含特定中间件实现的自定义(通常是生成的)代码。

In the context of providing binary packages of ROS (e.g., Debian packages) this implies that a significant part of them (at least all packages containing message definitions) would be specific to the selected middleware implementation.

> 在提供 ROS 二进制包(例如 Debian 包)的背景下，这意味着其中的大部分(至少所有包含消息定义的包)将特定于所选中间件实现。

                    +-----------+
                    | user land |
                    +-----------+
                         |||
          +--------------+|+-----------------+
          |               |                  |
          v               v                  v
    +-----------+   +-----------+   +-----------------+   All three packages
    | msg pkg 1 |   | msg pkg 2 |   | middleware impl |   on this level contain
    +-----------+   +-----------+   +-----------------+   middleware implementation specific code

### Static vs. dynamic message types with DDS

DDS has two different ways to use and interact with messages.

> DDS 有两种不同的方式来使用和与消息交互。

On the one hand the message can be specified in an IDL file from which usually a DDS implementation specific program will generate source code. The generated code for C++, e.g., contains types specifically generated for the message.

> 一方面，消息可以在 IDL 文件中指定，通常 DDS 实现的特定程序将生成源代码。 例如，为 C ++生成的代码包含专门为消息生成的类型。

On the other hand the message can be specified programmatically using the DynamicData API of the [XTypes](http://www.omg.org/spec/DDS-XTypes/) specification. Neither an IDL file nor a code generation step is necessary for this case.

> 在另一方面，可以使用[XTypes](http://www.omg.org/spec/DDS-XTypes/)规范的 DynamicData API 编程指定消息。在这种情况下，不需要 IDL 文件或代码生成步骤。

Some custom code must still map the message definition available in the ROS .msg files to invocations of the DynamicData API. But it is possible to write generic code which performs the task for any ROS .msg specification passed in.

> 一些自定义代码仍然需要将 ROS .msg 文件中可用的消息定义映射到 DynamicData API 的调用。但是可以编写通用代码来执行传入的任何 ROS .msg 规范的任务。

                    +-----------+
                    | user land |
                    +-----------+
                         |||
          +--------------+|+----------------+
          |               |                 |
          v               v                 |
    +-----------+   +-----------+           |            Each message provides its specification
    | msg pkg 1 |   | msg pkg 2 |           |            in a way which allows a generic mapping
    +-----------+   +-----------+           |            to the DynamicData API
          |               |                 |
          +-------+-------+                 |
                  |                         |
                  v                         v
     +-------------------------+   +-----------------+   Only the packages
     | msg spec to DynamicData |   | middleware impl |   on this level contain
     +-------------------------+   +-----------------+   middleware implementation specific code

However the performance using the DynamicData API will likely always be lower compared to the statically generated code.

> 然而，使用 DynamicData API 的性能很可能总是低于静态生成的代码。

### Switch between different implementations

When ROS supports different middleware implementations it should be as easy and low effort as possible for users to switch between them.

> 当 ROS 支持不同的中间件实现时，用户在它们之间切换应该尽可能简单且低努力。

#### Decide at compile time

One obvious way will be for a user to build all ROS packages from source selecting a specific middleware implementation. While the workflow won't be too difficult (probably half a dozen command-line invocations), it still requires quite some build time.

> 一种显而易见的方式是用户从源代码中构建所有 ROS 包，选择特定的中间件实现。虽然工作流程不会太困难(可能有半打命令行调用)，但仍需要相当多的构建时间。

To avoid the need to build from source a set of binary packages could be provided which chose the middleware implementation at build time. While this would reduce the effort for the user the buildfarm would need to build a completely separate set of binary packages. The effort to support N packages with M different middleware implementations would require significant resources (M \* N) for maintaining the service as well as the necessary computing power.

> 为了避免需要从源代码构建，可以提供一组二进制包，以在构建时选择中间件实现。虽然这可以减少用户的工作量，但构建农场仍需要构建一组完全独立的二进制包。支持 N 个包和 M 种不同中间件实现所需的资源(M \* N)将需要大量的资源来维护服务以及必要的计算能力。

#### Decide at runtime

The alternative is to support selecting a specific middleware implementation at runtime. This requires that the to be selected middleware implementation was available at compile time in order for the middleware specific type support to be generated for every message package.

> 另一种选择是在运行时支持选择特定的中间件实现。这需要在编译时可用要选择的中间件实现，以便为每个消息包生成中间件特定类型的支持。

When building ROS with a single middleware implementation the result should follow the design criteria:

> 在使用单一中间件实现构建 ROS 时，结果应遵循设计准则：

> Any features that you do not use you do not pay for.

This implies that there should be no overhead for neither the build time nor the runtime due to the ability to support different middleware implementations. However the additional abstraction due to the middleware interface is still valid in order to hide implementation details from the user.

> 这意味着，由于支持不同的中间件实现，构建时间和运行时应该没有额外的开销。但是，由于中间件接口的额外抽象，仍然可以隐藏实现细节，从而不让用户看到。

## Design of the _middleware interface_

The API is designed as a pure function-based interface in order to be implemented in C. A pure C interface can be used in ROS Client Libraries for most other languages including Python, Java, and C++ preventing the need to reimplement the core logic.

> API 被设计为纯函数式接口，以便在 C 中实现。大多数其他语言(包括 Python、Java 和 C++)的 ROS 客户端库可以使用纯 C 接口，从而避免重新实现核心逻辑。

### Publisher interface

Based on the general structure of ROS nodes, publishers and messages for the case of publishing messages the ROS client library need to invoke three functions on the middleware interface:

> 根据 ROS 节点的一般结构，发布消息的情况下，ROS 客户端库需要调用中间件接口的三个函数：

- `create_node()`
- `create_publisher()`
- `publish()`

#### Essential signature of `create_node`

Subsequent invocations of `create_publisher` need to refer to the specific node they should be created in. Therefore the `create_node` function needs to return a _node handle_ which can be used to identify the node.

> 后续调用`create_publisher`需要指向它们应该被创建的特定节点。因此，`create_node`函数需要返回一个*节点句柄*，用于标识节点。

    NodeHandle create_node();

#### Essential signature of `create_publisher`

Besides the _node handle_ the `create_publisher` function needs to know the _topic name_ as well as the _topic type_. The type of the _topic type_ argument is left unspecified for now.

> 除了*节点句柄*外，`create_publisher`函数还需要知道*主题名称*以及*主题类型*。目前尚未指定*主题类型*参数的类型。

Subsequent invocations of `publish` need to refer to the specific publisher they should send messages on. Therefore the `create_publisher` function needs to return a _publisher handle_ which can be used to identify the publisher.

> 后续调用`publish`需要指向特定的发布者，以发送消息。因此，`create_publisher`函数需要返回一个*发布者句柄*，用于标识发布者。

    PublisherHandle create_publisher(NodeHandle node_handle, String topic_name, .. topic_type);

The information encapsulated by the _topic type_ argument is highly dependent on the middleware implementation.

> 参数封装的信息严重依赖于中间件的实现。

##### Topic type information for the DynamicData API

In the case of using the DynamicData API in the implementation there is no C / C++ type which could represent the type information. Instead the _topic type_ must contain all information needed to describe the format of the message. This information includes:

> 在实现中使用 DynamicData API 时，没有 C/C++类型可以表示类型信息。取而代之的是，*主题类型*必须包含描述消息格式所需的所有信息，包括：

- the name of the package in which the message is defined
- the name of the message
- the list of fields of the message where each includes:

  - the name for the message field
  - the type of the message field (which can be either a built-in type or another message type), optionally the type might be an unbounded, bounded or fixed size array
  - the default value

- the list of constants defined in the message (again consisting of name, type and value)

In the case of using DDS this information enables one to:

> 在使用 DDS 的情况下，这些信息可以让人们：

- programmatically create a _DDS TypeCode_ which represents the message structure
- register the _DDS TypeCode_ with the _DDS Participant_
- create a _DDS Publisher_, _DDS Topic_ and _DDS DataWriter_
- convert data from a _ROS message_ into a _DDS DynamicData_ instance
- write the _DDS DynamicData_ to the _DDS DataWriter_

##### Topic type information for statically generated code

In the case of using statically generated code derived from an IDL there is are C / C++ types which represent the type information. The generated code contains functions to:

> 在使用从 IDL 生成的静态代码的情况下，存在 C/C++类型来表示类型信息。生成的代码包含以下功能：

- create a _DDS TypeCode_ which represents the message structure

Since the specific types must be defined at compile time the other functionalities can not be implemented in a generic (not specific to the actual message) way. Therefore for each message the code to perform the following tasks must be generated separately:

> 由于特定类型必须在编译时定义，因此其他功能不能以通用(不特定于实际消息)方式实现。因此，必须单独生成每条消息的代码来执行以下任务：

- register the _DDS TypeCode_ with the _DDS Participant_
- create a _DDS Publisher_, _DDS Topic_ and _DDS DataWriter_
- convert data from a _ROS message_ into a _DDS DynamicData_ instance
- write the _DDS DynamicData_ to the _DDS DataWriter_

The information encapsulated by the _topic type_ must include the function pointers to invoke these functions.

> 所封装在*主题类型*中的信息必须包括用于调用这些函数的函数指针。

#### `get_type_support_handle`

Since the information encapsulated by the _topic type_ argument is so fundamentally different for each middleware implementation it is actually retrieved through an additional function of the middleware interface:

> 由于*主题类型*参数封装的信息对于每个中间件实现来说是如此根本性的不同，因此实际上需要通过中间件接口的额外功能来获取它。

    MessageTypeSupportHandle get_type_support_handle();

Currently this function is a template function specialized on the specific ROS message type. For C compatibility an approach using a macro will be used instead which mangles the type name into the function name.

> 目前，此功能是一个专门针对特定 ROS 消息类型的模板函数。为了兼容 C，将使用宏而不是函数名将类型名称改变为函数名。

#### Essential signature of `publish`

Beside the _publisher handle_ the `publish` function needs to know the _ROS message_ to send.

> 除了出版者句柄，`publish`函数还需要知道要发送的 ROS 消息。

    publish(PublisherHandle publisher_handle, .. ros_message);

Since ROS messages do not have a common base class the signature of the function can not use a known type for the passed ROS message. Instead it is passed as a void pointer and the actual implementation reinterprets it according to the previously registered type.

> 由于 ROS 消息没有共同的基类，函数的签名不能使用已知类型传递 ROS 消息。相反，它被传递为一个 void 指针，实际实现根据先前注册的类型重新解释它。

#### Type of the returned handles

The returned handles need to encapsulate arbitrary content for each middleware implementation. Therefore these handles are just opaque objects from a user point of view. Only the middleware implementation which created it knows how to interpret it.

> 返回的句柄需要为每个中间件实现封装任意内容。因此，从用户的角度来看，这些句柄只是不透明的对象。只有创建它的中间件实现才知道如何解释它。

In order to ensure that these information are passed back to the same middleware implementation each handle encodes a unique identifier which the middleware implementation can check before interpreting the handles payload.

> 为了确保这些信息被传回到同一个中间件实现，每个句柄编码了一个唯一的标识符，中间件实现可以在解释句柄有效负载之前检查这个标识符。

### Subscriber interface

The details of the interface necessary for the subscriber side are not (yet) described in this document.

> 本文档尚未描述订阅者端所需的接口细节。

### Optionally exposing native handles

The RMW interface only exposes middleware agnostic handles. But the middleware implementation can optionally provide additional API to provide native handles. E.g. for a given ROS publisher handle a specific implementation can provide an API to access publisher related handles specific to the implementation.

> RMW 界面只暴露中间件无关的句柄。但中间件实现可以选择性地提供额外的 API 来提供本机句柄。例如，对于给定的 ROS 发布者句柄，特定实现可以提供 API 来访问与实现相关的句柄。

While using such a feature would make the user land code specific to the middleware implementation it provides a way to use features of a middleware implementation which are not exposed through the ROS interface. See [this](https://github.com/ros2/demos/tree/master/demo_nodes_cpp_native) demo for an example.

> 使用此功能可以让用户在中间件实现中定位代码，它提供了一种使用中间件实现的功能的方法，而这些功能并未通过 ROS 接口暴露出来。有关示例，请参见[此处](https://github.com/ros2/demos/tree/master/demo_nodes_cpp_native)。

## Current implementation

The described concept has been implemented in the following packages:

> 这个概念已经在以下包中实现：

- The package [rmw](https://github.com/ros2/rmw/tree/master/rmw) defines the middleware interface.

  - The functions are declared in [rms/rmw.h](https://github.com/ros2/rmw/blob/master/rmw/include/rmw/rmw.h).
  - The handles are defined in [rmw/types.h](https://github.com/ros2/rmw/blob/master/rmw/include/rmw/types.h).

- The package [rosidl_typesupport_introspection_cpp](https://github.com/ros2/rosidl/tree/master/rosidl_typesupport_introspection_cpp) generates code which encapsulated the information from each ROS msg file in a way which makes the data structures introspectable from C++ code.

- The package [rmw_fastrtps_cpp](https://github.com/ros2/rmw_fastrtps/tree/master/rmw_fastrtps_cpp) implements the middleware interface using [eProsima Fast-RTPS](http://www.eprosima.com/index.php/products-all/eprosima-fast-rtps) based on the introspection type support.

- The package [rosidl_generator_dds_idl](https://github.com/ros2/rosidl_dds/tree/master/rosidl_generator_dds_idl) generates DDS IDL files based on ROS msg files which are being used by all DDS-based RMW implementations which use static / compile type message types.

- The package [rmw_connext_cpp](https://github.com/ros2/rmw_connext/tree/master/rmw_connext_cpp) implements the middleware interface using [RTI Connext DDS](http://www.rti.com/products/dds/index.html) based on statically generated code.

  - The package [rosidl_typesupport_connext_cpp](https://github.com/ros2/rmw_connext/tree/master/rosidl_typesupport_connext_cpp) generates:

    - the DDS specific code based on IDL files for each message
    - additional code to enable invoking the register/create/convert/write functions for each message type

- The package [rmw_connext_dynamic_cpp](https://github.com/ros2/rmw_connext/tree/master/rmw_connext_dynamic_cpp) implements the middleware interface using _RTI Connext DDS_ based on the `DynamicData` API of Connext and the introspection type support.

- The package [rmw_opensplice_cpp](https://github.com/ros2/rmw_opensplice/tree/master/rmw_opensplice_cpp) implements the middleware interface using [PrismTech OpenSplice DDS](http://www.prismtech.com/opensplice) based on statically generated code.

  - The package [rosidl_typesupport_opensplice_cpp](https://github.com/ros2/rmw_opensplice/tree/master/rosidl_typesupport_opensplice_cpp) generates:

    - the DDS specific code based on IDL files for each message
    - additional code to enable invoking the register/create/convert/write functions for each message type

- The package [rmw_implementation](https://github.com/ros2/rmw_implementation/tree/master/rmw_implementation) provides the mechanism to switch between compile time and runtime selection of the middleware implementation.

  - If only one implementation is available at compile time it links directly against it.
  - If multiple implementations are available at compile time it implements the middleware interface itself and acts according to the strategy pattern by loading the shared library of a specific middleware implementation identified by an environment variable at runtime and pass all calls along.

### One or multiple type support generators

The packages contributing to the message generation process of each message are called _type support generators_. Their package name starts with the prefix `rosidl_typesupport_`.

> 每条消息的消息生成过程中贡献的包称为*类型支持生成器*。它们的包名以`rosidl_typesupport_`为前缀。

Each message package will contain the generated code from all type support generators which are available when the package is configured. This can be only one (when building against a single middleware implementation) or multiple type support generators.

> 每个消息包将包含在配置包时可用的所有类型支持生成器生成的代码。这可以是一个(在针对单个中间件实现构建时)或多个类型支持生成器。

### Mapping between DDS and ROS concepts

Every ROS node is one DDS participant. If multiple ROS nodes are being run in a single process they are still mapped to separate DDS participants. If the containing process exposes its own ROS interface (e.g. to load nodes into the process at runtime) it is acting as a ROS node itself and is therefore also mapped to a separate DDS participant.

> 每个 ROS 节点都是一个 DDS 参与者。如果在单个进程中运行多个 ROS 节点，它们仍然会映射到不同的 DDS 参与者。如果包含进程公开其自己的 ROS 接口(例如，在运行时加载节点)，它就是作为一个 ROS 节点本身而行动，因此也映射到一个单独的 DDS 参与者。

The ROS publishers and subscribers are mapped to DDS publishers and subscribers. The DDS DataReader and DataWriter as well as DDS topics are not exposed through the ROS API.

> ROS 发布者和订阅者映射到 DDS 发布者和订阅者。DDS DataReader 和 DataWriter 以及 DDS 主题不通过 ROS API 公开。

The ROS API defines queue sizes and a few [Quality of Service parameters](http://design.ros2.org/articles/qos.html) which are being mapped to their DDS equivalent. The other DDS QoS parameters are not being exposed through the ROS API.

> ROS API 定义了队列大小和一些[服务质量参数](http://design.ros2.org/articles/qos.html)，它们被映射到它们的 DDS 等效物。ROS API 不暴露其他 DDS QoS 参数。
