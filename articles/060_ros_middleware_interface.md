---
tip: translate by openai@2023-05-28 11:12:50
...
---
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

## The *middleware interface*

### Why does ROS 2 have a *middleware interface*


The ROS client library defines an API which exposes communication concepts like publish / subscribe to users.

> ROS 客户端库定义了一个 API，它向用户暴露发布/订阅等通信概念。


In ROS 1 the implementation of these communication concepts was built on custom protocols (e.g., [TCPROS](http://wiki.ros.org/ROS/TCPROS)).

> 在ROS 1中，这些通信概念的实现是基于自定义协议（例如[TCPROS](http://wiki.ros.org/ROS/TCPROS)）构建的。


For ROS 2 the decision has been made to build it on top of an existing middleware solution (namely [DDS](http://en.wikipedia.org/wiki/Data_Distribution_Service)).

> 对于ROS 2，我们已经决定基于现有的中间件解决方案（即[DDS](http://en.wikipedia.org/wiki/Data_Distribution_Service)）进行构建。

The major advantage of this approach is that ROS 2 can leverage an existing and well developed implementation of that standard.

> 这种方法的主要优点是ROS 2可以利用现有和完善的标准实现。


ROS could build on top of one specific implementation of DDS.

> ROS 可以建立在特定的DDS实现之上。

But there are numerous different implementations available and each has its own pros and cons in terms of supported platforms, programming languages, performance characteristics, memory footprint, dependencies and licensing.

> 但是有许多不同的实现可用，每个实现在支持的平台、编程语言、性能特征、内存占用、依赖关系和许可证方面都有其自身的优缺点。


Therefore ROS aims to support multiple DDS implementations despite the fact that each of them differ slightly in their exact API.

> 因此，尽管每个DDS实现的API稍有不同，但ROS仍旧旨在支持多种DDS实现。

In order to abstract from the specifics of these APIs, an abstract interface is being introduced which can be implemented for different DDS implementations.

> 为了抽象出这些API的具体内容，引入了一个抽象接口，可以用于不同的DDS实现。

This *middleware interface* defines the API between the ROS client library and any specific implementation.

> 这个中间件接口定义了ROS客户端库和任何特定实现之间的API。


Each implementation of the interface will usually be a thin adapter which maps the generic *middleware interface* to the specific API of the middleware implementation.

> 每个接口的实现通常都是一个薄适配器，它将通用的中间件接口映射到中间件实现的特定API。

In the following the common separation of the adapter and the actual middleware implementation will be omitted.

> 在接下来的内容中，适配器和实际的中间件实现的常见分离将被忽略。

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

## Why should the *middleware interface* be agnostic to DDS


The ROS client library should not expose any DDS implementation specifics to the user.

> ROS 客户端库不应向用户暴露任何 DDS 实现细节。

This is primarily to hide the intrinsic complexity of the DDS specification and API.

> 这主要是为了隐藏DDS规范和API的内在复杂性。


While ROS 2 only aims to support DDS based middleware implementations it can strive to keep the *middleware interface* free of DDS specific concepts to enable implementations of the interface using a different middleware.

> 尽管ROS 2只旨在支持基于DDS的中间件实现，但它可以努力使中间件接口不受DDS特定概念的限制，以便使用不同的中间件实现该接口。

It would also be feasible to implement the interface by tying together several unrelated libraries providing the necessary functions of discovery, serialization and publish / subscribe.

> 也可以通过将提供必要发现、序列化和发布/订阅功能的几个不相关的库捆绑在一起来实现该接口。

    +-----------------------------------+
    |             user land             |   no middleware implementation specific code
    +-----------------------------------+
    |        ROS client library         |   above the interface
    +-----------------------------------+
    |       middleware interface        |   ---
    +-----------------------------------+
    | mw impl 1 | mw impl 2 | mw impl 3 |
    +-----------+-----------+-----------+

## How does the information flow through the *middleware interface*


One goal of the *middleware interface* is to not expose any DDS specific code to the user land code.

> 中间件接口的一个目标是不向用户层代码暴露任何DDS特定的代码。

Therefore the ROS client library "above" the *middleware interface* needs to only operate on ROS data structures.

> 因此，ROS客户端库必须只在中间件接口上操作ROS数据结构。

ROS 2 will continue to use ROS message files to define the structure of these data objects and derive the data structures for each supported programming language from them.

> ROS 2 仍然会使用 ROS 消息文件来定义这些数据对象的结构，并从中为每种支持的编程语言派生出数据结构。


The middleware implementation "below" the *middleware interface* must convert the ROS data objects provided from the client library into its own custom data format before passing it to the DDS implementation.

> 中间件实现必须在中间件接口*之下*将客户端库提供的ROS数据对象转换为自定义数据格式，然后才能将其传递给DDS实现。

In reverse custom data objects coming from the DDS implementation must be converted into ROS data objects before being returned to the ROS client library.

> 从DDS实现返回的自定义数据对象必须在返回给ROS客户端库之前转换成ROS数据对象。


The definition for the middleware specific data types can be derived from the information specified in the ROS message files.

> 可以从ROS消息文件中指定的信息中导出中间件特定数据类型的定义。

A defined mapping between the primitive data types of ROS message and middleware specific data types ensures that a bidirectional conversion is possible.

> 一种在ROS消息的原始数据类型和中间件特定数据类型之间的定义映射，确保可以进行双向转换。

The functionality to convert between ROS types and the implementation specific types or API is encapsulated in the `type support` (see below for different kind of type supports).

> 功能用于在ROS类型和特定实现类型或API之间进行转换，被封装在`类型支持`中（参见下面的不同类型支持）。

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

> 根据中间件实现，可以通过直接从ROS消息实现序列化函数以及反序列化函数到ROS消息来避免额外的转换。

## Considered use cases


The following use cases have been considered when designing the middleware interface:

> 在设计中间件接口时，已经考虑了以下用例：

### Single middleware implementation


ROS applications are not built in a monolithic way but distributed across several packages.

> ROS应用程序不是采用单一方式构建，而是分布在几个软件包中。

Even with a *middleware interface* in place the decision of which middleware implementation to use will affect significant parts of the code.

> 即使有一个中间件接口，选择哪种中间件实现将会影响代码的重要部分。


For example, a package defining a ROS message will need to provide the mapping to and from the middleware specific data type.

> 例如，定义ROS消息的包将需要提供与中间件特定数据类型之间的映射。

Naively each package defining ROS messages might contain custom (usually generated) code for the specific middleware implementation.

> 每个定义ROS消息的包可能包含用于特定中间件实现的自定义（通常是生成的）代码，这是一种天真的做法。


In the context of providing binary packages of ROS (e.g., Debian packages) this implies that a significant part of them (at least all packages containing message definitions) would be specific to the selected middleware implementation.

> 在提供ROS的二进制包（例如Debian包）的上下文中，这意味着其中的大部分（至少所有包含消息定义的包）将特定于所选中间件实现。

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

> DDS有两种不同的方式来使用和与消息交互。


On the one hand the message can be specified in an IDL file from which usually a DDS implementation specific program will generate source code.

> 一方面，消息可以在IDL文件中指定，通常DDS实现特定的程序将生成源代码。

The generated code for C++, e.g., contains types specifically generated for the message.

> 生成的C++代码，例如，包含专门为消息生成的类型。


On the other hand the message can be specified programmatically using the DynamicData API of the [XTypes](http://www.omg.org/spec/DDS-XTypes/) specification.

> 另一方面，消息可以使用[XTypes](http://www.omg.org/spec/DDS-XTypes/)规范的DynamicData API编程指定。

Neither an IDL file nor a code generation step is necessary for this case.

> 在这种情况下，既不需要IDL文件，也不需要代码生成步骤。


Some custom code must still map the message definition available in the ROS .msg files to invocations of the DynamicData API.

> 一些自定义代码仍然需要将ROS .msg文件中可用的消息定义映射到DynamicData API的调用。

But it is possible to write generic code which performs the task for any ROS .msg specification passed in.

> 但是可以编写通用代码，用于执行传入的任何ROS .msg规范的任务。

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

> 然而，使用DynamicData API的性能很可能总是低于静态生成的代码。

### Switch between different implementations


When ROS supports different middleware implementations it should be as easy and low effort as possible for users to switch between them.

> 当ROS支持不同的中间件实现时，用户在它们之间切换应该尽可能简单且低努力。

#### Decide at compile time


One obvious way will be for a user to build all ROS packages from source selecting a specific middleware implementation.

> 一种明显的方法是用户可以从源代码中构建所有ROS包，选择特定的中间件实现。

While the workflow won't be too difficult (probably half a dozen command-line invocations), it still requires quite some build time.

> 流程虽然不会太困难（可能只需要几个命令行调用），但仍然需要相当长的构建时间。


To avoid the need to build from source a set of binary packages could be provided which chose the middleware implementation at build time.

> 为了避免需要从源代码构建，可以提供一组二进制包，在构建时选择中间件实现。

While this would reduce the effort for the user the buildfarm would need to build a completely separate set of binary packages.

> 这样可以减少用户的工作量，但是构建农场需要构建一整套完全不同的二进制软件包。

The effort to support N packages with M different middleware implementations would require significant resources (M * N) for maintaining the service as well as the necessary computing power.

> 为了支持N个包和M种不同的中间件实现，需要投入大量资源（M*N）来维护服务和必要的计算能力。

#### Decide at runtime


The alternative is to support selecting a specific middleware implementation at runtime.

> 另一种选择是在运行时支持选择特定的中间件实现。

This requires that the to be selected middleware implementation was available at compile time in order for the middleware specific type support to be generated for every message package.

> 这需要在编译时可用的被选中的中间件实现，以便为每个消息包生成特定类型的支持。


When building ROS with a single middleware implementation the result should follow the design criteria:

> 当使用单一的中间件实现构建ROS时，结果应遵循设计准则：

> Any features that you do not use you do not pay for.


This implies that there should be no overhead for neither the build time nor the runtime due to the ability to support different middleware implementations.

> 这意味着，由于支持不同的中间件实现，构建时间和运行时都不会有额外的开销。

However the additional abstraction due to the middleware interface is still valid in order to hide implementation details from the user.

> 然而，由于中间件接口带来的额外抽象仍然有效，可以隐藏实现细节对用户。

## Design of the *middleware interface*


The API is designed as a pure function-based interface in order to be implemented in C.

> API被设计成一个纯函数式接口，以便在C中实现。

A pure C interface can be used in ROS Client Libraries for most other languages including Python, Java, and C++ preventing the need to reimplement the core logic.

> 一个纯C接口可以用在ROS客户端库中，包括Python、Java和C++，避免重新实现核心逻辑。

### Publisher interface


Based on the general structure of ROS nodes, publishers and messages for the case of publishing messages the ROS client library need to invoke three functions on the middleware interface:

> 根据ROS节点的一般结构，对于发布消息的情况，ROS客户端库需要在中间件接口上调用三个函数：

    - `create_node()`
    - `create_publisher()`
    - `publish()`

#### Essential signature of `create_node`


Subsequent invocations of `create_publisher` need to refer to the specific node they should be created in.

> 下一次调用`create_publisher`时需要指明应该在哪个特定节点上创建。

Therefore the `create_node` function needs to return a *node handle* which can be used to identify the node.

> 因此，`create_node`函数需要返回一个*节点句柄*，用于标识该节点。

    NodeHandle create_node();

#### Essential signature of `create_publisher`


Besides the *node handle* the `create_publisher` function needs to know the *topic name* as well as the *topic type*.

> 除了*节点句柄*之外，`create_publisher`函数还需要知道*主题名称*以及*主题类型*。

The type of the *topic type* argument is left unspecified for now.

> 这个*主题类型*参数的类型暂时未指定。


Subsequent invocations of `publish` need to refer to the specific publisher they should send messages on.

> 后续调用`publish`需要指定它们应该发送消息的具体发布者。

Therefore the `create_publisher` function needs to return a *publisher handle* which can be used to identify the publisher.

> 因此，`create_publisher`函数需要返回一个*出版商句柄*，用于标识出版商。

    ```cpp
    PublisherHandle create_publisher(NodeHandle node_handle, String topic_name, .. topic_type);
    ```


The information encapsulated by the *topic type* argument is highly dependent on the middleware implementation.

> 这个*主题类型*参数封装的信息高度依赖中间件实现。

##### Topic type information for the DynamicData API


In the case of using the DynamicData API in the implementation there is no C / C++ type which could represent the type information.

> 在使用DynamicData API实现的情况下，没有C/C++类型可以表示类型信息。

Instead the *topic type* must contain all information needed to describe the format of the message.

> 代替，*主题类型*必须包含描述消息格式所需的所有信息。

This information includes:

> 这些信息包括：


- the name of the package in which the message is defined

> 名称为消息定义的包

- the name of the message

> 这条消息的名称

- the list of fields of the message where each includes:

> 消息的字段列表，每个字段包括：


  - the name for the message field

> 消息字段的名称

  - the type of the message field (which can be either a built-in type or another message type), optionally the type might be an unbounded, bounded or fixed size array

> 类型消息字段（可以是内置类型或其他消息类型），可选择无界、有界或固定大小的数组。

  - the default value

> 默认值


- the list of constants defined in the message (again consisting of name, type and value)

> 列出在消息中定义的常量（包括名称、类型和值）


In the case of using DDS this information enables one to:

> 在使用DDS的情况下，这些信息可以使人们：


- programmatically create a *DDS TypeCode* which represents the message structure

> .

程序性地创建一个代表消息结构的DDS TypeCode。

- register the *DDS TypeCode* with the *DDS Participant*

> 将DDS TypeCode注册到DDS参与者中

- create a *DDS Publisher*, *DDS Topic* and *DDS DataWriter*

> 创建一个DDS发布者、DDS主题和DDS数据写入器。

- convert data from a *ROS message* into a *DDS DynamicData* instance

> 将数据从*ROS消息*转换为*DDS DynamicData*实例

- write the *DDS DynamicData* to the *DDS DataWriter*

> 将DDS DynamicData写入DDS DataWriter

##### Topic type information for statically generated code


In the case of using statically generated code derived from an IDL there is are C / C++ types which represent the type information.

> 在使用从IDL生成的静态代码的情况下，有一些C/C++类型来表示类型信息。

The generated code contains functions to:

> 生成的代码包含函数：


- create a *DDS TypeCode* which represents the message structure

> 创建一个表示消息结构的DDS TypeCode。


Since the specific types must be defined at compile time the other functionalities can not be implemented in a generic (not specific to the actual message) way.

> 由于特定类型必须在编译时定义，因此其他功能不能以通用（不特定于实际消息）的方式实现。

Therefore for each message the code to perform the following tasks must be generated separately:

> 因此，必须为每条消息单独生成代码来执行以下任务：


- register the *DDS TypeCode* with the *DDS Participant*

> 将DDS TypeCode注册到DDS参与者上

- create a *DDS Publisher*, *DDS Topic* and *DDS DataWriter*

> 创建一个DDS发布者、DDS主题和DDS数据写入器。

- convert data from a *ROS message* into a *DDS DynamicData* instance

> 将数据从ROS消息转换为DDS DynamicData实例

- write the *DDS DynamicData* to the *DDS DataWriter*

> 将*DDS DynamicData*写入*DDS DataWriter*


The information encapsulated by the *topic type* must include the function pointers to invoke these functions.

> 所封装在*主题类型*中的信息必须包括用于调用这些函数的函数指针。

#### `get_type_support_handle`


Since the information encapsulated by the *topic type* argument is so fundamentally different for each middleware implementation it is actually retrieved through an additional function of the middleware interface:

> 由于由*主题类型*参数封装的信息对每个中间件实现都是如此根本不同，因此实际上是通过中间件接口的额外功能来检索它的。

    MessageTypeSupportHandle get_type_support_handle();


Currently this function is a template function specialized on the specific ROS message type.

> 目前，此功能是一个专门针对特定ROS消息类型的模板函数。

For C compatibility an approach using a macro will be used instead which mangles the type name into the function name.

> .

为了兼容C，将使用宏来实现，宏将类型名称映射到函数名称中。

#### Essential signature of `publish`


Beside the *publisher handle* the `publish` function needs to know the *ROS message* to send.

> 除了发布者句柄，发布函数还需要知道要发送的ROS消息。

    publish(PublisherHandle publisher_handle, .. ros_message);


Since ROS messages do not have a common base class the signature of the function can not use a known type for the passed ROS message.

> 由于ROS消息没有公共的基类，函数的签名无法使用已知类型来传递ROS消息。

Instead it is passed as a void pointer and the actual implementation reinterprets it according to the previously registered type.

> 它被传递为一个空指针，而实际实现根据之前注册的类型重新解释它。

#### Type of the returned handles


The returned handles need to encapsulate arbitrary content for each middleware implementation.

> 返回的句柄需要为每个中间件实现封装任意内容。

Therefore these handles are just opaque objects from a user point of view.

> 因此，从用户的角度来看，这些句柄只是不透明的对象。

Only the middleware implementation which created it knows how to interpret it.

> 只有创建它的中间件实现才知道如何解释它。


In order to ensure that these information are passed back to the same middleware implementation each handle encodes a unique identifier which the middleware implementation can check before interpreting the handles payload.

> 为了确保这些信息被传回同一中间件实现，每个句柄都编码一个唯一的标识符，中间件实现可以在解释句柄有效负载之前检查这个标识符。

### Subscriber interface


The details of the interface necessary for the subscriber side are not (yet) described in this document.

> 此文档尚未描述订户端所需的接口细节。

### Optionally exposing native handles


The RMW interface only exposes middleware agnostic handles.

> RMW接口仅暴露中间件无关的句柄。

But the middleware implementation can optionally provide additional API to provide native handles.

> 但是中间件实现可以可选地提供额外的API来提供本地句柄。

E.g. for a given ROS publisher handle a specific implementation can provide an API to access publisher related handles specific to the implementation.

> 例如，对于给定的ROS发布者句柄，特定实现可以提供一个API来访问与该实现相关的发布者句柄。


While using such a feature would make the user land code specific to the middleware implementation it provides a way to use features of a middleware implementation which are not exposed through the ROS interface.

> 使用这样的功能可以使用户降落到与中间件实现相关的代码，它提供了一种通过ROS接口无法访问中间件实现功能的方法。

See [this](https://github.com/ros2/demos/tree/master/demo_nodes_cpp_native) demo for an example.

> 看[这个](https://github.com/ros2/demos/tree/master/demo_nodes_cpp_native)演示作为一个例子。

## Current implementation


The described concept has been implemented in the following packages:

> 该概念已在以下软件包中实现：


- The package [rmw](https://github.com/ros2/rmw/tree/master/rmw) defines the middleware interface.

> - 包[rmw](https://github.com/ros2/rmw/tree/master/rmw)定义了中间件接口。


  - The functions are declared in [rms/rmw.h](https://github.com/ros2/rmw/blob/master/rmw/include/rmw/rmw.h).

> 函数在[rms/rmw.h](https://github.com/ros2/rmw/blob/master/rmw/include/rmw/rmw.h)中声明。

  - The handles are defined in [rmw/types.h](https://github.com/ros2/rmw/blob/master/rmw/include/rmw/types.h).

> 手柄定义在[rmw/types.h](https://github.com/ros2/rmw/blob/master/rmw/include/rmw/types.h)中。


- The package [rosidl_typesupport_introspection_cpp](https://github.com/ros2/rosidl/tree/master/rosidl_typesupport_introspection_cpp) generates code which encapsulated the information from each ROS msg file in a way which makes the data structures introspectable from C++ code.

> - [rosidl_typesupport_introspection_cpp](https://github.com/ros2/rosidl/tree/master/rosidl_typesupport_introspection_cpp) 包生成的代码，将每个ROS消息文件中的信息封装起来，使得从C++代码中可以对数据结构进行内省。


- The package [rmw_fastrtps_cpp](https://github.com/ros2/rmw_fastrtps/tree/master/rmw_fastrtps_cpp) implements the middleware interface using [eProsima Fast-RTPS](http://www.eprosima.com/index.php/products-all/eprosima-fast-rtps) based on the introspection type support.

> - 该包[rmw_fastrtps_cpp](https://github.com/ros2/rmw_fastrtps/tree/master/rmw_fastrtps_cpp)使用基于内省类型支持的[eProsima Fast-RTPS](http://www.eprosima.com/index.php/products-all/eprosima-fast-rtps)实现中间件接口。


- The package [rosidl_generator_dds_idl](https://github.com/ros2/rosidl_dds/tree/master/rosidl_generator_dds_idl) generates DDS IDL files based on ROS msg files which are being used by all DDS-based RMW implementations which use static / compile type message types.

> - [rosidl_generator_dds_idl](https://github.com/ros2/rosidl_dds/tree/master/rosidl_generator_dds_idl) 包根据ROS消息文件生成DDS IDL文件，这些文件被所有使用静态/编译类型消息类型的基于DDS的RMW实现所使用。


- The package [rmw_connext_cpp](https://github.com/ros2/rmw_connext/tree/master/rmw_connext_cpp) implements the middleware interface using [RTI Connext DDS](http://www.rti.com/products/dds/index.html) based on statically generated code.

> - 该软件包[rmw_connext_cpp](https://github.com/ros2/rmw_connext/tree/master/rmw_connext_cpp)使用[RTI Connext DDS](http://www.rti.com/products/dds/index.html)基于静态生成代码实现中间件接口。


  - The package [rosidl_typesupport_connext_cpp](https://github.com/ros2/rmw_connext/tree/master/rosidl_typesupport_connext_cpp) generates:

> - [rosidl_typesupport_connext_cpp](https://github.com/ros2/rmw_connext/tree/master/rosidl_typesupport_connext_cpp)包生成：

    - the DDS specific code based on IDL files for each message
    - additional code to enable invoking the register/create/convert/write functions for each message type


- The package [rmw_connext_dynamic_cpp](https://github.com/ros2/rmw_connext/tree/master/rmw_connext_dynamic_cpp) implements the middleware interface using *RTI Connext DDS* based on the ``DynamicData`` API of Connext and the introspection type support.

> - 该软件包[rmw_connext_dynamic_cpp](https://github.com/ros2/rmw_connext/tree/master/rmw_connext_dynamic_cpp)使用基于Connext的``DynamicData`` API和内省类型支持实现了基于RTI Connext DDS的中间件接口。


- The package [rmw_opensplice_cpp](https://github.com/ros2/rmw_opensplice/tree/master/rmw_opensplice_cpp) implements the middleware interface using [PrismTech OpenSplice DDS](http://www.prismtech.com/opensplice) based on statically generated code.

> - 该软件包[rmw_opensplice_cpp](https://github.com/ros2/rmw_opensplice/tree/master/rmw_opensplice_cpp)使用[PrismTech OpenSplice DDS](http://www.prismtech.com/opensplice)基于静态生成的代码实现中间件接口。


  - The package [rosidl_typesupport_opensplice_cpp](https://github.com/ros2/rmw_opensplice/tree/master/rosidl_typesupport_opensplice_cpp) generates:

> - 该包[rosidl_typesupport_opensplice_cpp](https://github.com/ros2/rmw_opensplice/tree/master/rosidl_typesupport_opensplice_cpp)生成：

    - the DDS specific code based on IDL files for each message
    - additional code to enable invoking the register/create/convert/write functions for each message type


- The package [rmw_implementation](https://github.com/ros2/rmw_implementation/tree/master/rmw_implementation) provides the mechanism to switch between compile time and runtime selection of the middleware implementation.

> - 这个[rmw_implementation](https://github.com/ros2/rmw_implementation/tree/master/rmw_implementation) 包提供了在编译时和运行时切换中间件实现的机制。


  - If only one implementation is available at compile time it links directly against it.

> 如果只有一个实现可以在编译时使用，它会直接链接到它。

  - If multiple implementations are available at compile time it implements the middleware interface itself and acts according to the strategy pattern by loading the shared library of a specific middleware implementation identified by an environment variable at runtime and pass all calls along.

> 如果在编译时可用多个实现，它会自己实现中间件接口，并根据策略模式在运行时加载由环境变量标识的特定中间件实现的共享库，并将所有调用传递下去。

### One or multiple type support generators


The packages contributing to the message generation process of each message are called *type support generators*.

> 每条消息的消息生成过程中贡献的包称为*类型支持生成器*。

Their package name starts with the prefix `rosidl_typesupport_`.

> 他们的包名以`rosidl_typesupport_`为前缀。


Each message package will contain the generated code from all type support generators which are available when the package is configured.

> 每个消息包都将包含在配置包时可用的所有类型支持生成器生成的代码。

This can be only one (when building against a single middleware implementation) or multiple type support generators.

> 这可以是针对单一中间件实现的唯一类型支持生成器，也可以是多种类型支持生成器。

### Mapping between DDS and ROS concepts


Every ROS node is one DDS participant.

> 每个ROS节点都是一个DDS参与者。

If multiple ROS nodes are being run in a single process they are still mapped to separate DDS participants.

> 如果在一个进程中运行多个ROS节点，它们仍然会映射到不同的DDS参与者。

If the containing process exposes its own ROS interface (e.g. to load nodes into the process at runtime) it is acting as a ROS node itself and is therefore also mapped to a separate DDS participant.

> 如果包含进程公开其自己的ROS接口（例如在运行时加载节点），它就像一个ROS节点，因此也映射到一个单独的DDS参与者。


The ROS publishers and subscribers are mapped to DDS publishers and subscribers.

> ROS发布者和订阅者映射到DDS发布者和订阅者。

The DDS DataReader and DataWriter as well as DDS topics are not exposed through the ROS API.

> DDS的DataReader和DataWriter以及DDS主题不能通过ROS API暴露。


The ROS API defines queue sizes and a few [Quality of Service parameters](http://design.ros2.org/articles/qos.html) which are being mapped to their DDS equivalent.

> ROS API 定义了队列大小和一些[服务质量参数](http://design.ros2.org/articles/qos.html)，这些参数被映射到它们的DDS等效物。

The other DDS QoS parameters are not being exposed through the ROS API.

> 其他DDS QoS参数没有通过ROS API暴露出来。
