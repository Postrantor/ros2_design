---
tip: translate by openai@2023-05-29 09:08:48
...
---
    layout: default
    title: ROS 2 Message Research
    abstract:
      This article captures the research done in regards to the serialization component, including an overview of the current implementation in ROS 1 and the alternatives for ROS 2.
    published: true
    author: '[Dirk Thomas](https://github.com/dirk-thomas) and [Esteve Fernandez](https://github.com/esteve)'
    date_written: 2013-12
    last_modified: 2019-05
    Authors: {{ page.author }}
    Date Written: {{ page.date_written }}
    Last Modified: {% if page.last_modified %}{{ page.last_modified }}{% else %}{{ page.date_written }}{% endif %}
---


This document pre-dates the decision to build ROS 2 on top of DDS. This is an exploration of possible message interfaces and the relation of the underlying message serialization. This paper is focused on specifying the message API and designing the integration with the serialization with performance as well as flexibility in mind. It is expected that there are one or more message serialization implementations which can be used, such as Protobuf, MessagePack and Thrift.

> 这份文件早于建立ROS 2在DDS之上的决定。这是对可能的消息接口以及底层消息序列化之间的关系的探索。本文旨在指定消息API并设计与序列化的集成，同时考虑性能和灵活性。预计可以使用一种或多种消息序列化实现，例如Protobuf，MessagePack和Thrift。

## Background


The messages are a crucial part of ROS since they are the interface between functional components and are exposed in every userland code. A future change of this API would require a significant amount of work. So a very important goal is to make the message interface flexible enough to be future-proof.

> 消息是ROS的重要组成部分，因为它们是功能组件之间的接口，并且在每个用户层代码中都有所体现。未来对此API的更改将需要大量的工作。因此，一个非常重要的目标是使消息接口足够灵活，以便未来可以免受影响。

## Existing Implementations


ROS 1 messages are data objects which use member-based access to the message fields. While the message specification is not very feature rich the serializer is pretty fast. The ROS distribution contains message serializers implemented in C++, Python and Lisp. Besides that the community provided implementations for other languages, like C, Ruby, MatLab, etc.

> ROS 1 消息是使用基于成员的访问消息字段的数据对象。虽然消息规范不是很丰富，但序列化器非常快。ROS 分发包含了用 C++、Python 和 Lisp 实现的消息序列化器。此外，社区还为其他语言（如 C、Ruby、MatLab 等）提供了实现。


Other existing serialization libraries provide more features and are tailored for specific needs ranging from small memory footprint over small wire protocol to low performance impact. For the usage on a small embedded device the constraints regarding the programming language and the available resources is very different from when being used on a desktop computer. Similar depending on the network connectivity the importance of the size of the wire format varies. The needs might even be different within one ROS graph but different entities.

> 其他现有的序列化库提供更多功能，并针对从小内存足迹到小线路协议到低性能影响的特定需求进行定制。对于小型嵌入式设备的使用，关于编程语言和可用资源的约束与在台式计算机上使用时非常不同。类似地，根据网络连接性，线路格式的大小的重要性也有所不同。ROS图中的不同实体的需求甚至可能不同。

## Areas for Improvement

### Future-proof API


Due to the broad domains where ROS is being (and will be) used and their different requirements and constraints we could not identify a single serialization library which matches all of them perfectly well. It is also likely that in the near future more libraries with new advantages (and disadvantages) will come up. So in order to design the message API in a future-proof manner it should not expose the serialization library used but make the actually used serialization library an implementation detail.

> 由于ROS正在（并将来）使用的广泛领域及其不同的要求和限制，我们无法确定一个完全符合所有要求的序列化库。未来可能会出现更多具有新优势（和劣势）的库。因此，为了以未来为导向地设计消息API，应将实际使用的序列化库作为实现细节而不是暴露出来。

### Serialization should be optional


With the goal to dynamically choose between the former node and nodelet style of composing a system the important the amount of scenarios where messages are actually serialized (rather than passed by reference) is likely to decrease. Therefore it would be good if no serialization library needs to be linked if the functionality is not used at all (e.g. on a self-contained product without external connections). This approach would also encourage a clean modular design.

> 为了动态地选择前节点和节点样式来组成系统，重要的是减少实际序列化消息（而不是通过引用传递）的场景数量。因此，如果不使用功能（例如在没有外部连接的自包含产品上），则最好不要链接序列化库。这种方法也将鼓励清晰的模块化设计。

### Use existing library


In order to reduce the future maintenance effort existing libraries should be used in order not to specify and implement yet another wire protocol.

> 为了减少未来的维护工作，应该使用现有的库，而不是另外指定和实施另一种线路协议。

### Support more features

#### Fixed length messages


Optional variant of a message which avoids dynamic memory allocation e.g. in real time systems. Since this use case implies severe constraints that are not optimal for scenarios where dynamic memory allocation is feasible this should not limit the solution but should be provided as an alternative implementation.

> 可选的消息变体，可以避免在实时系统中使用动态内存分配。由于这种用例意味着严格的限制，不适用于可以使用动态内存分配的场景，因此这不应该限制解决方案，而应该提供一种替代实现。

#### Optional fields, default values


In ROS 1, messages and services require all data members and arguments to be specified. By using optional fields and default values, we can define simpler APIs so that users\' code can be more succinct and more readable. At the same time we could also provide sane values for certain APIs, such as for sensors.

> 在ROS 1中，消息和服务需要指定所有数据成员和参数。通过使用可选字段和默认值，我们可以定义更简单的API，以使用户的代码更加简洁易读。同时，我们也可以为某些API提供合理的值，例如传感器。

#### Additional field types: dictionary


Dictionaries or maps are widely used in several programming languages (e.g. Python), thanks to being built-in data types. However, in order to support dictionaries in as many languages as we can, we have to take into consideration whether all languages provide mechanisms for supporting them. Certain semantics will have to be considered in the IDL, such as what datatypes can be used as keys. This feature would also imply backwards-incompatible changes to the ROS IDL.

> 字典或映射在许多编程语言（例如Python）中被广泛使用，这要归功于它们作为内置数据类型。但是，为了支持尽可能多的语言的字典，我们必须考虑所有语言是否都提供支持它们的机制。在IDL中必须考虑某些语义，例如哪些数据类型可以用作键。此功能还将意味着ROS IDL的向后不兼容的更改。

## Considerations

### Member-based vs. method-based access


A message interface which utilizes member-based access to the message fields is a straightforward API. By definition each message object stores its data directly in its members which implies the lowest overhead at runtime. These fields are then serialized directly into the wire format. (This does not imply that the message is a POD - depending on the used field types it can not be mem-copied.)

> 一种利用基于成员的访问消息字段的消息界面是一个直接的API。根据定义，每个消息对象直接在其成员中存储其数据，这意味着在运行时具有最低的开销。然后，这些字段将直接序列化到线路格式。（这并不意味着消息是一个POD - 取决于所使用的字段类型，它不能mem-copied。）


When considering existing libraries for serialization this approach implies a performance overhead since the message must either be copied into the object provided by the serialization library (implying an additional copy) or custom code must be developed which serializes the message fields directly into the wire format (while bypassing the message class of the serialization library). Furthermore member-based access makes it problematic to add support for optional field.

> 在考虑现有的序列化库时，这种方法意味着性能开销，因为消息必须复制到序列化库提供的对象中（意味着额外的复制），或者必须开发自定义代码，直接将消息字段序列化到线格式（而不使用序列化库的消息类）。此外，基于成员的访问使得添加对可选字段的支持变得有问题。


On the other hand a method-based interface allows to implement arbitrary storage paradigms behind the API. The methods could either just access some private member variable directly or delegate the data storage to a separate entity which could e.g. be the serialization library specific data object. Commonly the methods can be inlined in languages like C++ so they don’t pose a significant performance hit but depending on the utilized storage the API might not expose mutable access to the fields which can imply an overhead when modifying data in-place.

> 另一方面，基于方法的接口允许在API背后实现任意的存储范式。这些方法可以直接访问一些私有成员变量，也可以将数据存储委托给一个单独的实体，比如特定的序列化库数据对象。通常，这些方法可以在像C++这样的语言中内联，因此不会带来显著的性能损失，但是根据所使用的存储，API可能不暴露可变的字段访问，这可能会导致在原地修改数据时出现开销。


Each serialization library has certain pros and cons depending on the scenario. The features of a serialization library can be extrinsic (exposed functionality through API, e.g. optional fields) or intrinsic (e.g. compactness of wire format). Conceptually only the intrinsic features can be exploited when a serialization library is used internally, e.g. behind a method-based message interface.

> 每个序列化库都有其特定的优点和缺点，取决于情境。序列化库的特性可以是外在的（通过API暴露的功能，例如可选字段）或内在的（例如线格式的紧凑性）。从概念上讲，只有内在特征才能在使用序列化库内部时利用，例如在基于方法的消息接口之后。

### Support pluggable serialization library


The possible approaches to select the serialization library vary from a compile decision to being able to dynamically select the serialization library for each communication channel. Especially when a ROS graph spans multiple devices and networks the needs within one network are likely already different. Therefore a dynamic solution is preferred.

> 可能的选择序列化库的方法从编译决定到能够为每个通信通道动态选择序列化库。特别是当ROS图跨越多个设备和网络时，一个网络中的需求可能已经不同。因此，首选动态解决方案。


TODO add more benefits from whiteboard picture Serializatoin Pluggability

> TODO：添加来自白板图片序列化的更多优势，可插拔性

### Possible message storage and serialization process

#### Pipeline A


The message used by the userland code stores its data directly. For each communication channel the message data is then copied into the serialization specific message representation. The serialization library will perform the serialization into the wire format from there.

> 用户层代码使用的消息直接存储其数据。对于每个通信通道，消息数据然后被复制到序列化特定的消息表示中。然后，序列化库将执行序列化到线路格式。

<img src="/img/serialization_pipeline_a.png"/>

#### Pipeline B


The message fields can be serialized directly into the wire format using custom code. While this avoids the extra data copy it requires a significant effort for implementing the custom serialization routine.

> 消息字段可以使用自定义代码直接序列化到线格式中。虽然这样可以避免额外的数据复制，但需要花费大量精力来实现自定义序列化例程。

<img src="/img/serialization_pipeline_b.png"/>

#### Pipeline C


The message delegates the data storage to an internally held storage backend, e.g. the serialization library specific message representation. Since the data is stored directly in the serialization library specific representation copying the data before serializing it is not necessary anymore.

> 这条消息将数据存储委托给内部持有的存储后端，例如序列化库特定的消息表示。由于数据直接存储在序列化库特定的表示中，因此不再需要在序列化之前复制数据。


This assumes that the API of the serialization library specific representation can be wrapped inside the ROS message API (see [Technical Issues -> Variances in field types](#variances_in_field_types)).

> 这假设序列化库特定表示的API可以包装在ROS消息API中（请参见[技术问题->字段类型的差异]（#variances_in_field_types））。

<img src="/img/serialization_pipeline_c.png"/>

### Select message storage


Under the assumption that we want to avoid implementing the serialization process from a custom message class into each supported serialization format (pipeline B) the process will either require one extra copy of the data (pipeline A) or the message must directly store its data in the specific message representation of the used serialization library (pipeline C).

> 在假定我们想要避免将自定义消息类的序列化过程实现到每种支持的序列化格式（管道B）的情况下，这个过程要么需要一个额外的数据副本（管道A），要么消息必须直接存储在使用的序列化库的特定消息表示中（管道C）。


For the later approach the decision can be made transparent to the userland code. Each publisher can act as a factory for message instances and create messages for the userland code which fit the currently used communication channels and their serialization format best. If multiple communication channels use different serialization formats the publisher should still choose one of them as the storage format for the created message instance to avoid at least one of the necessary storage conversions.

> 对于后续的方法，可以将决定透明地传递给用户代码。每个发布者都可以充当消息实例的工厂，并为用户代码创建最适合当前使用的通信渠道及其序列化格式的消息。如果多个通信渠道使用不同的序列化格式，发布者仍应将其中一种作为创建的消息实例的存储格式，以避免至少一种必要的存储转换。

### Binary compatibility of message revisions


When message objects are used in nodelets one problem is that two nodelets which run in the same process might have been linked against different definitions of a message. E.g. if we add optional fields to the message IDL one might contain the version without the optional field while the other does contain the extended version of the message.

> 当在Nodelet中使用消息对象时，一个问题是，运行在同一进程中的两个Nodelet可能会关联到不同消息定义。例如，如果我们向消息IDL添加可选字段，其中一个可能包含没有可选字段的版本，而另一个则包含扩展版本的消息。


The two different binary representations will break the ability to exchange them using a shared pointer.

> 两种不同的二进制表示将破坏使用共享指针交换它们的能力。


This can be the case for any of the pipelines. In the case of pipeline A where only the ROS message is part of the nodelet library (the serialization specific code is only part of the nodelet manager) both revisions must be binary compatible. In the case of pipeline C some serialization libraries (e.g. Protobuf) are definitely not binary compatible when features like optional fields are being used (check this assumption).

> 在任何管道的情况下，都可能是这种情况。在管道A的情况下，只有ROS消息是节点库的一部分（序列化特定代码只是节点管理器的一部分），这两个修订版必须是二进制兼容的。在管道C的情况下，当使用可选字段等功能时，一些序列化库（例如Protobuf）肯定不是二进制兼容的（检查这个假设）。

### Minimal code around existing libraries


Both pipeline A as well as C a possible to implement using a small layer around an external serialization libraries to adapt them to the message API and make them pluggable into the ROS message system.

> 两种管道A和C都可以通过在外部序列化库周围添加一层小层来实现，以适应消息API并使其可插入ROS消息系统。

### Generate POD messages for embedded / real time use


Generate a special message class which acts as a POD which is mem-copyable as well as without any dynamic memory allocation. Although this can be done from any language, one particularly useful situation is for portable C99, for use in everything from microcontrollers to soft-core processors on FPGA's, to screwed hard real-time environments.

> 生成一个特殊的消息类，它可以作为POD使用，并且无需动态内存分配。虽然可以使用任何语言完成，但特别有用的是可移植的C99，可用于从微控制器到FPGA上的软核处理器，到螺丝紧的实时环境。


For each set of max size constraints the message class would require a “mangled” name: e.g. in C99 Foo_10_20 would represent a message Foo where the two dynamic sized fields have max sizes of 10 and 20. For each type a default max size could be provided as well as each field could have a specific custom override in the message IDL. See [https://github.com/ros2/prototypes/tree/master/c_fixed_msg](https://github.com/ros2/prototypes/tree/master/c_fixed_msg) for a prototype illustrating the concept.

> 对于每组最大尺寸约束，消息类将需要一个“扭曲”的名称：例如，在C99中，Foo_10_20表示一个消息Foo，其中两个动态大小字段的最大尺寸分别为10和20。对于每种类型，可以提供默认的最大尺寸，也可以在消息IDL中为每个字段提供特定的自定义覆盖。请参见[https://github.com/ros2/prototypes/tree/master/c_fixed_msg](https://github.com/ros2/prototypes/tree/master/c_fixed_msg)，以查看概念的原型。

## Performance Evaluation

### Member/method-based access and message copy vs. serialization


Serializing messages (small as well as big ones) is at least two orders of magnitude slower than accessing message fields and copying messages in memory. Therefore performing a message copy (from one data representation to the serialization library data representation) can be considered a neglectable overhead since the serialization is the clear performance bottleneck. The selection of one serialization library has a much higher impact on the performance. See [https://github.com/ros2/prototypes/tree/master/c_fixed_msg](https://github.com/ros2/prototypes/blob/master/serialization_benchmark/results.md) for benchmark results of serialization libraries.

> 序列化消息（无论大小）至少慢了两个数量级，比访问消息字段和在内存中复制消息要慢。因此，从一种数据表示到序列化库数据表示的消息复制可以被认为是可忽略的开销，因为序列化是明显的性能瓶颈。选择一个序列化库对性能的影响要大得多。有关序列化库基准测试结果，请参见[https://github.com/ros2/prototypes/tree/master/c_fixed_msg](https://github.com/ros2/prototypes/blob/master/serialization_benchmark/results.md)。

Choosing pipeline A over pipeline B or C should therefore not impose any significant performance hit.

> 选择管道A而不是管道B或C，因此不会造成任何重大性能损失。

### Member-based vs. method-based access (with storage in member variable)


As the results of produce_consume_struct and produce_consume_method show the performance difference is not measurable. Therefore a method-based interface is preferred as it allows future customizations (e.g. changing the way the data is stored internally).

> 根据produce_consume_struct和produce_consume_method的结果表明，性能差异不可测量。因此，基于方法的接口更受青睐，因为它允许未来的定制（例如改变数据内部存储的方式）。

### Storage in message vs. storage in templated backend (both using method-based access)


As the results of produce_consume_method and produce_consume_backend_plain show the performance difference is again not measurable. Therefore a templated backend is preferred as it allows customizations (e.g. add value range validation, defer storage, implement thread safety, custom logging for debugging/introspection purposes) and enable to drop in custom storage backends (e.g. any message class of an existing serialization which suits our API).

> 根据produce_consume_method和produce_consume_backend_plain的结果表明，性能差异再次不可测量。因此，模板后端更可取，因为它允许自定义（例如添加值范围验证、延迟存储、实现线程安全、调试/内省目的自定义日志），并允许插入自定义存储后端（例如符合我们API的现有序列化的任何消息类）。

## Technical Issues

### Variances in field types


Different serialization libraries specify different field type when e.g. generating C++ code for the messages. The major problem is the mapping between those types in an efficient manner. From a performance point of view the message interface should expose const references (especially to “big” fields). But those can only be mapped to the specific API of the serialization library if the types are exchangeable. But for example a byte array is represented differently in C++ in the various serialization libraries:

> 不同的序列化库在生成例如C++代码的消息时会指定不同的字段类型。主要问题是以有效的方式映射这些类型。从性能的角度来看，消息接口应该暴露常量引用（尤其是“大”字段）。但是如果类型是可交换的，这些只能映射到序列化库的特定API。但是例如在C++中，在各种序列化库中字节数组的表示形式是不同的：


- Protobuf:

> Protobuf：Protocol Buffer（协议缓冲）


  - dynamic array&lt;T&gt;: RepeatedField&lt;T&gt; (STL-like interface)

> 动态数组<T>：RepeatedField<T>（类似STL的接口）

  - fixed array&lt;T&gt;: ---

> 固定数组<T>：

  - binary/string: std::string

> 二进制/字符串：std::string


- Thrift:

> - 勤俭：


  - dynamic array&lt;T&gt;: std::vector&lt;T&gt;

> 动态数组<T>：std::vector<T>

  - fixed array&lt;T&gt;: ---

> 固定数组<T>：

  - binary/string: std::string

> 二进制/字符串：std::string


- ROS:

> - ROS：机器人操作系统


  - array&lt;T&gt;: std::vector&lt;T&gt;

> 数组<T>：std::vector<T>

  - fixed array&lt;T, N&gt;: boost::array&lt;T, N&gt;

> 固定数组<T, N>：boost::array<T, N>

  - string: std::string

> std::string转换成简体中文：字符串


Furthermore the serialization library specific message API might not expose mutable access which could therefore not be provided by RO either when using pipeline C.

> 此外，序列化库特定的消息API可能不暴露可变访问，因此在使用管道C时RO也可能不提供。

## Preliminary Conclusion

### Serialization Pipeline


Due to the mentioned problems and added complexity the pipeline C is not viable. Since the amount of time necessary for the memory copy before a serialization is orders of magnitudes smaller than the actual serialization pipeline A is selected for further prototyping. This still allows us to implement the optimization described as pipeline B for e.g. the default serialization library if need is.

> 由于提到的问题和增加的复杂性，管道C不可行。由于在序列化之前执行内存复制所需的时间比实际序列化管道A所需的时间要少几个数量级，因此选择管道A进行进一步原型设计。这仍然允许我们实现管道B中描述的优化，例如默认序列化库。

### Message Interface


Under the assumption that a method-based access is not significantly impacting the performance it is preferred over a member-based access in order to enable changing the storage backend in the future and enabling overriding it with a custom implementation.

> 在假设方法访问不会显著影响性能的情况下，为了将来可以更改存储后端并允许使用自定义实现来覆盖它，建议优先使用基于方法的访问而不是基于成员的访问。

### Update


With the decision to build ROS 2 on top of DDS the Pipeline B will be used. The previous conclusion to switch from member to method based access has been revisited. Since we do not have the need to make the storage backend exchangeable anymore and we might prefer keeping the member based access of messages to keep it similar with ROS 1.

> 随着决定在DDS上构建ROS 2，将使用管道B。重新审视了从成员到基于方法的访问的先前结论。由于我们不再需要使存储后端可交换，我们可能更喜欢保持消息的成员访问方式，以便与ROS 1保持相似。
