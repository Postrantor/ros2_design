---
tip: translate by openai@2023-05-28 11:11:10
...
---
    layout: default
    title: Parameter API design in ROS
    permalink: articles/ros_parameters.html
    abstract:
      This article is proposed design for the interfaces for interacting with parameters in ROS 2.
      We focus here on specifying the system design and leave the implementation unspecified.
    author: '[Tully Foote](https://github.com/tfoote)'
    date_written: 2015-09
    last_modified: 2019-05
    published: true
    Authors: {{ page.author }}
    Date Written: {{ page.date_written }}
    Last Modified: {% if page.last_modified %}{{ page.last_modified }}{% else %}{{ page.date_written }}{% endif %}
---

## Background


In ROS 1 the parameters were implemented in a 'blackboard model' with unrestricted read and write access from all nodes.

> 在ROS 1中，参数采用“黑板模型”实施，所有节点都可以无限制地读写。

The data model proved useful in many cases, but there were many cases where the lack of control or ownership proved to be a problem.

> 数据模型在许多情况下都很有用，但是由于缺乏控制或所有权，也有很多情况出现了问题。

One of the common shortcomings was for setting parameters on drivers.

> 一个常见的缺点是设置驱动程序的参数。

A tool called dynamic_reconfigure was developed to address this use case.

> 动态重新配置是一个专门为解决这种情况而开发的工具。

It provided a service based interface to interact with parameters of other nodes.

> 它提供了一个基于服务的界面来与其他节点的参数进行交互。

### Other resources


Other resources related to the parameter design process for ROS 2 include:

> 关于ROS 2参数设计过程的其他资源包括：


- Gonzalo's research on parameters.

> Gonzalo的参数研究。

    - Discussion: <https://groups.google.com/forum/#!topic/ros-sig-ng-ros/YzCmoIsN0o8> and <https://groups.google.com/forum/#!searchin/ros-sig-ng-ros/parameter/ros-sig-ng-ros/fwDBcei5Ths/L6ORPfjUDXYJ>
    - Prototype: <https://github.com/abellagonzalo/dynamic_config>
    - Final Notes:  [http://wiki.ros.org/sig/NextGenerationROS/Parameters](http://wiki.ros.org/sig/NextGenerationROS/Parameters)


- Thibault's nodeparam draft REP: <https://github.com/tkruse/rep/blob/nodeparam/nodeparam-REP.rst>

> Thibault的节点参数草案REP：<https://github.com/tkruse/rep/blob/nodeparam/nodeparam-REP.rst>

## Ideal System


It is useful to consider an ideal system to understand how it would work and how it would relate to the current system.

> 考虑一个理想的系统有助于了解它将如何运作以及它将如何与现有系统相关联。

An ideal system would support for the combined use cases of ROS 1.0's built-in parameters as well as ROS 1.0's dynamic parameters system.

> 理想的系统应该支持ROS 1.0内置参数和ROS 1.0动态参数系统的组合使用。

Based on that criteria an ideal system would be able to:

> 根据这些标准，理想的系统能够：


- Set parameter values

> .

设定参数值


  This includes setting groups of parameter values atomically.

> 这包括原子地设置参数值组。


- Get parameter values

> 取得参数值


  This includes getting groups of parameter values atomically.

> 这包括原子性地获取参数值组。


- Unset parameter values

> - 取消设置的参数值


  This includes unsetting groups of parameter values atomically, but may be a special case of setting groups of parameters atomically.

> 这包括原子性地取消设置参数组的值，但可能是原子性设置参数组的特殊情况。


- List currently set parameters

> 列出当前设定的参数


  Since the number of parameters might be large this needs to be able to provide subsets of the parameters.

> 由于参数的数量可能很大，因此需要能够提供参数的子集。

  E.g. the parameters could be queried incrementally for a tree-like GUI.

> 例如，可以逐步查询参数以获得树状图形用户界面。


- Provide notifications when parameters are added and removed or their value has been changed

> 当参数被添加、移除或值发生改变时，提供通知。


- Reject changes to parameter values

> 拒绝对参数值的更改


  This implies that some entity has the authority to reject or accept a change based on arbitrary criteria.

> 这意味着某个实体有权根据任意标准拒绝或接受变更。

  This would also include the ability to convey at least part of the criteria for the acceptance of a change to external actors.

> 这也包括向外部演员传达接受变化的至少部分标准的能力。

  For example, communicating the range for an integer or a few choices for a string.

> 例如，传达整数的范围或字符串的几个选择。

  This type of information would be used to generate generic user interfaces, but might not capture all criteria.

> 这种信息可以用来生成通用的用户界面，但可能不能满足所有的标准。

  Since the validation criteria can be arbitrary complex and can potentially not be communicated to a client the parameter server could offer to validate an atomic set of parameters and respond with a boolean flag if the values would be accepted (based on the current criteria).

> 由于验证标准可能非常复杂，可能无法与客户端沟通，参数服务器可以提供验证原子参数集的功能，并根据当前标准返回布尔值以表示是否接受这些值。

  Obviously the result might be different when the values are set shortly after but it would allow to implement validators in e.g. a GUI generically.

> 显然，当值设置得很短的时候，结果可能会有所不同，但这将允许在例如GUI中通用地实现验证器。


- Provide visibility into what parameters are expected to pass validation vs be rejected

> 提供可见性，以便了解哪些参数有望通过验证，哪些参数将被拒绝。


  When updating a value it can be valuable to know if the parameter update would be accepted without actually requesting the change to happen.

> 当更新一个值时，知道参数更新是否会被接受而不实际要求发生变化可能是有价值的。


- Provide clear rules on the lifetime of a parameter

> 提供关于参数生命周期的明确规则


  These rules would define what the lifetime of the parameter will be and what conditions will clear its value.

> 这些规则将定义参数的生命周期以及什么条件会清除其值。


- Address all parameters without ambiguity in the names

> 清楚地在名称中指明所有参数。


  For example, one of the challenges of the current system is that there is a naming ambiguity between nodes and parameters `/foo/bar/baz` could be a node `/foo/bar/baz` or a private parameter `baz` on node `/foo/bar`.

> 例如，当前系统的一个挑战是节点和参数之间的命名模糊性，`/foo/bar/baz`可以是节点`/foo/bar/baz`，也可以是节点`/foo/bar`上的私有参数`baz`。


- Be logged for playback and analysis

> 登录以进行播放和分析。

## Proposed Approach


To cover the feature set above, the ROS 2 parameter system is proposed as follows.

> 为了满足上面的功能集，提出了ROS 2参数系统如下。

### Parameters Hosted in Nodes


For the sake of validating parameter lifecycle, all parameters will be hosted on a node.

> 为了验证参数的生命周期，所有参数将被托管在一个节点上。

Their lifetime will be implicitly tied to the nodes lifetime.

> 他们的寿命将与节点的寿命隐式地联系在一起。

The node will be responsible for validating current values.

> 节点将负责验证当前值。

The node could also implement persistence of the parameters to reload the previous values after being restarted.

> 节点也可以实现参数的持久性，以便在重新启动后重新加载以前的值。

### Parameter addressing


All parameters will be addressed by two elements: the full node name and the parameter name.

> 所有参数将由两个元素处理：完整的节点名称和参数名称。

### Supported datatypes


Each parameter consists of a key and a value.

> 每个参数包括一个键和一个值。

The key is a string value.

> 钥匙是一个字符串值。

The value can be one of the following datatypes:

> 值可以是以下数据类型之一：

    - `bool`
    - `int64`
    - `float64`
    - `string`
    - `byte[]`
    - `bool[]`
    - `int64[]`
    - `float64[]`
    - `string[]`


The datatypes are chosen as non-complex datatypes, as defined in the [interface definitions article](interface_definition.html).

> 数据类型选择为非复杂数据类型，如[接口定义文章](interface_definition.html)中所定义的。

The full complement of datatypes of different bitdepth and unsigned types are avoided to allow interpretation from text based configuration files.

> 避免使用不同位深度和无符号类型的完整数据类型，以便从基于文本的配置文件中解释。


Array datatypes support a variety of use cases, e.g.,

> 数组数据类型支持多种用例，例如，

- `byte[]` is included to allow the storage of binary blobs.

> `byte[]`包括允许存储二进制大对象的功能。

Its use is not recommended but can be very convenient, and explicitly supporting it is better than having people try to abuse other datatypes such as strings.

> 它的使用不推荐，但可以非常方便，明确支持它比试图滥用字符串等其他数据类型更好。

- `bool[]` can be useful as a mask for other array parameters.

> 布尔数组可以用作其他数组参数的掩码。

- `int64[]` and `float64[]` can be used to express numerical vector and matrix parameters.

> int64[]和float64[]可用于表示数值向量和矩阵参数。

- `string[]` can be used to express groups of names, such as a set of robots in a multi-robot context.

> `string[]` 可以用来表达一组名称，比如一组机器人在多机器人环境中。


While the use of array parameters increases the complexity of the API, their omission would necessitate complex naming schemes for parameters like matrices, which are common in robotics.

> 虽然数组参数增加了API的复杂性，但如果省略它们，就需要为像机器人中常见的矩阵这样的参数设计复杂的命名方案。


Only homogenous arrays of datatypes will be supported. This is to avoid the unnecessary complexity of the introspection needed to handle multidimensionality and heterogeneity within the arrays.

> .

只支持同类型数组。这是为了避免处理多维和异质数组所需的内省带来的不必要的复杂性。


Arrays should not be abused, however; users should rely on namespacing and explicit variable names wherever possible.

> 数组不应该被滥用，用户应该尽可能依赖命名空间和明确的变量名称。

### Required functionality


Each node is responsible for providing the following functionality.

> 每个节点负责提供以下功能。


- Get Parameters

> 获取参数


  Given a list of parameter names it will return the parameter values.

> 给定一个参数名称列表，它将返回参数值。


- Set Parameters

> 设定参数


  Given a list of parameter names, it will request an update of the values subject to validation of the values.

> 给定一个参数名称列表，它将请求更新值，并经过值的验证。

  The updated values can include unsetting the value.

> 更新的值可以包括取消设置值。

  It will provide an API that can atomically update a set of values such that if any of the values fail validation, none of the values are set.

> 它将提供一个API，可以原子性地更新一组值，以便如果任何值验证失败，则不会设置任何值。

  The success or failure of this call will be available to the client for each update unit.

> 这次电话的成功或失败会在每次更新单元时向客户提供。

  Validation of the values is expected to return as quickly as possible and only be related to accepting or rejecting the set request.

> 预期要求快速验证值，并仅与接受或拒绝设置请求有关。

  It should not take into account how the changed value may or may not affect ongoing system performance of the node or the greater system.

> 它不应考虑改变的值如何可能会或不会影响节点或整个系统的持续系统性能。


- List Parameters

> - 参数列表


  Provide a list of parameter names which are currently set.

> 提供当前设置的参数名称列表。


- Describe Parameters

> 描述参数


  Given a list of parameter names, return their datatype.

> 给定一个参数名称列表，返回它们的数据类型。


This functionality will be exposed through a user API which will support both local API calls as well as invocations on remote nodes via a ROS Service API.

> 这个功能将通过用户API暴露，该API将支持本地API调用以及通过ROS服务API远程节点上的调用。

### Parameter update validation


The node can validate incoming parameter changes and either accept or reject them.

> 节点可以验证传入的参数变化，并接受或拒绝它们。

### Backwards compatibility Parameter Server like behavior


There are use cases where the older behavior with parameter server was useful.

> 有一些使用案例，旧的参数服务器行为很有用。

Both persisting beyond the duration of a specific node is valuable as well as having parameters with no specific association to a node which would potentially own or validate the values.

> 两者都能延续超过特定节点的持续时间是有价值的，而且还有没有特定关联到节点的参数，这些参数可能拥有或验证这些值。

To this end we propose to write a simple node which emulates the policy of the ROS 1.0 parameter server: it runs in namespace `/` and simply accepts all changes requested.

> 为此，我们建议编写一个简单的节点，模拟ROS 1.0参数服务器的策略：它在命名空间`/`中运行，并简单地接受所有请求的更改。

The parameters held by this parameter server node would persist for the lifetime of the parameter server node.

> 这个参数服务器节点所持有的参数将持续存在于参数服务器节点的整个生命周期中。

Specific instances could be launched in different namespaces to support different parameter persistence models.

> 特定的实例可以在不同的命名空间中启动，以支持不同的参数持久性模型。

### Search parameter behavior


A pattern developed in ROS 1.0 was the `searchParam` mode where a parameter could be set in a namespace and the parameter query would walk up the namespace to search for the parameter.

> 在ROS 1.0中开发的一种模式是`searchParam`模式，在该模式下，可以在命名空间中设置一个参数，而参数查询将沿着命名空间向上搜索该参数。

A similar behavior can be implemented by allowing the search parameter implementation to walk across the different nodes in hierarchical order.

> 类似的行为可以通过允许搜索参数实现按照层次顺序跨越不同节点来实现。

### Parameter API


The client libraries will provide the following API for interfacing with the Core Parameter API for both local and remote nodes including return codes.

> 客户端库将提供以下API，用于与本地和远程节点的核心参数API进行交互，包括返回代码。

### Parameter Events


Each node will provide a topic on which parameter events will be published.

> 每个节点将提供一个参数事件将被发布的主题。

This topic is to support monitoring parameters for change.

> 这个主题是为了支持监控参数的变化。

It is expected that client libraries will implement the ability to register callbacks for specific parameter changes using this topic.

> 预计客户端库将使用此主题实现注册特定参数更改的回调的能力。

### Logging and playback


When logging an entire system, the parameter changes can be logged via standard topic recording of the events channel.

> 当记录一个整个系统时，参数更改可以通过事件通道的标准主题记录来记录。

An implementation of the playback mechanism could listen to the parameter event streams and invoke the set parameter calls via the remote API.

> 一个播放机制的实现可以监听参数事件流，并通过远程API调用设置参数调用。

## Current implementation


The above specification has been prototyped; the implementation can be found in [rclcpp](https://github.com/ros2/rclcpp).

> 以上规范已经原型化；实现可以在[rclcpp](https://github.com/ros2/rclcpp)中找到。

The definition of the services to use for interacting remotely are contained in the [rcl_interfaces package](https://github.com/ros2/rcl_interfaces)

> 服务用于远程交互的定义包含在[rcl_interfaces package](https://github.com/ros2/rcl_interfaces)中。

### Unimplemented


Currently there a few parts of the specification unimplemented.

> 目前规范的一些部分尚未实现。


- No parameter subscription registration.

> 无参数订阅注册。

  The events are published, but there is not way to register a callback for changes to a specific parameter.

> 事件已发布，但没有办法为特定参数的更改注册回调。

  You can currently register for a callback on all changes for parameters of a node.

> 您目前可以为节点的参数所有变化注册回调。


- The ability to register callback to validate parameter updates prior to them being updated is not available.

> 不提供在更新参数之前注册回调来验证参数更新的功能。


- There has been no work on logging and playback of logged parameter changes.

> 没有对日志记录和记录参数变化的回放进行任何工作。


- The ability to list and get expected validation policy has not been implemented.

> 还没有实现列出和获取预期验证策略的能力。

  It is expected to operate at a slightly higher level than parameters, and it possibly will be related to the component life cycle.

> 预计它的运行水平会略高于参数，可能会与组件的生命周期有关。

## Topics not covered at the moment


Going forward, there are still topics to discuss and flesh out either in this document or others.

> 往前看，还有话题需要在本文档或其他文档中讨论和完善。

A few to highlight:

> 几个要强调的：

### Parameter initialization


There are several ways to load parameters at startup including command line arguments, roslaunch arguments, and potentially parameter files.

> 有几种方法可以在启动时加载参数，包括命令行参数、roslaunch参数和可能的参数文件。

This is something which should be addressed in conjunction with the new launch system.

> 这是应该与新发射系统一起解决的事情。

### Predeclared interface to support static checking/validation


The ability to declare an API which can help with static checks and prevent logical errors which arise from setting the wrong parameter based on a typo.

> 能够声明一个可以帮助进行静态检查并防止由于打错字而导致的逻辑错误的API。

The node could enforce this by rejecting unexpected names, but there are some cases where knowing the expected parameter names would be useful for developer tools.

> 节点可以通过拒绝意外的名称来强制执行这一点，但是在一些情况下，了解预期的参数名称对开发者工具有用。
