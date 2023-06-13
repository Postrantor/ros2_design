---
tip: translate by openai@2023-05-30 09:04:14
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
Authors: 
Date Written: 
Last Modified:
---
## Background

In ROS 1 the parameters were implemented in a 'blackboard model' with unrestricted read and write access from all nodes. The data model proved useful in many cases, but there were many cases where the lack of control or ownership proved to be a problem. One of the common shortcomings was for setting parameters on drivers. A tool called dynamic_reconfigure was developed to address this use case. It provided a service based interface to interact with parameters of other nodes.

> 在 ROS 1 中，参数采用“黑板模型”实现，所有节点都可以无限制地读写。该数据模型在许多情况下都很有用，但也有许多情况下缺乏控制或所有权会成为一个问题。其中一个常见的缺点是设置驱动程序的参数。为了解决这个问题，开发了一个名为 dynamic_reconfigure 的工具。它提供了一个基于服务的接口来与其他节点的参数进行交互。

### Other resources

Other resources related to the parameter design process for ROS 2 include:

> 其他与 ROS 2 参数设计流程相关的资源包括：

- Gonzalo's research on parameters.

  - Discussion: [https://groups.google.com/forum/#!topic/ros-sig-ng-ros/YzCmoIsN0o8](https://groups.google.com/forum/#!topic/ros-sig-ng-ros/YzCmoIsN0o8) and [https://groups.google.com/forum/#!searchin/ros-sig-ng-ros/parameter/ros-sig-ng-ros/fwDBcei5Ths/L6ORPfjUDXYJ](https://groups.google.com/forum/#!searchin/ros-sig-ng-ros/parameter/ros-sig-ng-ros/fwDBcei5Ths/L6ORPfjUDXYJ)
  - Prototype: [https://github.com/abellagonzalo/dynamic_config](https://github.com/abellagonzalo/dynamic_config)
  - Final Notes: [http://wiki.ros.org/sig/NextGenerationROS/Parameters](http://wiki.ros.org/sig/NextGenerationROS/Parameters)
- Thibault's nodeparam draft REP: [https://github.com/tkruse/rep/blob/nodeparam/nodeparam-REP.rst](https://github.com/tkruse/rep/blob/nodeparam/nodeparam-REP.rst)

## Ideal System

It is useful to consider an ideal system to understand how it would work and how it would relate to the current system. An ideal system would support for the combined use cases of ROS 1.0's built-in parameters as well as ROS 1.0's dynamic parameters system. Based on that criteria an ideal system would be able to:

> 考虑一个理想系统有助于了解它将如何工作以及它将如何与当前系统相关联。理想系统将支持 ROS 1.0 内置参数和 ROS 1.0 动态参数系统的组合用例。根据这一标准，理想系统能够：

- Set parameter values

  This includes setting groups of parameter values atomically.

> 这包括原子性地设置参数值组。

- Get parameter values

  This includes getting groups of parameter values atomically.

> 这包括原子方式获取参数值组。

- Unset parameter values

  This includes unsetting groups of parameter values atomically, but may be a special case of setting groups of parameters atomically.

> 这包括原子地取消设置参数组的值，但可能是原子地设置参数组的特殊情况。

- List currently set parameters

  Since the number of parameters might be large this needs to be able to provide subsets of the parameters. E.g. the parameters could be queried incrementally for a tree-like GUI.

> 由于参数的数量可能很大，因此需要能够提供参数的子集。例如，可以逐步查询参数以获得树形图形用户界面。

- Provide notifications when parameters are added and removed or their value has been changed
- Reject changes to parameter values

This implies that some entity has the authority to reject or accept a change based on arbitrary criteria. This would also include the ability to convey at least part of the criteria for the acceptance of a change to external actors. For example, communicating the range for an integer or a few choices for a string. This type of information would be used to generate generic user interfaces, but might not capture all criteria. Since the validation criteria can be arbitrary complex and can potentially not be communicated to a client the parameter server could offer to validate an atomic set of parameters and respond with a boolean flag if the values would be accepted (based on the current criteria). Obviously the result might be different when the values are set shortly after but it would allow to implement validators in e.g. a GUI generically.

> 这意味着某个实体有权根据任意标准拒绝或接受更改。这也包括向外部演员传达接受更改的至少部分标准的能力。例如，传达整数的范围或字符串的几个选择。这种信息将用于生成通用用户界面，但可能无法捕获所有标准。由于验证标准可能是任意复杂的，并且可能无法传达给客户端，因此参数服务器可以提供验证原子参数集并返回布尔值标志，如果根据当前标准接受值(基于当前标准)。显然，当设置值稍后会有所不同，但它可以允许在例如 GUI 中实现验证器。

- Provide visibility into what parameters are expected to pass validation vs be rejected

  When updating a value it can be valuable to know if the parameter update would be accepted without actually requesting the change to happen.

> 当更新一个值时，知道参数更新是否会被接受而不用实际请求更改可能是很有价值的。

- Provide clear rules on the lifetime of a parameter

  These rules would define what the lifetime of the parameter will be and what conditions will clear its value.

> 这些规则将定义参数的生命周期以及什么条件可以清除其值。

- Address all parameters without ambiguity in the names

  For example, one of the challenges of the current system is that there is a naming ambiguity between nodes and parameters `/foo/bar/baz` could be a node `/foo/bar/baz` or a private parameter `baz` on node `/foo/bar`.

> 例如，目前系统的一个挑战是，节点和参数之间存在命名模糊，`/foo/bar/baz` 可以是节点 `/foo/bar/baz`，也可以是节点 `/foo/bar` 上的私有参数 `baz`。

- Be logged for playback and analysis

## Proposed Approach

To cover the feature set above, the ROS 2 parameter system is proposed as follows.

> 为了满足上述功能，提出了 ROS 2 参数系统。

### Parameters Hosted in Nodes

For the sake of validating parameter lifecycle, all parameters will be hosted on a node. Their lifetime will be implicitly tied to the nodes lifetime. The node will be responsible for validating current values. The node could also implement persistence of the parameters to reload the previous values after being restarted.

> 为了验证参数的生命周期，所有参数都将放在一个节点上。它们的寿命将隐含地与节点的寿命相关联。节点将负责验证当前值。节点还可以实现参数的持久性，以便在重新启动后重新加载以前的值。

### Parameter addressing

All parameters will be addressed by two elements: the full node name and the parameter name.

> 所有参数将由两个元素来处理：完整的节点名称和参数名称。

### Supported datatypes

Each parameter consists of a key and a value. The key is a string value. The value can be one of the following datatypes:

> 每个参数由键和值组成。键是字符串值。值可以是以下数据类型之一：

- `bool`
- `int64`
- `float64`
- `string`
- `byte[]`
- `bool[]`
- `int64[]`
- `float64[]`
- `string[]`

The datatypes are chosen as non-complex datatypes, as defined in the [interface definitions article](interface_definition.html). The full complement of datatypes of different bitdepth and unsigned types are avoided to allow interpretation from text based configuration files.

> 数据类型选择为非复杂数据类型，根据[接口定义文章](interface_definition.html)定义。为了使文本配置文件可以被解释，避免使用不同位深度和无符号类型的完整数据类型。

Array datatypes support a variety of use cases, e.g.,

> 数组数据类型支持多种用例，例如，

- `byte[]` is included to allow the storage of binary blobs. Its use is not recommended but can be very convenient, and explicitly supporting it is better than having people try to abuse other datatypes such as strings.
- `bool[]` can be useful as a mask for other array parameters.
- `int64[]` and `float64[]` can be used to express numerical vector and matrix parameters.
- `string[]` can be used to express groups of names, such as a set of robots in a multi-robot context.

While the use of array parameters increases the complexity of the API, their omission would necessitate complex naming schemes for parameters like matrices, which are common in robotics.

> 虽然数组参数增加了 API 的复杂性，但如果省略它们，就需要为矩阵等在机器人领域很常见的参数设计复杂的命名方案。

Only homogenous arrays of datatypes will be supported. This is to avoid the unnecessary complexity of the introspection needed to handle multidimensionality and heterogeneity within the arrays.

> 只支持同类型数组。这是为了避免处理多维性和数组内异质性所需的自省的不必要的复杂性。

Arrays should not be abused, however; users should rely on namespacing and explicit variable names wherever possible.

> 数组不应该被滥用，但是用户应该尽可能依赖命名空间和明确的变量名。

### Required functionality

Each node is responsible for providing the following functionality.

> 每个节点负责提供以下功能。

- Get Parameters

  Given a list of parameter names it will return the parameter values.

> 给定一个参数名称列表，它将返回参数值。

- Set Parameters

  Given a list of parameter names, it will request an update of the values subject to validation of the values. The updated values can include unsetting the value. It will provide an API that can atomically update a set of values such that if any of the values fail validation, none of the values are set. The success or failure of this call will be available to the client for each update unit. Validation of the values is expected to return as quickly as possible and only be related to accepting or rejecting the set request. It should not take into account how the changed value may or may not affect ongoing system performance of the node or the greater system.

> 给定一个参数名称列表，它将请求更新值的更新，以符合值的验证。更新的值可以包括取消设置值。它将提供一个 API，可以原子性地更新一组值，以便如果任何值未通过验证，则不会设置任何值。此调用的成功或失败将对客户端可用于每个更新单元。期望尽快返回值的验证，只与接受或拒绝设置请求有关。它不应考虑更改的值如何或者不如何影响节点或整个系统的正常运行。

- List Parameters

  Provide a list of parameter names which are currently set.

> 提供目前设置的参数名称列表。

- Describe Parameters

  Given a list of parameter names, return their datatype.

> 给定一个参数名称列表，返回它们的数据类型。

This functionality will be exposed through a user API which will support both local API calls as well as invocations on remote nodes via a ROS Service API.

> 这个功能将通过用户 API 暴露，该 API 将支持本地 API 调用以及通过 ROS 服务 API 远程节点的调用。

### Parameter update validation

The node can validate incoming parameter changes and either accept or reject them.

> 节点可以验证传入的参数变化，然后接受或拒绝它们。

### Backwards compatibility Parameter Server like behavior

There are use cases where the older behavior with parameter server was useful. Both persisting beyond the duration of a specific node is valuable as well as having parameters with no specific association to a node which would potentially own or validate the values. To this end we propose to write a simple node which emulates the policy of the ROS 1.0 parameter server: it runs in namespace `/` and simply accepts all changes requested. The parameters held by this parameter server node would persist for the lifetime of the parameter server node. Specific instances could be launched in different namespaces to support different parameter persistence models.

> 在使用参数服务器的旧行为有用的情况下，持续超过特定节点的持续时间是有价值的，以及没有特定关联到节点的参数，该节点可能拥有或验证这些值。为此，我们建议编写一个简单的节点，模拟 ROS 1.0 参数服务器的策略：它运行在命名空间“/”中，只接受所有请求的更改。此参数服务器节点保留的参数将持续参数服务器节点的生命周期。可以在不同的命名空间中启动特定实例以支持不同的参数持久性模型。

### Search parameter behavior

A pattern developed in ROS 1.0 was the `searchParam` mode where a parameter could be set in a namespace and the parameter query would walk up the namespace to search for the parameter. A similar behavior can be implemented by allowing the search parameter implementation to walk across the different nodes in hierarchical order.

> 在 ROS 1.0 中开发的一种模式是 `searchParam` 模式，在该模式下，可以在命名空间中设置一个参数，参数查询将沿着命名空间向上搜索参数。可以通过允许搜索参数实现以分层顺序跨越不同节点来实现类似的行为。

### Parameter API

The client libraries will provide the following API for interfacing with the Core Parameter API for both local and remote nodes including return codes.

> 客户端库将为本地和远程节点接口的核心参数 API 提供以下 API，包括返回码。

### Parameter Events

Each node will provide a topic on which parameter events will be published. This topic is to support monitoring parameters for change. It is expected that client libraries will implement the ability to register callbacks for specific parameter changes using this topic.

> 每个节点将提供一个主题，用于发布参数事件。此主题旨在支持监控参数的变化。预计客户端库将实现使用此主题注册特定参数更改的回调的能力。

### Logging and playback

When logging an entire system, the parameter changes can be logged via standard topic recording of the events channel. An implementation of the playback mechanism could listen to the parameter event streams and invoke the set parameter calls via the remote API.

> 当记录整个系统时，可以通过事件通道的标准主题记录来记录参数更改。回放机制的实现可以监听参数事件流并通过远程 API 调用设置参数调用。

## Current implementation

The above specification has been prototyped; the implementation can be found in [rclcpp](https://github.com/ros2/rclcpp). The definition of the services to use for interacting remotely are contained in the [rcl_interfaces package](https://github.com/ros2/rcl_interfaces)

> 以上规范已经原型化；实现可以在 [rclcpp](https://github.com/ros2/rclcpp) 中找到。用于远程交互的服务定义包含在 [rcl_interfaces package](https://github.com/ros2/rcl_interfaces) 中。

### Unimplemented

Currently there a few parts of the specification unimplemented.

> 目前还有一些规格未实施。

- No parameter subscription registration. The events are published, but there is not way to register a callback for changes to a specific parameter. You can currently register for a callback on all changes for parameters of a node.
- The ability to register callback to validate parameter updates prior to them being updated is not available.
- There has been no work on logging and playback of logged parameter changes.
- The ability to list and get expected validation policy has not been implemented. It is expected to operate at a slightly higher level than parameters, and it possibly will be related to the component life cycle.

## Topics not covered at the moment

Going forward, there are still topics to discuss and flesh out either in this document or others. A few to highlight:

> 今后，还有一些议题需要在本文档或其他文档中讨论和完善，其中一些可以突出如下：

### Parameter initialization

There are several ways to load parameters at startup including command line arguments, roslaunch arguments, and potentially parameter files. This is something which should be addressed in conjunction with the new launch system.

> 有几种方法可以在启动时加载参数，包括命令行参数、roslaunch 参数和可能的参数文件。这是应该在新的启动系统中解决的问题。

### Predeclared interface to support static checking/validation

The ability to declare an API which can help with static checks and prevent logical errors which arise from setting the wrong parameter based on a typo. The node could enforce this by rejecting unexpected names, but there are some cases where knowing the expected parameter names would be useful for developer tools.

> 能够声明一个 API 可以帮助进行静态检查，并防止因为打错字而导致的逻辑错误。节点可以通过拒绝意外的名称来强制执行，但是有些情况下，知道预期的参数名称对开发者工具来说是很有用的。
