---
tip: translate by openai@2023-05-29 08:56:15
layout: default
title: External configurability of QoS policies.
permalink: articles/qos_configurability.html
abstract:
  This article describes a mechanism to allow reconfiguration of QoS settings at startup time.
author: '[Ivan Santiago Paunovic](https://github.com/ivanpauno)'
date_written: 2020-11
last_modified: 2020-11
published: true
Authors: {{ page.author }}
Date Written: {{ page.date_written }}
Last Modified: {% if page.last_modified %}{{ page.last_modified }}{% else %}{{ page.date_written }}{% endif %}
---

## Summary

Provide an standard way to configure QoS settings of different entities in the ROS graph at startup time, overriding the settings specified in code.

> 提供一种标准的方式来在启动时配置 ROS 图中不同实体的 QoS 设置，以覆盖代码中指定的设置。

## Motivation

Up to ROS 2 Foxy, there's no standard mechanism to override QoS settings when creating a node. It's quite common in the ROS 2 community to reuse nodes that other people have written and in a lot of use cases allowing QoS profiles to be changed make sense.

> 到 ROS 2 Foxy 为止，**没有标准机制来覆盖创建节点时的 QoS 设置**。在 ROS 2 社区中重用其他人编写的节点是很常见的，在许多用例中允许更改 QoS 配置文件是有意义的。

Up to now, different workarounds have been used in order to provide this reconfigurability:

> 到目前为止，为了提供这种可重新配置性，已经使用了不同的解决方案。

- Command line arguments.
- ROS parameters to set particular policies.
- Combining parameters, the system default QoS profile, and vendor specific QoS profiles.

> - 命令行参数
> - 设置特定政策的 ROS 参数
> - 结合参数、系统默认的 QoS 配置文件以及供应商特定的 QoS 配置文件。

Here are some examples applying these mechanisms:

> 这里有一些应用这些机制的例子：

- rosbag2 has an ad-hoc mechanism to override QoS profiles when recording or doing playback ([docs](https://index.ros.org/doc/ros2/Tutorials/Ros2bag/Overriding-QoS-Policies-For-Recording-And-Playback/)).

- Image pipeline nodes use some parameters to allow changing some of the policies ([PR](https://github.com/ros-perception/image_pipeline/pull/521)).

- Ouster drivers also use some parameters to allow changing some policies ([PR](https://github.com/ros-drivers/ros2_ouster_drivers/pull/26)).

- Gazebo ROS packages allows configuring QoS profiles of the plugins in the SDF file ([issue](https://github.com/ros-simulation/gazebo_ros_pkgs/issues/1079)).

> - rosbag2 有一种临时机制可以在录制或播放时覆盖 QoS 配置文件([文档](https://index.ros.org/doc/ros2/Tutorials/Ros2bag/Overriding-QoS-Policies-For-Recording-And-Playback/))。
> - 图像管道节点使用一些参数来允许更改一些策略([PR](https://github.com/ros-perception/image_pipeline/pull/521))。
> - Ouster 驱动程序也使用一些参数来允许更改一些策略([PR](https://github.com/ros-drivers/ros2_ouster_drivers/pull/26))。
> - Gazebo ROS 包允许在 SDF 文件中配置插件的 QoS 配置文件([问题](https://github.com/ros-simulation/gazebo_ros_pkgs/issues/1079))。

Not providing an standard mechanism for reconfiguring QoS results in a lot of slightly different interfaces to do the same thing, creating a pain point for end users when figuring out how to reconfigure QoS for nodes in their system.

> 不提供标准的 QoS 重新配置机制会导致大量略有不同的界面来完成同一件事，当用户试图弄清楚如何为他们系统中的节点重新配置 QoS 时，就会产生痛苦的体验。

## Design goals and other constraints

This proposal was written with the following goals in mind:

> 本提案是根据以下目标书写的：

- The node's author should be able to control which entities have reconfigurable QoS, and if a profile is valid or not.
- Rationale: Allowing to change all QoS policies could break a contract the node's implementer assumed.

> 节点的作者应该能够控制哪些实体具有可重新配置的 QoS，以及配置文件是否有效。
> 理由：**允许更改所有 QoS 策略可能会打破节点实施者所假定的合同**。

For example, the node's implementer may have assumed that the data was durable and not volatile.

> 例如，节点的实施者可能假定数据是持久的而不是易失的。

- QoS should not be dynamically reconfigurable.

- Rationale: Allowing runtime reconfiguration of QoS is complex, and in many cases it involves tearing down the previous entity and creating a new one.

> QoS 不应该动态重新配置。
> 理由：**允许运行时重新配置 QoS 是复杂的，在许多情况下，它涉及拆除之前的实体并创建一个新的实体**。

> [!NOTE]
> 这里给出了答案，不能允许运行时配置 QoS，这会涉及到销毁之前建立的实体，即重新创建实体
> 和之前添加 ownership 机制的想法的一样的，不能通过获得 data_reader 或者 data_writer 的 handle 来重新配置 qos，这点在 dds 底层的设计实现上也是不支持的，没有对 handles 提供对相应的 api。
> 最后，还是逐层修改源码，添加 qos 的设置，而不是动态的修改。

That process can cause loss of messages, and thus providing an automatic mechanism for it is not ideal.

> 这个过程**可能会导致消息的丢失**，因此提供一个自动机制来解决这个问题并不理想。

> [!NOTE]
> 即，重新创建实体会导致消息的丢失，从而与其他的 QoS 策略产生冲突。

- It should be possible to reuse the same QoS profile for different entities.
- Rationale: Not having a way of setting the same QoS profile to different entities can cause errors, considering that in many cases it's desired to have matching qos.

> 应该可以将相同的 QoS 配置文件用于不同的实体。
> 理由：如果没有一种方法来**设置相同的 QoS 配置文件给不同的实体**，可能会出错，因为在许多情况下，人们希望有相匹配的 QoS。

The following constraint was also considered in the design:

> 在设计中也考虑了以下约束：

- Only apply to publisher/subscriber QoS.
- Rationale: QoS profiles for services and actions are not well defined currently, and modifying them is not recommended.

> 只适用于发布者/订阅者服务质量。
> 理由：目前尚未明确定义服务和操作的 QoS 配置文件，不建议进行修改。

Thus, making them reconfigurable doesn't make much sense right now.

> 因此，现在没有必要使它们可重新配置。

## Adding a new mechanism vs using parameters

A previous proposal (see PR [#296](https://github.com/ros2/design/pull/296)) suggested using a new file format for reconfiguring QoS policies.

> 之前的提案(参见 PR [#296](https://github.com/ros2/design/pull/296))建议使用一种新的文件格式来重新配置 QoS 策略。

Some concerns were raised by the community:

> 社区提出了一些担忧。

1. It adds a different mechanism to the ones that already exist. Newcomers have to learn yet another thing.
2. QoS settings are not conceptually different to other parameters, e.g.: publishing rate.
3. It adds a new mechanism to configure QoS profiles.

> 这增加了另一种机制，这是除已有的机制外的。新来者还得再学一种东西。
> QoS 设置与其他参数(如发布率)没有概念上的不同。
> 它增加了一种新的机制来配置 QoS 配置文件。

Based on that feedback, this PR will explore a design based on parameters.

> 基于反馈，本次提交将探索基于参数的设计。

> [!NOTE]
> 综上所述，即添加 `QosOverridingOptions{true}` 的标志允许通过其他方式对 OoS 策略进行覆盖。
> 或者是指定一个 `callback()` 函数。

## Introduction

Current code will not allow QoS reconfigurability if not changed:

> 当前的代码如果不更改，将不允许 QoS 重新配置。

```cpp
    node->create_publisher(
      "chatter",
      KeepLast{10});
```

To make reconfigurability easy, only a flag is needed to automatically create the parameters:

> 要使重新配置变得容易，只需要一个标志就可以自动创建参数：

```cpp
    node->create_publisher(
      "chatter",
      KeepLast{10},
      QosOverridingOptions{true});  // allow_reconfigurable all qos
```

That will automatically declare the parameters for reconfiguring the QoS policies that can be overridden in the following way:

> 这将自动声明参数以重新配置可以以下列方式**覆盖的 QoS 策略**：

```yaml
/my/ns/node_name:
  ros__parameters:
    qos_overrides: # group for all the qos settings
      "my/fully/qualified/topic/name/here":
        publisher:
          reliability: reliable
          history_depth: 100
          history: keep_last
```

A callback to accept/reject the configured profile can also be specified:

> 也可以指定一个回调来接受/拒绝配置的配置文件：

```cpp
    node->create_publisher(
      "chatter",
      KeepLast(10),
      [] (const QoSProfile & qos) -> bool {
          return qos.is_reliable();  // only accept reliable communication, we should add getters to rclcpp::QoSProfile class for this.
      });
```

The user provided QoS callback will be internally used as a [parameters callback](https://github.com/ros2/rclcpp/blob/3a4ac0ca2093d12035070443692798b0c9f9da3a/rclcpp/include/rclcpp/node_interfaces/node_parameters_interface.hpp#L183), but we provide a more user-friendly interface.

> 用户提供的 QoS 回调将被内部用作[参数回调](https://github.com/ros2/rclcpp/blob/3a4ac0ca2093d12035070443692798b0c9f9da3a/rclcpp/include/rclcpp/node_interfaces/node_parameters_interface.hpp#L183)，但我们提供了一个更加用户友好的界面。

> [NOTE]
> 更详细的内容，参见：`C:\Users\trantor\Downloads\Documents\ChatGPT\dialogue\ros2\qos_20230529.md`

## Analysis of parameter API features and limitations

### Read-only parameters

ROS 2 currently provides read only parameters. They can be modified when constructing a node by providing overrides (e.g. `--ros-args -p <param_name> <param_value>`), but they cannot be dynamically changed after the node was constructed.

> ROS 2 目前只提供只读参数。它们可以通过提供覆盖来在构建节点时修改(例如`--ros-args -p <param_name> <param_value>`)，但在构建节点后不能动态更改。

This perfectly matches the goal of the proposal of not making QoS settings reconfigurable during runtime.

> 这完全符合不在运行时重新配置 QoS 设置的提案的目标。

### Parameter events

Up to ROS 2 Foxy, read-only parameters generate a parameter event with its initial value when declared. If we start declaring parameters for each QoS policies of many entities, the amount of parameter events at startup time will significantly increase, as the declaration of each parameter generates an event.

> 随着 ROS 2 Foxy 的发布，只读参数在声明时会产生一个带有初始值的参数事件。如果我们开始为许多实体的 QoS 策略声明参数，那么启动时的参数事件数量将大大增加，因为每个参数的声明都会产生一个事件。

To avoid this issue, a way to opt-out parameter events when declaring a parameter could be added.

> 为了避免这个问题，可以在声明参数时添加一个选择退出参数事件的方法。

### Parameter callbacks

If we want the following example to work:

> 如果我们想让以下例子运行：

```cpp
    node->create_publisher(
      "chatter",
      KeepLast(10),
      [] (const QoSProfile & qos) -> bool{
          return qos.is_reliable();
      });
```

We could completely ignore parameters [on set callbacks](https://github.com/ros2/rclcpp/blob/3defa8fc9d7410bd833ecd95b305ac94bb9b627a/rclcpp/include/rclcpp/node_interfaces/node_parameters_interface.hpp#L180-L192), the publisher creation code would do something like:

> 我们可以完全忽略参数[在设置回调](https://github.com/ros2/rclcpp/blob/3defa8fc9d7410bd833ecd95b305ac94bb9b627a/rclcpp/include/rclcpp/node_interfaces/node_parameters_interface.hpp#L180-L192)，发布者创建代码可能会像这样：

```cpp
    rcl_interfaces::msg::ParameterDescription policy_x_description{};
    policy_x_description.read_only = true;
    policy_x_description.description = "<string with a description of the policy, including node and topic name>";
    auto policy_x_value = node->declare_parameter(
        "<name of the parameter>", qos_provided_in_code.<getter_for_policy_x>(), policy_x_description);
    // Repeat for all policies
    rclcpp::QoS final_qos();
    // construct `final_qos` from the parameter values read above
    if (!user_provided_callback(final_qos) {
        // qos profile rejected, throw error here
    }
```

In that way, we avoid installing a parameter callback that will be triggered when any parameter is modified.

> 通过这种方式，我们可以避免安装一个参数回调，当任何参数被修改时将触发该回调。

```cpp
  // cpp：C++(计算机编程语言)
  auto callback_handle = node->add_on_set_parameters_callback(wrapper_for_qos_callback(user_provided_callback));
  node->declare_parameters_atomically(...);  // names and defaults for all parameters given here
  // callback_handle goes out of scope, the callback is auto-removed.

```

`declare_parameters_atomically` would be needed, if not you cannot have conditions like `history == keep_all or history_depth > 10`.

> 如果没有`declare_parameters_atomically`，就不能有像`history == keep_all 或 history_depth > 10`这样的条件。

### Parameters URIs

Parameters URIs currently look like:

> 目前参数 URIs 的样式看起来像：

```
rosparam://my/full/namespace/node_name/param_group.nested_param_group.param_name
```

For the following QoS policy:

- Full node name: `/asd/bsd/my_node`
- Entity: `publisher`
- Topic name: `/foo/bar/topic_name`
- Policy: `reliability`

> 完整的节点名称：`/asd/bsd/my_node`
> 实体：pub
> 主题名称：/foo/bar/topic_name
> 政策：可靠性

The URI of the automatically generated parameter would be:

> 自动生成的参数的 URI 将是：

```
rosparam://asd/bsd/my_node/qos_overrides.publisher./foo/bar/topic_name.reliability
```

The `/` in one of the subgroups looks odd, but it's currently accepted.

> 这个子组中的`/`看起来有点奇怪，但目前还是被接受的。

If in the future the addressing changes described in [#241](https://github.com/ros2/design/pull/241) are implemented, the URI would be:

> 如果将来实施[#241](https://github.com/ros2/design/pull/241)中描述的地址变更，URI 将是：

```
rosparam://asd.bsd.my_node/qos_overrides/publisher/foo/bar/topic_name/reliability
```

but in both cases the previous parameters file example wouldn't change.

> 在这两种情况下，先前的参数文件示例不会改变。

## Caveats

### Creating more than one publisher/subscription in the same topic with different QoS

If only the topic name is used for the parameter name, it would be impossible to create two publishers with different QoS in the same topic.

> 如果只使用主题名称作为参数名称，在同一主题中就不可能创建具有不同 QoS 的发布者。

That could be solved by adding an optional extra identifier:

> 那可以通过添加一个可选的额外标识符来解决：

```cpp
    node->create_publisher(
      "chatter",
      KeepLast(10),
      QosOverridingOptions {
        [] (const QoSProfile & qos) -> bool {
            return qos.is_reliable();  // only accept reliable communication
        },
        "id1"
      });
    node->create_publisher(
      "chatter",
      KeepLast(10),
      QosOverridingOptions {
        [] (const QoSProfile & qos) -> bool {
          return !qos.is_reliable();  // only accept best effort communication
        },
        "id2"
      });
```

```yaml
    /my/ns/node_name:
      ros__parameters:
        qos_overrides:
          'my/fully/qualified/topic/name/here':
            publisher_id1:  # {entity_type}_{id}
              reliability: reliable
              history_depth: 100
              history: keep_last
          'my/fully/qualified/topic/name/here':
            publisher_id2:  # {entity_type}_{id}
              reliability: best_effort
              history_depth: 100
              history: keep_last
```

### Lowering the amount of declared parameters

This two subsections describe some optional mechanism to lower the amount of declared parameters. The two mechanism are compatible with what it was said before and are not mutually exclusive.

> 这两个子节描述了一些可选机制来降低声明参数的数量。这两种机制与之前所说的兼容，而且互不排斥。

#### Hidden parameters

Currently, ROS 2 has the concept of hidden topics and services. Those aren't shown by the cli tools, except when explicitly requested:

> 目前，ROS 2 拥有隐藏主题和服务的概念。除非明确请求，否则这些不会由 cli 工具显示：

```bash
ros2 topic list  # Will not list hidden topics.
ros2 topic --include-hidden-topics list  # Will list hidden topics.
```

> [!NOTE]
> 还有隐藏主题？！

Similarly, hidden parameters could be added. All the parameters used for overriding QoS could be declared as hidden, thus avoiding "noise" in commands like `ros2 param list`.

> 同样，可以添加隐藏参数。所有用于覆盖 QoS 的参数都可以声明为隐藏，从而避免“噪声”，如“ros2 param list”命令。

#### Only declaring some parameters

Instead of hidden parameters that correspond to the QoS policies, we could only declare parameters for the policies that the user want to be reconfigurable. That will lower the amount of declared parameters, but make publisher/subscription creation a bit more verbose.

> 代替与 QoS 策略相对应的隐藏参数，我们只能声明用户希望可重新配置的策略参数。这将降低声明参数的数量，但会使发布者/订阅创建变得更加冗长。

Example API:

```cpp
enum class QosPolicyKind {
  Reliability,
  ...,
};

/// Options that are passed in subscription/publisher constructor
class QosOverridingOptions {
    /// Constructor, allowing to declare all the "default" parameters. If `false` is passed, reconfiguring QoS is not allowed.
    explicit QosOverridingOptions(bool declare_default_parameters, std::string id = "");
    /// Implicit constructor, a parameter for each policy provided in the initializer list will be declared.
    QosOverridingOptions(std::initializer_list<QosPolicyKind>, std::string id = "");
    /// Same with a callback for accepting/rejecting the qos profile.
    QosOverridingOptions(std::initializer_list<QosPolicyKind>, QosCallback callback, std::string id = "");
    /// Declares the "default" parameters and pass a callback.
    QosOverridingOptions(QosCallback callback, std::string id = "");
};
```

```cpp
// New method in Node class
template<
  typename MessageT,
  typename AllocatorT = std::allocator<void>,
  typename PublisherT = rclcpp::Publisher<MessageT, AllocatorT>>
  std::shared_ptr<PublisherT>
create_publisher(
  const std::string & topic_name,
  const rclcpp::QoS & qos,
  const rclcpp::QosOverridingOptions & qos_options = QosOverridingOptions{false},
  const PublisherOptionsWithAllocator<AllocatorT> & options = PublisherOptionsWithAllocator<AllocatorT>()
);
```

```cpp
// example usage
node->create_publisher(
    "my_topic",
    user_qos_provided_in_code_1,
    {  // implicit, to lower verbosity
        {QosPolicyKind::Reliability, QosPolicyKind::History, QosPolicyKind::HistoryDepth}, // only declare parameters for history depth, history kind, reliability
        [](const QoSProfile qos) -> bool {
            return qos.is_keep_all() || qos.history_depth() > 10u;  // constrains involving more than one QoS in callbacks
        },
        "my_id"  // id to disambiguate entities in same topic
    });

// nothing is reconfigurable
node->create_publisher(
    "my_topic",
    user_qos_provided_in_code_2);

// allow reconfiguring durability
node->create_publisher(
    "other_topic",
    user_qos_provided_in_code_3,
    QosOverridingOptions{QosPolicyKind::Durability});

// "default" policies are reconfigurable
node->create_publisher(
    "another_topic",
    user_qos_provided_in_code_4,
    QosOverridingOptions{true});
```

The intent of being able to opt-in a set of "default" policies is to make the API easier to use and less verbose.

> 意图能够**选择一组"默认"策略是为了让 API 更容易使用，更少的冗长**。

> [!NOTE]
> 对 QoS 提供默认值的原因、好处
> 对其他的地方也应该是同样的设计理由

All policies could be included in the "set" of default reconfigurable policies, with some exceptions:

> 所有的政策都可以包括在默认可重新配置的政策集合中，但也有一些例外。

- Liviliness kind: It requires special care by the node's author.
- Lifespan: It only applies to publishers, thus it shouldn't be declared for subscribers.

> Liviliness kind：需要节点作者特别照顾。
> **Lifespan：它只适用于 pub，因此不应该声明给 sub**。

> [!NOTE]
> 不太理解这里为啥要说，**指适用于 pub**
> 是从 writer 中删除“过期”数据吗？是的，参考：`C:\Users\trantor\Documents\Hirain\Project\doc\ros2_design\articles\qos_deadline_liveliness_lifespan.md:357`

It also only applies for "transient local" durability, so it shouldn't be reconfigurable if durability isn't.

> 它只适用于“瞬态本地”持久性，因此如果持久性不可用，则不应重新配置。

We could also have a restricted list of default policies to be declared in parameters:

> 我们也可以有一个受限制的默认策略列表，可以在参数中声明。

- Reliability
- History kind
- History depth

> 可靠性
> 历史 类型
> 历史 深度

which are the ones that usually require reconfigurability.

> 哪些通常需要重新配置？

### Reusing profiles

It isn't currently possible to reuse a group of parameters in a parameter file.

> 目前不可能在参数文件中重复使用一组参数。

We could, for example, leverage yaml anchors to allow this:

> 我们可以利用 YAML 锚来实现这一点：

```yaml
    /my/ns/node_name:
      ros__parameters:
        qos_overrides:
          'my/fully/qualified/topic/name/here':
            publisher: &profileA
              reliability: reliable
              history_depth: 1
              history: keep_last
    /my/ns/node_name:
      ros__parameters:
        qos_overrides:
          'my/fully/qualified/topic/name/here':
            subscription:
              <<: *profileA,
              history_depth: 100  # override the history depth
```

## Reconfigurability of topics the ROS 2 core creates

TBD: discuss reconfigurability of topics packages in the ROS 2 core creates here, e.g.: clock, rosout, tf, tf_static, etc.

> 待定：讨论 ROS 2 核心中主题包的可重组性，例如：时钟、rosout、tf、tf_static 等。

### Extending the ROS QoS profile with a rmw vendor specific payload

TBD

## QoS resolution order

- The QoS provided by the user in the publisher/subscription constructor is used as a default.
- Those defaults are overridden with the values provided in parameters, if the user allowed this kind of reconfigurability.
- After the two items above were completed, if a policy value is `RMW_QOS_*_POLICY_DEFAULT` then the qos resolution mechanism provided by the rmw vendor will be applied (e.g. DDS XML profile files).

> 用户在发布者/订阅构造函数中提供的 QoS 将作为默认值使用。
> 如果用户允许这种重新配置，那么用参数提供的值将覆盖这些默认值。
> 在完成上述两项之后，如果策略值为 `RMW_QOS_*_POLICY_DEFAULT`，则将应用 rmw 供应商提供的 qos 解析机制(例如 DDS XML 配置文件)。
