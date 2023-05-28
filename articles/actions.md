---
tip: translate by openai@2023-05-29 08:53:21
layout: default
title: Actions
permalink: articles/actions.html
abstract:
Actions are one of the three core types of interaction between ROS nodes.
This article specifies the requirements for actions, how they've changed from ROS 1, and how they're communicated.
author: '[Geoffrey Biggs](https://github.com/gbiggs) [Jacob Perron](https://github.com/jacobperron) [Shane Loretz](https://github.com/sloretz)'
date_written: 2019-03
last_modified: 2020-05
published: true
Authors: {{ page.author }}
Date Written: {{ page.date_written }}
Last Modified: {% if page.last_modified %}{{ page.last_modified }}{% else %}{{ page.date_written }}{% endif %}
---

## Background

There are three forms of communication in ROS: topics, services, and actions. Topic publishers broadcast to multiple subscribers, but communication is one-way. Service clients send a request to a service server and get a response, but there is no information about the progress. Similar to services, action clients send a request to an action server in order to achieve some goal and will get a result. Unlike services, while the action is being performed an action server sends progress feedback to the client.

> 在 ROS 中有三种通信形式：主题、服务和动作。主题发布者向多个订阅者广播，但通信是单向的。服务客户端向服务服务器发送请求并获得响应，但没有有关进度的信息。与服务类似，动作客户端向动作服务器发出请求以实现某些目标，并获得结果。与服务不同，当动作正在执行时，动作服务器会向客户端发送进度反馈。

Actions are useful when a response may take a significant length of time. They allow a client to track the progress of a request, get the final outcome, and optionally cancel the request before it completes.

> 行动在响应可能需要很长时间时很有用。它们允许客户跟踪请求的进度，获得最终结果，并可选择在完成之前取消请求。

This document defines requirements for actions, how they are specified in the ROS Message IDL, and how they are communicated by the middleware.

> 这份文件定义了动作的要求，它们是如何在 ROS 消息 IDL 中指定的，以及中间件如何传达它们。

## Entities Involved in Actions

There are two entities involved in actions: an **action server** and an **action client**.

> 有两个实体参与行动：一个**动作服务器**和一个**动作客户端**。

### Action Server

An action server provides an action. Like topics and services, an action server has a _name_ and a _type_. The name is allowed to be namespaced and must be unique across action servers. This means there may be multiple action servers that have the same type running simultaneously (under different namespaces).

> 一个动作服务器提供一个动作。像主题和服务一样，动作服务器有一个*名字*和一个*类型*。允许名称有命名空间，并且必须在动作服务器之间是唯一的。这意味着可能有多个具有相同类型的动作服务器同时运行（在不同的命名空间下）。

It is responsible for:

> 它负责：

- advertising the action to other ROS entities

> 宣传这个行动给其他 ROS 实体

- accepting or rejecting goals from one or more action clients

> 接受或拒绝来自一个或多个行动客户的目标

- executing the action when a goal is received and accepted

> 当收到并接受目标时执行该动作

- optionally providing feedback about the progress of all executing actions

> 可选择提供有关所有正在执行操作的进度反馈。

- optionally handling requests to cancel one or more actions

> 可选择性地处理取消一个或多个操作的请求。

- sending the result of a completed action, including whether it succeeded, failed, or was canceled, to a client that makes a result request.

> 将已完成操作的结果（包括是否成功、失败或被取消）发送给发出结果请求的客户端。

### Action Client

An action client sends one or more goals (an action to be performed) and monitors their progress. There may be multiple clients per server; however, it is up to the server to decide how goals from multiple clients will be handled simultaneously.

> 一个动作客户端可以发送一个或多个目标（要执行的动作）并监控其进度。每个服务器可以有多个客户端；但是，服务器要决定如何处理来自多个客户端的目标同时进行。

It is responsible for:

> 它负责：

- sending goals to the action server

> 发送目标到动作服务器

- optionally monitoring the user-defined feedback for goals from the action server

> 可选择监控用户定义的来自动作服务器的目标反馈。

- optionally monitoring the current state of accepted goals from the action server (see [Goal States](#goal-states))

> 可选地监控从行动服务器接受目标的当前状态（参见[目标状态]（＃目标状态））

- optionally requesting that the action server cancel an active goal

> 可选择要求动作服务器取消一个活动目标。

- optionally checking the result for a goal received from the action server

> 选择性地检查从动作服务器接收到的目标的结果

## Differences between ROS 1 and ROS 2 actions

### First Class Support

In ROS 1, actions are implemented as a separate library, [actionlib](http://wiki.ros.org/actionlib) that is built on top of the client libraries. This was done to avoid increasing the work required to create a client library in a new language, but actions turned out to be very important to packages like the [Navigation Stack](http://wiki.ros.org/navigation) and [MoveIt!](https://moveit.ros.org/)<sup>[1](#separatelib)</sup>.

> 在 ROS 1 中，动作被实现为一个单独的库[actionlib](http://wiki.ros.org/actionlib)，它建立在客户端库之上。这样做是为了避免增加在新语言中创建客户端库所需的工作，但是动作最终被证明对[导航堆栈](http://wiki.ros.org/navigation)和[MoveIt！](https://moveit.ros.org/)<sup>[1](#separatelib)</sup>等软件包非常重要。

In ROS 2, actions will be included in the client library implementations. The work of writing a client library in a new language will be reduced by creating a common implementation in C.

> 在 ROS 2 中，动作将包含在客户端库实现中。通过在 C 中创建一个通用实现，将减少编写新语言的客户端库的工作。

### Services used for Actions

In ROS 1, actions were implemented using a set of topics under a namespace taken from the action name. ROS 1 services were not used because they are inherently synchronous and actions need to be asynchronous. Actions also needed to send status/feedback and be cancelable.

> 在 ROS 1 中，动作是使用从动作名称获取的命名空间下的一组主题实现的。ROS 1 服务没有被使用，因为它们本质上是同步的，而动作需要是异步的。动作还需要发送状态/反馈并可以取消。

In ROS 2, services are asynchronous in the common C implementation, so actions will use a combination of services and topics.

> 在 ROS 2 中，服务在常见的 C 实现中是异步的，因此动作将使用服务和主题的组合。

### Goal Identifiers

In ROS 1, an action client can create a goal ID when submitting a goal. This potentially leads to scenarios where mutliple clients independently generate the same goal ID. If an empty goal ID is sent, the action server will create one instead, which is not very useful since the client has no way to know the goal ID.

> 在 ROS 1 中，行动客户端在提交目标时可以创建一个目标 ID。这可能会导致多个客户端独立生成相同的目标 ID 的情况。如果发送空的目标 ID，行动服务器将会创建一个，这对客户端来说没有什么用处，因为客户端无法知道目标 ID。

In ROS 2, action clients will be the sole entities responsible for generating the goal ID. This way clients always know the goal ID for an action. Futhermore, a UUID will be used for each goal to mitigate the issue of goal ID collision across multiple clients.

> 在 ROS 2 中，动作客户端将是唯一负责生成目标 ID 的实体。这样，客户端总是知道一个动作的目标 ID。此外，每个目标将使用 UUID 来解决多个客户端之间的目标 ID 冲突问题。

### Namespacing of Generated Messages and Services

Multiple message and service definitions are generated from a single action definition. In ROS 1, the generated messages were prefixed with the name of the action to avoid conflicts with other messages and services. In ROS 2, the generated service and message definitions will exist in a different namespace to be impossible to conflict with non-action message and service definitions. For example, in Python the code from the generated definitions should be in the module `action` instead of `srv` and `msg`. In C++, the generated code should be in the namespace and folder `action` instead of `srv` and `msg`.

> 当一个动作定义被生成时，多个消息和服务定义也会被生成。在 ROS 1 中，生成的消息会被加上动作的名字以避免与其他消息和服务的冲突。在 ROS 2 中，生成的服务和消息定义会存在于一个不同的命名空间中，这样就不可能与非动作消息和服务定义冲突。例如，在 Python 中，生成的定义代码应该在模块`action`中，而不是`srv`和`msg`中。在 C++中，生成的代码应该在命名空间和文件夹`action`中，而不是`srv`和`msg`中。

### Visibility of Action Services and Topics

In ROS 1, `rostopic list` would show all action topics in its output. In ROS 2, `ros2 topic list` and `ros2 service list` will not show topics and services used by actions by default. They can still be shown by passing an option to the commands to show hidden services and topics. The tool `ros2 action list` will produce list of action names provided by action servers (see [Introspection tools](#introspection-tools)).

> 在 ROS 1 中，`rostopic list`会在其输出中显示所有动作主题。在 ROS 2 中，`ros2 topic list`和`ros2 service list`默认不会显示动作使用的主题和服务。仍然可以通过传递选项给命令来显示隐藏的服务和主题。工具`ros2 action list`将产生由动作服务器提供的动作名称列表（参见[内省工具]（#introspection-tools））。

## Action Interface Definition

Actions are specified using a form of the ROS Message IDL. The specification contains three sections, each of which is a message specification:

> 行动是使用 ROS 消息 IDL 的形式来指定的。该规范包含三个部分，每个部分都是消息规范：

1. Goal

> 目标

This describes what the action should achieve and how it should do it.

> 这描述了该行动应该达到的目标以及它应该如何实现这一目标。

It is sent to the action server when it is requested to execute an action.

> 它在请求执行操作时被发送到动作服务器。

1. Result

> 结果

This describes the outcome of an action.

> 这描述了一个行动的结果。

It is sent from the server to the client when the action execution ends, whether successfully or not.

> 它在操作执行结束时从服务器发送到客户端，无论是否成功。

1. Feedback

> 反馈

This describes the progress towards completing an action.

> 这描述了完成一项行动的进展情况。

It is sent to the client of the action from the action server between commencing action execution and prior to the action completing.

> 这是从行动服务器发送给行动客户端的，在开始行动执行和行动完成之前。

This data is used by the client to understand the progress of executing the action.

> 这些数据被客户用来了解执行操作的进度。

Any of these sections may be empty. Between each of the three sections is a line containing three hyphens, `---`. Action specifications are stored in a file ending in `.action`. There is one action specification per `.action` file.

> 任何这些部分都可以为空。每三个部分之间都有一行包含三个连字符，`---`。动作规范存储在以`.action`结尾的文件中。每个`.action`文件有一个动作规范。

### Example

```
      # Define a goal of washing all dishes
      bool heavy_duty  # Spend extra time cleaning
      ---
      # Define the result that will be published after the action execution ends.
      uint32 total_dishes_cleaned
      ---
      # Define a feedback message that will be published during action execution.
      float32 percent_complete
      uint32 number_dishes_cleaned
```

## Introspection tools

Actions, like topics and services, are introspectable from the command line.

> 行为，就像主题和服务一样，可以从命令行中内省。

The command line tool, `ros2 action`, will be able to:

> 命令行工具“ros2 action”能够：

- list action names associated with any running action servers or action clients

> - 列出与任何正在运行的动作服务器或动作客户端相关联的动作名称

- list action servers and action clients

> - 列出动作服务器和动作客户端

- display active goals on an action server

> 显示行动服务器上的活跃目标

- display the arguments for an action goal

> 显示行动目标的参数

- display the type of an action's goal, feedback, and result

> 显示行动的目标、反馈和结果的类型

- find actions by action type

> 按动作类型查找动作

- echo feedback, status, and result for an action goal

> 回显反馈、状态和行动目标的结果

- call an action, display feedback as it is received, display the result when received, and cancel the action (when the tool is terminated prematurely).

> 调用一个动作，实时显示反馈，收到结果时显示结果，如果工具被意外终止，取消该动作。

Each action will be listed and treated as a single unit by this tool.

> 这个工具会将每个行为列出并视为单独的单元。

## Goal States

The action server maintains a state machine for each goal it accepts from a client. Rejected goals are not part of the state machine.

> 服务器为每个从客户端接受的目标维护一个状态机。被拒绝的目标不属于状态机。

![Action Goal State Machine](../img/actions/goal_state_machine.png)

There are three active states:

> 有三个活跃状态：

- **ACCEPTED** - The goal has been accepted and is awaiting execution.

> - **已接受** - 目标已被接受，正在等待执行。

- **EXECUTING** - The goal is currently being executed by the action server.

> - **正在执行** - 行动服务器正在执行目标。

- **CANCELING** - The client has requested that the goal be canceled and the action server has accepted the cancel request.

> - **取消** - 客户已请求取消目标，动作服务器已接受取消请求。

This state is useful for any user-defined "clean up" that the action server may have to do.

> 这个状态对于动作服务器可能需要做的任何用户定义的“清理”都很有用。

And three terminal states:

> 和三个终端状态：

- **SUCCEEDED** - The goal was achieved successfully by the action server.

> - **成功** - 行动服务器已成功完成目标。

- **ABORTED** - The goal was terminated by the action server without an external request.

> - **中止** - 没有外部请求，目标已经被行动服务器终止。

- **CANCELED** - The goal was canceled after an external request from an action client.

> -**已取消**-由行动客户的外部请求而取消目标。

State transitions triggered by the action server according to its designed behavior:

> 根据设计行为，动作服务器触发的状态转换：

- **execute** - Start execution of an accepted goal.

> 执行 - 开始执行已接受的目标。

- **succeed** - Notify that the goal completed successfully.

> - **成功** - 通知目标已经成功完成。

- **abort** - Notify that an error was encountered during processing of the goal and it had to be aborted.

> - **中止** - 通知在处理目标期间遇到错误，必须中止。

- **canceled** - Notify that canceling the goal completed successfully.

> - **取消** - 通知完成取消目标的成功。

State transitions triggered by the action client:

> 客户端触发的状态转换

- **send_goal** - A goal is sent to the action server. The state machine is only started if the action server _accepts_ the goal.

> - **发送目标** - 向动作服务器发送一个目标。 只有在动作服务器*接受*目标时，状态机才会启动。

- **cancel_goal** - Request that the action server stop processing the goal. A transition only occurs if the action server _accepts_ the request to cancel the goal.

> - **取消目标** - 请求行动服务器停止处理目标。只有当行动服务器*接受*取消目标的请求时，才会发生转换。

## API

Usage examples can be found in the [examples](https://github.com/ros2/examples) repository.

> 可以在[示例](https://github.com/ros2/examples)仓库中找到使用示例。

C++:

> C++：中文简体

- [examples/rclcpp/minimal_action_server](https://github.com/ros2/examples/tree/master/rclcpp/actions/minimal_action_server)
- [examples/rclcpp/minimal_action_client](https://github.com/ros2/examples/tree/master/rclcpp/actions/minimal_action_client)

Python:

> Python：

- [examples/rclpy/actions](https://github.com/ros2/examples/tree/master/rclpy/actions)

## Middleware implementation

Under the hood, an action is made up of three services and two topics. In this section, they are descibed in detail.

> 在引擎盖下，一个行动由三个服务和两个主题组成。在本节中，它们将详细描述。

![Action Client/Server Interaction Overview](../img/actions/interaction_overview.png)

### Send Goal Service

- **Direction**: client calls server

> 客户端调用服务器

- **Request**: description of goal and a UUID for the goal ID

> 请求：目标的描述和用于目标 ID 的 UUID

- **Response**: whether goal was accepted or rejected and the time when the goal was accepted

> - **回复**：目标是否被接受或拒绝以及目标被接受的时间。

The purpose of this service is to send a goal to the action server. It is the first service called to begin an action, and is expected to return quickly. The description of the goal in the request is user-defined as part of the [Action Interface Definition](#action-interface-definition).

> 本服务的目的是向动作服务器发送一个目标。它是第一个被称为开始动作的服务，预计会迅速返回。请求中的目标描述是用户定义的[动作接口定义]的一部分。

The QoS settings of this service should be set so the client is guaranteed to receive a response or an action could be executed without a client being aware of it.

> 本服务的 QoS 设置应该设置得如此，客户端可以保证收到回复，或者在客户端不知情的情况下执行某项操作。

### Cancel Goal Service

- **Direction**: client calls server

> 客户端调用服务器

- **Request**: goal ID and timestamp

> 请求：目标 ID 和时间戳

- **Response**: response code and a list of goals that have transitioned to the CANCELING state

> -**响应**：响应代码和一个已转换到取消状态的目标列表

The purpose of this service is to request the cancellation of one or more goals on the action server. The response code indicates any failures in processing the request (e.g. `OK`, `REJECTED` or `INVALID_GOAL_ID`). The list of goals in the response indicates which goals will be attempted to be canceled. Whether or not a goal transitions to the CANCELED state is indicated by the status topic and the result service.

> 此服务的目的是请求取消动作服务器上的一个或多个目标。响应代码表示处理请求时的任何失败（例如`OK`、`REJECTED`或`INVALID_GOAL_ID`）。响应中的目标列表指示将尝试取消哪些目标。目标是否转换为 CANCELED 状态由状态主题和结果服务指示。

The cancel request policy is the same as in ROS 1.

> 取消请求政策与 ROS 1 相同。

- If the goal ID is empty and timestamp is zero, cancel all goals

> 如果目标 ID 为空，时间戳为零，取消所有目标。

- If the goal ID is empty and timestamp is not zero, cancel all goals accepted at or before the timestamp

> 如果目标 ID 为空，且时间戳不为零，则取消在时间戳之前接受的所有目标。

- If the goal ID is not empty and timestamp is zero, cancel the goal with the given ID regardless of the time it was accepted

> 如果目标 ID 不为空，且时间戳为零，则无论接受时间如何，都取消具有给定 ID 的目标。

- If the goal ID is not empty and timestamp is not zero, cancel the goal with the given ID and all goals accepted at or before the timestamp

> 如果目标 ID 不为空且时间戳不为零，则取消具有给定 ID 的目标以及在时间戳之前或之后接受的所有目标。

### Get Result Service

- **Direction**: client calls server

> -**方向**：客户端调用服务器

- **Request**: goal ID

> 请求：目标 ID

- **Response**: status of goal and user defined result

> - **回应**：目标状态和用户定义的结果

The purpose of this service is to get the final result of a goal. After a goal has been accepted the client should call this service to receive the result. The result will indicate the final status of the goal and any user defined data as part of the [Action Interface Definition](#action-interface-definition).

> 本服务的目的是获取目标的最终结果。在接受目标后，客户端应调用此服务以接收结果。结果将指示目标的最终状态以及作为[动作接口定义](#action-interface-definition)一部分的任何用户定义数据。

#### Result caching

The server should cache the result once it is ready so multiple clients have to opportunity to get it. This is also useful for debugging/introspection tools.

> 服务器在结果准备就绪时应该缓存结果，以便多个客户端有机会获取它。这对于调试/内省工具也很有用。

To free up resources, the server should discard the result after a configurable timeout period. The timeout can be set as part of options to the action server. If the timeout is configured to have value **-1**, then goal results will be "kept forever" (until the action server shuts down). If the timeout is configured to have value **0**, then goal results are discarded immediately (after responding to any pending result requests).

> 为了释放资源，服务器应该在可配置的超时期限之后丢弃结果。超时可以作为动作服务器的选项之一设置。如果超时被配置为**-1**，那么目标结果将被“永久保留”（直到动作服务器关闭）。如果超时被配置为**0**，那么目标结果将立即被丢弃（在响应任何待处理结果请求之后）。

### Goal Status Topic

- **Direction**: server publishes

> 方向：服务器发布

- **Content**: list of in-progress goals with goal ID, time accepted, and an enum indicating the status

> 内容：具有目标 ID、接受时间和指示状态的正在进行的目标列表。

This topic is published by the server to broadcast the status of goals it has accepted. The purpose of the topic is for introspection; it is not used by the action client. Messages are published when transitions from one status to another occur.

> 这个主题是由服务器发布的，用于广播它所接受的目标的状态。该主题的目的是用于内省；它不被动作客户端使用。当从一个状态转换到另一个状态时，会发布消息。

The QoS settings for an action server can be configured by the user. The default QoS settings for a DDS middleware should be TRANSIENT LOCAL with a history size of 1. This allows new subscribers to always get the latest state.

> 用户可以配置操作服务器的 QoS 设置。DDS 中间件的默认 QoS 设置应为 TRANSIENT LOCAL，历史记录大小为 1。这样可以让新的订阅者总能得到最新的状态。

The possible statuses are:

> 可能的状态有：

- _Accepted_ - The goal has been accepted by the action server

> - 接受 - 目标已被行动服务器接受

- _Executing_ - The goal is currently being executing by the action server

> -_正在执行_ - 目标目前正由行动服务器执行

- _Canceling_ - The action server will try to cancel the indicated goal

> -_取消_ - 服务器将尝试取消指定的目标。

- _Succeeded_ - The action server successfully reached the goal

> - 成功 - 行动服务器成功达到目标

- _Aborted_ - The action server failed reached the goal

> - _中止_ - 行動伺服器未能達到目標

- _Canceled_ - The action server successfully canceled the goal

> - _已取消_ - 服务器成功取消了目标

### Feedback Topic

- **Direction**: server publishes

> - **方向**：服务器发布

- **Content**: goal ID, user defined feedback message

> 内容：目标 ID，用户自定义反馈消息

This topic is published by the server to send progress feedback about the goal that is user-defined as part of the [Action Interface Definition](#action-interface-definition). It is up to the author of the action server to decide how often to publish the feedback.

> 这个主题是由服务器发布的，用于发送有关用户定义的目标的进度反馈，这是[动作接口定义](#action-interface-definition)的一部分。动作服务器的作者决定发布反馈的频率。

The QoS settings for feedback coming from an action server can be configured by the user. It is up to the clients to use compatible QoS settings.

> 用户可以配置反馈来自动作服务器的 QoS 设置。客户端使用兼容的 QoS 设置取决于客户端。

### Client/Server Interaction Examples

Here are a couple of sequence diagrams depicting typical interactions between an action client and action server.

> 这里有几个序列图描述了动作客户端和动作服务器之间典型交互的情况。

#### Example 1

In this example, the action client requests a goal and gets a response from the server accepting the goal (synchronous). Upon accepting the goal, the action server starts a user defined execution method for completing the goal. Following the goal request, the client makes an asynchronous request for the result. The user defined method publishes feedback to the action client as it executes the goal. Ultimately, the user defined method populates a result message that is used as part of the result response.

> 在这个例子中，动作客户端请求一个目标，并从服务器获得接受目标的响应（同步）。接受目标后，动作服务器开始执行用户定义的执行方法来完成目标。在目标请求之后，客户端发出异步请求以获取结果。用户定义的方法在执行目标时向动作客户端发布反馈。最终，用户定义的方法填充一个结果消息，用作结果响应的一部分。

![Action Interaction Example 0](../img/actions/interaction_example_0.png)

#### Example 2

This example is almost identical to the first, but this time the action client requests for the goal to be canceled mid-execution. Note that the user defined method is allowed to perform any shutdown operations after the cancel request before returning with the cancellation result.

> 这个例子几乎和第一个相同，但这次行动客户端要求在执行过程中取消目标。注意，在取消请求之后，用户定义的方法可以执行任何关闭操作，然后返回取消结果。

![Action Interaction Example 1](../img/actions/interaction_example_1.png)

#### Example 3

Here is a more complex example involving multiple goals.

> 这里有一个涉及多个目标的更复杂的例子。

![Action Interaction Example 2](../img/actions/interaction_example_2.png)

### Topic and Service Name Examples

Topic and service names for actions will include a token `_action`. The leading underscore makes the name hidden, and the combined text allows tools to identify topics and services used by actions. Topic and Service names will be generated by prefixing the action name to one of `/_action/status`, `/_action/feedback`, `/_action/get_result`, `/_action/cancel_goal`, or `/_action/send_goal`. The resulting topic or service name is expanded to a fully qualified name as usual.

> 主题和服务名称的行动将包括一个令牌`_action`。前导下划线使名称隐藏，组合文本允许工具识别行动使用的主题和服务。主题和服务名称将通过将行动名称前缀添加到`/_action/status`、`/_action/feedback`、`/_action/get_result`、`/_action/cancel_goal`或`/_action/send_goal`之一而生成。随后，扩展的主题或服务名称通常会进行完全限定。

#### Example: fully qualified action name

Given:

> 给予：

- Action name: `/action/name`

> 行动名称：`/action/name`

- Namespace: `/name/space`

> 命名空間：`/name/space`

- Node name: `nodename`

> 节点名称：nodename

Topic Names:

> 主题名称：

- `/action/name/_action/status`

> - `/行动/名称/_行动/状态`

- `/action/name/_action/feedback`

> `-`/行动/名称/\_行动/反馈`

Service Names:

> 服务名称

- `/action/name/_action/send_goal`

> `-`/动作/名称/\_动作/发送目标`

- `/action/name/_action/cancel_goal`

> - `/行动/名称/_行动/取消_目标`

- `/action/name/_action/get_result`

> `-` /操作/名称/*操作/获取*结果

#### Example: relative action name

Given:

> 给出：

- Action name: `action/name`

> 行动名称：`action/name`

- Namespace: `/name/space`

> 命名空间：`/name/space`

- Node name: `nodename`

> - 节点名称：`nodename`

Topic Names:

> 主题名称：

- `/name/space/action/name/_action/status`

> `- /名稱/空間/行動/名稱/_行動/狀態`

- `/name/space/action/name/_action/feedback`

> - `/名字/空间/行动/名字/_行动/反馈`

Service Names:

> 服务名称

- `/name/space/action/name/_action/send_goal`

> - `/名称/空间/行动/名称/_行动/发送目标`

- `/name/space/action/name/_action/cancel_goal`

> - `/名称/空间/行动/名称/_行动/取消_目标`

- `/name/space/action/name/_action/get_result`

> `- `/名称/空间/行动/名称/*行动/获得*结果`

#### Example: private action name

Given:

> 给定：

- Action name: `~/action/name`

> 行动名称：~/action/name

- Namespace: `/name/space`

> 命名空间：/name/space

- Node name: `nodename`

> 节点名称：nodename

Topic Names:

> 主题名称：

- `/name/space/nodename/action/name/_action/status`

> - `/名字/空间/节点名/行动/名称/_行动/状态`

- `/name/space/nodename/action/name/_action/feedback`

> - `/名字/空间/节点名/行动/名称/_行动/反馈`

Service Names:

> 服务名称：

- `/name/space/nodename/action/name/_action/send_goal`

> - `/名称/空间/节点名称/行动/名称/_行动/发送目标`

- `/name/space/nodename/action/name/_action/cancel_goal`

> - `/名称/空间/节点名称/动作/名称/_动作/取消_目标`

- `/name/space/nodename/action/name/_action/get_result`

> - `/名字/空间/节点名/行动/名称/_行动/获取结果`

## Bridging between ROS 1 and ROS 2

TODO

> 待办

## Alternatives

These alternative approaches to actions in ROS 2 were considered.

> 这些针对 ROS 2 行动的替代方法已经被考虑过了。

### Actions in rmw

An alternative to using services and topics is to implement actions in the rmw layer. This would enable using middleware specific features better suited to actions. The default middleware in ROS 2 uses DDS, and there don't appear to be any DDS features better for actions than what are used for services and topics. Additionally implementing actions in the rmw implementations increases the complexity of writing an rmw implementation. For these reasons actions will be implemented at a higher level.

> 另一种方法是在 rmw 层实现动作，而不是使用服务和主题。这将使用更适合动作的特定中间件功能。ROS 2 中的默认中间件使用 DDS，似乎没有任何 DDS 功能比服务和主题更适合动作。此外，在 rmw 实现中实施动作会增加编写 rmw 实现的复杂性。出于这些原因，动作将在更高层次实现。

### Multiple topics for feedback and status

When there are many goals from many clients, the choice to have a single feedback (and status) topic per action server is suboptimal in terms of processing and bandwidth resource. It is up to clients to filter out feedback/status messages that are not pertinent to them. In this scenario, M goals are sent to N clients there is an unecessary use of bandwidth and processing; especially in extreme cases where M and N are large.

> 当有许多客户端的目标时，将每个动作服务器的反馈（和状态）主题设置为单一的是在处理和带宽资源方面不太理想的选择。客户端需要过滤掉与他们无关的反馈/状态消息。在这种情况下，M 个目标被发送给 N 个客户端，会不必要地使用带宽和处理能力，特别是在 M 和 N 都很大的极端情况下。

One approach is to use multiple topics (distinguished by goal IDs) or the "keyed data" feature in DDS that allows for "content filtered subscription". This would allow clients to only subscribe to the feedback and status messages that they care about.

> 一种方法是使用多个主题（由目标 ID 区分）或 DDS 中的“键数据”功能，允许进行“内容过滤订阅”。这将允许客户只订阅他们关心的反馈和状态消息。

A second approach is to give clients the option to specify a custom feedback topic as part of the goal request. This would be useful to alleviate extreme cases without the overhead of creating/destroying topics for every goal when the number of goals/clients is small.

> 第二种方法是让客户在请求目标时指定一个自定义反馈主题。当目标/客户的数量很少时，这将有助于缓解极端情况，而无需为每个目标创建/销毁主题。

Reasons against using separate topics for feedback and status:

> 反对使用不同主题来反馈和状态的原因：

- Most anticipated use cases will not involve many goals/clients (premature optimization)

> 最期待的用例不会涉及太多目标/客户（过早的优化）

- Topics dynamically namespaced (e.g. by goal ID) would complicate ROS security by not having deterministic topic names before runtime and outside user control.

> 动态命名空间的主题（例如通过目标 ID）会使 ROS 安全变得复杂，因为在运行时和用户控制之外没有确定性的主题名称。

- Added complexity in C implementation and client libraries

> 在 C 实现和客户端库中增加了复杂性

It seems reasonable in the future that the "keyed data" feature in DDS can be employed to reduce overhead in the "many goal/client" scenario. This will require that the feature be exposed in the middleware, which it is not at the time of writing this proposal.

> 似乎在未来，DDS 中的“键入数据”功能可以用于减少“多目标/客户端”场景中的开销。这需要在中间件中暴露该功能，但在撰写本提案时，尚未暴露该功能。

### Server-side goal ID generation

Since new goals are created by an action client making a service call to an action server, it is possible for the action server to generate the UUID for the goal and return it as part of the service response to the action client. The action server can better ensure uniqueness of the goal ID with this approach, but should still handle the potential race of two goal requests arriving simulataneously.

> 由于新目标是由动作客户端通过调用动作服务器创建的，因此动作服务器可以为目标生成 UUID 并将其作为服务响应的一部分返回给动作客户端。采用此方法，动作服务器可以更好地确保目标 ID 的唯一性，但仍应处理两个目标请求同时到达的潜在竞争。

On the other hand, it would be nice to expose the method for generating goal IDs and let the user correlate the goals with other libraries/systems. Imagine an action client being used to request a robot to perform tasks from a database where each task already has a UUID associated with it. In this case, it is still the action servers responsibility to ensure goal ID uniqueness and handle any potential races with concurrent goal requests, but we have the added benefit of the user being able to correlate goals to other existing entities.

> 另一方面，暴露用于生成目标 ID 的方法并允许用户将目标与其他库/系统相关联将是很好的。想象一下，一个动作客户端正在使用数据库中的每个任务都具有关联的 UUID 来请求机器人执行任务。在这种情况下，确保目标 ID 的唯一性以及处理任何潜在的与并发目标请求的竞争仍然是动作服务器的责任，但是我们有了额外的好处，即用户能够将目标与其他现有实体相关联。

## References

1. <a name="separatelib" href="https://discourse.ros.org/t/actions-in-ros-2/6254/5">https://discourse.ros.org/t/actions-in-ros-2/6254/5</a>

> 1. <a name="separatelib" href="https://discourse.ros.org/t/actions-in-ros-2/6254/5">https://discourse.ros.org/t/actions-in-ros-2/6254/5</a>： ROS 2 中的操作。
