---
tip: translate by openai@2023-05-29 23:03:18
layout: default
title: ROS 2 Launch System
permalink: articles/roslaunch.html
abstract:
  The launch system in ROS is responsible for helping the user describe the configuration of their system and then execute it as described. The configuration of the system includes what programs to run, where to run them, what arguments to pass them, and ROS specific conventions which make it easy to reuse components throughout the system by giving them each different configurations. Also, because the launch system is the process (or the set of processes) which executes the user's processes, it is responsible for monitoring the state of the processes it launched, as well as reporting and/or reacting to changes in the state of those processes.
author: '[William Woodall](https://github.com/wjwwood)'
date_written: 2019-09
last_modified: 2019-09
published: true
Authors: 
Date Written: 
Last Modified:
---
## Context

This article describes the launch system for ROS 2, and as the successor to the launch system in ROS 1 it makes sense to summarize the features and roles of `roslaunch` from ROS 1 and compare them to the goals of the launch system for ROS 2.

> 这篇文章描述了 ROS 2 的 launch 系统，作为 ROS 1 launch 系统的继任者，有必要总结一下 roslaunch 在 ROS 1 中的功能和角色，并将其与 ROS 2 launch 系统的目标进行比较。

### Description of `roslaunch` from ROS 1

From the description of `roslaunch` from the wiki ([https://wiki.ros.org/roslaunch](https://wiki.ros.org/roslaunch)):

> 从 roslaunch 的描述([https://wiki.ros.org/roslaunch](https://wiki.ros.org/roslaunch))：roslaunch 是 ROS 系统中一种用于启动和管理多个 ROS 节点的命令行工具。它可以同时加载和启动一组节点，并且可以自动处理依赖关系，以便节点在正确的顺序启动。它还可以动态地更改参数，以便在运行时调整系统行为。

roslaunch is a tool for easily launching multiple ROS nodes locally and remotely via SSH, as well as setting parameters on the Parameter Server. It includes options to automatically respawn processes that have already died. roslaunch takes in one or more XML configuration files (with the .launch extension) that specify the parameters to set and nodes to launch, as well as the machines that they should be run on.

> roslaunch 是一个方便地在本地和通过 SSH 远程启动多个 ROS 节点以及在参数服务器上设置参数的工具。它包括自动重新启动已经死亡进程的选项。roslaunch 接受一个或多个 XML 配置文件(扩展名为.launch)，指定要设置的参数和要启动的节点以及它们应该在哪台机器上运行。

This description lays out the main roles of `roslaunch` from ROS 1 as:

> 这个描述概述了 ROS 1 中 `roslaunch` 的主要角色：

- launch nodes
- launching nodes remotely via SSH
- setting parameters on the parameter server
- automatic respawning of processes that die
- static, XML based description of the nodes to launch, parameters to set, and where to run them

Further more the wiki goes on to say ([https://wiki.ros.org/roslaunch/Architecture](https://wiki.ros.org/roslaunch/Architecture)): roslaunch was designed to fit the ROS architecture of complexity via composition: write a simple system first, then combine it with other simple systems to make more complex systems. In roslaunch, this is expressed through several mechanisms:

> 更进一步，维基还说([https://wiki.ros.org/roslaunch/Architecture](https://wiki.ros.org/roslaunch/Architecture))：ROSlaunch 旨在通过组合实现 ROS 复杂性的体系结构：首先编写一个简单的系统，然后将其与其他简单系统组合起来，以构建更复杂的系统。在 roslaunch 中，这一点通过几种机制表达出来：

1. `<include>` s: you can easily include other .launch files and also assign them a namespace so that their names do not confict with yours.

> 1. `<include>`: 你可以轻松地包含其他的.launch 文件，并且给它们分配一个命名空间，以便它们的名字不会与你的名字冲突。

2. `<group>` s: you can group together a collection of nodes to give them the same name remappings.

> 2. < 群组 >：您可以将一组节点组合起来，给它们相同的名称重映射。

3. aliased `<machine>` s: you can separate machine definitions and node definitions into separate .launch files and use aliases to define which machines get used at runtime. This allows you to reuse the same node definitions for multiple robots. For example, instead of saying that a laser_assembler runs on 'foo.willowgarage.com', you can say that it runs on the 'tilt-laser' machine. The machine definitions then take care of which host is the 'tilt-laser' machine.

> 您可以将机器定义和节点定义分开放入单独的.launch 文件中，并使用别名来定义在运行时使用哪些机器。这样可以让您为多个机器重复使用相同的节点定义。例如，而不是说 laser_assembler 运行在'foo.willowgarage.com'上，您可以说它运行在'tilt-laser'机器上。然后机器定义将负责哪台主机是“tilt-laser”机器。

roslaunch also contains a variety of tools to help you write your .launch files as portably as possible. You can use the `<env>` tag to specify environment variables that need to be set for a particular machine or node. The $(find pkg) syntax let you specify file paths relative to a ROS package, instead of specifying their location on a particular machine. You can also use the $(env ENVIRONMENT_VARIABLE) syntax within include tags to load in .launch files based on environment variables (e.g. MACHINE_NAME).

> roslaunch 也包含各种工具，帮助您尽可能端口化地编写.launch 文件。您可以使用 `<env>` 标签指定特定机器或节点需要设置的环境变量。$(find pkg)语法允许您相对于ROS包指定文件路径，而不是指定其在特定机器上的位置。您还可以在include标签中使用$(env ENVIRONMENT_VARIABLE)语法来加载基于环境变量(例如 MACHINE_NAME)的.launch 文件。`

From this, there are a few more design goals and roles for `roslaunch` from ROS 1:

> 从这里，ROS 1 中有更多的设计目标和 roslaunch 的角色。

- composition of systems into systems of systems to manage complexity
- use include semantic to reuse fragments rather than writing each from scratch
- use groups to apply settings (e.g. remappings) to collections of nodes/processes/included launch files
  - also use groups with namespaces to form hierarchies
- portability through abstraction of operating system concepts, e.g. environment variables
- utilities to locate files on the filesystem in a relocatable and portable way, e.g. `$(find <package_name>)`

That covers most of the features and design goals of `roslaunch` from ROS 1, but in the next subsection we'll discuss what is different for the launch system in ROS 2 due to changes in ROS 2 and how it might improve on the launch system from ROS 1.

> 这涵盖了 ROS 1 中 `roslaunch` 的大多数功能和设计目标，但在下一小节中，我们将讨论由于 ROS 2 的变化而对 ROS 2 中启动系统有何不同，以及它如何改进 ROS 1 中的启动系统。

### Differences in ROS 2

One of the objectives of the launch system in ROS 2 is to emulate the features of the launch system in ROS 1, but due to architectural changes in ROS 2, some of the features, goals, and terminology need to change.

> 一个 ROS 2 中 launch 系统的目标是模拟 ROS 1 中 launch 系统的功能，但由于 ROS 2 的架构变化，一些功能、目标和术语需要更改。

#### Relationship Between Nodes and Processes

In ROS 1, there could only ever be one node per process and so the goals of `roslaunch` from ROS 1 reflect that by using "ROS nodes" and "processes" almost interchangeably.

> 在 ROS 1 中，每个进程只能有一个节点，因此 ROS 1 中 `roslaunch` 的目标几乎可以互换地使用“ROS 节点”和“进程”。

Even for the ROS 1 feature called 'nodelet' (where you could emulate having more than one node per process), the conceptual mapping from node or nodelet to process was preserved by proxy processes. For example, you would run a "NodeletManager" and then run a process for each nodelet you wanted to run in that manager. This allowed nodelet's which exited to be detected by `roslaunch` from ROS 1, as well as allowing them to respond to signals that it sent to the proxy process.

> 即使是 ROS 1 的一个叫做 **“nodelet”的功能(可以模拟一个进程中有多个节点)** ，节点或 nodelet 到进程的概念映射也是通过代理进程保留的。例如，你可以运行一个“NodeletManager”，然后为要在该管理器中运行的每个 nodelet 运行一个进程。这允许 ROS 1 中的 roslaunch 检测到退出的 nodelet，并允许它们响应代理进程发送的信号。

Since you can have many nodes per process in ROS 2, it is no longer necessary to conflate nodes and processes. Due to this, the design and documentation for the launch system in ROS 2 will need to be clearer when talking about processes and nodes. Additionally, the way that configuration (e.g. parameters and remappings) get passed to nodes by the launch system needs to be adapted, though this part overlaps with the design documents for static remapping_remapping[remappingemapping] and for parameters[^parameters].

> 由于 ROS 2 中每个进程可以拥有多个节点，因此不再需要把节点和进程合并。因此，ROS 2 中的启动系统的设计和文档在谈论进程和节点时需要更加清晰。此外，启动系统将配置(例如参数和重映射)传递给节点的方式也需要进行调整，尽管这部分与静态重映射_remapping[remappingemapping]和参数[^parameters]的设计文档有重叠。

Also, since there can be multiple nodes per process, shutting down a node no longer always means sending a unix signal to a single process. Other mechanisms might need to be used to have more granular shutdown control in multi-node processes.

> 也由于每个进程可以有多个节点，关闭节点不再总是意味着向单个进程发送 Unix 信号。为了在多节点进程中拥有更细粒度的关闭控制，可能需要使用其他机制。

#### Launching Nodes (Processes) Remotely and Portability

The launch system in ROS 1 only really ever was supported on Linux and other Unix-like operating systems like BSD and macOS. These machines all have SSH, which is the mechanism which is specifically called out to be used when launching processes on remote machines. It also played a role in defining what you specified and how when configuring `roslaunch` from ROS 1 to be able to launch processes on remote machines.

> 系统 launch 在 ROS 1 中只支持 Linux 和其他类 Unix 操作系统，如 BSD 和 macOS。这些机器都有 SSH，这是在远程机器上启动进程时特别提到的机制。当从 ROS 1 配置“roslaunch”以在远程机器上启动进程时，它还发挥了一定的作用，以确定您指定的内容和方式。

In ROS 2, Windows has been added to the list of targeted platforms, and as of the writing of this document it does not support SSH natively. So unless that changes (more possible than it sounds), a different, more portable mechanism might be required to support this feature everywhere. At the very least, an alternative solution would need to be used on Windows even if SSH was still used on Unix-like operating systems.

> 在 ROS 2 中，Windows 已经添加到目标平台列表中，但是在此文档撰写时，它不支持 SSH 本机。因此，除非发生变化(可能性比它听起来更大)，否则可能需要使用不同的、更可移植的机制来支持这一功能。即使在类 Unix 操作系统上仍使用 SSH，也至少需要在 Windows 上使用替代方案。

#### Parameters

In ROS 1, there was a global parameter server which stored all parameters and nodes would get and set all parameters through this server. The server was tightly integrated into `roslaunch` from ROS 1, and was also used by the other kind of parameters from ROS 1, which were called "dynamic reconfigure parameters".

> 在 ROS 1 中，有一个全局参数服务器，它存储所有参数，节点可以通过这个服务器获取和设置所有参数。该服务器与 ROS 1 中的 `roslaunch` 紧密集成，也用于 ROS 1 中的其他类型参数，称为“动态重新配置参数”。

In ROS 2, there are only one kind of parameters and they work differently. In general they work more like "dynamic reconfigure parameters" from ROS 1, in that they are node specific (no truly global parameters) and they are managed by the node (the node can refuse changes and parameters can only be read and changed while the node is running). More details can be found in the parameters design document[^parameters].

> 在 ROS 2 中，只有一种参数，它们的工作方式不同。通常，它们更像 ROS 1 中的“动态重新配置参数”，因为它们是节点特定的(没有真正的全局参数)，并且由节点管理(节点可以拒绝更改，只能在节点运行时读取和更改参数)。更多细节可以在参数设计文档中找到[^parameters]。

There can (and probably will) still be a "global parameter server" in ROS 2, but it will simply be implemented as a node which accepts all changes and could be run along with the launch system automatically or could be invoked explicitly by the user (a la `roscore` from ROS 1), but it should not be required for basic functionality.

> 在 ROS 2 中可能(也可能会)仍然有一个“全局参数服务器”，但它将被实现为一个节点，可以接受所有更改，可以与启动系统一起自动运行，或者可以被用户(类似 ROS 1 中的 `roscore`)显式调用，但不需要基本功能。

This fundamental difference in how parameters work will affect both the architecture of the launch system in ROS 2 and how users specify parameters for nodes via the launch system.

> 这种参数工作方式的根本区别将影响 ROS 2 中 launch 系统的体系结构，以及用户如何通过 launch 系统为节点指定参数。

#### Process Related Events and Responses

In `roslaunch` from ROS 1 there were only a few ways that it could react to changes in the system, and they were both related to a process "dieing" (either a clean or unclean exit):

> 在 ROS 1 中的 `roslaunch` 中，只有几种反应系统变化的方式，它们都与一个进程“死亡”(无论是干净的还是不干净的退出)有关：

- respawn a process if it died
- shutdown the whole launch system if a required process died

This is somewhere that the launch system in ROS 2 can hopefully improve on what `roslaunch` from ROS 1 had to offer, and it can do so by providing not only these common reactions to processes exiting, but also by providing more granular information about the process exit (and other events), and by letting the user specify arbitrary responses to these type of events.

> 这是 ROS 2 中的 launch 系统可以在 ROS 1 中的 roslaunch 所提供的内容上进行改进的地方，它不仅可以提供进程退出时的常见反应，还可以提供有关进程退出(和其他事件)的更细粒度的信息，并允许用户对这些类型的事件指定任意响应。

#### Deterministic Startup

In the ROS 1 wiki for `roslaunch`, it says ([https://wiki.ros.org/roslaunch/Architecture](https://wiki.ros.org/roslaunch/Architecture)):

> 在 ROS 1 的 wiki 上关于 `roslaunch`，它说([https://wiki.ros.org/roslaunch/Architecture](https://wiki.ros.org/roslaunch/Architecture))：

roslaunch does not guarantee any particular order to the startup of nodes -- although this is a frequently requested feature, it is not one that has any particular meaning in the ROS architecture as there is no way to tell when a node is initialized.

> .

roslaunch 不能保证节点启动的任何特定顺序——尽管这是一个经常被要求的功能，但在 ROS 架构中没有任何方法可以告诉你何时初始化一个节点，因此它没有任何特定的意义。

Hopefully this is another case on which the launch system for ROS 2 can improve, at least for nodes with a lifecycle, a.k.a. Managed Nodes[^lifecycle]. For Managed Nodes, it would not be possible to apply constraints on when something is launched, rather than how it is in `roslaunch` from ROS 1, where things are run in a non-deterministic order.

> 希望这是另一个可以改进 ROS 2 启动系统的案例，至少对于具有生命周期的节点，即托管节点[^lifecycle]。对于托管节点，不可能应用约束来确定何时启动某些内容，而不是 ROS 1 中的 `roslaunch`，其中内容以非确定性顺序运行。

In order to do this, the launch system in ROS 2 will need to model the dependencies between processes and/or nodes where they exist, and the constraints on those dependencies. For example, a user might express that an image processing node has a dependency on a camera driver node with the constraint that it should not be launched (what ever the action to do that might be, e.g. run a process or something else) until the camera driver node reaches the "Active" state. These constraints can be arbitrarily defined by the user or common constraints could be modeled directly by the launch system.

> 为了做到这一点，ROS 2 中的启动系统需要模拟进程和/或节点之间的依赖关系，以及这些依赖关系的约束。例如，用户可以表达出镜像处理节点与相机驱动节点之间存在依赖关系，并且该依赖关系的约束是在相机驱动节点达到“Active”状态之前不应启动(无论采取什么行动，例如运行一个进程或其他东西)。这些约束可以由用户任意定义，或者可以由启动系统直接建模常见的约束。

Also, these constraints don't have to be related to ROS specific events like lifecycle state changes. For example, a user might express that a plain process should be launched (in this case executed as a subprocess) after another process has been running for ten seconds. The launch system in ROS 2, could either choose to let the user define a predicate which satisfied that constraint, or it could provide a generic constraint like: "launch N seconds after another process".

> 也可以不涉及 ROS 特定事件(如生命周期状态更改)来定义约束。例如，用户可以表达一个普通的进程应该在另一个进程运行了十秒之后被启动(在这种情况下，作为一个子进程执行)。ROS 2 中的启动系统可以选择让用户定义一个满足该约束的谓词，或者提供一个像“在另一个进程运行 N 秒之后启动”这样的通用约束。

#### Node Related Events and Responses

Also leveraging Managed Nodes when possible, the launch system in ROS 2 could export, aggregate and export, or react to lifecycle events of nodes. For example, it might be possible to say that a node, rather than a process, is "required" such that the launch system shutdowns if that node's state ends up in the "Finalized" state, which would be similar to a process exiting with the "required=true" setting for `roslaunch` from ROS 1.

> 使用可能的管理节点，ROS 2 中的启动系统可以导出、聚合和导出节点的生命周期事件，或对其作出反应。例如，可以将节点视为“必需”，即如果该节点的状态最终进入“完成”状态，启动系统将关闭，这与 ROS 1 中的“roslaunch”的“required = true”设置类似。

#### Static Description and Programmatic API

Most users of `roslaunch` from ROS 1 used it by defining a static XML description of what they wanted executed and which parameters they wanted to set. There is an API for `roslaunch` in ROS 1, but in our experience few people use this interface. We can only speculate as to why, but the API is not very well documented and is not prevalent in the tutorials and examples. Sticking strictly to the XML description has caused two different approaches to dynamic behavior/configuration to become more popular:

> 大多数 ROS 1 中使用 roslaunch 的用户通过定义所需执行的静态 XML 描述以及要设置的参数来使用它。ROS 1 中有一个 roslaunch 的 API，但根据我们的经验，很少有人使用这个接口。我们只能猜测原因，但 API 的文档不是很全面，在教程和示例中也不太普遍。严格遵守 XML 描述已经导致两种不同的动态行为/配置方法变得更加流行：

- preprocessing with an XML preprocessor, like `xacro` or some other general purpose templating system
- more sophisticated expressions as XML tags in the `roslaunch` from ROS 1 syntax, e.g. `$(eval expression)` (added in ROS Kinetic) or the `if=$(arg ...)` and `unless=$(arg ...)` attributes

Often when these kind of "dynamic" features are discussed the question of "why is roslaunch (from ROS 1) a static description and not a script"? The direct answer is that "it doesn't have to be", but the API for doing it programmatically is not very well documented or easy to use.

> 经常在讨论这类“动态”功能时，会问到“为什么 ROS 1 中的 roslaunch 是静态描述而不是脚本？” 直接的答案是“它不必是”，但是用于以编程方式实现它的 API 文档不是很完善或易于使用。

There are pro's and con's to both scripted launch files as well as static, declarative launch files, but that will be covered in its own section later in this article. But even if the preference is for a static launch file format like is common in ROS 1, it's a goal of the launch system in ROS 2 to have a more accessible public API which is used to execute that static launch file, so a programmatic approach will always be an option.

> 有利有弊的脚本启动文件以及静态声明式启动文件，但这将在本文的后面部分进行介绍。但是，即使偏好使用像 ROS 1 中常见的静态启动文件格式，ROS 2 中的启动系统的目标也是拥有更易于访问的公共 API，用于执行该静态启动文件，因此编程方法将始终是一个选择。

#### Locating Files

It's often the case that you need to express the location of a file when describing your system to the launch system, whether it be an executable to run, a file to be passed as an argument, or a file from which to load parameters. In the launch system for ROS 2, like the launch system for ROS 1 the concept of packages is used to group related resources and programs together to make this easier, but it will also support some other kinds of relative paths (other than just package share folders). But where ROS 1 and ROS 2 differ in this topic is how the packages will be found, which folders a package can be associated with, and therefore probably also the syntax for how to get that relative path.

> 通常，您需要在向启动系统描述系统时表达文件的位置，无论是要运行的可执行文件，作为参数传递的文件，还是要加载参数的文件。在 ROS 2 的启动系统中，就像 ROS 1 的启动系统一样，使用包的概念将相关资源和程序组合在一起，以使其更容易，但它还将支持其他类型的相对路径(除了包共享文件夹之外)。但是 ROS 1 和 ROS 2 在此主题上的不同之处在于如何找到包，一个包可以与哪些文件夹关联，因此可能也是获取相对路径的语法。

### Similarities with ROS 1

The previous subsection dealt with what may be different for the launch system in ROS 2, but in this subsection the similarities will be enumerated (not necessarily exhaustively). The launch system in ROS 2 will:

> 上一小节讨论了 ROS 2 中 launch 系统可能有何不同，但在本小节中，将列举出它们的相似之处(不一定是详尽的)。ROS 2 中的 launch 系统将：

- convert common ROS concepts like remapping and changing the namespace into appropriate command line arguments and configurations for nodes so the user doesn't have to do so
- manage complexity through composition of simpler systems (launch files)
- allow including of other launch files
- use groups to apply settings to collections of nodes and processes
- provide operating system portability where possible

and possibly other things, all of which it will share in common with `roslaunch` from ROS 1.

> 以及可能的其它事情，它们都会和 ROS 1 中的 roslaunch 有共同之处。

## Separation of Concern

The launch system can be considered in parts, separated by concern. The coarse breakdown is like so:

> 可以把 launch 系统按关注点分解成几个部分。粗略的划分如下：

- Calling Conventions for Processes and Various Styles of Nodes
- Reporting System for Events
- System Description and Static Analysis
- Execution and Verification of the System Description
- Testing

The purpose of the following sections is to enumerate what the launch system could do and the things with which it could interact, but is not the requirements list for the launch system in ROS 2. The requirements for the launch system will be enumerated in section below based on what's possible in these sections.

> 以下部分的目的是列出 launch 系统可以做什么以及它可以与之交互的事情，但不是 ROS 2 中 launch 系统的要求列表。根据这些部分中可能的情况，将在下面的部分中列出 launch 系统的要求。

## Calling Conventions

In order for the launch system to execute a described system, it needs to understand how it can achieve the description. The phrase "calling conventions" is an existing phrase in Computer Science_convention_wikipedia[convention_wikipediawikipedia], but this section is not talking specifically about the compiler defined calling convention, through it is appropriating the term to describe a similar relationship. In this case, the phrase "calling conventions" is meant to describe the "interface" or "contract" the launch system has with anything it is executing and monitoring. This contract covers initial execution, activity during runtime, signal handling and behavior of the launch system, and shutdown.

> 为了让 launch 系统执行一个描述的系统，它需要理解如何实现该描述。“调用约定”是计算机科学中一个现有的术语_convention_wikipedia[convention_wikipediawikipedia]，但是本节没有具体谈论编译器定义的调用约定，尽管它拿来描述一种类似的关系。在这种情况下，“调用约定”的意思是描述 launch 系统与其执行和监视的任何东西之间的“接口”或“合同”。该合同涵盖了初始执行、运行时的活动、信号处理和 launch 系统的行为以及关闭。

### Operating System Processes

The most basic version of these entities, and the foundation for the other entities, are operating system processes.

> 最基本的这些实体版本，以及其他实体的基础，是操作系统进程。

#### Execution

For these, the launch system needs to know how to execute them, and to do that it needs:

> 为了这些，启动系统需要知道如何执行它们，为此它需要：

- name of the executable (just the name, relative path, or absolute path)
- environment variables (`PATH`, `LD_LIBRARY_PATH`, `DL_PRELOAD`, etc...)
- command line arguments
- working directory (directory from which to execute the process)
- launch prefix (used to inject things like `gdb`, `valgrind`, etc...)

    <div class="alert alert-warning" markdown="1">

RFC:

> RFC：请求变更

Missing from this list is the user which should be used to execute the process. It's possible that it would be necessary or at least useful to change the user based on the launch description. However, it can always be done in a user written script and supporting it in our Python implementation in a portable way looks to be difficult.

> 缺少的是用于执行进程的用户。根据启动描述可能有必要或至少有用地更改用户。但是，它总是可以通过用户编写的脚本来实现，并且在我们的 Python 实现中以便携的方式支持它似乎很困难。

</div>

With this information the launch system can execute any arbitrary operating system process on the local machine.

> 这些信息可以帮助 launch 系统在本地机器上执行任意操作系统进程。

#### Runtime

During runtime, the launch system may monitor all operating system process's:

> 在运行时，启动系统可能会监控所有操作系统进程。

- `stdout` pipe
- `stderr` pipe

The launch system may choose to either capture these pipes, for logging or suppressing output to the console, or it can connect the pipes to an existing `pty`, like the terminal's `stdout` and/or `stderr` pipes or a null pipe (e.g. `/dev/null`).

> 系统可以选择捕获这些管道，用于记录或抑制输出到控制台，或者它可以将管道连接到现有的 `pty`，比如终端的 `stdout` 和/或 `stderr` 管道或空管道(例如 `/dev/null`)。

When capturing the output pipes of a process, the launch system could report this data in a way that the user may process them in real-time or could pass the data through user defined filters, generating a user-handled event when the filter matches.

> 当捕获进程的输出管道时，启动系统可以以便用户实时处理的方式报告这些数据，也可以通过用户定义的过滤器传递数据，当过滤器匹配时生成用户处理的事件。

In addition, the launch system may interact with, or allow the user to interact with, an operating system process's:

> 此外，启动系统可以与操作系统进程交互，或者允许用户与之交互：

- `stdin` pipe
- signals (`SIGINT`, `SIGTERM`, `SIGUSR1`, etc...)

Regardless of how the user uses the launch system to interact with these items, they should be exposed by the launch system, which is the only entity which can interact with them directly.

> 不管用户如何使用 launch 系统与这些物品交互，它们应该由 launch 系统暴露出来，launch 系统是唯一可以直接与它们交互的实体。

#### Termination

If the operating system process terminates, and therefore returns a return code, the launch system will report this event and it can be handled in a user defined way. Termination covers expected termination (e.g. return from `main()` or use `exit()`) and unexpected termination (e.g. the abort trap or a segmentation fault or bus error).

> 如果操作系统进程终止并返回一个返回码，启动系统将报告此事件，用户可以以自定义的方式处理它。终止包括预期终止(例如从 `main()` 返回或使用 `exit()`)和意外终止(例如中止陷阱或段错误或总线错误)。

Historically, ROS 1's `roslaunch` allowed a few common exit handling cases:

> 历史上，ROS 1 的 `roslaunch` 允许一些常见的退出处理情况：

- `require=true`: if this process exits (any reason) shutdown everything else, as it's "required"
- `respawn=true`: if this process exits (any reason) restart it with the same settings as startup
  - `respawn_delay=N`: if restarting it, delay a number of seconds between attempts

The launch system may initiate the termination of an operating system process. This starts with the signaling of `SIGINT` on the child process. If this does not result in the termination of the process, then one of a few things can happen based on the configuration of the launch system:

> 如果 launch 系统发出 SIGINT 信号，操作系统进程可能会被终止。如果这不能导致进程的终止，那么根据 launch 系统的配置，可能会发生几件事情。

- after a period of time, signal `SIGTERM`
- after a period of time, signal `SIGKILL`
- nothing

By default, the launch system will:

> 默认情况下，启动系统将会：

- send `SIGINT`
- after a short period of time, send `SIGTERM`
- after an additional short period of time, send `SIGKILL`

The latter two steps can be skipped, or the time until escalation can be adjusted, on a per process basis.

> 这两个步骤可以跳过，或者可以根据每个流程调整升级的时间。

The launch system will initiate this process when an event (built-in or user generated) initiates shutdown, e.g. when a process with the equivalent of the `require=true` exit handler terminates, or when the launch system itself receives the `SIGINT` signal.

> 系统启动将在一个事件(内置或用户生成)发起关闭时启动此过程，例如，当具有等效于“require=true”退出处理程序的进程终止时，或者当启动系统本身接收“SIGINT”信号时。

If the launch system itself receives the `SIGTERM` signal it will send the `SIGKILL` signal to all child processes and exit immediately.

> 如果启动系统本身接收到 `SIGTERM` 信号，它将向所有子进程发送 `SIGKILL` 信号，并立即退出。

The rationale for the previous rule is that if someone attempts to `SIGTERM` the launch system, they probably did so out of impatience after sending `SIGINT` to the launch system, and therefore the launch system should attempt to exit quickly. Exiting quickly will hopefully avoid encouraging a user to `SIGKILL` the launch system, which might cause the subprocesses to be improperly shutdown and perhaps even become zombie processes.

> 上一条规则的理由是，如果有人试图向 launch 系统发送 `SIGTERM`，他们可能是在发送 `SIGINT` 到 launch 系统后感到不耐烦，因此 launch 系统应尽快退出。希望快速退出能避免鼓励用户对 launch 系统发送 `SIGKILL`，这可能会导致子进程被不正确关闭，甚至变成僵尸进程。

#### Shell Evaluation

A special case of operating system processes, shell evaluation would simply be passing shell script code as an argument to the default system shell. This is a problematic thing to support because it is hard/messy to make it portable to all operating systems.

> 在操作系统进程的一种特殊情况下，shell 评估只是将 shell 脚本代码作为参数传递给默认系统 shell。支持这种情况是很困难/混乱的，因为很难使其在所有操作系统上都具有可移植性。

A kind of in-between entity is an operating system process which uses shell evaluation to expand a relative executable name to an absolute path using the PATH environment variable.

> 一种中间实体是操作系统进程，它使用 shell 评估来使用 PATH 环境变量将相对可执行名称扩展为绝对路径。

#### Remote Operating System Processes

Any of the entities based on an operating system process can be made into a remote operating system process by simply adding the requirement information needed to gain access to the other machine and execute it. This is a feature that ROS 1's `roslaunch` has, and is useful in multi machine robots.

> 任何基于操作系统进程的实体都可以通过简单地添加获取其他机器访问并执行所需信息的要求而变成远程操作系统进程。这是 ROS 1 的 `roslaunch` 具有的功能，对于多机器人来说很有用。

```
<div class="alert alert-warning" markdown="1">
TODO: figure out what we need to do here in terms of portability and configuration
</div>
```

### ROS Nodes

Any operating system process can become ROS specific by having at least one ROS Node within it. Having one or more "plain" ROS nodes in a process doesn't add new standardized ways to get information into or out of the operating system process that contains them, though ROS topics, services, parameters, etc. can be accessed during runtime. It does however, add some specific kinds of inputs during execution and it also can affect how the process reacts to signals.

> 任何操作系统进程都可以通过在其中拥有至少一个 ROS 节点来变成 ROS 特定的进程。在进程中拥有一个或多个“普通”ROS 节点不会为操作系统进程添加新的标准化方式来获取信息，但是可以在运行时访问 ROS 主题、服务、参数等。然而，它确实会在执行过程中添加一些特定类型的输入，并且也可以影响进程对信号的反应。

This applies to "plain" ROS nodes, but there is more that the launch system can use in Managed ROS Nodes, which is described in the next section.

> 这适用于“普通”ROS 节点，但在受管理的 ROS 节点中，启动系统可以使用更多内容，这在下一节中有描述。

#### Execution

In addition to the "Execution" subsection of the "Operating System Processes" section, processes with ROS Nodes in them may need to consider additional elements, like:

> 除了"操作系统进程"部分的"执行"子部分外，包含 ROS 节点的进程可能还需要考虑其他元素，例如：

- "Package name + executable name" rather than "executable name + PATH" (i.e. `ros2 run` equivalent)
- ROS specific environment variables (e.g. `ROS_DOMAIN_ID`, `RMW_IMPLEMENTATION`, console output formatting, etc...)
- ROS specific command line arguments
  - Varies for single Node processes and multi Node processes
  - Change node name or namespace
  - Remap topics, services, actions, parameters, etc...
  - Initialize parameter values

The specific syntax of these extra environment variables and command line arguments are defined in other documents_wiki[wikiging_wiki] _remapping[remappingemapping].

> 这些额外的环境变量和命令行参数的具体语法定义在其他文档_wiki[wikiging_wiki] _remapping[remappingemapping]中。

In each of these cases, the ROS specific constructs can be expressed with the existing mechanisms described by the "Execution" subsection for "Operating System Processes", i.e. the ROS specific constructs can be expanded into either command line arguments or environment variables. Therefore the launch system is able to take ROS specific declarations, e.g. "remap 'image' to 'left/image'", and convert them implicitly into terms that a normal operating system process can consume like environment variables or command line arguments, e.g. adding `image:=left/image` to the command line arguments. However, what a given ROS specific declaration is converted into depends on how the nodes are used within the process, but later sections will go into details about that.

> 在这些情况下，ROS 特定的构造可以用“操作系统进程”下的“执行”子节中描述的现有机制表达，即 ROS 特定的构造可以扩展为命令行参数或环境变量。因此，启动系统可以隐式地将 ROS 特定的声明(例如“remap 'image' to 'left/image'”)转换为正常操作系统进程可以使用的项，如环境变量或命令行参数，例如将 `image:=left/image` 添加到命令行参数中。但是，给定的 ROS 特定声明转换为什么取决于节点如何在进程中使用，但后面的章节将详细介绍。

#### Runtime

During runtime a "plain" ROS node doesn't expose anything new beyond what an operating system process does. It does have ROS topics, services, parameters, etc. but none that are standardized in a way that's useful for the launch system at this time.

> 运行时，一个“普通”的 ROS 节点除了操作系统进程所具有的功能外，不会暴露出任何新的东西。它确实有 ROS 主题、服务、参数等，但这些都没有被标准化，因此无法用于目前的启动系统。

It also does not react in any special way to `stdin`, but processes containing ROS nodes do tend to have a signal handler for `SIGINT` which does a more graceful shutdown, but that is not enforced. Sending the `SIGINT` signal typically causes most nodes to shutdown if they are using one of the "spin" functions in `rclcpp` or are polling `rclcpp::ok()`, as is recommended.

> 它也不会对'stdin'做出特殊反应，但包含 ROS 节点的进程通常会为 `SIGINT` 设置一个信号处理程序，以实现更优雅的关机，但这不是强制的。发送 `SIGINT` 信号通常会导致大多数节点关闭，如果它们正在使用 `rclcpp` 中的“旋转”功能或正在轮询 `rclcpp：：ok()`，如推荐的那样。

#### Termination

Termination of a ROS Node (the node, not the process) is not externally observable beyond what is observed with an operating system process (the return code).

> 终止一个 ROS 节点(节点，而不是进程)在操作系统进程(返回码)观察到的以外，不能外部可观察到。

### Managed ROS Nodes

For ROS nodes that have a lifecycle, a.k.a. Managed ROS Nodes[^lifecycle], each node will have additional runtime state, which the launch system could access and either utilize directly, pass through to the event system, or aggregate before passing it through the event system.

> 对于具有生命周期的 ROS 节点，即所谓的受管理的 ROS 节点[^lifecycle]，每个节点将具有额外的运行时状态，启动系统可以访问并直接使用、通过事件系统传递，或在传递到事件系统之前进行聚合。

Building yet again on previous entities, the "Managed ROS Nodes" inherits all of the execution, runtime, and termination characteristics from normal ROS nodes and therefore operating system processes.

> 在之前的实体基础上，“管理 ROS 节点”继承了所有的执行、运行时和终止特征，因此操作系统进程也是如此。

#### Execution

Managed ROS Nodes do not add any additional inputs or specific configurations at execution time on top of what "plain" ROS nodes add, at least not at this time.

> 管理的 ROS 节点目前不会在执行时除了“普通”ROS 节点添加的以外，再增加任何额外的输入或特定的配置。

In the future this might change, so reference the design doc[^lifecycle] or future documentation on the subject.

> 在未来，这可能会改变，因此参考设计文档[^生命周期]或有关主题的未来文档。

#### Runtime

During runtime, a Managed ROS node emits events anytime the state of the node changes. This is at least emitted on a topic, but could also be captured, aggregated, and/or communicated in other ways too. These state changes could be consumed by either the launch system itself or by the user, either of which could react to these changes.

> 在运行期间，管理的 ROS 节点会在节点状态改变时发出事件。这至少会在主题上发出，但也可以捕获，聚合和/或以其他方式传输。这些状态更改可以由启动系统本身或用户消费，其中任何一方都可以对这些更改做出反应。

For example, the user could express something like "when node 'A' enters the `Active` state, launch nodes 'B' and 'C'" or "if node 'A' exits with a return code or enters the `Finalized` state, shutdown everything".

> 例如，用户可以表达：“当节点'A'进入'Active'状态时，启动节点'B'和'C'”，或者“如果节点'A'以返回码退出或进入'Finalized'状态，则关闭所有内容”。

#### Termination

Managed ROS Nodes have some additional observable effects when terminating (the node, not necessarily the process containing it). A managed node enters the `Finalized` state after passing through the `ShuttingDown` transition state on termination. Since these are state transitions, they are observable via the lifecycle event system, at least through the ROS topic `lifecycle_state` (subject to change, always reference the managed nodes design document[^lifecycle]).

> 管理 ROS 节点在终止(节点，而不一定是包含它的进程)时会有一些额外的可观察效果。经过终止的“关闭”状态转换后，管理节点进入“完成”状态。由于这些是状态转换，它们可以通过生命周期事件系统观察到，至少通过 ROS 主题“生命周期状态”(始终参考管理节点设计文档[^lifecycle])。

The mechanism for how Managed ROS Nodes transition to the Finalized state (or any other state) will not be decided in this document. Instead, the implementations or some other lifecycle specific documentation will cover that. However, you could imagine this transition could be handled by the Node itself automatically, or by launch sending a state transition request, or by launch sending a specific signal.

> 本文档不会决定 Managed ROS 节点如何转换到最终状态(或其他状态)的机制。相反，实现或其他生命周期特定文档将涵盖此内容。但是，您可以想象这种转换可以由节点本身自动处理，或者由启动发送状态转换请求，或者由启动发送特定信号。

### Process with a Single Node

In this subsection, and the following subsections of the "Calling Conventions" section, the different possible combinations of nodes and processes is explained. In each case they "inherit" any behaviors from either the "ROS nodes" or the "Managed ROS nodes" subsections above, but in these subsections the "how" of communicating ROS specific options is described in more detail.

> 在本小节及“调用约定”部分的其余小节中，将解释不同可能的节点和进程组合。在每种情况下，它们会从“ROS 节点”或“管理 ROS 节点”小节中继承任何行为，但在这些小节中，更详细地描述了传输 ROS 特定选项的“如何”。

The first is a single process with a single ROS node within it. This was likely the most commonly used type of entity launched in ROS 1, as you could only have one node per process in ROS 1. In ROS 2, this will likely be less common because you can have one to many nodes per process, but will may still be used quite a bit in the form of quickly developed scripts and drivers or GUI tools which might require control over the main thread.

> 第一种是单进程单 ROS 节点。在 ROS 1 中，这可能是最常用的实体启动类型，因为每个进程只能有一个节点。在 ROS 2 中，这可能不太常见，因为每个进程可以有一到多个节点，但仍然可能经常使用，例如快速开发的脚本和驱动程序或 GUI 工具，可能需要控制主线程。

Since there is only one ROS node, the command line arguments do not need to be explicit about to which node they apply. For example, changing the namespace of the single node could be expressed with the command line argument `__ns:=new_namespace`.

> 由于只有一个 ROS 节点，命令行参数不需要明确指出它们适用于哪个节点。例如，可以使用命令行参数 `__ns:=new_namespace` 来改变单个节点的命名空间。

Even though there is only one node in the process, that node does not need to start when the process starts, nor does the process need to end when the node is shutdown and/or destroyed. If it is a managed node, the lifecycle of the node is best tracked using the lifecycle events. In fact, a process with a single node could start a node, run for a while, later destroy it, and then create it again.

> 即使只有一个节点在过程中，该节点在过程开始时不需要启动，也不需要在节点关闭和/或销毁时结束过程。如果是一个受管理的节点，最好使用生命周期事件来跟踪节点的生命周期。事实上，一个只有一个节点的进程可以启动一个节点，运行一段时间，之后销毁它，然后再次创建它。

So the biggest impact of a single node process is that the configuration, in terms of command line arguments and environment variables, can be simplified.

> 所以单节点进程的最大影响是，可以简化命令行参数和环境变量的配置。

### Process with Multiple Nodes

In a process with multiple nodes, things are much the same as with a process with a single node, but the configuration, again in terms of command line arguments and environment variables, need to be more specific in order to discriminate between the various nodes being instantiated in the process. The remapping design document_remapping[remappingemapping] goes into detail on how you can selectively configure multiple nodes using command line arguments, so check there for up-to-date details.

> 在一个具有多个节点的流程中，情况和具有单个节点的流程相同，但是配置(以命令行参数和环境变量为准)需要更加具体以区分实例化的各个节点。_remapping[remappingemapping] 重新映射设计文档详细介绍了如何使用命令行参数有选择地配置多个节点，因此请查看最新详情。

However, as an example of a process with multiple nodes, consider a program that instantiates two camera driver nodes called "camera1" and "camera2" by default. You could configure their namespaces separately by doing something like `camera1:__ns:=left camera2:__ns:=right`.

> 然而，作为多节点过程的一个例子，考虑一个程序，它默认情况下实例化两个称为“camera1”和“camera2”的摄像头驱动程序节点。您可以通过做一些像 `camera1:__ns:=left camera2:__ns:=right` 这样的事情来单独配置它们的名称空间。

#### Dynamically loading Nodes

Dynamically loading a node means spawning it in a container process that does not know about the node until it is asked to load it. A container process is a stand alone executable that loads and executes nodes within itself.

> 动态加载节点意味着在一个容器进程中产生它，该容器进程在被要求加载它之前不知道该节点。容器进程是一个独立的可执行文件，可以在其中加载和执行节点。

##### Container Process API

While there will be standard container processes, custom container processes would allow using custom executors or client libraries. Therefore, there must be a container process API for the launch system to communicate which nodes should be loaded.

> 尽管会有标准的容器流程，但自定义容器流程将允许使用自定义执行程序或客户端库。因此，必须有一个容器流程 API，用于启动系统来通信哪些节点应该被加载。

The launch system must be able tell the container process what arguments to give to a dynamically loaded node. This includes command line arguments and client library specific options (e.g. `rclcpp` has `use_intra_process_comms`). Since the launch system cannot know about all custom containers, the API must include a way to pass unknown arguments (e.g. by passing key-value pairs).

> 系统启动必须能够告诉容器进程应该给动态加载的节点提供什么参数。这包括命令行参数和客户端库特定选项(例如，`rclcpp` 具有 `use_intra_process_comms`)。由于启动系统不能了解所有自定义容器，因此 API 必须包括一种传递未知参数的方法(例如，通过传递键值对)。

The API will not include setting environment variables per loaded node. Many languages have APIs to get environment variables, and there is no way to isolate them within a process. The following options for an API are being considered.

> API 不包括设置每个载入的节点的环境变量。许多语言都有 API 来获取环境变量，而且无法将它们隔离在一个进程中。正在考虑以下 API 选项。

###### API using Command Line Configuration File

One option for a container processes API is to pass a configuration file with nodes to load via the command line.

> 一个容器进程 API 的选项是通过命令行传递一个配置文件，以加载节点。

Advantages:

> 优势：

- No waiting for an API to become available

> 不用等待 API 可用。

Disadvantages:

> 缺点：

- Requires write access to the file system

> \*需要对文件系统有写访问权限

- Requires parsing a config file

> \*需要解析配置文件

- Cannot tell from the outside if a container process supports this interface

> 无法从外部判断容器进程是否支持此接口。

- Cannot tell if and when nodes are loaded or unloaded

> - 无法确定节点何时加载或卸载。

This API could have very low latency to launch nodes since it does not require waiting for discovery. However, there is no way to get feedback about the success or failure of loaded nodes. There is also no way to tell a container process to unload a composable node.

> 这个 API 可以具有非常低的延迟来启动节点，因为它不需要等待发现。但是，没有办法获得有关已加载节点成功或失败的反馈。也没有办法告诉容器进程卸载可组合节点。

###### API using STDIN

Another option for a container process API is to pass configuration in via STDIN.

> 另一种容器进程 API 的选择是通过 STDIN 传递配置。

Advantages:

> 优势：

- No waiting for an API to become available

> 没有等待 API 可用。

- Works with read-only file systems

> \*可以使用只读文件系统

Disadvantages:

> 缺点：

- Requires parsing a config

> \*需要解析配置

- Cannot tell from the outside if a container process supports this interface

> 无法从外部判断容器进程是否支持此接口。

- Cannot tell if and when nodes are loaded or unloaded

> - 不能确定节点什么时候加载或卸载。

- Cannot stop dynamically loaded nodes from reading STDIN

> \*无法阻止动态加载的节点从读取 STDIN。

This API could also have very low latency to launch nodes. However, there also is no way to get feedback about the success or failure of loaded nodes. STDOUT cannot be used because a composable node logging messages to STDOUT is assumed to be very common and would conflict. Since STDIN is always available, it would be possible to unload a node via this API.

> 这个 API 可以提供非常低的延迟来启动节点。但是，也没有办法获得有关加载节点成功或失败的反馈。STDOUT 不能用，因为假定一个可组合的节点将消息记录到 STDOUT 会产生冲突。由于 STDIN 总是可用的，因此可以通过此 API 卸载节点。

###### API using ROS Services or Topics

Lastly, a container process API may be defined by ROS services or topics.

> 最后，容器进程 API 可以通过 ROS 服务或主题定义。

Advantages:

> 优势：

- No config file parsing

> - 没有配置文件解析

- Works with read-only file systems

> \*可以操作只读文件系统

- Can indicate if a node was successfully loaded

> - 可以指示节点是否加载成功

- Can create API to trigger launch events

> \*可以创建 API 来触发 launch 事件。

- Can tell if a container process supports this interface

> .

- 可以判断一个容器进程是否支持这个接口

Disadvantages:

> 缺点：

- Must wait for the service API to become available

> \*必须等待服务 API 可用。

- Cannot stop dynamically loaded nodes from creating the same services

> \*无法阻止动态加载的节点创建相同的服务。

This is the only option discussed which can communicate the success or failure of dynamically launched nodes. It is also the only option that allows introspection. However, this option has the highest potential delay from when the container process is spawned to when nodes may be loaded.

> 这是唯一讨论过的可以传达动态启动节点成功与失败的选项。它也是唯一允许内省的选项。然而，这个选项从容器进程被产生到节点可能被加载的最高潜在延迟。

##### Proposed Container process API

This is a proposal for an API a launch system will use to interact with container processes.

> 这是一个提案，用于推出系统与容器进程交互的 API。

###### Command Line Arguments

A container process must accept command line arguments including log level, remapping, and parameters. These command line arguments must not be applied to dynamically launched nodes. The launch system will pass these arguments to a container process in the same way it would pass them to a node. If a remap rule would apply to a launch service, the launch system should try to use the remapped service name instead.

> 容器进程必须接受包括日志级别、重映射和参数在内的命令行参数。这些命令行参数不得应用于动态启动的节点。启动系统将这些参数以与传递给节点相同的方式传递给容器进程。如果重映射规则适用于启动服务，则启动系统应尝试使用重映射后的服务名称。

###### ROS Services

A container process must offer all of the following services.

> 一个容器进程必须提供以下所有服务。

- `~/_container/load_node`

> - `~/_container/加载节点`

- `~/_container/unload_node`

> - `~/_container/卸载节点`

- `~/_container/list_nodes`

> - `~/_容器/列出节点`

The services are hidden to avoid colliding with user created services. `load_node` will be called by the launch system when a composable node is to be dynamically loaded, and `unload_node` destroys a composable node. `list_nodes` is not called by launch system, and is only provided for introspection.

> 服务被隐藏以避免与用户创建的服务发生冲突。当需要动态加载可组合节点时，会由启动系统调用 `load_node`，而 `unload_node` 则用于销毁可组合节点。`list_nodes` 不会被启动系统调用，只提供了内省功能。

1. load_node

> 1. 加载节点

If a container process is asked to load a node with a full node name matching an existing node, then it must reject the request. This is to avoid conflicts in features that assume node name uniqueness, like parameters.

> 如果容器进程被要求加载一个具有与现有节点完全匹配的节点名称，则必须拒绝该请求。这是为了避免假设节点名称唯一性的功能上的冲突，比如参数。

A container process must assign the node a unique id when it is loaded. The id of a loaded node instance never changes. Two nodes in the same container process must never have the same id, and there should be a significant time delay before an id is reused.

> 一个容器进程在加载时必须给节点分配一个唯一的 ID。加载的节点实例的 ID 永不改变。同一个容器进程中的两个节点不应该有相同的 ID，而且在重复使用 ID 之前应该有一个显著的时间延迟。

The interface for this Service was added in ROS 2 Dashing:

> 这个服务的界面已经在 ROS 2 Dashing 中添加了。

[https://github.com/ros2/rcl_interfaces/blob/dashing/composition_interfaces/srv/LoadNode.srv](https://github.com/ros2/rcl_interfaces/blob/dashing/composition_interfaces/srv/LoadNode.srv)

> [https://github.com/ros2/rcl_interfaces/blob/dashing/composition_interfaces/srv/LoadNode.srv](https://github.com/ros2/rcl_interfaces/blob/dashing/composition_interfaces/srv/LoadNode.srv) 的简体中文翻译是：

LoadNode.srv：

请求：
string package_name
string plugin_name
string node_name

响应：
bool success
string error_message

2. unload_node

> 2. 卸载节点

The interface for this Service was added in ROS 2 Dashing:

> 这个服务的界面已在 ROS 2 Dashing 中添加。

[https://github.com/ros2/rcl_interfaces/blob/dashing/composition_interfaces/srv/UnloadNode.srv](https://github.com/ros2/rcl_interfaces/blob/dashing/composition_interfaces/srv/UnloadNode.srv)

> [https://github.com/ros2/rcl_interfaces/blob/dashing/composition_interfaces/srv/UnloadNode.srv](https://github.com/ros2/rcl_interfaces/blob/dashing/composition_interfaces/srv/UnloadNode.srv)
> 卸载节点服务

3. list_nodes

> 3. 列出节点

The interface for this Service was added in ROS 2 Dashing:

> 这个服务的界面已经在 ROS 2 Dashing 中添加了。

[https://github.com/ros2/rcl_interfaces/blob/dashing/composition_interfaces/srv/ListNodes.srv](https://github.com/ros2/rcl_interfaces/blob/dashing/composition_interfaces/srv/ListNodes.srv)

> [https://github.com/ros2/rcl_interfaces/blob/dashing/composition_interfaces/srv/ListNodes.srv](https://github.com/ros2/rcl_interfaces/blob/dashing/composition_interfaces/srv/ListNodes.srv)：列出节点服务

###### Exit Code

If the container process is asked to shutdown due to normal [Termination], then the exit code must be 0. If it exits due to an error then exit code must be any other number.

> 如果容器进程因正常[终止]而被要求关闭，则退出代码必须为 0。如果由于错误而退出，则退出代码必须为其他数字。

##### Parallel vs Sequential Loading of Nodes

If it is possible to load multiple nodes in parallel, then it needs to be decided how to load the nodes. The container process should load nodes as soon as it is asked. It should be up to the launch system to decide whether to load nodes in parallel or sequentially. If multiple nodes of the same type are to be launched, then the launch system should load the nodes sequentially so each is able to remap it's name before the next is loaded. If they are of different types then the launch system may choose to try to load them in parallel, where the exact order they get loaded is determined by chance or the container process.

> 如果可以并行加载多个节点，则需要决定如何加载节点。容器进程应在被要求时立即加载节点。应由启动系统决定是并行加载还是顺序加载。如果要启动多个相同类型的节点，则启动系统应顺序加载节点，以便每个节点在加载下一个节点之前都能重新映射其名称。如果它们是不同类型的，则启动系统可以选择尝试并行加载它们，其加载的具体顺序由机会或容器进程决定。

##### Registration of Composable Nodes

How Composable nodes are registered is not defined by this document. Instead, the container process is responsible for knowing how to find nodes it is asked to load. For example, a container process might use pluginlib for rclcpp nodes, or python entry points for rclpy nodes.

> 在本文档中没有定义如何注册可组合节点。相反，容器进程负责知道如何查找它被要求加载的节点。例如，容器进程可以使用 pluginlib 来加载 rclcpp 节点，或者使用 python 入口点来加载 rclpy 节点。

## Event Subsystem

This section of the article covers the event subsystem within the launch system, which is responsible for generating events and reporting them to users and itself so that those events can be handled.

> 这篇文章的这一部分涵盖了 launch 系统内的事件子系统，它负责生成事件并向用户和自身报告，以便这些事件可以处理。

### Categories of Events by Source

Events produced by the event subsystem of the launch system can fall broadly into two categories: events that only the launch system can directly observe and events that the launch system may relay for convenience but is directly observable by other systems too. Therefore, the events that only the launch system can observe must be exposed via the event system if we want them to be used by other applications or users.

> 事件子系统产生的事件可以大致分为两类：只有 launch 系统可以直接观察到的事件，以及 launch 系统可以为方便起见进行转发的事件，但其他系统也可以直接观察到的事件。因此，如果我们希望其他应用程序或用户使用这些只有 launch 系统可以观察到的事件，就必须通过事件系统公开这些事件。

Another way to categorize events is by their source.

> 另一种分类事件的方法是按其来源分类。

#### Launch System Events

The most basic events are related solely to things that happen within the launch system itself. This can be as simple as a timed event, either a specific amount of time has passed, or a specific time of day has passed, or an "idle" event which might be used to implement a "call later" type of callback.

> 最基本的事件只与 launch 系统本身发生的事情有关。这可以很简单，比如定时事件，要么是特定的时间流逝了，要么是特定的时间点流逝了，要么是“空闲”事件，可以用来实现“稍后调用”类型的回调。

However, these events can be more specific to the launch system, like when a launch description is included, or when the launch system needs to shutdown. These events might also contain pertinent information like why a launch description was included, e.g. given as an argument to the launch system, included by another launch file, requested to be included by asynchronous request (maybe via a ROS service call), or in the case of a shutting down event, maybe why the launch system is shutting down, e.g. a required process exited, or it received the SIGINT signal.

> 然而，这些事件可以更具体地关联到启动系统，比如当包含启动描述时，或者当启动系统需要关闭时。这些事件也可能包含相关信息，比如为什么包含启动描述，例如作为启动系统的参数提供，由另一个启动文件包含，通过异步请求(可能是通过 ROS 服务调用)要求包含，或者在关闭事件的情况下，也许是为什么启动系统正在关闭，例如必需的进程已退出，或者它收到了 SIGINT 信号。

#### Operating System Events

Other events will be specific to any process that is executed by the launch system, like when a process is started or when a process exits. You could also imagine events which get fired when `stdout` or `stderr` data is received from the process by the launch system, assuming it captures that information.

> 其他事件将特定于由启动系统执行的任何进程，例如当进程启动时或当进程退出时。您还可以想象当启动系统从进程接收到 `stdout` 或 `stderr` 数据时触发的事件，假设它捕获了该信息。

#### ROS Specific Events

ROS specific events would most likely occur in processes that launch is executing, but using ROS topics and/or services launch could observe these events and generate equivalent events within the launch event system. For example, if a process being run by launch contains a node with a life cycle, launch could observe any life cycle state transitions the node makes and create an event each time one of those transitions occur. Using this a user could, for example, wait for a node to reach the "active" state and only then start another process.

> ROS 特定事件很可能发生在启动执行的进程中，但使用 ROS 主题和/或服务启动可以观察到这些事件并在启动事件系统中生成等效事件。例如，如果启动运行的进程包含一个具有生命周期的节点，启动可以观察节点所做的任何生命周期状态转换，并在每次发生转换时创建一个事件。使用此，用户可以例如等待节点达到“活动”状态，然后才开始另一个进程。

### Reporting and Handling of Events

Without getting into implementation details (e.g. libraries and class hierarchies), this subsection will try to express what information should be contained within events, and how they can be accessed and what the behavior of the event system is with respect to delivery.

> 本小节将不涉及实现细节(如库和类层次结构)，而是尝试描述事件应包含的信息、如何访问它们以及事件系统在传递方面的行为。

#### Information Provided by Events

Like many other event systems, the events should be capable of not only notifying that an event has occurred, but it should be able to communicate data associated with the event.

> 跟其他事件系统一样，事件不仅要能够通知发生了事件，还要能够传达与事件相关的数据。

A simple example of an event without extra data might be an event for "call later", where it doesn't matter who initiated the "call later" or how long it has been since that occurred (though it could include that if it wished), and so this events existence is sufficient to notify waiting actions to proceed.

> 一个不带额外数据的简单事件的例子可能是"稍后打电话"的事件，无论谁发起了"稍后打电话"，或者距离发起已经过去了多长时间(尽管它可以包括这些信息)，这个事件的存在就足以通知等待的行动继续进行。

A simple example of an event with extra data might be a "process exited" event, which would include enough information to identify which process it was as well as the return code of the process.

> 一个带有额外数据的简单事件示例可能是"进程退出"事件，它将包括足够的信息来确定是哪个进程，以及该进程的返回代码。

Again, like many other event systems, the events should have a type (either as an attribute or as a child class) which can be used to filter events to particular handlers.

> 再次，像其他许多事件系统一样，事件应该有一个类型(作为属性或子类)，可以用来将事件过滤到特定的处理程序。

#### Event Handlers

Events can be handled by registering an event handler with the launch system. The only required form of event handler is one that is a function, registered locally with the launch system.

> 事件可以通过向 launch 系统注册事件处理程序来处理。唯一必需的事件处理程序形式是一个在 launch 系统本地注册的函数。

Other kinds of event handlers could be supported by building on a locally defined function. They could be something like a user-defined "lambda" defined in the description of the launch file, or even a built-in event handler function which just publishes the events as ROS messages. In the latter case, it could be either be a subscription to a topic (which needs no a priori registration with the launch system) or a service call (which was registered with the launch system a priori). So if it is a topic, the subscription object, with its callback, could be considered an event handler. In the case of a service, which would be called by the launch system and handled by a user defined service server, the service server (and it's response) would be considered the event handler.

> 其他类型的事件处理程序可以通过建立在本地定义的函数上来支持。它们可以是在启动文件描述中定义的用户自定义“lambda”，甚至是内置的事件处理程序函数，只是将事件发布为 ROS 消息。在后一种情况下，它可以是对主题的订阅(无需与启动系统事先注册)或服务调用(事先与启动系统注册)。因此，如果是主题，具有回调的订阅对象可以被视为事件处理程序。在服务的情况下，将由启动系统调用，并由用户定义的服务服务器处理，服务服务器(及其响应)将被视为事件处理程序。

#### Handling of Events

By default, events are passed to all event handlers and there is no way for an event handler to "accept" an event to prevent it from being delivered to other events.

> 默认情况下，事件会传递给所有事件处理程序，而没有办法让事件处理程序“接受”事件，以防止其传递给其他事件。

While event handlers have no comparison operators between one another (so no sorting), the order of delivery of events to event handlers should be deterministic and should be in the reverse order of registration, i.e. "first registered, last delivered". Note that delivery to asynchronous event handlers (e.g. a subscription to a ROS topic for events, sent via a ROS publisher), will be sent in order, but not necessarily delivered in order.

> 事件处理程序之间没有比较运算符(因此没有排序)，但事件传递给事件处理程序的顺序应该是确定的，应该按照注册的相反顺序，即“先注册，后传递”。注意，异步事件处理程序(例如 ROS 主题的事件订阅，通过 ROS 发布者发送)的传递顺序是有序的，但不一定是按顺序传递的。

##### Event Filters

Like the Qt event system, it will be possible to create event filters, which emulate the ability to accept events and prevent them from being sent "downstream". _event_filters[event_filtersters]

> 像 Qt 事件系统一样，可以创建事件过滤器，以模拟接受事件并阻止它们被发送到“下游”的能力。_event_filters[event_filtersters]

Unlike the Qt event system, an event filter is simply like any other event handler, and will not prevent other event handlers from receiving the event. Instead, each event filter will have its own list of event handlers, each of which can accept or reject an event, allowing or denying further processing of the event within the event filter, respectively.

> 不像 Qt 事件系统，事件过滤器只是像其它事件处理程序一样，不会阻止其他事件处理程序接收事件。相反，每个事件过滤器都有自己的事件处理程序列表，每个处理程序都可以接受或拒绝事件，分别允许或拒绝在事件过滤器中进一步处理事件。

Any event handler can be added to an event filter, but "pure event sinks" are unable to accept an event, e.g. things like a ROS topic. This is because there is no feedback mechanism, i.e. a subscription to a topic has no way of indicating if an event has been accepted or rejected as it does not have a return type. Whereas, other event handlers which are functions or lambda's withing the launch system itself or ROS service calls can have a return type and therefore can accept or reject an event.

> 任何事件处理程序都可以添加到事件过滤器中，但是“纯事件接收器”无法接受事件，例如 ROS 主题。这是因为没有反馈机制，即订阅主题无法指示事件是否被接受或拒绝，因为它没有返回类型。而其他事件处理程序，如启动系统本身或 ROS 服务调用中的函数或 lambda，可以有返回类型，因此可以接受或拒绝事件。

#### Sending Events

It should be possible for users of the launch system send events, in addition to the system being able to do so itself.

> 用户应该可以使用 launch 系统发送事件，除了系统本身也可以这样做。

## System Description

The system description is a declarative set of actions and reactions that describe what the user wants to launch in a format that the launch system can interpret.

> 系统描述是一组声明性的操作和反应，描述用户想要以可由 launch 系统解释的格式 launch 的内容。

The goal of the system description is to capture the intentions of the user describing the system to be launched, with as few side effects as possible. The reason for doing this is so that a launch description can be visualized and statically analyzed without actually launching the described system. Having a tool that can allow a developer to visualize and modify the launch description in a WYSIWYG (what you see is what you get) editor is an important use case for the system description.

> 系统描述的目标是捕获用户描述要启动的系统的意图，尽可能少的副作用。这样做的原因是，可以在不实际启动所描述的系统的情况下可视化和静态分析启动描述。拥有一个可以让开发人员在 WYSIWYG(你所看到的就是你所得到的)编辑器中可视化和修改启动描述的工具，是系统描述的重要用例。

First, this section will describe in a programming language, or text markup, agnostic way what can be expressed in the system description and how it maps to the calling conventions and event handling described in previous sections, as well as how it maps to launch system specific behaviors. After that, it will suggest how this agnostic system description can be applied to Python and XML, but also how it might be able to be extended to support other languages and markups.

> 首先，本节将以编程语言或文本标记中无关的方式描述系统描述中可以表达的内容，以及它如何映射到前面章节中描述的调用约定和事件处理，以及它如何映射到特定启动系统的行为。之后，它将建议如何将这种无关的系统描述应用于 Python 和 XML，以及如何能够将其扩展以支持其他语言和标记。

### Launch Descriptions

The system is described in parts which we'll refer to here as "Launch Descriptions". Launch descriptions are made of up of an ordered list of actions and groups of actions. It may also contain substitutions throughout the description, which are used to add some flexibility and configuration to the descriptions in a structured way.

> 系统在此被描述为“launch 描述”的部分。launch 描述由有序的动作列表和动作组成。它也可能包含在描述中的替换，用于以结构化的方式为描述添加一些灵活性和配置。

#### Actions

Actions may be one of several things, and each action has a type (associated with the action's function) and may also contain arbitrary configurations. Actions represent an intention to do something, but a launch description is parsed first, then actions are taken in order of definition later. This allows actions to be interpreted and then statically introspected without actually executing any of them unless desired.

> 行动可能有几种，每种行动都有一种类型(与行动的功能相关联)，也可能包含任意配置。行动表示有意做某事的意图，但首先要解析启动描述，然后按定义顺序采取行动。这样，可以解释行动，然后静态地检查它们，除非有必要，否则不执行任何行动。

By separating the declaration of an action from the execution of an action, tools may use the launch descriptions to do things like visualize what a launch description will do without actually doing it. The launch system will simply use the interpreted actions in the launch descriptions to actually execute the actions.

> 通过将动作声明与动作执行分离，工具可以使用启动描述来做一些事情，比如可以在不实际执行的情况下可视化启动描述会做什么。启动系统将简单地使用启动描述中解释的动作来实际执行动作。

Basic actions include being able to:

> 基本操作包括：

- include another launch description
- modify the launch system configurations at the current scope
- execute a process
- register/unregister an event handler
- emit an event
- additional actions defined by extensions to the launch system

Actions may also yield more actions and groups rather than perform an actual task. For example, an action to "run a node" may end up resulting in "executing two process" or in "executing a process and registering an event handler". These kind of actions could be thought of a launch description generators or macros, since they effectively generate the same contents as a launch description, but look like an action to the user.

> 行动也可能会产生更多的行动和群体，而不是执行实际的任务。例如，运行节点的行动可能最终导致"执行两个进程"或"执行一个进程并注册事件处理程序"。这类行动可以被认为是启动描述生成器或宏，因为它们有效地生成与启动描述相同的内容，但对用户来说看起来像一个行动。

This allows for more complex actions which might include, but not be limited to:

> 这允许更复杂的行动，可能包括但不限于：

- include a launch description from a file with a certain markup type
- set an environment variable
- run a single-node process
- run a multi-node process
- run a node container
- run a node proxy to load into a node container
- run a process on a remote computer
- declare launch description arguments
  - exposed as either:
    - command line arguments for top-level launch descriptions
    - or additional arguments to the "include another launch description" action
  - stored in "launch system configuration"
- various OS actions, e.g. touch a file, read a file, write to a file, etc...

Each of these actions would be able to generate one or more other actions. This can be used to run one or more processes with a single action statement, or to simply provide some "syntactic sugar" For example, a "run a single-node process" action might take ROS specific configurations, then expand them to generic configurations for one of the basic actions like the "execute a process" action.

> 这些操作中的每一个都可以产生一个或多个其他操作。这可以用于用一个操作语句运行一个或多个进程，或者只是提供一些“语法糖”。例如，“运行单节点进程”操作可以采用 ROS 特定配置，然后将其扩展为基本操作(如“执行进程”操作)的通用配置。

##### Including Another Launch Description

One of the simplest actions is to include another launch description. This launch description is processed in its entirety, including parsing of any launch descriptions it includes recursively. Therefore processing of launch descriptions is in order, and depth first.

> 一个最简单的行动就是包括另一个启动描述。这个启动描述被完整地处理，包括递归解析任何包含的启动描述。因此，启动描述的处理是有序的，而且是深度优先的。

Included launch descriptions inherit all configurations of the current launch description, and any changes to the launch system configurations made in the included launch description will affect actions after the include action.

> 包含的启动描述继承当前启动描述的所有配置，并且在包含的启动描述中对启动系统配置所做的任何更改都将影响 include 操作之后的动作。

However, it should also be possible to control which configurations are inherited by an included launch description and also to "scope" an included launch description so that it cannot affect the configuration above it.

> 然而，也应该可以控制哪些配置被包含的启动描述继承，并且还可以“作用域”包含的启动描述，以便它不能影响上面的配置。

In all cases, the desired behavior may be achieved though selective use of optionally scoped group actions.

> 在所有情况下，通过选择性使用可选的范围组操作，都可以实现所需的行为。

##### Launch System Configuration

The "modify the launch system configurations at the current scope" action mentioned above is able to mutate a local scope of configurations which can affect other actions which come after the modification. Actions may use this local state to uniformly apply certain settings to themselves.

> 上面提到的“修改当前范围的启动系统配置”动作能够改变一个本地配置范围，这可能会影响修改之后的其他动作。动作可以使用这个本地状态来统一应用某些设置给它们自己。

For example, the environment variables which are set when running an operating system process would be taken from the launch system configuration, and therefore can be modified with an action. Then any "execute a process" actions which come after it will be affected by the configuration change.

> 例如，在运行操作系统进程时设置的环境变量将从启动系统配置中获取，因此可以通过操作进行修改。然后，任何“执行进程”操作都将受到配置更改的影响。

Changes to the local state by included launch descriptions persist, as they should be thought of as truly included in the same file, as if you had copied the contents of the included launch description in place of the include action. To avoid this, a group without a namespace could be used to produce a push-pop effect on the launch configurations.

> 本地状态的更改由包含的启动描述持久化，因为它们应该被认为是真正包含在同一个文件中，就好像您将包含的启动描述的内容复制到包含操作的位置一样。为了避免这种情况，可以使用不带命名空间的组来产生对启动配置的推送-弹出效果。

##### Execute a Process

Another basic action would be to execute a subprocess, with arguments and emitted events, as described in the calling conventions section under "operating system process".

> 另一个基本操作是执行子进程，根据“操作系统进程”下的调用约定条款中描述的参数和发出的事件。

This action will take a few required arguments, a few optional requirements, and also take settings from the launch system configurations if they're not explicitly given. The signature of this action should be similar to the API of Python's `subprocess.run` function_run[runbprocess_run]. Basically taking things like the executable file, arguments, working directory, environment, etc. as input and reporting the return code, stdout and stderr, and any errors as emitted events.

> 这个操作需要几个必需的参数，一些可选的要求，如果没有明确给出，还可以从启动系统配置中获取设置。该操作的签名应类似于 Python 的 `subprocess.run` 函数_run[runbprocess_run]的 API。基本上把可执行文件、参数、工作目录、环境等作为输入，并报告返回码、stdout 和 stderr 以及任何错误作为发出的事件。

Also, every executed process will automatically setup a few event handlers, so that the user can emit events to ask the launch system to terminate the process (following the signal escalation described in previous sections), signal the process explicitly, or write to the `stdin` of the process. More sophisticated calling conventions which are based on the "operating system process" may include other default event handlers.

> 每个执行的进程也会自动设置一些事件处理程序，以便用户可以发出事件来要求启动系统终止进程(遵循前面部分描述的信号升级)，显式发出信号给进程，或者写入进程的 stdin。基于“操作系统进程”的更复杂的调用约定可能还包括其他默认的事件处理程序。

##### Event Handlers

The launch description can also contain event handlers. An event handler is essentially a function which takes an event as input and returns a launch description to be included at the location of the event handler registration. The event handler will be executed asynchronously when the associated event is emitted.

> launch 描述也可以包含事件处理程序。事件处理程序本质上是一个函数，它将事件作为输入，并返回要包含在事件处理程序注册位置的 launch 描述。当相关事件发出时，事件处理程序将异步执行。

There are two actions associated with event handlers, registering one and unregistering one. How event types and event handlers are represented and tracked depends on the implementation of the launch system. However, as an example, a "launch file" written in Python might represent event's as classes which inherit from a base class. If instead the "launch file" is written in XML, event types might be expressed as a string, with launch system events using a "well-known name" and with user definable events being represented with unique strings as well. Similarly, the Python based "launch file" might use instances of objects to represent registered event handlers, therefore you might need that object to perform the unregister action. And in the XML based "launch file" the user might be required to give a unique name to all registered event handlers, so that they can unregistered with the same name later.

> 有两个与事件处理程序相关的操作，一个是注册，另一个是注销。事件类型和事件处理程序的表示和跟踪取决于启动系统的实现。但是，例如，用 Python 编写的“启动文件”可以将事件表示为从基类继承的类。如果“启动文件”是用 XML 编写的，事件类型可以用字符串表示，启动系统事件使用“已知名称”，用户定义的事件用唯一字符串表示。同样，基于 Python 的“启动文件”可以使用对象的实例来表示已注册的事件处理程序，因此您可能需要该对象来执行注销操作。而在基于 XML 的“启动文件”中，用户可能需要为所有已注册的事件处理程序提供唯一的名称，以便稍后可以使用相同的名称进行注销。

When an event handler finishes, it is able to return a launch description which is implicitly given to the include action at the location of the event handler's registration. This allows an event handler to cause any action upon completion, e.g. include another launch description, unregister an event handler, emit another event, run a process, start the termination of a process by emitting an event, etc...

> 当事件处理程序完成时，它可以返回一个启动描述，该描述隐式地给予事件处理程序注册位置的包含操作。这允许事件处理程序在完成时引起任何操作，例如包括另一个启动描述，注销事件处理程序，发出另一个事件，运行一个进程，通过发出一个事件启动一个进程的终止等等...

The lowest level of event handlers is the function which takes an event and returns a launch description. For example, a user defined event handler might look like this in Python:

> 最低级别的事件处理程序是一个接受事件并返回启动描述的函数。例如，用户定义的事件处理程序在 Python 中可能看起来像这样：

```python
    # This is a made up example of an API, consider it pseudo code...

    launch_description = LaunchDescription(...)
    # ...

    def my_process_exit_logger_callback(event: ProcessExitedEvent) -> LaunchDescription:
        print(f"process with pid ''")

    launch_description.register_event_handler(
        ProcessExitedEvent, my_process_exit_logger_callback, name='my_process_exit_logger')
```

However, to remove boilerplate code or to avoid programming in markup descriptions, common event handler patterns can be encapsulated in different event handler signatures.

> 然而，为了消除样板代码或避免在标记描述中编程，可以将常见的事件处理器模式封装在不同的事件处理器签名中。

For example, there might be the `on_event` event handler signature, which then returns a given set of actions or groups the user provides. This signature might be useful to after ten seconds start a node or include another launch file, and in XML it might look like this:

> 例如，可能会有 `on_event` 事件处理程序签名，然后返回用户提供的一组操作或组。这个签名可能有用，在 10 秒钟后启动一个节点或包括另一个启动文件，在 XML 中可能看起来像这样：

```xml
    <!-- This is a made up example of a markup, consider it pseudo code... -->
    <!-- Also this could be made even simpler by just having a tag which lets -->
    <!-- you specify the extra actions directly, rather than emitting an event and -->
    <!-- handling it, but this demonstrates the custom event handler signature -->

    <emit_event after="10" type="my_custom_timer_event" />

    <on_event event_type="my_custom_timer_event">
      <!-- actions to be included when this event occurs -->
      <include file="$(package-share my_package)/something.launch.xml" />
      <node pkg="my_package" executable="my_exec" />
      <group namespace="my_ns">
        <node pkg="my_package" executable="my_exec" />
      </group>
    </on_event>
```

##### Emitting Events

Another basic action that the launch system should support is the ability to emit events and if necessary declare new kinds of events before emitting them.

> launch 系统应该支持的另一个基本操作是能够发出事件，如有必要，在发出事件之前声明新的事件类型。

This feature could be used by users to filter a launch system event and then dispatch it to other user defined event handlers, or to create new or modified events based on existing events.

> 这个功能可以让用户根据系统事件进行过滤，然后将其分发到其他用户定义的事件处理器，或者基于现有事件创建新的或修改过的事件。

How events are defined is up to the implementation, but it should be possible to model the events so they can be emitted and then handled by registered event handlers.

> 事件的定义取决于实现，但应该可以模拟事件，以便它们可以被发出，然后由注册的事件处理程序处理。

#### Groups

<div class="alert alert-warning" markdown="1">

TODO:

> 待办：

- can be broken into: - "namespace" (like roslaunch), - conditionals (`if` and `unless`) (see: [https://wiki.ros.org/roslaunch/XML#if_and_unless_attributes)](https://wiki.ros.org/roslaunch/XML#if_and_unless_attributes)), and - scope (push-pop for configurations) - should consider what we're discussing to do in [https://github.com/ros2/launch/issues/313](https://github.com/ros2/launch/issues/313)

</div>

#### Substitutions

<div class="alert alert-warning" markdown="1">

TODO:

> 待办：

- equivalent to substitutions in ROS 1, see: [https://wiki.ros.org/roslaunch/XML#substitution_args](https://wiki.ros.org/roslaunch/XML#substitution_args)
- they've already been implemented in the reference implementation, they should at least be summarized as built here

</div>

### Mapping to Programming Languages and Markup Languages

<div class="alert alert-warning" markdown="1">

TODO:

> 待办事项：

- Explain in general how the features described in the previous sections would map to a programming language and/or markup language and any considerations therein.
- How it would map to Python (likely implementation)
- How it would map to XML (likely first markup language)

</div>

## Execution and Verification of the System Description

<div class="alert alert-warning" markdown="1">

TODO: Restructure notes on this and put them here.

> TODO：重新组织这里的笔记，并把它们放在这里。

Temporary summary:

> 暂时总结：

Whether described via static file or programmatically, once the system is described it has to be executed, and this section will cover all of that. Most of this is already covered in the "calling conventions" section, but this section will also cover some more details about execution, and then add on to that verification (starting another discussion about what the launch system should and should not do itself). Verification is runtime assertion that mirrors the static analysis that can be done off-line.

> 无论是通过静态文件还是程序描述，一旦系统被描述，它就必须被执行，本节将涵盖所有内容。大部分内容已经在“调用约定”部分进行了介绍，但本节还将涵盖有关执行的更多细节，然后再增加验证(开始另一个关于启动系统应该和不应该自己做什么的讨论)。验证是运行时断言，反映了可以离线进行的静态分析。

</div>

## Testing

<div class="alert alert-warning" markdown="1">

TODO: Restructure notes on this and put them here.

> TODO：重新组织这里的笔记，并把它们放在这里。

Temporary summary:

> 临时总结：

In ROS 1, `rostest` is an important extension of `roslaunch`, and so far in ROS 2 we're already using the foundation of launching (executing processes and reacting to their exit, return codes, and stdout/stderr), called `ros2/launch_testing` right now, to implement some tests. This section will cover how that happens and how it integrates with the static description files as well as the programmatic API, adding ROS specific concepts to what we're already doing with `ros2/launch_testing`.

> 在 ROS 1 中，`rostest` 是 `roslaunch` 的一个重要扩展，到目前为止，我们已经在 ROS 2 中使用启动(执行进程并对其退出、返回代码和 stdout / stderr 进行反应)的基础，称为 `ros2 / launch_testing`，来实现一些测试。本节将介绍这种情况以及它如何与静态描述文件以及编程 API 集成，为我们正在使用 `ros2 / launch_testing` 的内容添加 ROS 特定的概念。

</div>

## Requirements

<div class="alert alert-warning" markdown="1">

TODO: Reformat requirements list, possibly combine/reconcile with "separation of concerns section" (consider dropping in favor of renaming to something that implies requirements as well)

> TODO：重新格式化要求列表，可能与“职责分离部分”合并/协调(考虑放弃以改为暗示要求的名称)

</div>

## Reference Implementation Proposal

<div class="alert alert-warning" markdown="1">

TODO: This will outline what we have and what we need to build and how it should be separated.

> TODO：这将概述我们拥有什么，需要建立什么，以及它应该如何分离。

</div>

## Alternatives

<div class="alert alert-warning" markdown="1">

TODO: Anything we choose not to support in the requirements vs. the "separation of concern section", and also any alternatives which we considered but rejected in the reference implementation proposal.

> TODO：我们在需求 vs.“关注分离”部分没有选择支持的任何内容，以及在参考实施方案中我们考虑但拒绝的所有替代方案。

</div>

## References

[^calling_convention_wikipedia]: [https://en.wikipedia.org/wiki/Calling_convention](https://en.wikipedia.org/wiki/Calling_convention)


> [^calling_convention_wikipedia]: [https://zh.wikipedia.org/wiki/%E8%B0%83%E7%94%A8%E7%BB%93%E7%AE%97](https://zh.wikipedia.org/wiki/%E8%B0%83%E7%94%A8%E7%BB%93%E7%AE%97)
>

[^logging_wiki]: [https://github.com/ros2/ros2/wiki/Logging#console-output-configuration](https://github.com/ros2/ros2/wiki/Logging#console-output-configuration)


> [^logging_wiki]: [https://github.com/ros2/ros2/wiki/Logging#console-output-configuration](https://github.com/ros2/ros2/wiki/Logging#console-output-configuration)
>

[^logging_wiki]: [https://github.com/ros2/ros2/wiki/Logging#控制台输出配置](https://github.com/ros2/ros2/wiki/Logging#%E6%8E%A7%E5%88%B6%E5%8F%B0%E8%BE%93%E5%87%BA%E9%85%8D%E7%BD%AE)


[^static_remapping]: [http://design.ros2.org/articles/static_remapping.html#remapping-rule-syntax](http://design.ros2.org/articles/static_remapping.html#remapping-rule-syntax)


> [^static_remapping]: [http://design.ros2.org/articles/static_remapping.html#remapping-rule-syntax](http://design.ros2.org/articles/static_remapping.html#remapping-rule-syntax)
>

[^static_remapping]: [http://design.ros2.org/articles/static_remapping.html#remapping-rule-syntax](http://design.ros2.org/articles/static_remapping.html#remapping-rule-syntax)：静态重映射规则语法


[^lifecycle]: [http://design.ros2.org/articles/node_lifecycle.html](http://design.ros2.org/articles/node_lifecycle.html)


> [^生命周期]: [http://design.ros2.org/articles/node_lifecycle.html](http://design.ros2.org/articles/node_lifecycle.html)
>

[^parameters]: [http://design.ros2.org/articles/ros_parameters.html](http://design.ros2.org/articles/ros_parameters.html)


> [参数]: %5Bhttp://design.ros2.org/articles/ros_parameters.html%5D(http://design.ros2.org/articles/ros_parameters.html)

[^qt_event_filters]: [https://doc.qt.io/archives/qt-4.8/eventsandfilters.html#event-filters](https://doc.qt.io/archives/qt-4.8/eventsandfilters.html#event-filters)


> [^qt_event_filters]: [https://doc.qt.io/archives/qt-4.8/eventsandfilters.html#event-filters](https://doc.qt.io/archives/qt-4.8/eventsandfilters.html#event-filters)
>

_event_filters_event_filters[event_filtersters]：[https://doc.qt.io/archives/qt-4.8/eventsandfilters.html#event-filters](https://doc.qt.io/archives/qt-4.8/eventsandfilters.html#event-filters)

[^subprocess_run]: [https://docs.python.org/3.6/library/subprocess.html#subprocess.run](https://docs.python.org/3.6/library/subprocess.html#subprocess.run)


> [^subprocess_run]: [https://docs.python.org/3.6/library/subprocess.html#subprocess.run](https://docs.python.org/3.6/library/subprocess.html#subprocess.run)
>

[^subprocess_run]: [https://docs.python.org/3.6/library/subprocess.html#subprocess.run](https://docs.python.org/3.6/library/subprocess.html#subprocess.run) 简体中文版


\*[operating system process]: Operating System Process

> 操作系统进程：操作系统进程

\*[operating system processes]: Operating System Processes

> 操作系统进程
