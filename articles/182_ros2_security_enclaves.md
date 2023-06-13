---
tip: translate by openai@2023-05-29 22:38:31
layout: default
title: ROS 2 Security Enclaves
permalink: articles/ros2_security_enclaves.html
abstract:
  This article specifies the integration of security enclaves.
author:  >
  [Ruffin White](https://github.com/ruffsl),
  [Mikael Arguedas](https://github.com/mikaelarguedas)
date_written: 2020-05
last_modified: 2020-07
published: true
categories: Security
Authors: 
Date Written: 
Last Modified:
---
This design document formalizes the integration of ROS 2 with security enclaves. In summary, all secure processes must use an enclave that contains the runtime security artifacts unique to that enclave, yet each process may not necessarily have a unique enclave. Multiple enclaves can be encapsulated in a single security policy to accurately model the information flow control. Users can tune the fidelity of such models by controlling at what scope enclaves are applied at deployment. E.g. one unique enclave per OS process, or per OS user, or per device/robot, or per swarm, etc. The rest of this document details how enclaves can be organized and used by convention.

> 这份设计文档正式将 ROS 2 与安全区域集成起来。总的来说，所有安全进程必须使用一个包含该区域的运行时安全特征的区域，但是每个进程不一定有一个唯一的区域。多个区域可以封装在一个安全策略中，以准确地模拟信息流控制。用户可以通过控制部署时应用的范围来调整这种模型的保真度。例如，每个操作系统进程，或每个操作系统用户，或每个设备/机器人，或每个群体等等。本文档的其余部分详细说明了如何按照惯例组织和使用区域。

## Concepts

Before detailing the SROS 2 integration of the enclaves, the following concepts are introduced.

> 在详细介绍 SROS 2 与飞地的集成之前，首先介绍以下概念。

### Participant

Participant is the object representing a single entity on the network. In the case of DDS, the `Participant` is a DDS DomainParticipant, which has both access control permissions and a security identity.

> 参与者是代表网络上的单个实体的对象。在 DDS 的情况下，`参与者` 是一个 DDS DomainParticipant，它具有访问控制权限和安全身份。

### Namespaces

Namespaces are a fundamental design pattern in ROS and are widely used to organize and differentiate many types of resources as to be uniquely identifiable; i.e. for topics, services, actions, and node names. As such, the concept of namespacing is well known and understood by current users, as well as strongly supported by the existing tooling. Namespaces are often configurable at runtime via command line arguments or statically/programmatically via launch file declarations.

> 命名空间是 ROS 中的一种基本设计模式，广泛用于组织和区分许多类型的资源，以便唯一标识；即主题，服务，操作和节点名称。因此，命名空间的概念已经被当前用户所熟知和理解，并得到现有工具的强有力支持。命名空间通常可以通过命令行参数或静态/编程方式通过启动文件声明在运行时配置。

Previously, the Fully Qualified Name (FQN) of a node was used directly by a selected security directory lookup strategy to load the necessary key material. However, now that Participants map to contexts and not nodes, such a direct mapping of node FQN to security artifacts is no longer appropriate.

> 以前，完全限定名(FQN)的节点被选定的安全目录查找策略直接使用，以加载必要的密钥材料。但是，现在参与者映射到上下文而不是节点，这种节点 FQN 到安全工件的直接映射不再适用。

### Contexts

With the advent of ROS 2, multiple nodes may now be composed into one process for improved performance. Previously however, each node would retain it's one to one mapping to a separate middleware `Participant`. Given the non-negligible overhead incurred of multiple `Participant` s per process, a change was introduced to map a single `Participant` to a context, and allow for multiple nodes to share that context.

> 随着 ROS 2 的出现，多个节点现在可以组合成一个进程以提高性能。然而，以前每个节点都会保留其一对一映射到单独的中间件“Participant”。鉴于每个进程所产生的多个“Participant”的不可忽视的开销，引入了一项更改，将单个“Participant”映射到上下文，并允许多个节点共享该上下文。

Based on the DDS Security specification v1.1, a `Participant` can only utilise a single security identity; consequently the access control permissions applicable to every node mapped to a given context must be consolidated and combined into a single set of security artifacts, or enclave. Thus, all contexts and their respective participants in each process will then use that single enclave. As such, additional tooling and extensions to SROS 2 are necessary to support this new paradigm.

> 根据 DDS 安全规范 v1.1，`参与者` 只能使用单个安全标识；因此，映射到给定上下文的每个节点的访问控制权限必须合并并组合成单个安全工件或飞地。因此，每个过程中的所有上下文及其各自的参与者将使用该单个飞地。因此，需要额外的工具和 SROS 2 的扩展来支持这种新的范式。

## Keystore

With the addition of contexts, it’s perhaps best to take the opportunity to restructure the keystore layout as well. Rather than a flat directory of namespaced node security directories, we can push all such security directories down into a designated `enclaves` sub-folder. Similarly, private and public keystore materials can also be pushed down into their own respective sub-folders within the root keystore directory. This is reminiscent of the pattern used earlier in Keymint [1].

> 随着上下文的增加，最好利用这个机会重新组织密钥库布局。不是一个平面目录的命名空间节点安全目录，我们可以把所有这样的安全目录推到一个指定的“enclaves”子文件夹中。私钥和公钥库材料也可以推到根密钥库目录的各自子文件夹中。这与 Keymint [1]中使用的模式非常相似。

```
$ tree keystore/
keystore
├── enclaves
│   └── ...
│       └── ...
├── private
│   ├── ca.csr.pem
│   └── ca.key.pem
└── public
    ├── ca.cert.pem
    ├── identity_ca.cert.pem -> ca.cert.pem
    └── permissions_ca.cert.pem -> ca.cert.pem
```

### `public`

The `public` directory contains anything permissible as public, such as public certificates for the identity or permissions certificate authorities. As such, this can be given read access to all executables. Note that in the default case, both the `identity_ca` and `permissions_ca` points to the same CA certificate.

> `public` 目录包含任何可以公开的东西，比如身份或权限证书颁发机构的公共证书。因此，可以给所有可执行文件读取权限。请注意，在默认情况下，`identity_ca` 和 `permissions_ca` 都指向同一个 CA 证书。

### `private`

The `private` directory contains anything permissable as private, such as private key material for aforementioned certificate authorities. This directory should be redacted before deploying the keystore onto the target device/robot.

> 私有目录包含任何可以作为私有的东西，比如上述证书颁发机构的私钥材料。在将密钥库部署到目标设备/机器人之前，应该把这个目录资料屏蔽掉。

### `enclaves`

The `enclaves` directory contains the security artifacts associated with individual enclaves, and thus node directories are no longer relevant. Similar to node directories however, the `enclaves` folder may still recursively nest sub-paths for organizing separate enclaves. The `ROS_SECURITY_KEYSTORE` environment variable should by convention point to this directory.

> `enclaves` 目录包含与单个飞地相关的安全项，因此节点目录不再可用。与节点目录类似，`enclaves` 文件夹仍可以递归嵌套子路径以组织不同的飞地。`ROS_SECURITY_KEYSTORE` 环境变量应该按照惯例指向此目录。

## Integration

With the introduction of contexts into rcl, instead of relying on node namespaces to lookup security artifacts from the keystore, the enclave path is now completely disassociated from node namespaces, instead serving as its own unique resource identifier. Although having to book keep both identifier spaces simultaneously may introduce more degrees of freedom, it should still be possible to organize enclaves within the keystore to mimic node namespace hierarchy for transparent traceability of allocated permissions.

> 随着上下文的引入，rcl 不再依赖节点命名空间从密钥库查找安全性物件，而是将飞地路径完全与节点命名空间分离，而成为其自身独特的资源标识符。虽然同时需要跟踪两个标识空间可能会引入更多的自由度，但仍然可以在密钥库中组织飞地，以模仿节点命名空间层次，以便透明跟踪分配的权限。

## Future Work

Introspective tooling and launchfile interfaces should be updated to help lessen the complexity introduced with the migration to contexts and enclaves.

> 工具内省和启动文件接口应该更新，以帮助减少迁移到上下文和飞地时引入的复杂性。

### Runtime

Given the normative case where an enclave within a policy may be specific to a single node/container process, the namespace the node is remapped to will inevitably affect the required security permissions necessary within the enclave. To highlight this interdependency, and to help avoid enclave path collisions, a hierarchy borrowing namespaces is appropriate. By convention, ros2launch could be used to prefix relative enclave paths for single process node or containers using the namespace in scope, to enable a convention of composable launch files with adjustable and parameterized enclave paths. Given the runtime command argument for specifying the fully qualified enclave path, ros2launch would accordingly resolve relative enclave paths for executables, as defined by launch attributes.

> 给定一个政策内的隔离环境可能特定于单个节点/容器进程的规范情况，节点重映射到的命名空间将不可避免地影响隔离环境中所需的安全权限。为了突出这种相互依赖性，并帮助避免隔离环境路径冲突，借用命名空间的层次结构是合适的。按约定，可以使用 ros2launch 来为使用范围内的命名空间的单个进程节点或容器的相对隔离环境路径添加前缀，以实现可组合的启动文件，具有可调整和参数化的隔离环境路径。给定用于指定完全限定隔离环境路径的运行时命令参数，ros2launch 将相应地解析由启动属性定义的相对隔离环境路径中的可执行文件。

### Unqualified enclave path

For single process nodes with unqualified enclave paths, the enclave directory will subsequently default to the root level enclave.

> 对于具有不合格的保护区路径的单进程节点，保护区目录将随后默认为根级别保护区。

```xml
<launch>
  <node pkg="demo_nodes_cpp" exec="talker"/>
  <node pkg="demo_nodes_cpp" exec="listener"/>
</launch>
```

```
$ tree enclaves/
enclaves/
├── cert.pem
├── governance.p7s
├── identity_ca.cert.pem -> ../public/identity_ca.cert.pem
├── key.pem
├── permissions_ca.cert.pem -> ../public/permissions_ca.cert.pem
└── permissions.p7s
```

### Pushed unqualified enclave path

For single process nodes with unqualified enclave paths pushed by a namespace, the enclave directory will subsequently be pushed to the relative sub-folder.

> 对于由命名空间推送的未经认证的环境路径的单进程节点，环境目录将随后被推送到相关的子文件夹中。

```xml
<launch>
  <node pkg="demo_nodes_cpp" exec="talker"/>
  <group>
    <push_ros_namespace namespace="foo"/>
    <node pkg="demo_nodes_cpp" exec="listener"/>
  </group>
</launch>
```

```
$ tree --dirsfirst enclaves/
enclaves/
├── foo
│   ├── cert.pem
│   ├── governance.p7s
│   ├── identity_ca.cert.pem
│   ├── key.pem
│   ├── permissions_ca.cert.pem
│   └── permissions.p7s
├── cert.pem
├── governance.p7s
├── identity_ca.cert.pem
├── key.pem
├── permissions_ca.cert.pem
└── permissions.p7s
```

> Symbolic links suppressed for readability

### Relatively pushed qualified enclave path

For single process nodes with qualified enclave paths pushed by a namespace, the qualified enclave directory will subsequently be pushed to the relative sub-folder.

> 对于由命名空间推送的具有合格的秘密保护区路径的单进程节点，合格的秘密保护区目录将随后被推送到相对的子文件夹中。

```xml
<launch>
  <group>
    <push_ros_namespace namespace="foo"/>
    <node pkg="demo_nodes_cpp" exec="listener" enclave="bar"/>
  </group>
</launch>
```

```
$ tree --dirsfirst enclaves/
enclaves/
└── foo
    └── bar
        ├── cert.pem
        ├── governance.p7s
        ├── identity_ca.cert.pem
        ├── key.pem
        ├── permissions_ca.cert.pem
        └── permissions.p7s
```

### Fully qualified enclave path

For single process nodes with absolute enclave paths, namespaces do not subsequently push the relative sub-folder.

> 对于具有绝对隔离路径的单进程节点，命名空间不会随后推送相对子文件夹。

```xml
<launch>
  <group>
    <push_ros_namespace namespace="foo"/>
    <node pkg="demo_nodes_cpp" exec="listener" enclave="/bar"/>
  </group>
</launch>
```

```
$ tree --dirsfirst enclaves/
enclaves/
└── bar
    ├── cert.pem
    ├── governance.p7s
    ├── identity_ca.cert.pem
    ├── key.pem
    ├── permissions_ca.cert.pem
    └── permissions.p7s
```

## Alternatives

#### `<push_ros_namespace namespace="..." enclave="foo"/>`

One such approach could be done by adding a `enclave` attribute to `push_ros_namespace` element.

> 一种这样的方法可以通过向 `push_ros_namespace` 元素添加 `enclave` 属性来实现。

This also keeps the pushing of enclaves close/readable to pushing of namespaces.

> 这也使得推送封闭区域与推送名称空间的操作更加便捷可读。

#### `<push_ros_enclave enclave="foo"/>`

Another alternative approach could be to add an entirely new `push_ros_enclave` element.

> 另一种替代方法可以是添加一个全新的 `push_ros_enclave` 元素。

This could ensure the pushing of enclave path independent/flexable from namespaces.

> 这可以确保从命名空间中推送独立/灵活的飞地路径。

## Concerns

### Multiple namespaces per context

For circumstances where users may compose multiple nodes of dissimilar namespaces into a single context, the user must still subsequently specify a common enclave path that is applicable for all nodes composed. For circumstances where the enclave path is orthogonal to node namespace, the use of fully qualifying all relevant enclave paths could be tedious, but could perhaps could still be parametrized via the use of `<var/>`, and `<arg/>` substitution and expansion.

> 在用户可以将多个不同名称空间的节点组合成一个单独的上下文的情况下，用户仍然必须随后指定一个适用于所有组合节点的公共飞地路径。在飞地路径与节点名称空间正交的情况下，完全限定所有相关飞地路径可能会很繁琐，但也可以通过使用<var/>和<arg/>替换和展开来参数化。

### Modeling permissions of nodes in a process v.s. permission of the middleware `Participant`

Before the use of contexts, multiple nodes composed into a single process where each mapped to a separate `Participant`. Each `Participant` subsequently load a security identity and access control credential prevalent to its' respective node. However, all nodes in that process share the same memory space and can thus access data from other nodes. There is a mismatch between the middleware credentials/permissions loaded and the resources accessible within the process.

> 在使用上下文之前，多个节点组合成一个单独的进程，每个节点映射到一个单独的“参与者”。每个“参与者”随后加载适用于其各自节点的安全身份和访问控制凭据。然而，该进程中的所有节点共享相同的内存空间，因此可以访问来自其他节点的数据。中间件凭据/权限与进程中可访问的资源之间存在不匹配。

By using enclaves, all nodes in a context share the same security identity and access control credentials. This inevitably means that code compiled to node `foo` can access credentials/permissions only trusted to node `bar`. This consequence of composition could unintendedly subvert the minimal spanning policy as architected by the policy designer or measured/generated via ROS 2 tooling/IDL.

> 通过使用封闭空间，上下文中的所有节点共享相同的安全身份和访问控制凭据。这不可避免地意味着编译到节点“foo”的代码只能访问只信任给节点“bar”的凭据/权限。这种组合的后果可能会意外地颠覆由政策设计者或通过 ROS 2 工具/ IDL 测量/生成的最小跨越策略。

With the introduction of enclaves, it becomes possible to describe the union of access control permission by defining a collection of SROS 2 policy profiles as element within a specific enclave. This would allow for formal analysis tooling [2] to check for potential violations in information flow control given the composing of nodes at runtime. If a process loads a single enclave, this reconciles the permissions of a `Participant` and the ones of the process.

> 随着飞地的引入，通过定义一组 SROS 2 策略配置文件作为特定飞地内的元素，可以描述访问控制权限的联合。这将允许形式分析工具[2]在运行时组合节点时检查信息流控制的潜在违规行为。如果一个过程加载一个飞地，这就协调了“参与者”的权限和过程的权限。

However, should multiple enclaves be loaded per process, then such security guaranties are again lost because of shared same memory space. Thus it should be asked whether if multiple enclaves per process should even be supported.

> 然而，如果每个进程加载多个保护域，由于共享的相同的内存空间，这些安全保证又会丢失。因此，应该考虑是否应该支持每个进程多个保护域。

In summary, the distinction here is that before, the composition of multiple node permissions could not be conveyed to the tooling. Whether nodes could gain the permission of others in the same process space is not the hinge point of note; it's the fact that such side effects could not be formally modeled or accounted for by the designer. It will now be possible with enclaves, however allowing for multiple contexts per process that load separate enclaves would reintroduce and exacerbate the same modeling inaccuracies.

> 总之，这里的区别是，以前，多个节点权限的组合无法传达给工具。在同一个进程空间中，节点是否能够获得其他人的权限并不是值得注意的关键点；重要的是，这种副作用无法由设计者正式建模或核算。然而，使用隔离环境将可以实现，但是允许每个进程有多个上下文加载单独的隔离环境将会重新引入和加剧相同的建模不准确性。

### Composable launchfile includes

A particular challenge in using launchfiles with security enclaves is that of keeping the include hierarchy composable. An inherit tradeoff between simplicity and configurability can arise when writing launchfiles for downstream use. Authors can selectively choose what attributes to expose as input arguments, while users may implicitly override provided defaults.

> 在使用安全保护区域的启动文件时，一个特殊的挑战是保持包含层次结构可组合。在编写用于下游使用的启动文件时，可能会出现简单性和可配置性之间的折衷。作者可以选择性地选择要公开的属性作为输入参数，而用户可以隐式覆盖提供的默认值。

In case of enclaves, it is not inherently clear what best practices either package authors or users should employ to retain a composable and intuitive launchfile structure. E.g: Should authors parametrize enclave paths for each node as input arguments? Should users push namespaces of included launchfiles to unique enclaves?

> 在有飞地的情况下，包作者或用户应该采用什么样的最佳实践来保持可组合的和直观的启动文件结构尚不清楚。例如：作者应该将每个节点的飞地路径作为输入参数吗？用户应该将包含的启动文件的命名空间推送到唯一的飞地吗？

To be sure though, the setting of security environment variables from within launchfiles should be discouraged, as this would restrict the use of static analysis of launchfiles combined with Node IDL for procedural policy generation.

> 确保，不应该从 launchfiles 中设置安全环境变量，因为这会限制与节点 IDL 结合使用的静态分析 launchfiles 进行程序策略生成的使用。

### Composable nodes in container

Given that containers can be dynamic, where nodes can be added or removed at runtime, there is perhaps some question as to how containers should integrate with secure enclaves. In ros2launch, the namespace in scope at the container's instantiation could be used to resolve the container's specified relative enclave path, thus to all nodes/components inside that container. This should be further deliberated when eventually extending the launch API for containers.

> 考虑到容器可以是动态的，在运行时可以添加或删除节点，因此可能会有一些问题，即容器应该如何与安全隔离环境集成。在 ros2launch 中，可以使用容器实例化时的命名空间来解析容器指定的相对隔离路径，从而解析到该容器中所有的节点/组件。当最终扩展用于容器的启动 API 时，应进一步讨论这一点。

### Migration for RMW implementations

As it may take time before all RMW implementations implement the new system of contexts, a defined fallback behavior should still be designated. For such implementations, the enclave security directory determined by RCL should be loaded for the participant as specified in the "ROS 2 DDS-Security integration" design doc. This primarily desists the use of including the node name in the default lookup path, consequently getting users in the habit of creating separate enclaves for separate processes, or explicitly specifying unique enclave paths via launchfiles.

> 由于在所有 RMW 实现实现新的上下文系统之前可能需要一些时间，因此应该指定一种默认的行为。对于这些实现，应按照“ROS 2 DDS-Security 集成”设计文档中的说明，为参与者加载由 RCL 确定的保密目录。这主要是为了防止在默认查找路径中包括节点名称，从而使用户习惯于为不同的进程创建单独的保密空间，或者通过 launchfiles 明确指定唯一的保密路径。

## References

1. [Procedurally Provisioned Access Control for Robotic Systems](https://doi.org/10.1109/IROS.2018.8594462)

> 1. [机器人系统的程序化提供的访问控制](https://doi.org/10.1109/IROS.2018.8594462)

```bibtex
@inproceedings{White2018,
  title     = ,
  author    = ,
  year      = 2018,
  booktitle = ,
  doi       = ,
  issn      = ,
  url       = 
```

2. [Network Reconnaissance and Vulnerability Excavation of Secure DDS Systems](https://doi.org/10.1109/EuroSPW.2019.00013)

> 2. 安全 DDS 系统的网络侦察和漏洞挖掘

```bibtex
@inproceedings{White2019,
  title     = ,
  author    = ,
  year      = 2019,
  booktitle = ,
  doi       = ,
  pages     = ,
  url       = 
```
