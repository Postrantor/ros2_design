---
tip: translate by openai@2023-05-29 08:25:10
layout: default
title: ROS 2 Access Control Policies
permalink: articles/ros2_access_control_policies.html
abstract:
This article specifies the policy format used for access control when securing ROS subsystem.
author:  >
[Ruffin White](https://github.com/ruffsl),
[Kyle Fazzari](https://github.com/kyrofa)
date_written: 2019-08
last_modified: 2021-06
published: true
categories: Security
Authors: 
Date Written: 
Last Modified:
---
[SROS 2](/articles/ros2_dds_security.html) introduces several security properties, including encryption, authentication, and authorization.

> SROS 2 引入了几种安全属性，包括加密、身份验证和授权。

Authorization is obtained by combining the first two properties with a model for access control.

> 授权是通过将前两个属性与访问控制模型相结合来获得的。

Such models are often referred to as access control policies.

> 这类模型通常被称为访问控制策略。

A policy serves as a high-level abstraction of privileges associated with attributes that may then be transpiled into low-level permissions for individual identities, such as specific ROS nodes within a secure DDS network.

> 一项政策作为特定属性相关的高级抽象权限，可以转化为安全 DDS 网络中个人身份(如特定 ROS 节点)的低级权限。

## Concepts

Before detailing the SROS 2 policy design of the [access control](https://en.wikipedia.org/wiki/Computer_access_control), by which the system constrains the ability of a subject to access an object, it is important to establish a few concepts useful in formalizing the design approach in terms of security.

> 在详细介绍[访问控制](https://en.wikipedia.org/wiki/Computer_access_control)的 SROS 2 策略设计之前，为了从安全角度正式化设计方法，有必要建立一些有用的概念。

In this setting, a subject may be thought of as a participant on a distributed data-bus (e.g. a ROS node in the computation graph), whereas an object may be an instance of a particular subsystem (e.g. a ROS topic), and access is defined as the capability to act upon that object (e.g. publish or subscribe).

> 在这种环境下，主体可以被认为是分布式数据总线(例如计算图中的 ROS 节点)上的参与者，而对象可以是特定子系统(例如 ROS 主题)的实例，而访问权限是指可以对该对象(例如发布或订阅)进行操作的能力。

### Mandatory Access Control

[Mandatory Access Control](https://en.wikipedia.org/wiki/Mandatory_access_control) (MAC) refers to allowing access to an object if and only if rules exist that allow a given subject to access the resource; the term mandatory denotes this requirement that a subject’s access to an object must always be explicitly provisioned.

> 强制访问控制(Mandatory Access Control，MAC)是指只有当存在允许给定主体访问资源的规则时，才允许访问对象；强制这个术语表明主体对对象的访问必须始终明确授权。

Most importantly, contrary to discretionary access control (DAC), such policies are enforced by a set of authorization rules that cannot be overridden or modified by the subject either accidentally or intentionally.

> 最重要的是，与自主访问控制(DAC)相反，这些策略由一组无法被主体意外或故意覆盖或修改的授权规则强制执行。

This could also be referred to as “deny by default”.

> 这也可以称为“默认拒绝”。

### Principle of Least Privilege

[Principle of Least Privilege](https://en.wikipedia.org/wiki/Principle_of_least_privilege) (PoLP) requires that in a particular abstraction layer, every subject must be able to access only the resources necessary for its legitimate purpose.

> 原则上最低特权(PoLP)要求，在特定的抽象层中，每个主体只能访问其合法目的所必需的资源。

This is also known as the principle of minimal privilege or the principle of least authority.

> 这也被称为最小特权原则或最低权限原则。

Applying PoLP not only bolsters system security, but can also ease deployment and help improve system stability.

> 应用 PoLP 不仅可以加强系统安全性，还可以简化部署并有助于提高系统稳定性。

### Privilege Separation

[Privilege Separation](https://en.wikipedia.org/wiki/Privilege_separation) requires that a subject’s access be divided into parts which are limited to the specific privileges they require in order to perform a specific task.

> 隔离特权要求将主体的访问权限划分为有限的部分，以便为执行特定任务而获得所需的特定权限。

This is used to mitigate the potential damage of a security vulnerability.

> 这用于减轻安全漏洞的潜在损害。

Systems unable to comply with this requirement may consequently fail to satisfy PoLP as well.

> 系统无法满足此要求可能会导致无法满足 PoLP 的要求。

### Separation of Concerns

[Separation of Concerns](https://en.wikipedia.org/wiki/Separation_of_concerns) (SoC) is a design principle for separating a system into distinct sections, so that each section addresses a separate concern.

> 分离关注点(SoC)是一种将系统分解为不同部分的设计原则，以便每个部分都能够处理不同的关注点。

Separate concerns in this case may be how encryption is governed in a system versus how authorization is given to subjects.

> 在这种情况下，分离的关注可能是系统中加密的控制方式与给予主体授权的方式。

## Criteria

Design criteria for SROS 2 policies and for selecting the [Extensible Markup Language](https://en.wikipedia.org/wiki/XML) (XML) are discussed here.

> 讨论了 SROS 2 策略的设计准则和选择[可扩展标记语言](https://en.wikipedia.org/wiki/XML) (XML)的准则。

### Validation

Prior to interpreting any user configuration input, such as an access control policy, [data validation](https://en.wikipedia.org/wiki/Data_validation) should be applied to ensure inputs are compliant and correctly formatted.

> 在解释任何用户配置输入(如访问控制策略)之前，应该应用[数据验证](https://en.wikipedia.org/wiki/Data_validation)来确保输入符合要求并格式正确。

Incorrect inputs can affect the soundness of most programs or tools, yet guarding against general malformations may itself require meticulous validation logic.

> 不正确的输入可能会影响大多数程序或工具的正确性，但要防止一般的错误形式可能需要精心的验证逻辑。

Formalizing the description of the data using a precise schema allows for separate programs to assert inputs are compliant without replicating validation logic across implementations.

> 使用精确的模式对数据的描述进行形式化，可以使不同的程序断定输入是否符合要求，而无需在实现中重复验证逻辑。

In XML, this is achieved using [XSD](https://en.wikipedia.org/wiki/XML_schema); allowing the policy markup to be defined by an extendable standard definition rather than a canonical implementation.

> 在 XML 中，这是通过使用 [XSD](https://en.wikipedia.org/wiki/XML_schema) 来实现的；允许政策标记由可扩展的标准定义而不是规范实现来定义。

### Transformation

For usability and generalizability, access control policies can be expressed using domain specific abstractions, such as ROS based subjects and objects.

> 为了可用性和通用性，访问控制策略可以使用域特定的抽象表达，比如基于 ROS 的主体和对象。

However such abstractions may translate into different representations when applied to lower level transports and policy enforcement points.

> 然而，当应用于较低级别的传输和策略执行点时，这些抽象可能会转换成不同的表示。

Formalizing this [data transformation](https://en.wikipedia.org/wiki/Data_transformation) using a transformation language allows for separate programs to transpile without replicating conversion logic across implementations.

> 使用转换语言将这种数据转换形式化，可以让不同的程序进行转译而不需要复制转换逻辑在实现中。

In XML, this is achieved using [XSLT](https://en.wikipedia.org/wiki/XML_transformation_language); allowing the policy markup to be easily transpiled for various transports by simply swapping or extending transforms.

> 在 XML 中，可以使用 [XSLT](https://en.wikipedia.org/wiki/XML_transformation_language) 来实现这一目标；通过简单地替换或扩展转换，可以轻松地将策略标记转换为各种传输格式。

### Composition

When formulating an access control policy, many subjects may share fundamental privileges for basic access.

> 当制定访问控制策略时，许多主体可能共享基本访问权限。

To avoid unnecessary repetition that could exacerbate human errors or other discrepancies, policies should possess sufficient [expressive power](https://en.wikipedia.org/wiki/Expressive_power_(computer_science)) to remain [DRY](https://en.wikipedia.org/wiki/Don%27t_repeat_yourself).

> 为了避免不必要的重复，可能会加剧人为错误或其他差异，政策应具有足够的表达能力，以保持 DRY(不重复自己)。

In XML, this is achieved using [XInclude](https://en.wikipedia.org/wiki/XInclude); allowing the policy markup to easily include or substitute external reference to particular profiles and permissions that repeat across separate policies or profiles.

> 在 XML 中，可以使用 [XInclude](%5Bhttps://en.wikipedia.org/wiki/XInclude%5D(https://en.wikipedia.org/wiki/XInclude)) 来实现；允许策略标记轻松地包含或替换跨越不同策略或配置文件的特定档案和权限的外部参考。

## Schema

The SROS 2 policy schema is defined with XML.

> SROS 2 策略架构是用 XML 定义的。

The elements and attributes that make up a policy are described below.

> 以下描述了构成政策的元素和属性。

```xml

```

### `<policy>` Tag

Root tag of the policy file.

> 策略文件的根标签。

There must be only one `<policy>` tag per policy file.

> 每个策略文件只能有一个<policy>标签。

Attributes:

> 属性：

- **version**: declared version of schema version in use

> - **版本**: 使用的架构版本的声明版本

- Allows for advancing future revisions of the schema

> 允许推进未来版本的模式。

### `<enclaves>` Tag

Encapsulates a sequence of unique enclaves.

> 封装一系列独特的飞地。

This method of nesting sequences allows for additional tags to be extended to the `<policy>` root.

> 这种嵌套序列的方法允许将额外的标签扩展到 `<policy>` 根。

### `<enclave>` Tag

Encapsulates a collection of profiles.

> 封装一组配置文件。

This is specific to an enclave as determined by associative attributes.

> 这是由关联属性确定的特定飞地。

Attributes:

> 属性：

- **path**: Fully qualified enclave path

> - **路径**：完全限定的飞地路径

Given that multiple nodes can be composed into a single process, an enclave is used to contain the collection of profiles of all respective nodes.

> 给定多个节点可以组成一个单独的进程，围栏用来包含所有节点的配置文件集合。

An enclave may therefore be considered the union of contained profiles.

> 一个飞地可以被视为包含的轮廓的联合体。

> [NOTE]:
> E.g. if a profile asks for a permission but a matching permission has been explicitly denied by another profile in the enclave, the deny rule will take precedence.

> 例如，如果一个资料要求获得权限，但另一个资料在飞地中明确拒绝了匹配的权限，拒绝规则将优先考虑。

See section `<profile>` Tag for more info on how MAC is applied.

> 请参考“<profile>”标签来了解更多有关 MAC 的应用信息。

### `<profiles>` Tag

Encapsulates a sequence of unique profiles and designated metadata.

> 封装了一系列唯一的配置文件和指定的元数据。

This method of nesting sequences allows for additional tags to be extended to the `<enclave>` root, as well as associating particular metadata or constraints to the contained profile elements.

> 这种嵌套序列的方法允许将额外的标签扩展到 `<enclave>` 根，以及将特定的元数据或约束关联到包含的配置文件元素。

Attributes:

> 属性：

- **type**: Specifies the transport type of profiles and metadata

> - **类型**：指定配置文件和元数据的传输类型

### `<profile>` Tag

Encapsulates a collection of subject privileges.

> 封装一组主题特权。

This is specific to a unique node instance as determined by associative attributes.

> 这是由关联属性确定的特定于唯一节点实例的。

Attributes:

> 属性：

- **ns**: Namespace of the node

> - **ns**：节点的命名空间

- **node**: Name of the node

> 节点：节点的名称

In accordance with MAC, privileges must be explicitly qualified as allowed.

> 根据 MAC，特权必须明确指定为允许。

Additionally, as with many other MAC languages, while composed privileges may overlap, any particular denied privilege will suppress any similarly applicable allowed privileges.

> 此外，与许多其他 MAC 语言一样，虽然组合的权限可能会重叠，但任何特定的拒绝权限都将抑制任何类似的允许权限。

That is to say the priority of denied privileges conservatively supersedes allowed privileges, avoiding potential lapses in PoLP.

> 那就是说，拒绝权限的优先级保守地超越了允许的权限，避免了可能出现的操作管理策略(PoLP)失误。

This method of flatting privileges enables users to provision general access to a larger set of objects, while simultaneously revoking access to a smaller subset of sensitive objects.

> 这种权限平铺方法使用户能够为更大范围的对象提供通用访问权限，同时撤销对较小范围敏感对象的访问权限。

Although recursion of qualifiers is subsequently prevented, transformations are subsequently simplified, preventing potential for unintended access.

> 尽管限定符的递归被随后阻止，但转换被简化，从而防止潜在的意外访问。

### `<metadata>` Tag

Encapsulates arbitrary metadata or constraints.

> 封装任意元数据或约束。

This could include transport specific permission details applicable to sibling profile elements.

> 这可能包括与兄弟配置文件元素相关的特定交通权限细节。

There can only one `metadata` element per `profiles` parent element.

> 每个 `profiles` 父元素只能有一个 `metadata` 元素。

Attributes:

> 属性：

- To be defined

> 待定

Given the use cases for bridge interfaces where an enclave's credentials may be used to interconnect across multiple transports or to transport specific domains, it may be necessary to qualify certain profile sequences with particular constraints, while doing so multiple times for separate profiles per enclave.

> 在桥接接口的使用场景中，可能需要用某个飞地的凭据来跨多种传输协议或者传输特定的域，因此可能需要对某些配置序列进行特定约束，并且每个飞地可能需要多次进行这样的操作。

This allows advanced users to holistically control the intersect of permissions across transport domains, while retaining accurate model fidelity of security permissions.

> 这允许高级用户在保持安全权限的准确模型保真度的同时，在传输域之间整体控制权限的交叉点。

Given how security sensitive bridge interfaces are and the attack surface they expose, it is vital that information flow control within a bridge remains formally verifiable for safe and secure operation.

> 由于桥接接口的安全敏感性以及它们暴露的攻击面，为了安全操作，桥接器内部的信息流控制必须得到正式的可验证性。

#### Privileges

Privileges are defined as configuration of rules and permissions for object access.

> 特权定义为对象访问的规则和权限的配置。

As objects can be categorized by subsystem type, rules and respective permission are identically structured.

> 物件可以按子系统类型分类，规则及相应的权限也有相同的结构。

Given an average profile is likely to reference more unique objects with multiple permissions than number of rules, the subsequent hierarchy of rules/permissions/objects is chosen to minimize verbosity.

> 给定一个平均的配置文件往往会引用更多具有多种权限的唯一对象，而不是规则的数量，为了最大限度地减少冗长，选择了规则/权限/对象的后续层次结构。

| rule types | permissions        |
| ---------- | ------------------ |
| actions    | call, execute      |
| services   | reply, request     |
| topics     | publish, subscribe |

Each subsystem is associated to a given rule type, while permissions are expressed as attributes in their respective respective rule tags.

> 每个子系统都与特定的规则类型相关联，而权限则表示为各自规则标签中的属性。

Uniqueness or ordering of rules in this sequence is not required, as this is accounted for by transformation templates.

> 在这个序列中，不需要规则的唯一性或排序，因为这已由转换模板账户。

In fact, a profile may contain an empty set of privileges; particularly useful when a node may require no subsystem permissions, but must still be provisioned an identity nonetheless for discovery purposes in DDS.

> 事实上，一个配置文件可能包含一组空的特权；特别有用的是，当一个节点可能不需要任何子系统权限，但仍然必须为 DDS 提供一个身份认证，以便发现目的。

Each rule includes a sequence of objects that the permissions apply.

> 每条规则包含一系列对许可适用的对象序列。

For some secure transports, such as [Secure DDS](https://www.omg.org/spec/DDS-SECURITY/About-DDS-SECURITY), matching expressions may also be used to expand the scope further using globbing patterns, specifically those supported by [fnmatch](https://pubs.opengroup.org/onlinepubs/9699919799/functions/fnmatch.html) as specified in the POSIX standard.

> 对于某些安全传输，比如[安全 DDS](https://www.omg.org/spec/DDS-SECURITY/About-DDS-SECURITY)，也可以使用匹配表达式来进一步扩展范围，使用 [fnmatch](https://pubs.opengroup.org/onlinepubs/9699919799/functions/fnmatch.html) 支持的 globbing 模式，根据 POSIX 标准指定。

However, caution should be taken when using expression matching, as discussed further in the concerns section.

> 然而，在使用表达式匹配时应谨慎，具体内容可参见下文的担忧部分。

Basic fnmatch-style patterns are supported:

> 基本的 fnmatch 风格的模式被支持：

| Pattern     | Meaning                               |
| ----------- | ------------------------------------- |
| \*           | Matches everything                    |
| ?           | Matches any single character          |
| [sequence]  | Matches any character in sequence     |
| [!sequence] | Matches any character not in sequence |

### `<topics>` Tag

A group of `<topic>` tags with the specified permissions.

> 一组具有指定权限的 `<主题>` 标签。

Attributes:

> 属性：

- **publish**: Whether or not publication on this set of topics is allowed

> 是否允许在这组主题上发布

- i.e. whether the node can be a topic publisher

> - 也就是说，节点是否可以成为主题发布者？

- Valid values are "ALLOW" or "DENY"

> 有效值为“允许”或“拒绝”

- **subscribe**: Whether or not subscription on this set of topics is allowed

> 是否允许订阅此一组主题

- i.e. whether the node can be a topic subscriber

> - 也就是说，这个节点是否可以成为主题订阅者？

- Valid values are "ALLOW" or "DENY"

> 有效值为“允许”或“拒绝”

### `<services>` Tag

A group of `<service>` tags with the specified permissions.

> 一组具有指定权限的 < 服务 > 标签。

Attributes:

> 属性：

- **request**: Whether or not requesting the service is allowed

> 是否允许请求服务

- i.e. whether the node can be a service client

> - 也就是说，节点是否可以作为服务客户端？

- Valid values are "ALLOW" or "DENY"

> 有效值为“允许”或“拒绝”

- **reply**: Whether or not replying to service requests is allowed

> 回复服务请求是否允许？

- i.e. whether the node can be a service server

> - 即该节点是否可以作为服务器？

- Valid values are "ALLOW" or "DENY"

> 有效值为“允许”或“拒绝”

### `<actions>` Tag

A group of `<action>` tags with the specified permissions.

> 一组具有指定权限的 `<动作>` 标签。

Attributes:

> 属性：

- **call**: Whether or not calling the action is allowed

> 是否允许打电话

- i.e. whether the node can be an action client

> - 也就是说，节点是否可以成为一个动作客户端？

- Valid values are "ALLOW" or "DENY"

> 有效值为“允许”或“拒绝”

- **execute**: Whether or not executing the action is allowed

> 是否允许执行该操作

- i.e. whether the node can be an action server

> - 也就是说，节点是否可以作为动作服务器？

- Valid values are "ALLOW" or "DENY"

> .

有效值为“允许”或“拒绝”

## Templating

To transpile SROS 2 policies into security artifacts for a targeted access controlled transport, XSLT templates can be used to perform this level of document conversion.

> 可以使用 XSLT 模板将 SROS 2 策略转译为针对限制访问控制传输的安全工件，以执行此级别的文档转换。

This may include any number of optimisations or adjustments specific for the target transport.

> 这可能包括为目标运输特别定制的任何数量的优化或调整。

For example, the pipeline stages for targeting Secure DDS is as follows:

> 例如，用于安全 DDS 的管道阶段如下：

1. An XML document with a root of the SROS 2 policy is specified

> 一个具有 SROS 2 策略根的 XML 文档已被指定。

2. The document is fully expanded using XInclude to reference external elements

> 文档已使用 XInclude 完全展开，以引用外部元素。

2. The expanded document is then validated with the equivalent schema version

> 2. 然后，使用相同版本的架构对扩展的文档进行验证。

2. At this point the document tree may or may not be pruned to a particular profile

> 此时，文档树可能会或可能不会被修剪到特定的配置文件。

2. The valid document is then transpiled using the transform template

> 2. 然后使用转换模板对有效文档进行转译。

2. For each profile, a matching DDS grant is appended into the permission document

> 对于每个配置文件，将一个匹配的 DDS 授权附加到权限文件中。

- privileges and namespaces are remapped into a DDS centric representations

> 权限和名称空间被重新映射成以 DDS 为中心的表示形式。

- privileges with matching attributes are conjoined to reduce payload size

> 权限与匹配的属性被结合在一起，以减少消息体的大小。

- duplicate objects in the same privilege are pruned to reduce payload size

> 重复的对象在同一权限下被剪枝以减少报文大小

- privileges are sorted deny first, abiding the priority of qualifiers when using DDS

> 权限按照拒绝优先的顺序排列，在使用 DDS 时遵循资格证书的优先级。

- objects are also sorted alphabetically to improve readability and change diffs

> .

物件也按字母順序排序以提高可讀性和改變差異。

## Alternatives

This section lists concerns about the proposed design and alternatives that were considered, including different [Markup Languages](https://en.wikipedia.org/wiki/Markup_language) and policy formats.

> 这一节列出了有关拟议设计及其他考虑过的替代方案的担忧，包括不同的标记语言和政策格式。

### YAML

[YAML](https://en.wikipedia.org/wiki/YAML), a recursive acronym for “YAML Ain't Markup Language”, was originally adopted for specifying access control policies in the first version of SROS [1].

> YAML(递归缩写“YAML Ain't Markup Language”)最初被采用用于指定 SROS 第一个版本中的访问控制策略[1]。

Although the policy model used in [SROS 1](http://wiki.ros.org/SROS/Concepts/PolicyDissemination) was semantically equivalent, the YAML format lent it being quite verbose due to repetition of permissions per namespaced resource given the lack of clear element attributes.

> 尽管 [SROS 1](http://wiki.ros.org/SROS/Concepts/PolicyDissemination) 中使用的策略模型在语义上是等价的，但由于缺乏明确的元素属性，YAML 格式使其变得相当冗长，每个命名空间资源重复出现权限。

For SROS 2 we decided switched away from YAML to XML for many of the reasons weighed in the following pros and cons:

> 对于 SROS 2，我们决定放弃 YAML，改用 XML，原因如下列出的优缺点：

- Pros

> 优点

- Human Readable: Minimal Line Noise

  - YAML has very minimal syntax and is targeted for human editable configuration files, making it simple to read and write manually.
- Data Model: Intuitive Interpretation

  - YAML has very simple data model forming tree structure using key-value pair dictionaries and lists making it quite approachable.
- Cons

> 缺点

- Parsability: Implicit Type Casting
  - Given YAML is a data-serialization language, it may attempt to type cast where possible.

However this does not always have the desired effect and may lead to unintended behaviors.

> 然而，这并不总能达到预期的效果，可能会导致意外的行为。

Parsing of booleans v.s. strings are notable example of ambiguity.

> 解析布尔值与字符串之间的明显差异是一个重要的歧义例子。

- Interpreters: Validation and Transformation
  - Although YAML is supported for many programming languages, YAML itself provides no schema to enforce document structure.
  - Validation must be repeated for each interpreter implementation, rendering it non-agnostic to the programing language used.
  - Similarly, transformations for transpiling policies into transport security artifacts is less generalizable across implementations.
- Composability: Reuse of Profiles
  - Although YAML supports a degree of composability via Anchors, Aliases and Extensions, allowing documents to be more DRY, these do not extend to separate files or external resources.
- Expressiveness: Succinct Representation
  - Given YAML’s inherit data model, it’s expressive power is quite limited, necessitating either verbose file structures, or unintuitive options to achieve similar access control configurations.

### Custom

As an alternative to choosing an existing markup format, it would be possible to define our own formal language for expressing access control permissions for ROS 2 using a custom file syntax.

> 作为选择现有标记格式的替代方案，我们可以使用自定义文件语法为 ROS 2 定义自己的正式语言，以表达访问控制权限。

An example of a MAC based policy language would include that which is used in [AppArmor](https://gitlab.com/apparmor/apparmor/wikis/home).

> 一个基于 MAC 的策略语言的示例包括 [AppArmor](https://gitlab.com/apparmor/apparmor/wikis/home) 中使用的语言。

Although affording the flexibility to succinctly express profile permission while minimizing general syntactic overhead, this approach was not pursued for many of the reasons weighed in the following pros and cons:

> 虽然这种方法可以简洁地表达配置文件权限，同时最小化一般句法的开销，但出于以下优缺点的权衡，没有采取这种方法：

- Pros

> 优点

- Expressiveness: Succinct Representation

  - Complete control of syntax and interpretation, allowing for domain specific optimizations for SROS policy representation.
- Cons

> 缺点

- Interpreters: Validation and Transformation
  - Specification and implementation for parsing and interpreting a custom policy format would be considered undertaking.
  - Validation must be repeated for each interpreter implementation, rendering it non-agnostic to the programing language used.
  - Similarly, transformations for transpiling policies into transport security artifacts is less generalizable across implementations.
- Correctness: Policy Remaining Sound
  - Maintaining and synchronizing parsing support across multiple programing language could affect policy soundness.

### ComArmor

[ComArmor](https://github.com/ComArmor/comarmor) [2], a predecessor and inspiration for the existing XML ROS 2 policy format now used in SROS 2, is itself inspired from AppArmor’s policy language.

> [ComArmor](https://github.com/ComArmor/comarmor) [2]是现在 SROS 2 中使用的 XML ROS 2 策略格式的前身和灵感来源，它本身也受到了 AppArmor 策略语言的启发。

ComArmor facilitates composition through the use of a nested tree structure of policy/profile/permission primitives.

> ComArmor 通过使用策略/配置文件/权限原语的嵌套树结构来促进组成。

As with AppArmor, it also supports nesting of profiles, i.e. importation of child profiles into that of a parent profile.

> 正如 AppArmor 一样，它也支持配置文件的嵌套，即将子配置文件导入父配置文件中。

While this greatly extends the flexibility given the nesting of imported sun-profile hierarchies, it also adds complexity to the transpiling process when converting policies to security transport artifacts.

> 这大大增加了导入 sun-profile 层次结构时的灵活性，但在将策略转换为安全传输工件时也增加了转译过程的复杂性。

In an effort to strike a balance between simplicity and flexibility, a flat sequence of single level profiles was opted for SROS 2 instead, allowing the policy format to serve as both a grounded intermediate representation for higher level policy languages and tools to build upon, such as [XACML](https://en.wikipedia.org/wiki/XACML) or [Keymint](https://github.com/keymint/keymint_tools), while remaining succinctly expressive of ROS concepts for general use.

> 为了在简单性和灵活性之间取得平衡，SROS 2 选择了一个单层次的平面序列来提供策略格式，以作为更高级别的策略语言和工具的基础表示，例如 [XACML](https://en.wikipedia.org/wiki/XACML) 或 [Keymint](https://github.com/keymint/keymint_tools)，同时又保持简洁表达 ROS 概念，以便普遍使用。

## Concerns

### Separation of Privileges

ROS 2 subsystems such as topics, services, actions, and parameters must eventually map to transport layer interfaces, such as DDS topic, that can sufficiently enforce the desired access control policy in order to secure the ROS application layer.

> ROS 2 的子系统，如主题、服务、动作和参数，最终必须映射到传输层接口，如 DDS 主题，以便足够地执行所需的访问控制策略，以确保 ROS 应用层的安全性。

However, any quirks between mapping of subsystems and separation of privileges can degrade security.

> 然而，子系统映射和权限分离之间的任何怪癖都会降低安全性。

As an example, if granting access to all topics and services starting with `/foo` additionally grants access to all actions starting with `/foo`, this would be a weak example of privilege separation.

> 例如，如果授予所有以“/foo”开头的主题和服务的访问权限，同时又授予以“/foo”开头的所有操作的访问权限，这将是权限分离的一个较弱的例子。

Such can be exacerbated when using globbing expressions that include matching patterns, such as with `fnmatch`, leading to innocuous and sound policies being inaccurately applied to the underlying transport security.

> 当使用包含匹配模式的通配符表达式(如 `fnmatch`)时，这种情况可能会加剧，导致无害而又合理的策略不准确地应用于底层传输安全性。

While such privilege separation in [remains week between ROS 2 and DDS](https://github.com/ros2/design/pull/203), perhaps it is wise to discourage the use of expression matching for general use in permissions.

> 在 ROS 2 和 DDS 之间的特权分离仍然很弱，也许不建议一般情况下使用表达式匹配来控制权限。

### Separation of Concerns

Middleware transports, such as DDS, offer a myriad of features and options, such as those for QoS as well as security.

> 中间件传输，如 DDS，提供了多种功能和选项，如 QoS 以及安全性。

Drawing a boundary between many of them when deciding what to expose from a configuration standpoint can be tricky.

> 在决定从配置角度暴露什么时，在它们之间划定一个界限可能会很棘手。

Still, among the objective for ROS 2 includes remaining as agnostic of transport as reasonable.

> ROS 2 仍然将尽可能地保持对传输的中立性作为其目标之一。

Although the SROS 2 policy format was intentionally structured to mimic that of Secure DDS’s permission.xml format, care should be taken when adding extensions to surface non-functional security properties tangential to ROS 2 data flow, such as governance on encryption or discovery.

> 虽然 SROS 2 策略格式有意结构化为模仿 Secure DDS 的 permission.xml 格式，但在为 ROS 2 数据流表面添加非功能性安全属性(如加密或发现的治理)时应当小心。

Yet, if the intended purpose of SROS 2 policy becomes that of an intermediate representation across transports, and is subsequently auto generated from higher level tooling/representations, or composability is adequate preserved, then perhaps this concern is of lesser priority.

> 然而，如果 SROS 2 策略的预期目的成为跨转换的中间表示，并且随后从更高级别的工具/表示中自动生成，或者可组合性得到充分保留，那么这个问题可能不是那么重要了。

### Composability

ROS 2 allows for the remapping of many namespaced subsystems at runtime, such as when reusing launch files to orchestrate larger applications.

> ROS 2 允许在运行时重新映射许多命名空间的子系统，例如在重用启动文件来编排更大的应用程序时。

While it is perhaps unreasonable to expect this dynamic flexibility from staticky provisioned permissions without allocating such capabilities prior, it should be made possible to infer the necessary capabilities from composed launch files and similar orchestrations.

> 尽管没有预先分配这种能力，期望从静态提供的权限中获得这种动态灵活性或许是不合理的，但应该从组合启动文件和类似的编排中推断出必要的能力。

Static analysis of such remapping in conjunction with the setting of the nominal requirements of respective nodes could be used to auto generate the new satisfactory policies.

> 静态分析这种重映射结合各个节点的名义要求的设置可以用来自动生成新的满意的政策。

However, inferring such policies from the source code could be equated to the halting problem.

> 然而，从源代码推断出这些政策可以等同于停机问题。

Thus, it stands to reason nodes could instead provide a manifest or IDL defining these nominal requirements so that permission may as easily be remapped, at least at design time.

> 因此，有理由认为节点可以提供一个清单或 IDL 来定义这些名义要求，以便至少在设计时可以轻松重新映射权限。

## References

1. [SROS1: Using and Developing Secure ROS1 Systems](https://doi.org/10.1007/978-3-319-91590-6_11)

> 1. [SROS1：使用和开发安全的 ROS1 系统](https://doi.org/10.1007/978-3-319-91590-6_11)

```bibtex
@inbook{White2019,
  title     = ,
  author    = ,
  year      = 2019,
  booktitle = ,
  doi       = ,
  isbn      = ,
  url       = 
```

2. [Procedurally Provisioned Access Control for Robotic Systems](https://doi.org/10.1109/IROS.2018.8594462)

> 2. [机器人系统的程序性提供的访问控制](https://doi.org/10.1109/IROS.2018.8594462)

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
