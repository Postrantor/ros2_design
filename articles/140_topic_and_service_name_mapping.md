---
tip: translate by openai@2023-07-19 11:58:52
src: [](D:\Document\Hirain\Project\rolling\ros2\rcl\rcl\src\rcl\remap.c)
...

The special single character token `~` will be replaced with a namespace snippet that is a concatenation of the namespace for the node and the node name. For example, a node `node1` in a namespace `/foo` would result in `~` being replaced with `/foo/node1`. As another example, a node `node1` in a namespace `foo/bar` would result in `~` being replaced with `foo/bar/node1`. It must be used at the beginning of a non-fully qualified name, if at all. Here is a table with some example expansions:

> 特殊的单字符令牌`~`将被替换为命名空间片段，这是节点的命名空间和节点名称的连接。例如，在命名空间 `/foo` 中的节点 node1 将导致'~'被替换为/foo/node1。另一个例子是，在命名空间 foo/bar 中的节点 node1 将导致'~'被替换为 foo/bar/node1。它必须在非完全限定名称的开头使用，如果有的话。以下是一张带有一些示例扩展的表：

| **Input Name** | Node: `my_node` NS: none | Node: `my_node` NS: `/my_ns` |
| -------------- | ------------------------ | ---------------------------- |
| `ping`         | _`/ping`_                | _`/my_ns/ping`_              |
| `/ping`        | _`/ping`_                | _`/ping`_                    |
| `~`            | _`/my_node`_             | _`/my_ns/my_node`_           |
| `~/ping`       | _`/my_node/ping`_        | _`/my_ns/my_node/ping`_      |

### Substitutions

The bracket syntax (`{substitution_name}`) may be used in non-fully qualified names to substitute useful contextual information into the name. The set of substitution keys (names) are not set in this document, but some reasonable examples might be: `{node}` expands to the current node's name or `{ns}` expands to the current node's namespace.

> 括号语法(`{substitution_name}`)可以用于非完全限定的名称中，以将有用的上下文信息替换到该名称中。替换键(名称)的集合未在本文档中设定，但一些合理的示例可能是：`{node}`展开为当前节点的名称或`{ns}`展开为当前节点的命名空间。

Substitutions are expanded after the private namespace substitution character is expanded. Therefore a substitution may not contain the private namespace substitution character, i.e. `~`. For example, given the name `{private}foo` and a substitution called `{private}` which expands to `~/_`, you will get an error because the `~/_` will end up in the expanded name as `/my_ns/~/_foo` which is is not allowed to have a `~` in it.

> 替换符号在私有命名空间替换符号被展开后才会被展开。因此，替换符号不能包含私有命名空间替换符号，即`~`。例如，给定名称`{private}foo`和一个名为`{private}`的替换，它展开为`~/_`，您将会得到一个错误，因为`~/_`最终会以`/my_ns/~/\_foo`的形式出现在展开后的名称中，而不允许在其中出现`~`。

Substitutions are expanded in a single pass, so substitutions should not expand to contain substitutions themselves. For example, given the name `/foo/{bar_baz}` where `{bar_baz}` expands to `{bar}/baz` and where `{bar}` in turn expands to `bar`, you will get `/foo/{bar}/baz` as the final result, which is invalid, and not `/foo/bar/baz` as you might expect.

> 替换是在一次通过中展开的，因此替换本身不应该展开为包含替换的内容。例如，给定名称`/foo/{bar_baz}`，其中`{bar_baz}`展开为`{bar}/baz`，而`{bar}`又展开为`bar`，最终结果是`/foo/{bar}/baz`，这是无效的，而不是你可能期望的`/foo/bar/baz`。

Substitutions are also not allowed to be nested, i.e. substitutions may not contain other substitutions in their names. This is implicitly enforced by the rules above that say substitution names may only contain alphanumerics and underscores (`_`). For example, given the name `{% raw %}/foo/{{bar}_baz}{% endraw %}` would result in an error because `{` and `}` are not allowed in a substitution names and the substitution name `{bar}_baz` does contain them.

> 不允许嵌套替换，即替换名称不能包含其他替换。这可以通过上面的规则隐式地强制执行，即替换名称只能包含字母数字和下划线(`_`)。例如，给定名称`/foo/{{bar}_baz}`将导致错误，因为替换名称中不允许使用`{`和`}`，而替换名称`{bar}_baz`确实包含它们。

### Hidden Topic or Service Names

Any topic or service name that contains any tokens (either namespaces or a topic or service name) that start with an underscore (`_`) will be considered hidden and tools may not show them unless explicitly asked.

> 任何包含以下划线(`_`)开头的令牌(命名空间或 Topic 或服务名称)的 Topic 或服务名称，将被视为隐藏，除非明确要求，否则工具可能不会显示它们。

## Mapping of ROS 2 Topic and Service Names to DDS Concepts

The ROS topic and service name constraints allow more types of characters than the DDS topic names because ROS additionally allows the forward slash (`/`), the tilde (`~`), and the balanced curly braces (`{}`). These must be substituted or otherwise removed during the process of converting the topic or service name to DDS concepts. Since ROS 2 topic and service names are expanded to fully qualified names, any balanced bracket (`{}`) substitutions and tildes (`~`) will have been expanded. Additionally any URL related syntax, e.g. the `rostopic://` prefix, will be removed once parsed. Previously forward slashes (`/`) were disallowed in DDS topic names, now the restriction has been lifted (see [issue](https://issues.omg.org/issues/lists/dds-rtf5#issue-42236) on omg.org) and therefore the ROS topic names are first prefixed with ROS Specific Namespace prefix (described below) are then mapped directly into DDS topic names.

> ROS Topic 和服务名称约束允许比 DDS Topic 名称更多类型的字符，因为 ROS 还允许正斜杠(`/`)、波浪号(`~`)和平衡的花括号(`{}`)。在将 Topic 或服务名称转换为 DDS 概念的过程中，必须替换或删除这些字符。由于 ROS 2 Topic 和服务名称已扩展为完全限定的名称，因此任何平衡括号(`{}`)替换和波浪号(`~`)都将被展开。此外，一旦解析，将删除与 URL 相关的语法，例如`rostopic://`前缀。以前，DDS Topic 名称中不允许使用正斜杠(`/`)，现在已解除了此限制(请参见 omg.org 上的[issue])，因此 ROS Topic 名称首先使用 ROS 特定的命名空间前缀(如下所述)，然后直接映射到 DDS Topic 名称中。

### ROS Specific Namespace Prefix

In order to differentiate ROS topics easily, all DDS topics created by ROS shall be automatically prefixed with a namespace like `/rX`, where `X` is a single character that indicates to which subsystem of ROS the topic belongs. For example, a plain topic called `/foo` would translate to a DDS topic `rt/foo`, which is the result of implicitly adding `rt` to the namespace of a ROS topic which is in the root namespace `/` and has a base name `foo`. As another example, a topic called `/left/image_raw` would translate to a DDS topic `rt/left/image_raw`, which is the result of implicitly adding `rt` to the namespace of a ROS topic which is in the namespace `/left` and has a base name `image_raw`.

> **为了更容易区分 ROS Topic，ROS 创建的所有 DDS Topic 都将自动加上一个名称空间，如`/rX`**，其中`X`是指示属于 ROS 的哪个子系统的单个字符。例如，一个简单的 Topic`/foo`会转换为 DDS Topic`rt/foo`，这是隐式地将`rt`添加到处于根名称空间`/`且基本名称为`foo`的 ROS Topic 的名称空间中的结果。另一个例子，一个叫做`/left/image_raw`的 Topic 会转换为 DDS Topic`rt/left/image_raw`，这是隐式地将`rt`添加到处于名称空间`/left`且基本名称为`image_raw`的 ROS Topic 的名称空间中的结果。

For systems where Services are implemented with topics (like with OpenSplice), a different subsystem character can be used: `rq` for the request topic and `rr` for the response topic. On systems where Services are handled explicitly implemented, we consider a separate prefix, e.g. `rs`.

> 对于使用 Topic 实现服务(如 OpenSplice)的系统，可以使用不同的子系统字符：`rq`用于请求 Topic，`rr`用于响应 Topic。对于显式实现服务的系统，我们考虑使用单独的前缀，例如`rs`。

Here is a non-exhaustive list of prefixes:

| ROS Subsystem        | Prefix |
| -------------------- | ------ |
| ROS Topics           | rt     |
| ROS Service Request  | rq     |
| ROS Service Response | rr     |
| ROS Service          | rs     |
| ROS Parameter        | rp     |
| ROS Action           | ra     |

While all planned prefixes consist of two characters, i.e. `rX`, anything proceeding the first namespace separator, i.e. `/`, can be considered part of the prefix. The standard reserves the right to use up to 8 characters for the prefix in case additional prefix space is needed in the future.

> 所有计划的前缀都由两个字符组成，即`rX`，任何第一个命名空间分隔符`/`之后的内容都可以被认为是前缀的一部分。标准保留了在未来需要更多前缀空间时使用最多 8 个字符的前缀的权利。

### Examples of ROS Names to DDS Concepts

Here are some examples of how a fully qualified ROS name would be broken down into DDS concepts:

> 以下是如何将完全合格的 ROS 名称分解为 DDS 概念的一些示例：

| ROS Name                        | DDS Topic                         |
| ------------------------------- | --------------------------------- |
| `/foo`                          | `rt/foo`                          |
| `rostopic:///foo/bar`           | `rt/foo/bar`                      |
| `/robot1/camera_left/image_raw` | `rt/robot1/camera_left/image_raw` |

### ROS Topic and Service Name Length Limit

The length of the DDS topic must not exceed 256 characters. Therefore the length of a ROS Topic, including the namespace hierarchy, the base name of the topic and any ros specific prefixes must not exceed 256 characters since this is mapped directly as DDS topic.

> DDS Topic 的长度不得超过 256 个字符。因此，**ROS Topic 的长度**，包括命名空间层次结构、Topic 的基本名称和任何 ros 特定的前缀，不得超过 256 个字符，因为它们**直接映射为 DDS Topic**。

#### Considerations for RTI Connext

While testing our implementation with Connext, we encountered some additional limitations when it comes to the topic length. That is, for example the length of a service name has tighter limits than the length of the ROS Topics. The RTI Connext implementation for service names are suffixed with the GUID value of the DDS participant for the service response topic. Additionally, a content filtered topic (max length 256 characters) is created which is mapped from the suffixed service name. Therefore when linking against `rmw_connext_c` or `rmw_connext_cpp`, service names cannot be longer than 185 characters including the namespace hierarchy and any ros specific prefixes.

> 在使用 Connext 进行测试时，我们在 Topic 长度方面遇到了一些额外的限制。也就是说，例如服务名的长度比 ROS Topic 的长度更短。RTI Connext 实现的服务名会被 DDS 参与者的 GUID 值作为后缀。此外，还会创建一个内容过滤 Topic(最大长度为 256 个字符)，该 Topic 从带有后缀的服务名映射而来。因此，当链接到`rmw_connext_c`或`rmw_connext_cpp`时，服务名的长度不能超过包括命名空间层次结构和任何 ros 特定前缀在内的 185 个字符。

### Communicating with Non-ROS Topics

Since all ROS topics are prefixed when being converted to DDS topic names, it makes it impossible to subscribe to existing DDS topics which do not follow the same naming pattern. For example, if an existing DDS program is publishing on the `image` topic (and is using the DDS equivalent to the ROS message type) then a ROS program could not subscribe to it because of the name mangling produced by the implicit ROS specific namespace. Therefore to allow ROS programs to interoperate with "native" DDS topic names the API should provide a way to skip the ROS specific prefixing.

> 由于**所有 ROS Topic 在转换为 DDS Topic 名称时都会加上前缀**，因此无法订阅不遵循相同命名模式的现有 DDS Topic。例如，如果现有的 DDS 程序在`image`Topic 上发布(并使用 DDS 等效于 ROS 消息类型)，则 ROS 程序无法订阅它，因为由隐式 ROS 特定命名空间产生的名称混乱。因此，为了允许 ROS 程序与`原生`DDS Topic 名称进行互操作，API 应提供一种跳过 ROS 特定前缀的方法。

There is an option in the API, a boolean `avoid_ros_namespace_convention` in the qos_profile which can be set to `false` to use ROS prefix and `true` to not using ROS namespace prefixing.

> API 中有一个选项，qos_profile 中的布尔类型`avoid_ros_namespace_convention`，可以将其设置为`false`以使用 ROS 前缀，设置为`true`则不使用 ROS 命名空间前缀。

For example:

| ROS Name           | avoid_ros_namespace_conventions | DDS Topic  |
| ------------------ | ------------------------------- | ---------- |
| `rostopic://image` | `false`                         | `rt/image` |
| `rostopic://image` | `true`                          | `image`    |

#### Alternative(Idea)

## Compare and Contrast with ROS 1

In order to support a mapping to the - slightly more - restrictive DDS topic name rules, these rules are in some ways more constraining than the rules for ROS 1. Other changes have been proposed for convenience or to remove a point of confusion that existed in ROS 1. In ROS 2, topic and service names differ from ROS 1 in that they:

> 为了支持映射到更严格的 DDS Topic 名称规则，这些规则在某些方面比 ROS 1 的规则更加严格。为了方便起见，或者消除 ROS 1 中存在的混乱点，还提出了其他变化。在 ROS 2 中，Topic 和服务名称与 ROS 1 不同，它们：

- must separate the tilde (`~`) from the rest of the name with a forward slash (`/`)

  - This is done to avoid inconsistency with how `~foo` works in filesystem paths versus when used in a ROS name.

> 这是为了避免文件系统路径中的 `~foo` 与 ROS 名称中使用的 `~foo` 不一致。

- may contain substitutions which are delimited with balanced curly braces (`{}`)

  - This is a more generic extension of the idea behind the tilde (`~`).

- have length limits

  - This is driven by the topic name length limit of DDS.

- may be indicated as "hidden" by using a leading underscore (`_`) in one of the namespaces

  - This is used to hide common but infrequently introspected topics and services.

## Concerns/Alternatives

This section lists concerns about the proposed design and alternatives that were considered.

> 这一节列出了关于拟议设计的担忧以及考虑过的替代方案。

### Alternative Name Rules and Concerns About Name Rules

There were some suggested but then rejected alternatives to the rules for topic and service names.

> 有一些建议，但是被拒绝的替代规则，用于 Topic 和服务名称。

#### More Versatile Private Namespace Substitution Character

Currently the `~` private namespace substitution character may only be used at the beginning of the name, but it was also suggested that it could be placed anywhere within the name and could be substituted in place.

> 目前，`~`私有命名空间替换字符只能用于名称开头，但也建议可以放在名称的任何位置，并可以进行替换。

This was rejected because it was complicated to explain and did not behave the same as the `~` when used in filesystem paths on Unix machines. Also, it was difficult to justify its existence because all suggested use cases were quite contrived. It also behaved differently from how it worked in ROS 1, which was yet another negative for this alternative.

> 这被拒绝了，因为它很难解释，而且在 Unix 机器上的文件系统路径中使用`~`时行为不同。此外，由于所有建议的用例都非常牵强，很难证明它的存在。它的行为也与 ROS 1 中的行为不同，这又是这种替代方案的另一个负面。

#### Alternative Substitution Syntax

There were some alternative syntaxes proposed for substitutions in the names before the plain balanced curly braces syntax (`{}`) was selected:

> 在选择平衡大括号语法(`{}`)之前，曾提出过一些替换的替代语法：

- `%{sub}`
- `${sub}`
- `$sub`

The most serious alternatives considered were the "bash-like" syntax of `${sub}` and `$sub`. The `$sub` style syntax has the downside of being difficult to process and making it impossible to express some kinds of concatenation. The `${sub}` was a strong candidate, but ultimately was rejected because it would collide with use in shell scripts. For example, you can imagine a shell script that runs a node and remaps a topic name would contain these substitutions, but they would need to be escaped to prevent bash itself from trying to expand them. The `{}` syntax avoids this problem but is also easy to parse.

> 最被考虑的严重的替代方案是`类似 bash`的语法`${sub}`和`$sub`。`$sub`样式的语法有一个缺点，就是难以处理，并且使得一些种类的连接不可能表达出来。`${sub}`是一个很强的候选，但最终被拒绝了，因为它会与在 shell 脚本中的使用发生冲突。例如，你可以想象一个运行节点并重新映射 Topic 名称的 shell 脚本会包含这些替换，但它们需要被转义以防止 bash 本身试图展开它们。`{}`语法避免了这个问题，而且也很容易解析。

The `{}` syntax will collide with Python String substitution, but since that is an explicit action (unlike shell substitution which will implicitly always occur) it's less of an issue.

> `{}` 语法会与 Python 字符串替换发生冲突，但由于这是一个显式的操作(不像 shell 替换会自动发生)，所以问题不大。

#### Concerns About Substitutions

This document does not prescribe what substitutions should be supported by implementations. This was done to avoid blocking progress on this document on the need to agree on the set of required substitutions. However, this is just delaying the issue. For the substitutions to be useful, then all implementations that process topic and service names need to support them.

> 这份文件并未规定实施者应该支持哪些替换。这是为了避免因就所需的必要替换集合需要达成一致而阻碍本文件的进展。然而，这只是推迟了这个问题。为了使替换有用，处理 Topic 和服务名称的所有实施者都需要支持它们。

This compromise was made so that when more work is done on substitutions, it would hopefully not require changes to the name syntax, but instead revolve around what substitutions to support and whether or not that support is optional.

> 这一妥协是为了当更多的工作被做在替换上时，希望不需要改变名称语法，而是围绕什么替换来支持以及是否支持是可选的。

### Alternative ROS to DDS Mappings

#### Alternative using DDS Partitions

Previously the usage of forward slashes (`/`) was disallowed in DDS topic name and hence a strategy was proposed which used DDS partitions to address the forward slashes (`/`) which are present in ROS names. The main idea was to separate the ROS name into the "namespace" and the "base name", and then place the namespace, stripped of leading and trailing forward slashes (`/`), into a single DDS partition entry and the remaining base name into the DDS topic name. This addressed the issue because the ROS name's base name will not contain any forward slashes (`/`) by definition and so there are no longer any disallowed characters in the DDS topic name. The DDS partition would contain the ROS name's namespace, including any forward slashes (`/`) that made up the namespace and were not at the beginning or the end of the namespace. That is acceptable because DDS partitions are allowed to contain forward slashes (`/`) unlike the DDS topics previously but now DDS topic names allow forward slashes (`/`).

> 以前，DDS 主题名称中不允许使用前向斜线（`/`）的使用，因此提出了一种策略，该策略使用 DDS 分区来解决 ROS 名称中存在的前向斜线（`/`）。主要的想法是将 ROS 名称分离为“名称空间”和“基本名称”，然后将命名空间剥离，剥夺了前进和前进的斜线（`/`），将单个 DDS 分区条目和其余的基础放置名称为 DDS 主题名称。这解决了问题，因为 ROS 名称的基本名称将不包含任何正向斜线（`/`），因此 DDS 主题名称中不再有任何不允许字符。DDS 分区将包含 ROS 名称的名称空间，包括组成名称空间的任何前向斜线（`/`），并且不在名称空间的开始或结束时。这是可以接受的，因为与之前的 DDS 主题不同，允许 DDS 分区包含前向斜线（`/`），但现在 DDS 主题名称允许前向斜线（`/`）。

DDS partitions are implemented as an array of strings within the `DDS::Publisher` and `DDS::Subscriber` QoS settings and have no hierarchy or order, Each entry in the partition array is directly combined with the DDS topic and they are not sequentially combined. If a publisher has two partition entries, e.g. `foo` and `bar` with a base name of `baz`, this would be equivalent to having two different publishers on these topics: `/foo/baz` and `/bar/baz`. Therefore this proposal used only one of the strings in the partitions array to hold the entire ROS name's namespace.

> DDS 分区用作`DDS::Publisher`和`DDS::Subscriber`QoS 设置中的字符串数组实现，没有层次结构或顺序。分区数组中的每个条目直接与 DDS Topic 结合，它们不是顺序结合的。如果发布者有两个分区条目，例如`foo`和`bar`，基本名称为`baz`，这相当于在这些 Topic 上有两个不同的发布者：`/foo/baz`和`/bar/baz`。因此，本提案仅使用分区数组中的一个字符串来保存整个 ROS 名称的命名空间。

You can read more about partitions in RTI's documentation:

- [PARTITION_QosPolicy](https://community.rti.com/static/documentation/connext-dds/5.2.3/doc/manuals/connext_dds/html_files/RTI_ConnextDDS_CoreLibraries_UsersManual/Content/UsersManual/PARTITION_QosPolicy.htm)

Trade-offs (in comparison to using the whole ROS name along with the namespaces):

> 权衡(与使用整个 ROS 名称以及命名空间相比)：

- Splitting the ROS name into "namespace" and "base name", and placing the complete namespace into a field designed for another purpose seemed incorrect.

> 将 ROS 名称拆分为`命名空间`和`基本名称`，并将完整的命名空间放入专门用于其他目的的字段中似乎是不正确的。

- In general partitions are recommended to be used as a spare, but using partitions for all ROS names suggested otherwise.

> 一般来说，建议将分区用作备份，但使用分区来存储所有 ROS 名称的建议却有所不同。

- Major concern was reported in this [issue](https://github.com/ros2/rmw_connext/issues/234), where having two topics with same base name, although different namespace and different types caused problem. For example: topicA is `/camera/data` of type `Image` and topicB is `/imu/data` of type `Imu`. The base names for both topicA and topicB is `data`, generated errors as described in the [issue](https://github.com/ros2/rmw_connext/issues/234).

> 在这个问题中报告了一个主要的关注点，其中具有相同基本名称但不同命名空间和不同类型的两个 Topic 会导致问题。例如：topicA 是`/camera/data`类型为`Image`，topicB 是`/imu/data`类型为`Imu`。topicA 和 topicB 的基本名称都是`data`，如问题中所述，会产生错误。

- Newer standards such as [DDS-XRCE](https://www.omg.org/spec/DDS-XRCE) might not have partitions at all.

> 新一代的标准，如[DDS-XRCE](https://www.omg.org/spec/DDS-XRCE)可能根本不需要分区。

- Using the complete ROS name in the later strategy will cause a tighter length limit on base name because the DDS topic name would contain ROS prefix, namespace along with the base name which should not exceed DDS topic name limitation which is 256 characters.
  > - 使用后续策略中的完整 ROS 名称会导致基本名称的长度限制更紧，因为 DDS Topic 名称将包含 ROS 前缀、命名空间以及基本名称，它们的总长度不应超过 DDS Topic 名称的限制，即 256 个字符。

Rationale:

- With the decision from the DDS vendors to allow forward slashes (`/`) in DDS topic names, using the complete ROS name seemed simple and more intuitive than using partitions.

> 随着 DDS 供应商允许在 DDS Topic 名称中使用斜杠(`/`)，使用完整的 ROS 名称似乎比使用分区更简单、更直观。

#### Alternative Substitute the Namespace Delimiter

A previous proposal was to substitute the namespace delimiter, i.e. forward slash (`/`), with something that is allowed in DDS topic names, and then only use the DDS topic name to represent the full ROS name. For example in the simplest case, a topic `/foo/bar/baz` might become `__foo__bar__baz`, where the forward slash (`/`) is being replaced with a double underscore (`__`) and double underscores (`__`) were not allowed in ROS topic and service names.

> 之前的提议是用 DDS Topic 名称允许的东西替换命名空间分隔符，即前导斜杠(`/`)，然后只使用 DDS Topic 名称来表示完整的 ROS 名称。例如，在最简单的情况下，Topic`/foo/bar/baz`可能会变成`__foo__bar__baz`，其中前导斜杠(`/`)被双下划线(`__`)替换，而 ROS Topic 和服务名称中不允许使用双下划线(`__`)。

Trade-offs (in comparison to the use of DDS partitions):

- Has a tighter length limit, since it limited by just the DDS topic name and does not benefit from part of the ROS topic name going into the DDS partition.

> 它有一个更紧的长度限制，因为它受到 DDS Topic 名称的限制，而不能从 ROS Topic 名称的一部分进入 DDS 分区中受益。

- The replacement for the forward slash (`/`) had to be more than one character since all usable characters were already in allowed in both the ROS names and DDS topic names, so each namespace further reduced the topic length limit.

> 由于 ROS 名称和 DDS Topic 名称中都已经允许使用所有可用字符，因此替换斜杠('/')必须是多个字符，因此每个命名空间进一步减少了 Topic 长度限制。

- Implementation requires string replacement and validation afterwards, which is moderately complicated.

> 实施需要字符串替换和之后的验证，这是相当复杂的。

Rationale:

The DDS partition proposal is preferred over this alternative because it allows for longer total ROS names and because it is simpler to implement, i.e. splitting the string into base name and namespace is simpler than replacing the forward slashes (`/`) with double underscores (`__`) and then redoing length limit testing afterwards.

> DDS 分区建议优于此替代方案，因为它允许更长的总 ROS 名称，并且实施起来更简单，即将字符串拆分为基本名称和命名空间比替换斜线(`/`)为双下划线(`__`)，然后重新进行长度限制测试要简单。

#### Capital Letter Substitution Alternative

This is another alternative that was proposed in the context of the alternative described in the above section called "Alternative Substitute the Namespace Delimiter". Since the forward slash (`/`) is replaced with double underscores (`__`) when translating to DDS from ROS topic names, two characters out of the 256 character limit are lost with each additional namespace. One proposed alternative was to add a constraint that ROS topic names could not use capital letters, and then capital letters could be used as the stand in for the forward slashes (`/`).

> 这是另一种替代方案，它是在上一节中所描述的`替换命名空间分隔符`的情况下提出的。由于将斜杠(`/`)替换为双下划线(`__`)以将 ROS Topic 名称转换为 DDS，每添加一个命名空间就会损失 256 个字符中的两个字符。提出的一种替代方案是，ROS Topic 名称不能使用大写字母，然后大写字母可以用作斜杠(`/`)的替代。

Trade-offs:

- Uses one fewer character per namespace and makes it easier to calculate the maximum length of a ROS topic or service name.
- Prevents users from using capital letters in their names, which is constraining and might be a problem for backwards compatibility with ROS 1 topics and services.

> - 使用每个命名空间少一个字符，可以更容易计算 ROS Topic 或服务名称的最大长度。
> - 阻止用户在其名称中使用大写字母，这是限制性的，可能会对 ROS 1 Topic 和服务的向后兼容性造成问题。

Rationale:

Preventing users from using capital letters was too constraining for the added benefit.

> 阻止用户使用大写字母的好处太有限了。

#### ROS Prefix with Single Underscore

This is another variation that was proposed in the context of the alternative described in the above section called "Alternative Substitute the Namespace Delimiter". This alternative differs only in that it uses a single underscore in the prefix, i.e. `rt_` rather than `rt__` (`rt` + the leading `/`).

> 这是在上一节中所描述的`替换命名空间分隔符的替代方案`的背景下提出的另一种变体。这种替代方案仅在前缀中使用单下划线，即 rt\_而不是 rt\_\_(rt + 前导斜线/)不同。

Trade-offs:

- Uses one fewer character
- Less consistent with replacement of other forward slashes (`/`)

Rationale:

Slight preference given to the more consistent alternative.

#### Limited Prefixes Alternative

This is another variation that was proposed in the context of the alternative described in the above section called "Alternative Substitute the Namespace Delimiter".

> 这是在上一节中所描述的`替换命名空间分隔符`的另一种变体。

This alternative would:

- not prefix topics
- optionally prefix other kinds of "implementation detail topics"

Trade-offs:

- it would be easier to have ROS subscribe to a DDS created topic

  - e.g. DDS publisher on topic `image` could be subscribed to in ROS using just `image`
  - however, the types would need to match as well

  - in the current proposal the ROS topic `image` would become `rt__image`, so DDS topics would need to follow the ROS topic conversion scheme to interoperate with ROS components

> 在当前提案中，ROS Topic`image`将变成`rt__image`，因此 DDS Topic 需要遵循 ROS Topic 转换方案，以与 ROS 组件进行互操作。

- it would be hard to distinguish ROS created DDS topics and normal DDS topics
- services would still need to be differentiated

  - e.g. service `/foo` would need to make two topics, something like `foo_Request` and `foo_Reply`

> 例如，服务`/foo`需要创建两个 Topic，比如`foo_Request`和`foo_Reply`

Rationale:

Slight preference was given to easily categorizing ROS created topics with DDS created topics over easily connecting to existing DDS topics. Connecting to DDS topics could be achieved by having an option when subscribing or publishing to "alias" to an implementation topic name, e.g. something like `sub->alias_to_explicit_topic('dds_topic')`. Also, the workaround for separating ROS created topics from other DDS topics was considered to be more complicated than the suggested solution of allowing users to specify specific DDS topic names for their publishers and subscriptions.

> 有轻微的偏好，将 ROS 创建的 Topic 与 DDS 创建的 Topic 进行简单分类，而不是简单地连接到现有的 DDS Topic。可以通过在订阅或发布时提供一个选项来实现连接到 DDS Topic，例如`sub->alias_to_explicit_topic('dds_topic')`。此外，将 ROS 创建的 Topic 与其他 DDS Topic 分离的工作绕道被认为比建议的解决方案更复杂，即允许用户为其发布者和订阅者指定特定的 DDS Topic 名称。

#### Suffix Alternative

This is another variation that was proposed in the context of the alternative described in the above section called "Alternative Substitute the Namespace Delimiter".

> 这是在上面一节中称为`替换命名空间分隔符的替代方案`的背景下提出的另一种变体。

This alternative would:

- not prefix topics

  - this would be unique to user defined names because they cannot have a trailing underscore (`_`)

> 这对于用户定义的名称是独一无二的，因为它们不能以下划线(`_`)结尾。

Trade-offs:

- more difficult to distinguish ROS created DDS topics from normal or built-in DDS topics when listing them using DDS tools because they are not sorted by a ROS specific prefix
- if the service name is suffixed again by the DDS vendor (like in Connext's implementation of Request-Reply) then it would be potentially difficult to differentiate from a user's topic name
- e.g. service `/foo` might become topics: `foo_s_Request` and `foo_s_Reply` and the user could create a topic called `/foo_s_Request` too.
- this also applies to any other similar transformations that an RMW implementation might make to the topic

> 查看 DDS 工具中的 Topic 时，由于没有 ROS 特定的前缀，因此很难区分 ROS 创建的 DDS Topic 和普通或内置的 DDS Topic。
> 如果服务名称被 DDS 供应商(如 Connext 的 Request-Reply 实现)再次后缀，那么很可能很难从用户的 Topic 名称中区分开来。
> 例如，服务`/foo`可能会变成 Topic：`foo_s_Request`和`foo_s_Reply`，用户也可以创建一个名为`/foo_s_Request`的 Topic。
> 这也适用于 RMW 实施中可能对 Topic 进行的任何其他类似变换。

Rationale:

This alternative was not selected over the prefix solution because of a lack of advantages over the prefix solution. Also, it typically took one more character to express (`rt` versus `_rt_`; unless you also drop the implicit first namespace `/` then it's `rt__` versus `_rt_`) and the potential issues with ambiguity when the DDS implementation handles Request-Reply (added suffixes).

> 这个替代方案没有被选择，因为它比前缀解决方案没有优势。此外，它通常需要一个更多的字符来表达(`rt`与`_rt_`；除非你也把隐式的第一个命名空间`/`去掉，那么它就是`rt__`与`_rt_`)以及 DDS 实现 Request-Reply 时可能出现的模糊性问题(添加后缀)。

#### Limited Suffix Alternative

This is another variation that was proposed in the context of the alternative described in the above section called "Alternative Substitute the Namespace Delimiter".

> 这是在上一节中所描述的`替代命名空间分隔符`的情况下提出的另一种变体。

This alternative is the same as the "Suffix Alternative" except:

- Topics would not have a suffix or prefix at all

Trade-offs:

- same trade-offs as the "Suffix Alternative"
- but also easier to have ROS subscribe to a DDS created topic

  - e.g. DDS publisher on topic `image` could be subscribed to in ROS using just `image`
  - the types would need to match

  - in the current proposal the ROS topic `image` would become `rt__image`, so DDS topics would need to follow our naming scheme to interoperate with ROS components

> 在当前的提案中，ROS Topic`image`将变成`rt\_\_image`，因此 DDS Topic 需要遵循我们的命名约定，以与 ROS 组件进行互操作。

Rationale:

While this alternative provided the benefits of both the suffix and limited prefix alternatives, the rationale for the limited prefix alternative still applies here.

> 虽然这个替代方案提供了后缀和有限前缀替代方案的好处，但有限前缀替代方案的基本原理仍然适用于此。
