---
translate by baidu@2023-05-30 23:32:05
---

The special single character token `~` will be replaced with a namespace snippet that is a concatenation of the namespace for the node and the node name. For example, a node `node1` in a namespace `/foo` would result in `~` being replaced with `/foo/node1`. As another example, a node `node1` in a namespace `foo/bar` would result in `~` being replaced with `foo/bar/node1`. It must be used at the beginning of a non-fully qualified name, if at all. Here is a table with some example expansions:

> 特殊的单字符标记“~”将替换为名称空间片段，该片段是节点的名称空间和节点名称的串联。例如，名称空间“/foo”中的节点“node1”将导致“~”被替换为“/foo/node1”。另一个例子是，名称空间“foo/bar”中的节点“node1”将导致“~”被“foo/bar/node1”替换。它必须用于非完全限定名称的开头(如果有的话)。下面是一个带有一些扩展示例的表：

| **Input Name** | Node: `my_node` NS: none | Node: `my_node` NS: `/my_ns` |
| -------------- | ------------------------ | ---------------------------- |
| `ping`         | _`/ping`_                | _`/my_ns/ping`_              |
| `/ping`        | _`/ping`_                | _`/ping`_                    |
| `~`            | _`/my_node`_             | _`/my_ns/my_node`_           |
| `~/ping`       | _`/my_node/ping`_        | _`/my_ns/my_node/ping`_      |

### Substitutions

The bracket syntax (`{substitution_name}`) may be used in non-fully qualified names to substitute useful contextual information into the name. The set of substitution keys (names) are not set in this document, but some reasonable examples might be: `{node}` expands to the current node's name or `{ns}` expands to the current node's namespace.

> 括号语法(`{substitution_name}`)可以在非完全限定名称中使用，以将有用的上下文信息替换为名称。本文档中没有设置替换键(名称)集，但一些合理的示例可能是：“｛node｝”扩展到当前节点的名称或“｛ns｝”展开到当前节点名称空间。

Substitutions are expanded after the private namespace substitution character is expanded. Therefore a substitution may not contain the private namespace substitution character, i.e. `~`. For example, given the name `{private}foo` and a substitution called `{private}` which expands to `~/_`, you will get an error because the `~/_` will end up in the expanded name as `/my_ns/~/_foo` which is is not allowed to have a `~` in it.

> 在展开私有命名空间替换字符之后，将展开替换。因此，替换可能不包含私有命名空间替换字符，即“~”。例如，给定名称“｛private｝foo”和一个扩展为“~/_”的名为“｛private｝”的替换，您将得到一个错误，因为“~/_'”将在扩展名称中以“/my_ns/~/\ufoo”结尾，而扩展名称中不允许有“~”。

Substitutions are expanded in a single pass, so substitutions should not expand to contain substitutions themselves. For example, given the name `/foo/{bar_baz}` where `{bar_baz}` expands to `{bar}/baz` and where `{bar}` in turn expands to `bar`, you will get `/foo/{bar}/baz` as the final result, which is invalid, and not `/foo/bar/baz` as you might expect.

> 换人是在一次传球中扩大的，所以换人不应该扩大到包含换人本身。例如，给定名称“/foo/｛bar_baz｝”，其中“｛bar_blaz｝”扩展为“｛bar｝/baz”，而“｛bar｝”又扩展为“bar”，则最终结果为“/foo/{bar｝/baz”。

Substitutions are also not allowed to be nested, i.e. substitutions may not contain other substitutions in their names. This is implicitly enforced by the rules above that say substitution names may only contain alphanumerics and underscores (`_`). For example, given the name `{% raw %}/foo/{{bar}_baz}{% endraw %}` would result in an error because `{` and `}` are not allowed in a substitution names and the substitution name `{bar}_baz` does contain them.

> 替换也不允许嵌套，即替换的名称中不能包含其他替换。这是由上面的规则隐含地强制执行的，即替换名称只能包含字母数字和下划线(`_`)。例如，给定名称“｛%raw%｝/foo/｛｛bar｝\_baz｝｛%endraw%｝”将导致错误，因为在替换名称中不允许使用“｛”和“｝”，而替换名称“{bar｝\_baz”确实包含它们。

### Hidden Topic or Service Names

Any topic or service name that contains any tokens (either namespaces or a topic or service name) that start with an underscore (`_`) will be considered hidden and tools may not show them unless explicitly asked.

> 任何包含以下划线(`_`)开头的任何标记(命名空间或主题或服务名称)的主题或服务名都将被视为隐藏，除非明确要求，否则工具可能不会显示这些标记。

## Mapping of ROS 2 Topic and Service Names to DDS Concepts

The ROS topic and service name constraints allow more types of characters than the DDS topic names because ROS additionally allows the forward slash (`/`), the tilde (`~`), and the balanced curly braces (`{}`). These must be substituted or otherwise removed during the process of converting the topic or service name to DDS concepts. Since ROS 2 topic and service names are expanded to fully qualified names, any balanced bracket (`{}`) substitutions and tildes (`~`) will have been expanded. Additionally any URL related syntax, e.g. the `rostopic://` prefix, will be removed once parsed. Previously forward slashes (`/`) were disallowed in DDS topic names, now the restriction has been lifted (see [issue](https://issues.omg.org/issues/lists/dds-rtf5#issue-42236) on omg.org) and therefore the ROS topic names are first prefixed with ROS Specific Namespace prefix (described below) are then mapped directly into DDS topic names.

> ROS 主题和服务名称约束允许比 DDS 主题名称更多类型的字符，因为 ROS 还允许正斜杠(`/`)、波浪号(`~`)和平衡大括号(`{}`)。在将主题或服务名称转换为 DDS 概念的过程中，必须替换或删除这些内容。由于 ROS 2 主题和服务名称被扩展为完全限定名称，任何平衡括号(“｛｝”)替换和波浪号(“~”)都将被扩展。此外，任何与 URL 相关的语法，例如“rostopic://”前缀，一旦解析就会被删除。以前 DDS 主题名称中不允许使用正斜杠(`/`)，现在取消了限制(请参阅[问题](https://issues.omg.org/issues/lists/dds-rtf5#issue-42236)，因此ROS主题名称首先加上ROS特定命名空间前缀(如下所述)，然后直接映射到DDS主题名称中。

### ROS Specific Namespace Prefix

In order to differentiate ROS topics easily, all DDS topics created by ROS shall be automatically prefixed with a namespace like `/rX`, where `X` is a single character that indicates to which subsystem of ROS the topic belongs. For example, a plain topic called `/foo` would translate to a DDS topic `rt/foo`, which is the result of implicitly adding `rt` to the namespace of a ROS topic which is in the root namespace `/` and has a base name `foo`. As another example, a topic called `/left/image_raw` would translate to a DDS topic `rt/left/image_raw`, which is the result of implicitly adding `rt` to the namespace of a ROS topic which is in the namespace `/left` and has a base name `image_raw`.

> 为了容易区分 ROS 主题，ROS 创建的所有 DDS 主题都应自动加上一个名称空间前缀，如“/rX”，其中“X”是一个单独的字符，指示主题属于 ROS 的哪个子系统。例如，名为“/foo”的普通主题将转换为 DDS 主题“rt/foo”，这是将“rt”隐式添加到 ROS 主题的名称空间的结果，该 ROS 主题位于根名称空间“/”中并且具有基名称“foo”。作为另一个例子，称为“/left/image_raw”的主题将转换为 DDS 主题“rt/left/images_raw’，这是将“rt”隐式添加到 ROS 主题的名称空间的结果，该 ROS 主题在名称空间“/left”中并且具有基名称“image_raw”。

For systems where Services are implemented with topics (like with OpenSplice), a different subsystem character can be used: `rq` for the request topic and `rr` for the response topic. On systems where Services are handled explicitly implemented, we consider a separate prefix, e.g. `rs`.

> 对于使用主题实现服务的系统(如 OpenSplice)，可以使用不同的子系统字符：“rq”表示请求主题，“rr”表示响应主题。在明确实现服务的系统上，我们考虑单独的前缀，例如“rs”。

Here is a non-exhaustive list of prefixes:

> 以下是前缀的非详尽列表：

| ROS Subsystem        | Prefix |
| -------------------- | ------ |
| ROS Topics           | rt     |
| ROS Service Request  | rq     |
| ROS Service Response | rr     |
| ROS Service          | rs     |
| ROS Parameter        | rp     |
| ROS Action           | ra     |

While all planned prefixes consist of two characters, i.e. `rX`, anything proceeding the first namespace separator, i.e. `/`, can be considered part of the prefix. The standard reserves the right to use up to 8 characters for the prefix in case additional prefix space is needed in the future.

> 虽然所有计划的前缀都由两个字符组成，即“rX”，但在第一个命名空间分隔符(即“/”)之前的任何字符都可以被视为前缀的一部分。如果将来需要额外的前缀空间，标准保留最多使用 8 个字符作为前缀的权利。

### Examples of ROS Names to DDS Concepts

Here are some examples of how a fully qualified ROS name would be broken down into DDS concepts:

> 以下是如何将完全限定的 ROS 名称分解为 DDS 概念的一些示例：

| ROS Name                        | DDS Topic                         |
| ------------------------------- | --------------------------------- |
| `/foo`                          | `rt/foo`                          |
| `rostopic:///foo/bar`           | `rt/foo/bar`                      |
| `/robot1/camera_left/image_raw` | `rt/robot1/camera_left/image_raw` |

### ROS Topic and Service Name Length Limit

The length of the DDS topic must not exceed 256 characters. Therefore the length of a ROS Topic, including the namespace hierarchy, the base name of the topic and any ros specific prefixes must not exceed 256 characters since this is mapped directly as DDS topic.

> DDS 主题的长度不得超过 256 个字符。因此，ROS 主题的长度，包括命名空间层次结构、主题的基本名称和任何特定于 ROS 的前缀，不得超过 256 个字符，因为它被直接映射为 DDS 主题。

#### Considerations for RTI Connext

While testing our implementation with Connext, we encountered some additional limitations when it comes to the topic length. That is, for example the length of a service name has tighter limits than the length of the ROS Topics. The RTI Connext implementation for service names are suffixed with the GUID value of the DDS participant for the service response topic. Additionally, a content filtered topic (max length 256 characters) is created which is mapped from the suffixed service name. Therefore when linking against `rmw_connext_c` or `rmw_connext_cpp`, service names cannot be longer than 185 characters including the namespace hierarchy and any ros specific prefixes.

> 在用 Connext 测试我们的实现时，我们在主题长度方面遇到了一些额外的限制。也就是说，例如，服务名称的长度比 ROS 主题的长度有更严格的限制。服务名称的 RTI-Connext 实现以服务响应主题的 DDS 参与者的 GUID 值作为后缀。此外，还创建了一个内容过滤主题(最大长度 256 个字符)，该主题是从带后缀的服务名称映射而来的。因此，当链接到“rmw_connext_c”或“rmw-connext_cpp”时，服务名称不能超过 185 个字符，包括命名空间层次结构和任何特定于 ros 的前缀。

### Communicating with Non-ROS Topics

Since all ROS topics are prefixed when being converted to DDS topic names, it makes it impossible to subscribe to existing DDS topics which do not follow the same naming pattern. For example, if an existing DDS program is publishing on the `image` topic (and is using the DDS equivalent to the ROS message type) then a ROS program could not subscribe to it because of the name mangling produced by the implicit ROS specific namespace. Therefore to allow ROS programs to interoperate with "native" DDS topic names the API should provide a way to skip the ROS specific prefixing.

> 由于所有 ROS 主题在转换为 DDS 主题名称时都带有前缀，因此无法订阅不遵循相同命名模式的现有 DDS 主题。例如，如果一个现有的 DDS 程序正在“图像”主题上发布(并且使用相当于 ROS 消息类型的 DDS)，那么 ROS 程序就无法订阅它，因为隐含的 ROS 特定命名空间产生了名称篡改。因此，为了允许 ROS 程序与“本机”DDS 主题名进行互操作，API 应该提供一种跳过 ROS 特定前缀的方法。

There is an option in the API, a boolean `avoid_ros_namespace_convention` in the qos_profile which can be set to `false` to use ROS prefix and `true` to not using ROS namespace prefixing.

> API 中有一个选项，即 qos_profile 中的布尔值“avoid_ros_namespace_convention”，可以设置为“false”以使用 ros 前缀，设置为“true”以不使用 ros 命名空间前缀。

For example:

> 例如：

| ROS Name           | avoid_ros_namespace_conventions | DDS Topic  |
| ------------------ | ------------------------------- | ---------- |
| `rostopic://image` | `false`                         | `rt/image` |
| `rostopic://image` | `true`                          | `image`    |

#### Alternative(Idea)

> [NOTE]:
> | ROS Name | DDS Topic |
> |---------------------------------------|---------------------|
> | `rostopic://image` | `rt/image` |
> | `rostopic+exact://image` | `image` |
> | `rostopic+exact://camera_left/image` | `camera_left/image` |
> | `rostopic+exact:///camera_left/image` | `camera_left/image` |

## Compare and Contrast with ROS 1

In order to support a mapping to the - slightly more - restrictive DDS topic name rules, these rules are in some ways more constraining than the rules for ROS 1. Other changes have been proposed for convenience or to remove a point of confusion that existed in ROS 1. In ROS 2, topic and service names differ from ROS 1 in that they:

> 为了支持映射到限制性稍强的 DDS 主题名称规则，这些规则在某些方面比 ROS 1 的规则更具约束性。为了方便或消除 ROS 1 中存在的混淆点，已经提出了其他变化。在 ROS 2 中，主题和服务名称与 ROS 1 的不同之处在于：

- must separate the tilde (`~`) from the rest of the name with a forward slash (`/`)

  - This is done to avoid inconsistency with how `~foo` works in filesystem paths versus when used in a ROS name.

- may contain substitutions which are delimited with balanced curly braces (`{}`)

  - This is a more generic extension of the idea behind the tilde (`~`).

- have length limits

  - This is driven by the topic name length limit of DDS.

- may be indicated as "hidden" by using a leading underscore (`_`) in one of the namespaces

  - This is used to hide common but infrequently introspected topics and services.

## Concerns/Alternatives

This section lists concerns about the proposed design and alternatives that were considered.

> 本节列出了对拟议设计和考虑的备选方案的关注。

### Alternative Name Rules and Concerns About Name Rules

There were some suggested but then rejected alternatives to the rules for topic and service names.

> 有一些人提出了主题和服务名称规则的替代方案，但后来被拒绝了。

#### More Versatile Private Namespace Substitution Character

Currently the `~` private namespace substitution character may only be used at the beginning of the name, but it was also suggested that it could be placed anywhere within the name and could be substituted in place.

> 目前，“~”私有命名空间替换字符只能用在名称的开头，但也有人建议它可以放在名称内的任何位置，并且可以替换到位。

This was rejected because it was complicated to explain and did not behave the same as the `~` when used in filesystem paths on Unix machines. Also, it was difficult to justify its existence because all suggested use cases were quite contrived. It also behaved differently from how it worked in ROS 1, which was yet another negative for this alternative.

> 这被拒绝了，因为它解释起来很复杂，而且在 Unix 机器上的文件系统路径中使用时与“~”的行为不同。此外，很难证明它的存在，因为所有建议的用例都是人为的。它的表现也与它在 ROS 1 中的工作方式不同，ROS 1 是这种替代方案的另一个负面因素。

#### Alternative Substitution Syntax

There were some alternative syntaxes proposed for substitutions in the names before the plain balanced curly braces syntax (`{}`) was selected:

> 在选择纯平衡大括号语法(“｛｝”)之前，有一些替代语法被提议用于名称中的替换：

- `%{sub}`
- `${sub}`
- `$sub`

The most serious alternatives considered were the "bash-like" syntax of `${sub}` and `$sub`. The `$sub` style syntax has the downside of being difficult to process and making it impossible to express some kinds of concatenation. The `${sub}` was a strong candidate, but ultimately was rejected because it would collide with use in shell scripts. For example, you can imagine a shell script that runs a node and remaps a topic name would contain these substitutions, but they would need to be escaped to prevent bash itself from trying to expand them. The `{}` syntax avoids this problem but is also easy to parse.

> 考虑的最严重的替代方案是“${sub}”和“$sub”的“类似 bash”语法。“$sub”风格的语法的缺点是难以处理，并且无法表达某些类型的串联。“${sub}”是一个强有力的候选者，但最终被拒绝，因为它会与 shell 脚本中的使用相冲突。例如，您可以想象一个运行节点并重新映射主题名称的 shell 脚本将包含这些替换，但需要对它们进行转义，以防止 bash 本身试图扩展它们。“｛｝”语法避免了这个问题，但也很容易解析。

The `{}` syntax will collide with Python String substitution, but since that is an explicit action (unlike shell substitution which will implicitly always occur) it's less of an issue.

> “｛｝”语法将与 Python 字符串替换发生冲突，但由于这是一个显式操作(与总是隐式发生的 shell 替换不同)，所以问题不大。

#### Concerns About Substitutions

This document does not prescribe what substitutions should be supported by implementations. This was done to avoid blocking progress on this document on the need to agree on the set of required substitutions. However, this is just delaying the issue. For the substitutions to be useful, then all implementations that process topic and service names need to support them.

> 本文档没有规定实现应该支持哪些替换。这样做是为了避免阻碍本文件的进展，因为需要就所需的替换集达成一致。然而，这只是在拖延问题。为了使替换有用，那么所有处理主题和服务名称的实现都需要支持它们。

This compromise was made so that when more work is done on substitutions, it would hopefully not require changes to the name syntax, but instead revolve around what substitutions to support and whether or not that support is optional.

> 做出这种妥协是为了在替换方面做更多的工作时，希望不需要更改名称语法，而是围绕着支持什么替换以及这种支持是否是可选的。

### Alternative ROS to DDS Mappings

There was some discussion of alternatives and concerns with respect to the ROS -> DDS translation.

> 有一些关于 ROS->DDS 翻译的替代方案和关注点的讨论。

#### Alternative using DDS Partitions

Previously the usage of forward slashes (`/`) was disallowed in DDS topic name and hence a strategy was proposed which used DDS partitions to address the forward slashes (`/`) which are present in ROS names. The main idea was to separate the ROS name into the "namespace" and the "base name", and then place the namespace, stripped of leading and trailing forward slashes (`/`), into a single DDS partition entry and the remaining base name into the DDS topic name. This addressed the issue because the ROS name's base name will not contain any forward slashes (`/`) by definition and so there are no longer any disallowed characters in the DDS topic name. The DDS partition would contain the ROS name's namespace, including any forward slashes (`/`) that made up the namespace and were not at the beginning or the end of the namespace. That is acceptable because DDS partitions are allowed to contain forward slashes (`/`) unlike the DDS topics previously but now DDS topic names allow forward slashes (`/`).

> 以前，DDS 主题名称中不允许使用正斜杠(“/”)，因此提出了一种策略，该策略使用 DDS 分区来处理 ROS 名称中存在的正斜杠(‘/’)。其主要思想是将 ROS 名称分为“名称空间”和“基本名称”，然后将去掉前导和尾随斜杠(`/`)的名称空间放在一个 DDS 分区条目中，将剩余的基本名称放在 DDS 主题名称中。这解决了这个问题，因为 ROS 名称的基本名称根据定义将不包含任何正斜杠(`/`)，因此 DDS 主题名称中不再有任何不允许使用的字符。DDS 分区将包含 ROS 名称的名称空间，包括组成名称空间且不在名称空间的开头或结尾的任何前斜杠(“/”)。这是可以接受的，因为与以前的 DDS 主题不同，允许 DDS 分区包含正斜杠(“/”)，但现在 DDS 主题名称允许正斜杠(‘/’)。

DDS partitions are implemented as an array of strings within the `DDS::Publisher` and `DDS::Subscriber` QoS settings and have no hierarchy or order, Each entry in the partition array is directly combined with the DDS topic and they are not sequentially combined. If a publisher has two partition entries, e.g. `foo` and `bar` with a base name of `baz`, this would be equivalent to having two different publishers on these topics: `/foo/baz` and `/bar/baz`. Therefore this proposal used only one of the strings in the partitions array to hold the entire ROS name's namespace.

> DDS 分区被实现为`DDS:：Publisher`和`DDS:：Subscriber` QoS 设置中的字符串数组，并且没有层次结构或顺序。分区数组中的每个条目都与 DDS 主题直接组合，并且它们不按顺序组合。如果一个发布者有两个分区条目，例如基本名称为“baz”的“foo”和“bar”，这相当于在这些主题上有两个不同的发布者：“/foo/baz”和”/bar/baz“。因此，该提议仅使用分区数组中的一个字符串来保存整个 ROS 名称的命名空间。

You can read more about partitions in RTI's documentation:

> 您可以在 RTI 的文档中阅读更多关于分区的信息：

- [PARTITION_QosPolicy](https://community.rti.com/static/documentation/connext-dds/5.2.3/doc/manuals/connext_dds/html_files/RTI_ConnextDDS_CoreLibraries_UsersManual/Content/UsersManual/PARTITION_QosPolicy.htm)

Trade-offs (in comparison to using the whole ROS name along with the namespaces):

> 权衡(与使用整个 ROS 名称和名称空间相比)：

- Splitting the ROS name into "namespace" and "base name", and placing the complete namespace into a field designed for another purpose seemed incorrect.
- In general partitions are recommended to be used as a spare, but using partitions for all ROS names suggested otherwise.
- Major concern was reported in this [issue](https://github.com/ros2/rmw_connext/issues/234), where having two topics with same base name, although different namespace and different types caused problem. For example: topicA is `/camera/data` of type `Image` and topicB is `/imu/data` of type `Imu`. The base names for both topicA and topicB is `data`, generated errors as described in the [issue](https://github.com/ros2/rmw_connext/issues/234).
- Newer standards such as [DDS-XRCE](https://www.omg.org/spec/DDS-XRCE) might not have partitions at all.
- Using the complete ROS name in the later strategy will cause a tighter length limit on base name because the DDS topic name would contain ROS prefix, namespace along with the base name which should not exceed DDS topic name limitation which is 256 characters.

Rationale:

- With the decision from the DDS vendors to allow forward slashes (`/`) in DDS topic names, using the complete ROS name seemed simple and more intuitive than using partitions.

#### Alternative Substitute the Namespace Delimiter

A previous proposal was to substitute the namespace delimiter, i.e. forward slash (`/`), with something that is allowed in DDS topic names, and then only use the DDS topic name to represent the full ROS name. For example in the simplest case, a topic `/foo/bar/baz` might become `__foo__bar__baz`, where the forward slash (`/`) is being replaced with a double underscore (`__`) and double underscores (`__`) were not allowed in ROS topic and service names.

> 以前的一个建议是用 DDS 主题名称中允许的东西来替换名称空间分隔符，即正斜杠(`/`)，然后只使用 DDS 主题名称来表示完整的 ROS 名称。例如，在最简单的情况下，主题“/foo/bar/baz”可能会变成“**foo**bar**baz”，其中正斜杠(“/`”)被双下划线(“**`”)取代，并且ROS主题和服务名称中不允许使用双下划线('__`)。

Trade-offs (in comparison to the use of DDS partitions):

> 权衡(与 DDS 分区的使用相比)：

- Has a tighter length limit, since it limited by just the DDS topic name and does not benefit from part of the ROS topic name going into the DDS partition.
- The replacement for the forward slash (`/`) had to be more than one character since all usable characters were already in allowed in both the ROS names and DDS topic names, so each namespace further reduced the topic length limit.
- Implementation requires string replacement and validation afterwards, which is moderately complicated.

Rationale:

The DDS partition proposal is preferred over this alternative because it allows for longer total ROS names and because it is simpler to implement, i.e. splitting the string into base name and namespace is simpler than replacing the forward slashes (`/`) with double underscores (`__`) and then redoing length limit testing afterwards.

> 与此替代方案相比，DDS 分区方案更受欢迎，因为它允许更长的 ROS 总名称，而且实现起来更简单，即将字符串拆分为基名称和命名空间比用双下划线(`__`)替换前斜杠(`/`)然后重新进行长度限制测试更简单。

#### Capital Letter Substitution Alternative

This is another alternative that was proposed in the context of the alternative described in the above section called "Alternative Substitute the Namespace Delimiter". Since the forward slash (`/`) is replaced with double underscores (`__`) when translating to DDS from ROS topic names, two characters out of the 256 character limit are lost with each additional namespace. One proposed alternative was to add a constraint that ROS topic names could not use capital letters, and then capital letters could be used as the stand in for the forward slashes (`/`).

> 这是在上一节“替代命名空间分隔符”中描述的替代方案的背景下提出的另一个替代方案。由于在从 ROS 主题名称转换为 DDS 时，正斜杠(`/`)被双下划线(`__`)取代，因此每个额外的命名空间都会丢失 256 个字符限制中的两个字符。一个提议的替代方案是添加一个限制，即 ROS 主题名称不能使用大写字母，然后大写字母可以用作前斜杠(`/`)的替代。

Trade-offs:

> 权衡：

- Uses one fewer character per namespace and makes it easier to calculate the maximum length of a ROS topic or service name.
- Prevents users from using capital letters in their names, which is constraining and might be a problem for backwards compatibility with ROS 1 topics and services.

Rationale:

Preventing users from using capital letters was too constraining for the added benefit.

> 阻止用户使用大写字母对于额外的好处来说过于约束。

#### ROS Prefix with Single Underscore

This is another variation that was proposed in the context of the alternative described in the above section called "Alternative Substitute the Namespace Delimiter". This alternative differs only in that it uses a single underscore in the prefix, i.e. `rt_` rather than `rt__` (`rt` + the leading `/`).

> 这是在上一节“替换命名空间分隔符的替代方案”中描述的替代方案的背景下提出的另一个变体。这种替代方案的不同之处在于，它在前缀中使用了一个下划线，即“rt\_”而不是“rt\_\_”(“rt”+前导“/”)。

Trade-offs:

> 权衡：

- Uses one fewer character
- Less consistent with replacement of other forward slashes (`/`)

Rationale:

Slight preference given to the more consistent alternative.

> 略微倾向于更一致的替代方案。

#### Limited Prefixes Alternative

This is another variation that was proposed in the context of the alternative described in the above section called "Alternative Substitute the Namespace Delimiter".

> 这是在上一节“替换命名空间分隔符的替代方案”中描述的替代方案的背景下提出的另一个变体。

This alternative would:

> 这种替代方案将：

- not prefix topics
- optionally prefix other kinds of "implementation detail topics"

Trade-offs:

> 权衡：

- it would be easier to have ROS subscribe to a DDS created topic

  - e.g. DDS publisher on topic `image` could be subscribed to in ROS using just `image`
  - however, the types would need to match as well
  - in the current proposal the ROS topic `image` would become `rt__image`, so DDS topics would need to follow the ROS topic conversion scheme to interoperate with ROS components

- it would be hard to distinguish ROS created DDS topics and normal DDS topics

- services would still need to be differentiated

  - e.g. service `/foo` would need to make two topics, something like `foo_Request` and `foo_Reply`

Rationale:

Slight preference was given to easily categorizing ROS created topics with DDS created topics over easily connecting to existing DDS topics. Connecting to DDS topics could be achieved by having an option when subscribing or publishing to "alias" to an implementation topic name, e.g. something like `sub->alias_to_explicit_topic('dds_topic')`. Also, the workaround for separating ROS created topics from other DDS topics was considered to be more complicated than the suggested solution of allowing users to specify specific DDS topic names for their publishers and subscriptions.

> 与轻松连接到现有 DDS 主题相比，略微倾向于将 ROS 创建的主题与 DDS 创建的主题轻松分类。连接 DDS 主题可以通过在订阅或发布实现主题名称的“别名”时提供一个选项来实现，例如“sub->alias_to_explicit_topic('DDS_topic')”。此外，将 ROS 创建的主题与其他 DDS 主题分离的解决方案被认为比建议的允许用户为其发布者和订阅指定特定 DDS 主题名称的解决方案更复杂。

#### Suffix Alternative

This is another variation that was proposed in the context of the alternative described in the above section called "Alternative Substitute the Namespace Delimiter".

> 这是在上一节“替换命名空间分隔符的替代方案”中描述的替代方案的背景下提出的另一个变体。

This alternative would:

> 这种替代方案将：

- not prefix topics

- restructure prefixes to instead be suffixes, i.e. `rX<topic>` -> `<topic>_rX_`

  - this would be unique to user defined names because they cannot have a trailing underscore (`_`)

Trade-offs:

> 权衡：

- more difficult to distinguish ROS created DDS topics from normal or built-in DDS topics when listing them using DDS tools because they are not sorted by a ROS specific prefix

- if the service name is suffixed again by the DDS vendor (like in Connext's implementation of Request-Reply) then it would be potentially difficult to differentiate from a user's topic name

  - e.g. service `/foo` might become topics: `foo_s_Request` and `foo_s_Reply` and the user could create a topic called `/foo_s_Request` too.
  - this also applies to any other similar transformations that an RMW implementation might make to the topic

Rationale:

> 理由：

This alternative was not selected over the prefix solution because of a lack of advantages over the prefix solution. Also, it typically took one more character to express (`rt` versus `_rt_`; unless you also drop the implicit first namespace `/` then it's `rt__` versus `_rt_`) and the potential issues with ambiguity when the DDS implementation handles Request-Reply (added suffixes).

> 由于与前缀解决方案相比缺乏优势，因此没有选择此替代方案而不是前缀解决方案。此外，它通常还需要一个字符来表达(“rt”与“_rt_”；除非您也删除隐式的第一个命名空间“/”，否则它是“rt\__”与“\_rt_”)，以及 DDS 实现处理请求回复(添加后缀)时的潜在模糊问题。

#### Limited Suffix Alternative

This is another variation that was proposed in the context of the alternative described in the above section called "Alternative Substitute the Namespace Delimiter".

> 这是在上一节“替换命名空间分隔符的替代方案”中描述的替代方案的背景下提出的另一个变体。

This alternative is the same as the "Suffix Alternative" except:

> 此备选方案与“后缀备选方案”相同，除了：

- Topics would not have a suffix or prefix at all

Trade-offs:

> 权衡：

- same trade-offs as the "Suffix Alternative"

- but also easier to have ROS subscribe to a DDS created topic

  - e.g. DDS publisher on topic `image` could be subscribed to in ROS using just `image`
  - the types would need to match
  - in the current proposal the ROS topic `image` would become `rt__image`, so DDS topics would need to follow our naming scheme to interoperate with ROS components

Rationale:

> 理由：

While this alternative provided the benefits of both the suffix and limited prefix alternatives, the rationale for the limited prefix alternative still applies here.

> 虽然这种替代方案提供了后缀和有限前缀替代方案的好处，但有限前缀替代的基本原理仍然适用于此。
