---
tip: translate by openai@2023-05-28 11:39:04
layout: default
title: Topic and Service name mapping to DDS
permalink: articles/topic_and_service_names.html
abstract:
  This article describes the proposed mapping between ROS topic and service names to DDS topic and service names.
author: '[William Woodall](https://github.com/wjwwood)'
date_written: 2016-10
last_modified: 2018-06
published: true
Authors: {{ page.author }}
Date Written: {{ page.date_written }}
Last Modified: {% if page.last_modified %}{{ page.last_modified }}{% else %}{{ page.date_written }}{% endif %}
---

## Context

Before proposing constraints for ROS 2 topic and service names and a mapping to underlying DDS topics, this article will first summarize the existing guidelines and restrictions for both ROS 1 and DDS.

> 在提出 ROS 2 主题和服务名称的约束以及与底层 DDS 之间的映射之前，本文将首先总结 ROS 1 和 DDS 的现有指南和限制。

### ROS 1 Names

In ROS 1, topic and service name guidelines are all covered under the umbrella of "ROS Names" and have these restrictions:

> 在 ROS 1 中，主题和服务名称准则都涵盖在“ROS Names”的保护伞下，并具有以下限制：

_From [http://wiki.ros.org/Names](http://wiki.ros.org/Names):_

> .

_根据[http://wiki.ros.org/Names](http://wiki.ros.org/Names)：_

```
    A valid name has the following characteristics:

    * First character is an alpha character ([a-z|A-Z]), tilde (~) or forward slash (/)
    * Subsequent characters can be alphanumeric ([0-9|a-z|A-Z]), underscores (_), or forward slashes (/)

    Exception: base names (described below) cannot have forward slashes (/) or tildes (~) in them.
```

### DDS Topic Names

In DDS, topic names are restricted by these restrictions:

> 在 DDS 中，主题名称受到以下限制：

_From DDS 1.4 specification, or the [RTI documentation](http://community.rti.com/rti-doc/510/ndds/doc/html/api_cpp/group__DDSQueryAndFilterSyntaxModule.html):_

> _根据 DDS 1.4 规范或[RTI 文档](http://community.rti.com/rti-doc/510/ndds/doc/html/api_cpp/group__DDSQueryAndFilterSyntaxModule.html)：_

```
    TOPICNAME - A topic name is an identifier for a topic, and is defined as any series of characters 'a', ..., 'z', 'A', ..., 'Z', '0', ..., '9', '_' but may not start with a digit.
```

_Note:_ that the DDS specification has a known typo, where it says `-` are allowed, but the RTI documentation correctly lists `_` as allowed.

> _注意：_ DDS 规范中有一个已知的错误，它说 `-` 是允许的，但 RTI 文档正确地列出 `_` 也是允许的。

Additionally, DDS - or more precisely the underlaying RTPS protocol - has a hard limit on topic names of 256 characters, so an additional goal is to minimize the number of extra characters used when mapping from ROS to DDS names.

> 此外，DDS（或更精确地说，底层 RTPS 协议）对主题名称有 256 个字符的硬限制，因此，除了将 ROS 映射到 DDS 名称时尽量减少额外字符的目标外，还有另一个目标。

See The Real-time Publish-Subscribe Protocol (RTPS) DDS Interoperability Wire Protocol Specification, Table 9.12 for more details.

> 请参阅实时发布订阅协议（RTPS）DDS 互操作性无线协议规范，表 9.12 以获取更多详细信息。

## ROS 2 Topic and Service Name Constraints

In this section an outline of the proposed constrains for ROS 2 topic and service names will be enumerated along with rationales where appropriate.

> 在本节中，将列出 ROS 2 主题和服务名称的建议约束，并在适当的情况下附有理由。

For convenience here is a summary of all rules for topic and service names in ROS 2:

> 以下是 ROS 2 中主题和服务名称的所有规则的摘要，方便参考。

- must not be empty

> 不能为空

- may contain alphanumeric characters (`[0-9|a-z|A-Z]`), underscores (`_`), or forward slashes (`/`)

> 可以包含字母数字字符（[0-9|a-z|A-Z]）、下划线（\_）或斜杠（/）

- may use balanced curly braces (`{}`) for substitutions

> 可以使用平衡的花括号（{}）来进行替换

- may start with a tilde (`~`), the private namespace substitution character

> 可以以波浪符号（`~`）开头，这是私有命名空间替换字符。

- must not start with a numeric character (`[0-9]`)

> 不能以数字字符（`[0-9]`）开头

- must not end with a forward slash (`/`)

> 不得以斜杠（'/'）结尾

- must not contain any number of repeated forward slashes (`/`)

> 不得包含任何重复的斜杠（`/`）

- must not contain any number of repeated underscores (`_`)

> 不能包含任何重复的下划线（`_`）

- must separate a tilde (`~`) from the rest of the name with a forward slash (`/`), i.e. `~/foo` not `~foo`

> 必须用斜杠（`/`）将波浪号（`~`）与其余部分分开，即`~/foo`而不是`~foo`。

- must have balanced curly braces (`{}`) when used, i.e. `{sub}/foo` but not `{sub/foo` nor `/foo}`

> 必须在使用时有平衡的花括号（{}），例如{sub}/foo，而不是{sub/foo 或/foo}。

The content of substitutions, i.e. the string in side of balanced curly braces (`{}`), follow very similar rules to names.

> 替换的内容，即平衡大括号（`{}`）中的字符串，遵循与名称非常相似的规则。

The content of substitutions:

> 替换的内容：

- must not be empty

> 不能为空

- may contain alphanumeric characters (`[0-9|a-z|A-Z]`) and underscores (`_`)

> 可以包含字母数字字符（`[0-9|a-z|A-Z]`）和下划线（`_`）

- must not start with a numeric character (`[0-9]`)

> 不能以数字字符（`[0-9]`）开头

### Fully Qualified Names

The topic and service name rules allow for some convenience syntax, which in some cases requires additional context to expand to the fully qualified name and then to the DDS equivalent name.

> 主题和服务名称规则允许一些便利的语法，在某些情况下，需要额外的上下文来扩展到完全限定的名称，然后再扩展到 DDS 相应的名称。

For example, as outlined in further detail in the sections that follow, names may be relative (e.g. `foo` versus the absolute `/foo`), they may contain the private namespace substitution character (`~`), or arbitrary substitutions which are within the curly braces (`{}`) syntax.

> 例如，在后面的章节中详细阐述的，名称可以是相对的（例如`foo`与绝对的`/foo`），它们可以包含私有命名空间替换字符（`~`），或者在花括号（`{}`）语法中的任意替换。

With context, each of these features can be expanded to some simple string to form the fully qualified name.

> 随着上下文的变化，每个特征都可以扩展为一些简单的字符串，以形成完全限定的名称。

Fully qualified names have these additional restrictions:

> 完全限定名称有以下附加限制：

- must start with a forward slash (`/`), i.e. they must be absolute

> 必须以斜杠（`/`）开头，即必须是绝对路径。

- must not contain tilde (`~`) or curly braces (`{}`)

> 不得包含波浪号（`~`）或大括号（`{}`）

> [NOTE]:
> An example of an invalid substitution would be `{sub}/foo` and replace `{sub}` with a numeric value, which thus leads to a topic starting with a numeric character.

> .

一个无效的替换的例子是将“{sub}/foo”替换为数值，这样就导致主题以数字开头。

### Uniform Resource Locators (URLs)

Additionally, topic and service names can be represented in the [Uniform Resource Locator (URL)](https://en.wikipedia.org/wiki/Uniform_Resource_Locator) format to further disambiguate the resource name.

> 此外，主题和服务名称可以用[统一资源定位符（URL）](https://en.wikipedia.org/wiki/Uniform_Resource_Locator)格式表示，以进一步澄清资源名称。

A topic name may be preceded by a `rostopic://` scheme prefix, and a service name may be preceded by a `rosservice://` scheme prefix.

> 一个主题名称可能被`rostopic://`方案前缀所引导，而服务名称可能被`rosservice://`方案前缀所引导。

For example, the absolute name `/foo` could also be represented as a topic with `rostopic:///foo` or as a service with `rosservice:///foo`.

> 例如，绝对名称`/foo`也可以用`rostopic:///foo`表示为一个主题，或者用`rosservice:///foo`表示为一个服务。

> [NOTE]:
> A relative name `foo/bar` could would be represented (as a topic) with `rostopic://foo/bar`.

> 一个相对名称`foo/bar`可以用`rostopic://foo/bar`表示（作为一个主题）。

### ROS 2 Name Examples

For example, these are valid names:

> 例如，这些是有效的名称：

    | `foo`      | `abc123`   | `_foo`  | `Foo`               | `BAR`                |
    | `~`        | `foo/bar`  | `~/foo` | `{foo}_bar`         | `foo/{ping}/bar`     |
    | `foo/_bar` | `foo_/bar` | `foo_`  | `rosservice:///foo` | `rostopic://foo/bar` |

But these are not valid names:

> 但这些不是有效的名字：

    | `123abc`    | `123`  | `foo bar`  | ` `        | `foo//bar` |
    | `/~`        | `~foo` | `foo~`     | `foo~/bar` | `foo/~bar` |
    | `foo/~/bar` | `foo/` | `foo__bar` |            |            |

These are some valid fully qualified names:

> 以下是一些有效的完全限定名称：

    | `/foo`     | `/bar/baz` | `rostopic:///ping` | `/_private/thing` | `/public_namespace/_private/thing` |

### Namespaces

Topic and service names:

> 主题和服务名称：

- may be split into tokens using a forward slash (`/`) as a delimiter

> 可以使用斜杠（`/`）作为分隔符来拆分成令牌。

- see the "Name Tokens" section for more details on tokens

> 请参阅“名称令牌”部分以获取更多有关令牌的详细信息。

- must not end with a forward slash (`/`)

> 不得以斜杠(`/`)结尾

The last token is the topic or service base name, and any preceding tokens make up the namespace of the topic or service.

> 最后一个标记是主题或服务的基础名称，而任何前面的标记构成主题或服务的命名空间。

For example, the topic name `/foo/bar/baz` is composed of a topic or service with the base name `baz` in the `/foo/bar` namespace.

> 例如，主题名称`/foo/bar/baz`由基础名称`baz`在`/foo/bar`命名空间中的主题或服务组成。

In another example, the name `/foo` splits into one token, such that it is composed of a topic or service with the base name `foo` in the `/` namespace (the root namespace).

> 在另一个例子中，名称`/foo`被分割成一个令牌，这样它由一个主题或服务组成，基本名称为`foo`，在`/`命名空间（根命名空间）中。

Topic and service names:

> 主题和服务名称：

- may be specified as absolute or relative

> 可以指定为绝对值或相对值

- must start with a forward slash (`/`) if they are absolute

> 必须以斜杠（`/`）开头，如果它们是绝对的。

An absolute name begins with a forward slash (`/`) and does not respect namespaces, i.e. it can be considered "global".

> 一个绝对名称以斜杠（`/`）开头，不受命名空间的限制，可以被认为是“全局”的。

A relative name does not begin with a forward slash (`/`) and does respect the namespace of the node which created them.

> 相对名称不以斜杠（'/'）开头，并且尊重创建它们的节点的命名空间。

Relative names are appended to the namespace of the node which creates them.

> 相对名称被附加到创建它们的节点的命名空间。

For example, if the node is put into a namespace `/ping/pong` and the topic or service name is `foo/bar`, then the absolute name will become `/ping/pong/foo/bar`.

> 例如，如果将节点放入名称空间`/ping/pong`，主题或服务名称为`foo/bar`，那么绝对名称将变为`/ping/pong/foo/bar`。

However, if the topic or service name is `/foo/bar`, then the absolute name will stay just `/foo/bar`, ignoring the node's namespace.

> 然而，如果主题或服务名称是`/foo/bar`，那么绝对名称将保持为`/foo/bar`，忽略节点的命名空间。

### Name Tokens

"Name tokens" are the strings between the namespace delimiters, e.g. the tokens of the topic or service `/foo/bar/baz` are `foo`, `bar`, and `baz`.

> "名称令牌"是命名空间定界符之间的字符串，例如主题或服务/foo/bar/baz 的令牌是 foo、bar 和 baz。

Topic and service name tokens:

> 主题和服务名称令牌：

- must not be empty, e.g. the name `//bar` is not allowed

> 不能为空，例如名称`bar`不允许。

- rationale: it removes the chance for accidental `//` from concatenation and therefore the need to collapse `//` to `/`

> 理由：它消除了连接时不小心添加`//`的可能性，因此无需将`//`折叠为`/`。

- may use alphanumeric characters (`[0-9|a-z|A-Z]`), underscores (`_`), and/or balanced curly braces (`{}`)

> 可以使用字母数字字符（[0-9|a-z|A-Z]）、下划线（\_）和/或平衡的大括号（{}）

- must not start with numeric characters (`[0-9]`)

> 不得以数字字符（`[0-9]`）开头

- may be a single tilde character (`~`)

> 可能是一个波浪号字符（`~`）

### Private Namespace Substitution Character

The special single character token `~` will be replaced with a namespace snippet that is a concatenation of the namespace for the node and the node name.

> 特殊的单字符令牌`~`将被命名空间片段替换，该片段是节点的命名空间和节点名称的连接。

For example, a node `node1` in a namespace `/foo` would result in `~` being replaced with `/foo/node1`.

> 例如，在命名空间`/foo`中的节点`node1`将导致`~`被替换为`/foo/node1`。

As another example, a node `node1` in a namespace `foo/bar` would result in `~` being replaced with `foo/bar/node1`.

> 例如，在名称空间“foo/bar”中的节点`node1`将导致`~`被替换为`foo/bar/node1`。

It must be used at the beginning of a non-fully qualified name, if at all.

> 它必须在非完全限定名称开头使用，如果使用的话。

Here is a table with some example expansions:

> 这里有一张表格，里面有一些例子扩展：

    | **Input Name** | Node: `my_node` NS: none | Node: `my_node` NS: `/my_ns`   |
    |----------------|--------------------------|--------------------------------|
    | `ping`         | *`/ping`*                | *`/my_ns/ping`*                |
    | `/ping`        | *`/ping`*                | *`/ping`*                      |
    | `~`            | *`/my_node`*             | *`/my_ns/my_node`*             |
    | `~/ping`       | *`/my_node/ping`*        | *`/my_ns/my_node/ping`*        |

> [NOTE]:

### Substitutions

The bracket syntax (`{substitution_name}`) may be used in non-fully qualified names to substitute useful contextual information into the name.

> 括号语法（`{substitution_name}`）可以用于非完全限定的名称中，以将有用的上下文信息替换到名称中。

The set of substitution keys (names) are not set in this document, but some reasonable examples might be: `{node}` expands to the current node's name or `{ns}` expands to the current node's namespace.

> 这份文件中没有设置替换键（名称），但是一些合理的示例可能是：`{node}` 展开为当前节点的名称，或 `{ns}` 展开为当前节点的命名空间。

Substitutions are expanded after the private namespace substitution character is expanded.

> 私有命名空间替换字符被展开后，替换扩展。

Therefore a substitution may not contain the private namespace substitution character, i.e. `~`.

> 因此，替换不能包含私有命名空间替换字符，即`~`。

For example, given the name `{private}foo` and a substitution called `{private}` which expands to `~/_`, you will get an error because the `~/_` will end up in the expanded name as `/my_ns/~/_foo` which is is not allowed to have a `~` in it.

> 例如，给定名称`{private}foo`和一个名为`{private}`的替换，其展开为`~/_`，您将会得到一个错误，因为`~/_`最终会以`/my_ns/~/_foo`的形式出现在展开后的名称中，而这是不允许有`~`字符的。

Substitutions are expanded in a single pass, so substitutions should not expand to contain substitutions themselves.

> 替换在一次传递中被展开，因此替换本身不应包含替换。

For example, given the name `/foo/{bar_baz}` where `{bar_baz}` expands to `{bar}/baz` and where `{bar}` in turn expands to `bar`, you will get `/foo/{bar}/baz` as the final result, which is invalid, and not `/foo/bar/baz` as you might expect.

> 例如，给定名称`/foo/{bar_baz}`，其中`{bar_baz}`展开为`{bar}/baz`，其中`{bar}`又展开为`bar`，最终结果将是`/foo/{bar}/baz`，这是无效的，而不是您可能期望的`/foo/bar/baz`。

Substitutions are also not allowed to be nested, i.e. substitutions may not contain other substitutions in their names.

> 不允许嵌套替换，即替换名称中不得包含其他替换。

This is implicitly enforced by the rules above that say substitution names may only contain alphanumerics and underscores (`_`).

> 这通过上面的规则隐含地强制执行，即替换名称只能包含字母数字和下划线（`_`）。

For example, given the name `{% raw %}/foo/{{bar}_baz}{% endraw %}` would result in an error because `{` and `}` are not allowed in a substitution names and the substitution name `{bar}_baz` does contain them.

> 例如，给定名称`/foo/{{bar}_baz}`会导致错误，因为替换名称中不允许使用`{`和`}`，而替换名称`{bar}_baz`中包含它们。

### Hidden Topic or Service Names

Any topic or service name that contains any tokens (either namespaces or a topic or service name) that start with an underscore (`_`) will be considered hidden and tools may not show them unless explicitly asked.

> 任何包含以下划线（“\_”）开头的令牌（命名空间或主题或服务名称）的主题或服务名称，将被视为隐藏，工具可能不会显示它们，除非有明确的要求。

## Mapping of ROS 2 Topic and Service Names to DDS Concepts

The ROS topic and service name constraints allow more types of characters than the DDS topic names because ROS additionally allows the forward slash (`/`), the tilde (`~`), and the balanced curly braces (`{}`).

> ROS 主题和服务名称的约束比 DDS 主题名称允许更多的字符类型，因为 ROS 除了允许斜杠（'/'）、波浪号（'~'）和平衡的花括号（'{}'）之外。

These must be substituted or otherwise removed during the process of converting the topic or service name to DDS concepts.

> 这些必须在将主题或服务名称转换为 DDS 概念的过程中替换或移除。

Since ROS 2 topic and service names are expanded to fully qualified names, any balanced bracket (`{}`) substitutions and tildes (`~`) will have been expanded.

> 由于 ROS 2 主题和服务名称已扩展为完全限定名称，因此任何平衡括号（`{}`）替换和波浪号（`~`）都将被展开。

Additionally any URL related syntax, e.g. the `rostopic://` prefix, will be removed once parsed.

> 此外，一旦解析完成，与 URL 有关的语法，例如`rostopic://`前缀，将被删除。

Previously forward slashes (`/`) were disallowed in DDS topic names, now the restriction has been lifted (see [issue](https://issues.omg.org/issues/lists/dds-rtf5#issue-42236) on omg.org) and therefore the ROS topic names are first prefixed with ROS Specific Namespace prefix (described below) are then mapped directly into DDS topic names.

> 以前，正斜杠（`/`）不允许在 DDS 主题名称中使用，现在该限制已被取消（请参阅 omg.org 上的[问题](https://issues.omg.org/issues/lists/dds-rtf5#issue-42236)），因此 ROS 主题名称首先使用 ROS 特定的命名空间前缀（如下所述），然后直接映射到 DDS 主题名称。

### ROS Specific Namespace Prefix

In order to differentiate ROS topics easily, all DDS topics created by ROS shall be automatically prefixed with a namespace like `/rX`, where `X` is a single character that indicates to which subsystem of ROS the topic belongs.

> 为了容易区分 ROS 主题，ROS 创建的所有 DDS 主题都将自动添加一个名称空间前缀，如'/rX'，其中'X'表示该主题属于 ROS 的哪个子系统。

For example, a plain topic called `/foo` would translate to a DDS topic `rt/foo`, which is the result of implicitly adding `rt` to the namespace of a ROS topic which is in the root namespace `/` and has a base name `foo`.

> 例如，一个称为“/foo”的普通主题会被翻译成 DDS 主题“rt/foo”，这是在 ROS 主题的根命名空间`/`中隐式添加`rt`到基本名称`foo`的结果。

As another example, a topic called `/left/image_raw` would translate to a DDS topic `rt/left/image_raw`, which is the result of implicitly adding `rt` to the namespace of a ROS topic which is in the namespace `/left` and has a base name `image_raw`.

> .

例如，一个名为`/left/image_raw`的主题会隐式地在 ROS 主题的命名空间`/left`中添加`rt`，转换成 DDS 主题`rt/left/image_raw`。

For systems where Services are implemented with topics (like with OpenSplice), a different subsystem character can be used: `rq` for the request topic and `rr` for the response topic.

> 在使用主题（例如 OpenSplice）实现服务的系统中，可以使用不同的子系统字符：`rq`用于请求主题，`rr`用于响应主题。

On systems where Services are handled explicitly implemented, we consider a separate prefix, e.g. `rs`.

> 在处理服务的系统上，我们考虑使用一个单独的前缀，例如“rs”。

Here is a non-exhaustive list of prefixes:

> 以下是一份不全面的前缀列表：

    | ROS Subsystem        | Prefix |
    |----------------------|--------|
    | ROS Topics           | rt     |
    | ROS Service Request  | rq     |
    | ROS Service Response | rr     |
    | ROS Service          | rs     |
    | ROS Parameter        | rp     |
    | ROS Action           | ra     |

While all planned prefixes consist of two characters, i.e. `rX`, anything proceeding the first namespace separator, i.e. `/`, can be considered part of the prefix.

> 所有计划的前缀都由两个字符组成，即`rX`，第一个命名空间分隔符，即`/`，之后的任何内容都可以被视为前缀的一部分。

The standard reserves the right to use up to 8 characters for the prefix in case additional prefix space is needed in the future.

> 标准保留在未来需要额外前缀空间时使用最多 8 个字符的前缀的权利。

### Examples of ROS Names to DDS Concepts

Here are some examples of how a fully qualified ROS name would be broken down into DDS concepts:

> 以下是一些全限定 ROS 名称如何被分解成 DDS 概念的示例：

    | ROS Name                        | DDS Topic                         |
    |---------------------------------|-----------------------------------|
    | `/foo`                          | `rt/foo`                          |
    | `rostopic:///foo/bar`           | `rt/foo/bar`                      |
    | `/robot1/camera_left/image_raw` | `rt/robot1/camera_left/image_raw` |

### ROS Topic and Service Name Length Limit

The length of the DDS topic must not exceed 256 characters. Therefore the length of a ROS Topic, including the namespace hierarchy, the base name of the topic and any ros specific prefixes must not exceed 256 characters since this is mapped directly as DDS topic.

> DDS 主题的长度不得超过 256 个字符。因此，ROS 主题的长度，包括命名空间层次结构、主题的基本名称和任何 ros 特定的前缀，都不得超过 256 个字符，因为这直接映射为 DDS 主题。

#### Considerations for RTI Connext

While testing our implementation with Connext, we encountered some additional limitations when it comes to the topic length.

> .

在使用 Connext 进行测试实施时，我们在主题长度方面遇到了一些额外的限制。

That is, for example the length of a service name has tighter limits than the length of the ROS Topics.

> 例如，服务名称的长度比 ROS 主题的长度更加受限制。

The RTI Connext implementation for service names are suffixed with the GUID value of the DDS participant for the service response topic.

> RTI Connext 的服务名称后缀是 DDS 参与者的 GUID 值，用于服务响应主题。

Additionally, a content filtered topic (max length 256 characters) is created which is mapped from the suffixed service name.

> 另外，创建了一个内容过滤的主题（最大长度 256 个字符），它是从带后缀的服务名映射而来的。

Therefore when linking against `rmw_connext_c` or `rmw_connext_cpp`, service names cannot be longer than 185 characters including the namespace hierarchy and any ros specific prefixes.

> 因此，当链接`rmw_connext_c`或`rmw_connext_cpp`时，服务名称不得超过 185 个字符，包括命名空间层次结构和任何 ros 特定的前缀。

### Communicating with Non-ROS Topics

Since all ROS topics are prefixed when being converted to DDS topic names, it makes it impossible to subscribe to existing DDS topics which do not follow the same naming pattern.

> 由于所有 ROS 主题在转换为 DDS 主题名称时都带有前缀，因此无法订阅不遵循相同命名模式的现有 DDS 主题。

For example, if an existing DDS program is publishing on the `image` topic (and is using the DDS equivalent to the ROS message type) then a ROS program could not subscribe to it because of the name mangling produced by the implicit ROS specific namespace.

> 例如，如果一个现有的 DDS 程序正在发布`image`主题（并且正在使用与 ROS 消息类型相等的 DDS），那么 ROS 程序就无法订阅它，因为由 ROS 特定命名空间隐式产生的命名混乱。

Therefore to allow ROS programs to interoperate with "native" DDS topic names the API should provide a way to skip the ROS specific prefixing.

> 因此，为了使 ROS 程序能够与“本机”DDS 主题名称互操作，API 应提供一种跳过 ROS 特定前缀的方法。

There is an option in the API, a boolean `avoid_ros_namespace_convention` in the qos_profile which can be set to `false` to use ROS prefix and `true` to not using ROS namespace prefixing.

> API 中有一个选项，qos_profile 中的布尔值`avoid_ros_namespace_convention`可以设置为`false`来使用 ROS 前缀，设置为`true`则不使用 ROS 命名空间前缀。

For example:

> 例如：

    | ROS Name              | avoid_ros_namespace_conventions    | DDS Topic   |
    |-----------------------|------------------------------------|-------------|
    | `rostopic://image`    | `false`                            | `rt/image`  |
    | `rostopic://image`    | `true`                             | `image`     |

#### Alternative(Idea)

> [NOTE]:
> Another option would be to have some markup in the scheme name, for example:

> 另一个选择是在方案名称中添加一些标记，例如：

    | ROS Name                              | DDS Topic           |
    |---------------------------------------|---------------------|
    | `rostopic://image`                    | `rt/image`          |
    | `rostopic+exact://image`              | `image`             |
    | `rostopic+exact://camera_left/image`  | `camera_left/image` |
    | `rostopic+exact:///camera_left/image` | `camera_left/image` |

## Compare and Contrast with ROS 1

In order to support a mapping to the - slightly more - restrictive DDS topic name rules, these rules are in some ways more constraining than the rules for ROS 1.

> .

为了支持映射到更严格的 DDS 主题名称规则，这些规则在某些方面比 ROS 1 的规则更加严格。

Other changes have been proposed for convenience or to remove a point of confusion that existed in ROS 1.

> 其他变更是为了方便或消除 ROS 1 中存在的混乱点而提出的。

In ROS 2, topic and service names differ from ROS 1 in that they:

> 在 ROS 2 中，主题和服务名称与 ROS 1 不同，因为它们：

- must separate the tilde (`~`) from the rest of the name with a forward slash (`/`)

> 必须用斜杠（`/`）将波浪号（`~`）与其他名称分开。

- This is done to avoid inconsistency with how `~foo` works in filesystem paths versus when used in a ROS name.

> 这是为了避免在文件系统路径中使用`~foo`时与 ROS 名称中使用的不一致。

- may contain substitutions which are delimited with balanced curly braces (`{}`)

> 可能包含用平衡的大括号（{}）分隔的替换字符

- This is a more generic extension of the idea behind the tilde (`~`).

> 这是对波浪号（`~`）背后思想的更为普遍的扩展。

- have length limits

> 拥有长度限制

- This is driven by the topic name length limit of DDS.

> 这是由 DDS 的主题名称长度限制驱动的。

- may be indicated as "hidden" by using a leading underscore (`_`) in one of the namespaces

> 可以通过在其中一个命名空间中使用前导下划线（\_）来表示为“隐藏”。

- This is used to hide common but infrequently introspected topics and services.

> 这用于隐藏常见但不经常检查的主题和服务。

## Concerns/Alternatives

This section lists concerns about the proposed design and alternatives that were considered.

> 这一部分列出了关于拟议设计的担忧以及考虑过的替代方案。

### Alternative Name Rules and Concerns About Name Rules

There were some suggested but then rejected alternatives to the rules for topic and service names.

> 有一些建议，但被拒绝的替代规则，用于主题和服务名称。

#### More Versatile Private Namespace Substitution Character

Currently the `~` private namespace substitution character may only be used at the beginning of the name, but it was also suggested that it could be placed anywhere within the name and could be substituted in place.

> 目前，`~`私有命名空间替换字符只能在名称开头使用，但也建议可以在名称的任何位置放置它，并可以替换。

This was rejected because it was complicated to explain and did not behave the same as the `~` when used in filesystem paths on Unix machines.

> 这被拒绝了，因为它很难解释，而且在 Unix 机器上使用文件系统路径时，其行为与`~`不同。

Also, it was difficult to justify its existence because all suggested use cases were quite contrived.

> 也很难证明它的存在，因为所有建议的用例都很苛刻。

It also behaved differently from how it worked in ROS 1, which was yet another negative for this alternative.

> 它的行为也与 ROS 1 中的行为不同，这又是这种替代方案的另一个缺点。

#### Alternative Substitution Syntax

There were some alternative syntaxes proposed for substitutions in the names before the plain balanced curly braces syntax (`{}`) was selected:

> 在选择普通平衡大括号语法（`{}`）之前，曾提出一些替换的替代语法：

    - `%{sub}`
    - `${sub}`
    - `$sub`

The most serious alternatives considered were the "bash-like" syntax of `${sub}` and `$sub`.

> 最被考虑的严肃替代方案是“类似 bash”的语法 `${sub}` 和 `$sub`。

The `$sub` style syntax has the downside of being difficult to process and making it impossible to express some kinds of concatenation.

> `$sub`语法的缺点是难以处理，也无法表达某些种类的连接。

The `${sub}` was a strong candidate, but ultimately was rejected because it would collide with use in shell scripts.

> `${sub}`是一个很强的候选者，但最终被拒绝了，因为它会与在 Shell 脚本中的使用发生冲突。

For example, you can imagine a shell script that runs a node and remaps a topic name would contain these substitutions, but they would need to be escaped to prevent bash itself from trying to expand them.

> 例如，你可以想像一个运行节点并重新映射主题名称的 shell 脚本会包含这些替换，但是它们需要被转义以防止 bash 本身试图展开它们。

The `{}` syntax avoids this problem but is also easy to parse.

> `{}`语法可以避免这个问题，同时也很容易解析。

The `{}` syntax will collide with Python String substitution, but since that is an explicit action (unlike shell substitution which will implicitly always occur) it's less of an issue.

> `{}`语法会与 Python 字符串替换发生冲突，但由于这是一个明确的操作（不像壳子替换会隐式发生），因此问题不大。

#### Concerns About Substitutions

This document does not prescribe what substitutions should be supported by implementations.

> 这份文件不规定实施者应该支持哪些替换。

This was done to avoid blocking progress on this document on the need to agree on the set of required substitutions.

> 这样做是为了避免在需要就此文件中所需替换的一组内容达成一致时阻碍进展。

However, this is just delaying the issue.

> 然而，这只是在推迟问题。

For the substitutions to be useful, then all implementations that process topic and service names need to support them.

> 为了使替换有用，那么所有处理主题和服务名称的实现都需要支持它们。

This compromise was made so that when more work is done on substitutions, it would hopefully not require changes to the name syntax, but instead revolve around what substitutions to support and whether or not that support is optional.

> 这种妥协是为了在对替代品做出更多的工作时，有希望不需要更改名称语法，而是围绕什么替代品支持和是否支持是可选的。

### Alternative ROS to DDS Mappings

There was some discussion of alternatives and concerns with respect to the ROS -> DDS translation.

> 有关 ROS 到 DDS 转换的替代方案和关注点有一些讨论。

#### Alternative using DDS Partitions

Previously the usage of forward slashes (`/`) was disallowed in DDS topic name and hence a strategy was proposed which used DDS partitions to address the forward slashes (`/`) which are present in ROS names. The main idea was to separate the ROS name into the "namespace" and the "base name", and then place the namespace, stripped of leading and trailing forward slashes (`/`), into a single DDS partition entry and the remaining base name into the DDS topic name.

> 之前禁止在 DDS 主题名称中使用斜杠（`/`），因此提出了一种使用 DDS 分区来解决 ROS 名称中存在的斜杠（`/`）的策略。主要思想是将 ROS 名称分为“命名空间”和“基本名称”，然后将去掉前导和尾随斜杠（`/`）的命名空间放入一个 DDS 分区条目中，剩余的基本名称放入 DDS 主题名称中。

This addressed the issue because the ROS name's base name will not contain any forward slashes (`/`) by definition and so there are no longer any disallowed characters in the DDS topic name.

> .

这解决了这个问题，因为根据定义，ROS 名称的基本名称不会包含任何斜杠（`/`），因此 DDS 主题名称中不再有任何不允许的字符。

The DDS partition would contain the ROS name's namespace, including any forward slashes (`/`) that made up the namespace and were not at the beginning or the end of the namespace.

> DDS 分区将包含 ROS 名称的命名空间，包括构成命名空间的任何斜杠（`/`），这些斜杠不在命名空间的开头或结尾处。

That is acceptable because DDS partitions are allowed to contain forward slashes (`/`) unlike the DDS topics previously but now DDS topic names allow forward slashes (`/`).

> 这是可以接受的，因为 DDS 分区允许包含斜杠（`/`），不像以前的 DDS 主题，但现在 DDS 主题名称允许使用斜杠（`/`）。

DDS partitions are implemented as an array of strings within the `DDS::Publisher` and `DDS::Subscriber` QoS settings and have no hierarchy or order, Each entry in the partition array is directly combined with the DDS topic and they are not sequentially combined.

> DDS 分区在`DDS::Publisher`和`DDS::Subscriber` QoS 设置中实现为一个字符串数组，没有层次结构或顺序，分区数组中的每个条目都直接与 DDS 主题结合，它们不是按顺序组合的。

If a publisher has two partition entries, e.g. `foo` and `bar` with a base name of `baz`, this would be equivalent to having two different publishers on these topics: `/foo/baz` and `/bar/baz`.

> 如果一个发布者有两个分区条目，例如'foo'和'bar'，基本名称为'baz'，这相当于在这些主题上拥有两个不同的发布者：'/foo/baz'和'/bar/baz'。

Therefore this proposal used only one of the strings in the partitions array to hold the entire ROS name's namespace.

> 因此，本提案仅使用分区数组中的一个字符串来保存整个 ROS 名称的命名空间。

You can read more about partitions in RTI's documentation:

> 您可以在 RTI 的文档中阅读更多关于分区的信息：

- [PARTITION_QosPolicy](https://community.rti.com/static/documentation/connext-dds/5.2.3/doc/manuals/connext_dds/html_files/RTI_ConnextDDS_CoreLibraries_UsersManual/Content/UsersManual/PARTITION_QosPolicy.htm)

Trade-offs (in comparison to using the whole ROS name along with the namespaces):

> 折衷（与使用整个 ROS 名称以及名称空间相比）：

- Splitting the ROS name into "namespace" and "base name", and placing the complete namespace into a field designed for another purpose seemed incorrect.

> 将 ROS 名称拆分为“命名空间”和“基本名称”，并将完整的命名空间放入为另一个目的设计的字段中似乎是不正确的。

- In general partitions are recommended to be used as a spare, but using partitions for all ROS names suggested otherwise.

> 一般来说，建议将分区用作备份，但使用分区为所有 ROS 名称提出的另一种建议。

- Major concern was reported in this [issue](https://github.com/ros2/rmw_connext/issues/234), where having two topics with same base name, although different namespace and different types caused problem. For example: topicA is `/camera/data` of type `Image` and topicB is `/imu/data` of type `Imu`. The base names for both topicA and topicB is `data`, generated errors as described in the [issue](https://github.com/ros2/rmw_connext/issues/234).

> 报告了这个[问题](https://github.com/ros2/rmw_connext/issues/234)中的主要担忧，其中具有相同基本名称但不同命名空间和不同类型的两个主题会导致问题。例如：主题 A 是`/camera/data`类型的`Image`，主题 B 是`/imu/data`类型的`Imu`。主题 A 和主题 B 的基本名称均为`data`，如[问题](https://github.com/ros2/rmw_connext/issues/234)中所述，会产生错误。

- Newer standards such as [DDS-XRCE](https://www.omg.org/spec/DDS-XRCE) might not have partitions at all.

> 新一代的标准，比如[DDS-XRCE](https://www.omg.org/spec/DDS-XRCE)可能根本不需要分区。

- Using the complete ROS name in the later strategy will cause a tighter length limit on base name because the DDS topic name would contain ROS prefix, namespace along with the base name which should not exceed DDS topic name limitation which is 256 characters.

> 使用完整的 ROS 名称在后续策略中将会对基础名称的长度产生更紧的限制，因为 DDS 主题名称将包含 ROS 前缀、命名空间以及基础名称，这些都不应超过 DDS 主题名称的限制，也就是 256 个字符。

Rationale:

> .

理由：

- With the decision from the DDS vendors to allow forward slashes (`/`) in DDS topic names, using the complete ROS name seemed simple and more intuitive than using partitions.

> 随着 DDS 供应商允许在 DDS 主题名称中使用正斜杠（'/'），使用完整的 ROS 名称似乎比使用分区更简单，也更直观。

#### Alternative Substitute the Namespace Delimiter

A previous proposal was to substitute the namespace delimiter, i.e. forward slash (`/`), with something that is allowed in DDS topic names, and then only use the DDS topic name to represent the full ROS name.

> .

之前的建议是用 DDS 主题名允许的东西替换命名空间分隔符，即斜杠（`/`），然后只使用 DDS 主题名来表示完整的 ROS 名称。

For example in the simplest case, a topic `/foo/bar/baz` might become `__foo__bar__baz`, where the forward slash (`/`) is being replaced with a double underscore (`__`) and double underscores (`__`) were not allowed in ROS topic and service names.

> 例如，在最简单的情况下，主题“/foo/bar/baz”可能会变成“**foo**bar**baz”，其中斜杠（`/`）被双下划线（`**`）替换，而ROS主题和服务名称中不允许使用双下划线（`\_\_`）。

Trade-offs (in comparison to the use of DDS partitions):

> 与使用 DDS 分区相比，权衡利弊。

- Has a tighter length limit, since it limited by just the DDS topic name and does not benefit from part of the ROS topic name going into the DDS partition.

> 它有一个更紧的长度限制，因为它受 DDS 主题名称的限制，而不能从 ROS 主题名称的一部分进入 DDS 分区中受益。

- The replacement for the forward slash (`/`) had to be more than one character since all usable characters were already in allowed in both the ROS names and DDS topic names, so each namespace further reduced the topic length limit.

> 替换正斜杠（/）的字符必须多于一个，因为 ROS 名称和 DDS 主题名称中已经允许使用所有可用字符，因此每个命名空间都会进一步减少主题长度限制。

- Implementation requires string replacement and validation afterwards, which is moderately complicated.

> 实施需要字符串替换和验证，这是相当复杂的。

Rationale:

> 理由：

The DDS partition proposal is preferred over this alternative because it allows for longer total ROS names and because it is simpler to implement, i.e. splitting the string into base name and namespace is simpler than replacing the forward slashes (`/`) with double underscores (`__`) and then redoing length limit testing afterwards.

> DDS 分区建议优先于这个替代方案，因为它允许更长的 ROS 名称，并且更简单实现，也就是说，将字符串分割成基本名称和命名空间比替换正斜杠（/）为双下划线（\_\_），然后重新进行长度限制测试要简单。

#### Capital Letter Substitution Alternative

This is another alternative that was proposed in the context of the alternative described in the above section called "Alternative Substitute the Namespace Delimiter".

> 这是另一个替代方案，它是在上文中所描述的“替代命名空间分隔符”的背景下提出的。

Since the forward slash (`/`) is replaced with double underscores (`__`) when translating to DDS from ROS topic names, two characters out of the 256 character limit are lost with each additional namespace.

> 由于在从 ROS 主题名称翻译为 DDS 时，正斜杠（/）被双下划线（\_\_）替换，每添加一个命名空间，256 个字符的限制就会损失两个字符。

One proposed alternative was to add a constraint that ROS topic names could not use capital letters, and then capital letters could be used as the stand in for the forward slashes (`/`).

> 一个建议的替代方案是，添加一个约束，即 ROS 主题名称不能使用大写字母，然后可以用大写字母来代替斜杠（`/`）。

Trade-offs:

> 权衡利弊

- Uses one fewer character per namespace and makes it easier to calculate the maximum length of a ROS topic or service name.

> 使用每个命名空间少一个字符，使计算 ROS 主题或服务名称的最大长度变得更容易。

- Prevents users from using capital letters in their names, which is constraining and might be a problem for backwards compatibility with ROS 1 topics and services.

> 防止用户在名字中使用大写字母，这是限制性的，可能会与 ROS 1 主题和服务的向后兼容性存在问题。

Rationale:

> 理由：

Preventing users from using capital letters was too constraining for the added benefit.

> 阻止用户使用大写字母对于额外的好处来说太过于限制性了。

#### ROS Prefix with Single Underscore

This is another variation that was proposed in the context of the alternative described in the above section called "Alternative Substitute the Namespace Delimiter".

> 这是在上一节“替代名称空间分隔符”中提出的另一种变体。

This alternative differs only in that it uses a single underscore in the prefix, i.e. `rt_` rather than `rt__` (`rt` + the leading `/`).

> 这个替代方案的不同之处只在于它在前缀中使用单下划线，即`rt_`而不是`rt__`（`rt`加上前导`/`）。

Trade-offs:

> 权衡：

- Uses one fewer character

> 使用一个字符少一点

- Less consistent with replacement of other forward slashes (`/`)

> 不太一致地替换其它斜杠（/）

Rationale:

> 理由：

Slight preference given to the more consistent alternative.

> 对更加一致的选择稍微偏好一些。

#### Limited Prefixes Alternative

This is another variation that was proposed in the context of the alternative described in the above section called "Alternative Substitute the Namespace Delimiter".

> 这是在上一节中所描述的“替换命名空间分隔符”的另一种变体所提出的。

This alternative would:

> 这个替代方案将：

- not prefix topics

> 不要前缀话题

- optionally prefix other kinds of "implementation detail topics"

> 可选地为其他类型的“实施细节主题”添加前缀

Trade-offs:

> 权衡利弊。

- it would be easier to have ROS subscribe to a DDS created topic

> 这样，ROS 订阅一个 DDS 创建的主题会更容易。

- e.g. DDS publisher on topic `image` could be subscribed to in ROS using just `image`

> 例如，可以使用“image”来订阅 ROS 中的 DDS 发布者，主题为“image”。

- however, the types would need to match as well

> 然而，类型也需要匹配。

- in the current proposal the ROS topic `image` would become `rt__image`, so DDS topics would need to follow the ROS topic conversion scheme to interoperate with ROS components

> 在当前的提案中，ROS 主题`image`将变成`rt__image`，因此 DDS 主题需要遵循 ROS 主题转换方案才能与 ROS 组件进行互操作。

- it would be hard to distinguish ROS created DDS topics and normal DDS topics

> 很难区分 ROS 创建的 DDS 主题和普通 DDS 主题。

- services would still need to be differentiated

> 服务仍然需要区分。

- e.g. service `/foo` would need to make two topics, something like `foo_Request` and `foo_Reply`

> 例如服务'/foo'需要创建两个主题，比如`foo_Request`和`foo_Reply`

Rationale:

> 理由：

Slight preference was given to easily categorizing ROS created topics with DDS created topics over easily connecting to existing DDS topics.

> 对于将 ROS 创建的主题与 DDS 创建的主题进行容易分类而不是容易连接到现有 DDS 主题，略微偏爱。

Connecting to DDS topics could be achieved by having an option when subscribing or publishing to "alias" to an implementation topic name, e.g. something like `sub->alias_to_explicit_topic('dds_topic')`.

> 通过订阅或发布时有一个选项来实现连接到 DDS 主题，即将“别名”指向实现主题名称，例如`sub->alias_to_explicit_topic('dds_topic')`。

Also, the workaround for separating ROS created topics from other DDS topics was considered to be more complicated than the suggested solution of allowing users to specify specific DDS topic names for their publishers and subscriptions.

> 也考虑到，将 ROS 创建的主题与其他 DDS 主题分离的解决方案比允许用户为其发布者和订阅指定特定 DDS 主题名称的建议解决方案更加复杂。

#### Suffix Alternative

This is another variation that was proposed in the context of the alternative described in the above section called "Alternative Substitute the Namespace Delimiter".

> 这是在上面称为“替代替换命名空间分隔符”的部分中提出的另一种变体。

This alternative would:

> 这个替代方案将：

- not prefix topics

> 不要前缀主题

- restructure prefixes to instead be suffixes, i.e. `rX<topic>` -> `<topic>_rX_`

> 将前缀重新组织为后缀，即`rX<主题>`->`<主题>_rX_`

- this would be unique to user defined names because they cannot have a trailing underscore (`_`)

> 这对于用户定义的名称是独一无二的，因为它们不能以下划线（`_`）结尾。

Trade-offs:

> 权衡利弊。

- more difficult to distinguish ROS created DDS topics from normal or built-in DDS topics when listing them using DDS tools because they are not sorted by a ROS specific prefix

> 更难以使用 DDS 工具将 ROS 创建的 DDS 主题与普通或内置的 DDS 主题区分开来，因为它们没有按 ROS 特定的前缀排序。

- if the service name is suffixed again by the DDS vendor (like in Connext's implementation of Request-Reply) then it would be potentially difficult to differentiate from a user's topic name

> 如果服务名称再次由 DDS 供应商后缀（如 Connext 的 Request-Reply 实施），那么可能很难从用户的主题名称中区分出来。

- e.g. service `/foo` might become topics: `foo_s_Request` and `foo_s_Reply` and the user could create a topic called `/foo_s_Request` too.

> 例如，服务“/foo”可能会变成主题：“foo_s_Request”和“foo_s_Reply”，用户也可以创建一个名为“/foo_s_Request”的主题。

- this also applies to any other similar transformations that an RMW implementation might make to the topic

> 这也适用于 RMW 实施中对主题所做的任何其他类似转换。

Rationale:

> 理由：

This alternative was not selected over the prefix solution because of a lack of advantages over the prefix solution.

> 这个替代方案没有被选择，是因为它比前缀解决方案没有优势。

Also, it typically took one more character to express (`rt` versus `_rt_`; unless you also drop the implicit first namespace `/` then it's `rt__` versus `_rt_`) and the potential issues with ambiguity when the DDS implementation handles Request-Reply (added suffixes).

> 此外，表达（`rt`与`_rt_`之间；除非您也取消隐式的第一个命名空间`/`，那么它就是`rt__`与`_rt_`之间）通常需要一个额外的字符，以及 DDS 实现处理请求-回复（添加后缀）时可能出现的歧义问题。

#### Limited Suffix Alternative

This is another variation that was proposed in the context of the alternative described in the above section called "Alternative Substitute the Namespace Delimiter".

> 这是另一种变体，它是在上面一节中所描述的“替代命名空间分隔符”的背景下提出的。

This alternative is the same as the "Suffix Alternative" except:

> 这个替代方案与“后缀替代”相同，但是：

- Topics would not have a suffix or prefix at all

> 主题不会有任何前缀或后缀。

Trade-offs:

> 权衡：

- same trade-offs as the "Suffix Alternative"

> 同样的权衡，如“后缀替代”

- but also easier to have ROS subscribe to a DDS created topic

> 但也更容易让 ROS 订阅到一个 DDS 创建的主题

- e.g. DDS publisher on topic `image` could be subscribed to in ROS using just `image`

> 例如，可以使用“image”在 ROS 中订阅主题为“image”的 DDS 发布者。

- the types would need to match

> 类型需要匹配。

- in the current proposal the ROS topic `image` would become `rt__image`, so DDS topics would need to follow our naming scheme to interoperate with ROS components

> 在当前的提案中，ROS 主题`image`将变成`rt__image`，因此 DDS 主题需要遵循我们的命名规则，以与 ROS 组件进行互操作。

Rationale:

> 理由：

While this alternative provided the benefits of both the suffix and limited prefix alternatives, the rationale for the limited prefix alternative still applies here.

> 虽然这种替代方案提供了后缀和有限前缀替代方案的好处，但有限前缀替代方案仍然适用于此。
