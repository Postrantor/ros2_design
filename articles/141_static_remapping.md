---
tip: translate by openai@2023-05-30 22:55:57
layout: default
title: Remapping Names
permalink: articles/static_remapping.html
abstract:
  Topics, parameters, and services are identified by [Names](http://wiki.ros.org/Names). Names are hard coded in ROS nodes, but they can be changed at runtime through remapping. Without remapping every instance of a node would require changes in code. This article describes the requirements, rationale, and mechanisms for remapping names in ROS 2.
author: '[Shane Loretz](https://github.com/sloretz)'
date_written: 2017-03
last_modified: 2020-03
published: true

Authors: 
Date Written: 
Last Modified:
---
## Why remap names

Remapping names allows reusing the same node executable in different parts of the system. A robot that has multiple sensors of the same type could launch multiple instances of the same node with outputs remapped to different topics.

> 重新映射名称可以在系统的不同部分重复使用相同的节点可执行文件。一个拥有**多个相同类型传感器的机器人可以启动多个相同节点的实例**，并将输出重新映射到不同的主题上。

```
          +-------------+
(Lidar 1)--> node inst 1 +-->/head_scan
          +-------------+

          +-------------+
(Lidar 2)--> node inst 2 +-->/base_scan
          +-------------+
```

## Structure of a Name

The complete definition of a name is [here](http://design.ros2.org/articles/topic_and_service_names.html). It should be read before reading this article.

> 完整的名称定义在[这里](http://design.ros2.org/articles/topic_and_service_names.html)。在阅读本文之前应该先阅读它。

### Quick Summary

If a name begins with `/` it is called a **Fully Qualified Name** (FQN) otherwise it is called a **relative name**. The strings between slashes are called **tokens**. Names are conceptually divided into two pieces: **namespace** and **basename**. The basename is the last token in a name. The namespace is everything prior to the basename.

> 如果一个名称以“/”开头，它就被称为**完全限定名称**(FQN)，否则就称为**相对名称**。斜杠之间的字符串被称为**令牌**。概念上将名称分为两部分：**命名空间**和**基本名**。基本名是名称中的最后一个令牌。命名空间是基本名之前的所有内容。

### Example names

- `/foo`
- `/foo/bar`
- `~/foo/bar`
- `/bar`
- `bar`

## Structure of a Remapping Rule

**Remapping rules** are the instructions describing how a node should change the names it uses. Remapping rules have two parts. The first part is used to determine if the rule applies to a name. The second part is the replacement for a matched name. The act of replacing one name with another is **remapping**.

> **重映射规则**是指描述节点应如何更改其使用的名称的指令。重映射规则有两部分。第一部分用于确定规则是否适用于某个名称。第二部分是匹配名称的替换内容。将一个名称替换为另一个名称的行为称为**重映射**。

## ROS 2 Remapping Use cases

These use cases are being considered for remapping in ROS 2:

> 这些用例正在考虑在 ROS 2 中重新映射：

- Remap One Node in a Process
- Change a Namespace
- Change a Basename
- Change a Token
- Pre-FQN Remapping
- Exact FQN Replacement
- Exact Relative Name Replacement
- Remap via Command Line
- Change the Default Namespace
- Change the Node Name
- Remap Topic and Service Names Separately

### Remap One Node in a Process

This is the ability to apply remap rules to one node in a process without affecting the other nodes. Because processes in ROS 2 can contain multiple nodes, it is possible multiple nodes in a process may use the same name for different purposes. A user may want to change a name used in one node without affecting the rest.

> 这是一种能够将重映射规则应用于进程中的一个节点而不会影响其他节点的能力。由于 ROS 2 中的进程可以包含多个节点，因此可能有多个节点在进程中使用相同的名称用于不同的目的。用户可能想要更改一个节点中使用的名称而不影响其余节点。

### Change a Namespace

Nodes are said to be in a namespace or have a **default namespace**. This namespace gets prepended to all relative names used by the node. This use case is the ability to change the namespace of multiple names with one rule.

> 节点通常被认为处于一个名称空间或具有**默认名称空间**。此名称空间会被添加到节点使用的所有相对名称的前面。此用例是能够用一条规则改变多个名称的名称空间。

A popular ROS 1 package [actionlib](http://wiki.ros.org/actionlib) creates 5 topics with the same namespace. In ROS 1 remapping an actionlib client or server means creating 5 remapping rules. In ROS 2 just one rule could remap them all.

> 一个流行的 ROS 1 包 [actionlib](http://wiki.ros.org/actionlib) 创建了 5 个具有相同命名空间的主题。在 ROS 1 中，重新映射 actionlib 客户端或服务器意味着创建 5 个重新映射规则。在 ROS 2 中，只需一条规则就可以重新映射它们。

_Example:_

- Node provides an actionlib server `move_head` and checks a parameter called `move_head`
- A rule remaps namespace `move_head` to `move_head_check_collision`
- All 5 actionlib topics are remapped to `/move_head_check_collision`, but the parameter name remains unchanged

### Change a Basename

This is the ability to change the basename of multiple names with one rule. It's possible a user may want to change multiple instances of a basename to another token.

> 这是一种能够用一条规则改变多个名称的基本名称的能力。有可能用户想要将多个实例的基本名称更改为另一个令牌。

_Example:_

- A node uses names `/scan/head/scan`, `/base/scan`
- A user wants the node to subscribe to the same data after some processing
- The user remaps basename `scan` to `scan_filtered`
- The final topics are `/scan/head/scan_filtered`, `/base/scan_filtered`

### Change a Token

This is the ability to change a token in multiple names regardless of where it appears. It is possible a token is used throughout an interface, but is undesirable to the end user. This means it should be possible to make a rule that replaces all uses of this token.

> 这是一种能力，可以将一个标记改变为多个名称，而不管它出现在哪里。有可能一个标记被用在整个界面上，但对最终用户来说是不受欢迎的。这意味着应该有可能制定一条规则，替换掉所有使用这个标记的地方。

_Example:_

- A company sells a generic mobile robot base with a ROS 2 driver
- The driver uses lots of names with the company's name in it: `UmbrellaCorp`
- Another company incorporates the base into their product, and their customers want a ROS 2 interface
- The second company doesn't want their interface to contain `UmbrellaCorp`, so they remap the token to `mobile_base` when launching the driver

### Pre-FQN Remapping

This is the ability to match a name by how it is used in code. Doing so requires matching prior to FQN expansion. This could be useful when two different names expand to the same FQN.

> 这是一种按照代码中的使用方式匹配名称的能力。在 FQN 扩展之前进行匹配是必须的。当两个不同的名称扩展到相同的 FQN 时，这可能很有用。

_Example:_

- A node uses two names `cat` and `/ns/cat`
- The node is run in namespace `/ns/`, so the FQN of both names is `/ns/cat`
- A rule remaps Pre-FQN expansion `cat` to `lion`
- The final names are `/ns/lion` and `/ns/cat`

### Exact FQN Replacement

This is the ability to replace a name by exactly matching it. This is part of the behavior of ROS 1 remapping, so it has proven useful and including it will ease the transition to ROS 2.

> 这是指通过完全匹配来替换一个名称的能力。这是 ROS 1 重映射行为的一部分，因此已经证明是有用的，包括它将有助于过渡到 ROS 2。

_Example:_

- A node uses names `/ns/bar` and `/ns/barista`
- A rule is created to remap `/ns/bar` to `/ns/foo`
- The final names are `/ns/foo` and `/ns/barista`

### Exact Relative Name Replacement

This allows a user to remap a relative name to another name. It works by first expanding the relative name and then doing FQN replacement. This is also part of ROS 1 remapping.

> 这允许用户将相对名称重新映射到另一个名称。它通过首先展开相对名称，然后执行 FQN 替换来工作。这也是 ROS 1 重映射的一部分。

_Example:_

- A node is in namespace `/ns/` and uses name `bar`
- A rule is created to remap `bar` to `foo`
- The node's use of `bar` is expanded to `/ns/bar`
- Both sides of the remap rule are expanded to `/ns/bar` and `/ns/foo`
- Because the FQN match, the final name is `/ns/foo`

### Remap via Command Line

A user can supply node specific remapping arguments via the command line. Because a process can contain multiple nodes, there must be a way to uniquely identify a node in a process. This is a feature of ROS 1 remapping.

> 用户可以通过命令行提供特定节点的重映射参数。因为一个进程可以包含多个节点，必须有一种方法来唯一标识一个进程中的节点。这是 ROS 1 重映射的一个特性。

_Example:_

- Node is an executable at `/bin/my_node`
- User wants to change `/cat` to `/dog`
- User types `/bin/my_node /cat:=/dog`

### Change the Default Namespace

The default namespace is the one in which relative names get expanded to. This should be changeable without affecting FQN. ROS 1 has this feature using either the environment variable `ROS_NAMESPACE` or the argument `__ns`.

> 默认命名空间是相对名称扩展的命名空间。这应该可以改变而不影响 FQN。ROS 1 有这个功能，使用环境变量 `ROS_NAMESPACE` 或参数 `__ns`。

_Example:_

- Node is in namespace `/ns` and uses relative name `bar`
- User changes the default namespace to `/foo`
- The final name is `/foo/bar`

### Change the Node Name

The node name is used in log messages and to create private names. ROS 1 has this feature using the argument `__name`.

> 节点名称用于日志消息和创建私有名称。ROS 1 具有使用参数 `__name` 的此功能。

_Example:_

- Node is named `camera_driver` and uses a private name `camera_info`
- User changes the node name to `left_camera_driver`
- The final name is `/ns/left_camera_driver/camera_info` and log messages use `left_camera_driver`

### Remap Topic and Service Names Separately

This is the ability to create a rule that will remap only topics or only services.

> 这是一种能力，可以创建一个规则，只重新映射主题或只服务。

_Example:_

- Node subscribes to a topic `/map` and offers a service `/map`
- User changes the topic name to `/map_stream`
- The node is subscribed to topic `/map_stream` and offers a service `/map`

## Remapping Names in ROS 1

Remapping is a feature that also exists in ROS 1. In ROS 1 remapping works by passing in [arguments](http://wiki.ros.org/Remapping%20Arguments) to each node. Client libraries also have APIs in code to pass remapping rules when the node is initialized. A remap rule consists of two names: one that should be replaced with another.

> 重映射也存在于 ROS 1 中。在 ROS 1 中，重映射通过向每个节点传递[参数]来实现。客户端库还具有在初始化节点时传递重映射规则的代码 API。重映射规则由两个名称组成：一个应该替换为另一个。

ROS 1 remapping works on **Fully Qualified Names** (FQN). Both sides of a rule are [expanded to FQN](http://docs.ros.org/kinetic/api/roscpp/html/namespaceros_1_1names.html#a377ff8fede7b95398fd5d1c5cd49246b). Before a name is remapped it is also [expanded to FQN](http://docs.ros.org/kinetic/api/roscpp/html/namespaceros_1_1names.html#ab2eebaf734abfbdccb4122f8e24f547f). The name is remapped to the right side only if it exactly matches the left side of a rule.

> ROS 1 的重新映射工作在完全限定名(FQN)上。规则的两边都会被[扩展到 FQN]。在一个名字被重新映射之前，它也会被[扩展到 FQN]。只有当它完全匹配规则左边时，才会将名字重新映射到右边。

## Remapping rule syntax

This is a proposal for the ROS 2 remapping rule syntax. It attempts to be the same as ROS 1 syntax when possible.

> 这是一个 ROS 2 重映射规则语法的提案。尽可能地与 ROS 1 语法保持一致。

Use cases supported by this syntax:

- Remap One Node in a Process
- Change a Namespace
- Change a Basename
- Exact FQN Replacement
- Exact Relative Name Replacement
- Remap via Command Line
- Change the Default Namespace
- Change the Node Name
- Remap Topic and Service Names Separately

Not supported:

- Change a Token
- Pre-FQN Remapping

### How the Syntax Works

The structure of a remapping rule is `match:=replacement`. `match` tests if a name should be remapped. `replacement` says what the new name will be. `:=` behaves the same as it does in ROS 1.

> 翻译规则的结构是 `match:=replacement`。`match` 测试是否需要重新映射名称。`replacement` 表示新名称将是什么。`:=` 在 ROS 1 中的行为与其相同。

Example rules are:

- `foo:=bar`
- `/foo/bar:=fiz/buzz`
- `nodename:~/foo:=foo`
- `**/foo:=\1/bar`

#### Match Part of a Rule

The match part of a rule uses these operators:

> 规则的匹配部分使用这些运算符：

- `*` matches a single token
- `**` matches zero or more tokens delimited by slashes
- `rosservice://` prefixed to the match makes the rule apply to only services
- `rostopic://` prefixed to the match makes the rule apply to only topics
- `nodename:` prefixed to the match makes it apply only to a node with that name

The operators `*` and `**` are similar to the globbing behavior in bash. `**` behaves similar to its use in bash>=4.0 with the globstar option set.

> 运算符 `*` 和 `**` 类似于 bash 中的 globbing 行为。`**` 的行为类似于 bash>=4.0 中设置了 globstar 选项时的行为。

The URL schemes `rosservice://` and `rostopic://` may only be given to topic or service name rules. They may not be prefixed to a node name or namespace replacement rule (`__name`, `__node`, or `__ns`). If both a node name prefix and URL scheme are given, the node name prefix must come first.

> URL 方案 `rosservice://` 和 `rostopic://` 只能给主题或服务名规则使用，不能加在节点名或命名空间替换规则(`__name`，`__node` 或 `__ns`)之前。如果给出了节点名前缀和 URL 方案，则节点名前缀必须先出现。

`*`, and `**` match whole tokens only. `*bar` looks like it would match `foobar`, but that would mean matching a partial token. To avoid confusion they are required to be separated from tokens, substitutions, and each other by a `/`. For example `*/bar` `**/*` `~/*` are allowed, but `*bar` `***` `~*` are invalid.

Matching works on FQN only. When a name is to be tested the substitution operators (`~` and ``) in the name and in the rule are replaced with the content they stand for. Then the name is expanded to a FQN. If the match part of a rule does not begin with `/`, `*`, or `**` it is prefixed with `/namespace/` to make it a FQN. Finally the name is compared against the match part of the rule. If the name matches it is remapped.

> 匹配仅适用于 FQN。当要测试名称时，名称和规则中的替换运算符(“~”和“｛｝”)将替换为它们所代表的内容。然后将名称扩展为 FQN。如果规则的匹配部分不是以“/”、“\*”或“\*\*”开头，则会以“/namespace/”为前缀，使其成为 FQN。最后，将名称与规则的匹配部分进行比较。如果名称匹配，则会重新映射。

#### Replacement Part of a rule

These special operators are unique to the replacement part of a rule:

> 这些特殊运算符对于规则的替换部分是唯一的：

- `\1` - `\9` are replaced with the matched content of a `*` or `**`

The syntax for `\1` through `\9` was taken from backreferences in POSIX BRE. However, parenthesis are not used; the wild cards always capture.

> 从“\1”到“\9”的语法取自 POSIX BRE 中的反向引用。但是，不使用括号；外卡总是能抓住。

These references are required to be separated from tokens by a `/`. When this creates a name with `//` one slash is automatically deleted. For example `**/bar:=/bar/\1` matches the name `/foo/bar` with `**` capturing `/foo`, but the new name is `/bar/foo`.

> 这些引用需要与标记用“/”分隔。当创建一个带有“//”的名称时，会自动删除一个斜线。例如，“**/bar:=/bar/\1”将名称“/foo/bar”与“**` 捕获“/foo”匹配，但新名称为“/bar/foo”。

The replacement part of a rule may not have a URL scheme. This is to avoid a mismatch between the scheme type of the match side and of the replacement side.

> 规则的替换部分可能没有 URL 方案。这是为了避免匹配侧和替换侧的方案类型之间的不匹配。

The substitution operators (`~` and ``) are replaced first. Afterwards the reference operators are replaced with the matched content. Then if the replacment name does not begin with `/` it is automatically prefixed with the node's default namespace to make it a FQN. Finally the name is replaced with the replacement. For example, `/bar/*:=\1/bar` matches the name `/bar/foo` use by a node with default namespace `/ns` with `*` capturing `foo` and replacement name `/ns/foo/bar`.

> 替换运算符(“~”和“｛｝”)首先被替换。然后，引用运算符被匹配的内容替换。然后，如果 replacment 名称不是以“/”开头，则会自动以节点的默认名称空间为前缀，使其成为 FQN。最后，名称被替换。例如，“/bar/_：=\1/bar”将具有默认名称空间“/ns”的节点使用的名称“/bar/foo”与“_”捕获“foo”和替换名称“/ns/foo/bar”相匹配。

#### Special Rule for Changing the Default Namespace

The string `__ns` can be given on the match part of a rule to signal a change of the default namespace. On the match side `__ns` must be used by itself or with a `nodename:` prefix. The replacement side of a rule must have a FQN which will become the new default namespace.

> 字符串“**ns”可以在规则的匹配部分给出，以表示默认名称空间的更改。在匹配方面，“**ns”必须单独使用或带有“nodename:”前缀。规则的替换端必须有一个 FQN，该 FQN 将成为新的默认命名空间。

#### Special Rule for Changing the Node Name

The strings `__name` or `__node` can be given on the match part of a rule to signal a change of the node's name. On the match side it may be used by itself or with a `nodename:` prefix. The replacement must be a single token which will become the node's new name.

> 字符串“**name”或“**node”可以在规则的匹配部分给出，以表示节点名称的更改。在匹配方面，它可以自己使用，也可以带有“nodename:”前缀。替换必须是将成为节点新名称的单个令牌。

#### Order of Applying Remapping Rules

Remapping rules are applied in the following order:

> 重新映射规则按以下顺序应用：

1. Node name remapping

> 1.节点名称重映射

2. Namespace remapping

> 2.命名空间重映射

3. All other rules

> 3.所有其他规则

Within each category, the rules are applied in the order in which the user gave them.

> 在每个类别中，规则都是按照用户给出规则的顺序应用的。

**Example of topic/service remapping order:**

> **主题/服务重新映射顺序示例：**

- a node uses a name `/foo/bar`
- a user gives the node a rule `/*/*:=/asdf` and then `/foo/bar:=fizzbuzz`
- the final name is `/asdf` because the name did not match the second rule after being remapped by the first rule

**Example of node/namespace remapping order:**

> **节点/命名空间重新映射顺序示例：**

- A node has name `talker`
- A user gives the rules `talker:__ns:=/my_namespace` then `talker:__node:=foo`
- The final namespace is the default `/` because the node name remap is applied before the namespace remap

**Example of a default and node specific namespace remap:**

> **默认和特定于节点的命名空间重映射示例：**

- A node has name `talker`
- A user gives the rules `talker:__ns:=/foo` then `__ns:=/bar`
- talker's namespace is `/foo` because that rule was given first

### Applications of the syntax

The following sections explain how the syntax enables the use cases above.

> 以下部分解释语法如何启用上述用例。

#### Supporting: Remap One Node in a Process

Remapping a node in a process requires a way to uniquely identify a node. Assuming the node's name is unique in a process, a rule can be prefixed with the name of the target node and a `:`. If the node name is not prefixed, the rule will be applied to all nodes in the process.

> 重新映射流程中的节点需要一种唯一标识节点的方法。假设节点的名称在进程中是唯一的，则可以在规则前面加上目标节点的名称和“：”。如果节点名称没有加前缀，则该规则将应用于进程中的所有节点。

_Example:_

- Multiple nodes in a process use the name `scan`
- A rule is given to one node `node1:scan:=scan_filtered`
- Only the node named `node1` uses the rule

#### Supporting: Change a Namespace

There are two cases: changing part of a namespace, and changing the entire namespace. The first case requires a wildcard to match the rest of a namespace. The second requires a wildcard to match the basename at the end.

> 有两种情况：更改部分名称空间和更改整个名称空间。第一种情况需要一个通配符来匹配命名空间的其余部分。第二个需要一个通配符来匹配末尾的基名称。

_Example of partial namespace replacement:_

- Node uses names `/foo`, `/foo/bar`, `/foo/bar/baz`
- Node given rule `/foo/**:=/fizz/\1`
- Resulting names `/foo`, `/fizz/bar`, `/fizz/bar/baz`

_Example of full namespace replacement:_

- Node uses names `/foo/bar/baz`, `/foo/bar/fee/biz`
- Node given rule `/foo/bar/*:=/bar/foo/\1`
- Resulting names `/bar/foo/baz`, `/foo/bar/fee/biz`

#### Supporting: Change a Basename

Changing a basename requires a wildcard which matches the entire namespace. The wildcard `**` is useful because it matches every possible namespace when combined with a slash.

> 更改基名称需要一个与整个命名空间匹配的通配符。通配符“\*\*”很有用，因为它与斜杠组合时匹配所有可能的命名空间。

_Example:_

- Node uses names `/foo`, `/buz/foo`, `/biz/buz/foo`
- Node given rule `**/foo:=\1/bar`
- Resulting names `/bar`, `/buz/bar`, `/biz/buz/bar`

#### Supporting: Exact FQN Replacement

Exact FQN replacement requires no wildcards. This syntax is identical to ROS 1.

> 确切的 FQN 替换不需要通配符。此语法与 ROS 1 相同。

_Example rules:_

- `/foo/bar:=/fiz/buz`
- `/foo:=/foo/bar`

#### Supporting: Exact Relative Name Replacement

Exact relative replacement also requires no wildcards. It means relative names are first expanded to FQN, and then processed as during exact FQN replacement. This syntax is identical to ROS 1.

> 精确的相对替换也不需要通配符。这意味着相对名称首先扩展到 FQN，然后在确切的 FQN 替换过程中进行处理。此语法与 ROS 1 相同。

_Example rules:_

- `foo:=/foo/bar` in namespace `/ns` is identical to `/ns/foo:=/foo/bar`
- `foo:=bar` in namespace `/ns` is identical to `/ns/foo:=/ns/bar`
- `/foo/bar:=foo` in namespace `/ns` is identical to `/foo/bar:=/ns/foo`

#### Supporting: Remap via Command Line

The syntax here can be passed to a node via the command line. The syntax has been chosen to not conflict with special shell characters in bash. For example in bash, the character `*` only has special behavior if it is surrounded by whitespace, but remap rules don't contain whitespace. This character may still be difficult on other shells, like zsh.

> 这里的语法可以通过命令行传递给节点。语法的选择是为了不与 bash 中的特殊 shell 字符冲突。例如，在 bash 中，字符“\*”只有在被空白包围时才具有特殊行为，但重映射规则不包含空白。这个角色在其他 shell 上可能仍然很难，比如 zsh。

#### Supporting: Change the Default Namespace

This isn't really a remapping rule, but the syntax is similar. In ROS 1 the argument `__ns:=` could change the default namespace. Here the syntax is the same, and additionally it can be prefixed with a node's name. The replacement side must have a FQN with no special operators. All relative names are expanded to the new namespace before any remapping rules are applied to them.

> 这实际上不是一个重新映射规则，但语法是相似的。在 ROS 1 中，参数“__ns:=”可以更改默认名称空间。这里的语法是相同的，另外它可以以节点的名称为前缀。更换侧必须有一个 FQN，没有特殊的操作员。在对所有相对名称应用任何重新映射规则之前，所有相对名称都将扩展到新的名称空间。

_Examples:_

- `__ns:=/new/namespace`
- `node1:__ns:=/node1s/new/namespace`

#### Supporting: Change the Node Name

This also isn't a true remapping rule, but the syntax is similar. In ROS 1 the argument `__name:=` could change the node's name. Here the syntax is the same, and additionally it can be prefixed with a node's current name. The argument `__node:=` has the same effect. The replacement side must have a single token. Log messages use the new name immediately. All private names are expanded to the new name before any remapping rules are applied to them.

> 这也不是一个真正的重映射规则，但语法是相似的。在 ROS 1 中，参数“**name:=”可能会更改节点的名称。这里的语法是相同的，此外，它可以以节点的当前名称为前缀。参数“**node:=”具有相同的效果。替换方必须有一个令牌。日志消息立即使用新名称。在对所有私有名称应用任何重新映射规则之前，所有私有名称都将扩展为新名称。

_Examples:_

- `__name:=left_camera_driver`
- `__node:=left_camera_driver`
- `camera_driver:__name:=left_camera_driver`

#### Supporting: Remap Topic and Service Names Separately

Specifying a URL scheme on the match side of the rule makes it exclusive to one type of name. If no URL scheme is given then the rule applies to both topics and services.

> 在规则的匹配端指定 URL 方案使其仅限于一种类型的名称。如果没有给出 URL 方案，则该规则适用于主题和服务。

_Examples:_

- `rosservice:///foo/bar:=/bar/foo`
- `rostopic://foo/bar:=bar/foo`
- `nodename:rosservice://~/left:=~/right`

#### Not Supporting: Change a Token

The syntax can't change all uses of a token with one rule. Supporting this use case with a single rule is not a priority.

> 语法不能用一条规则改变一个令牌的所有使用。用一条规则支持这个用例并不是一个优先事项。

_Workaround using two rules_

- First rule remaps token used in namespace `**/foobar/**:=\1/fizzbuz/\2`
- Second rule remaps token used as basename `**/foobar:=\1/fizzbuz`

#### Not Supporting: Pre-FQN Remapping

The syntax doesn't have a way to specify that a rule should be applied Prior to FQN expansion. There is no workaround.

> 语法无法指定规则应在 FQN 扩展之前应用。没有变通办法。

### Fnmatch Syntax

A syntax like fnmatch is being considered. The character for the wild card `*` was chosen to match fnmatch. Because remapping needs to capture text to use during replacement, the C function `fnmatch()` cannot be used as the implementation. The extra wildcards `?` and `[]` don't appear to enable more uses cases above. Fnmatch syntax may or may not match text with slashes depending on the option `FNM_PATHNAME`.

> 正在考虑使用类似 fnmatch 的语法。通配符“\*”的字符被选择为匹配 fnmatch。因为重映射需要捕获文本以在替换过程中使用，所以不能将 C 函数“fnmatch()”用作实现。额外的通配符 `？` 和“[]”似乎无法启用以上更多的用例。Fnmatch 语法可能匹配带斜线的文本，也可能不匹配，具体取决于选项“FNM_PATHNAME”。

## Static Versus Dynamic Remapping

Static remapping is giving a node remapping rules at the time it is launched. Dynamic remapping is the ability to remap a name while a node is running. It may be useful for a developer who has started a node and wants to connect it to a different source. Because the user will see the name after it has been remapped by static rules, dynamic rules should be applied after static ones. This way the new rule matches against the name the user sees with introspection tools rather than the original name used in code.

> 静态重映射是在节点启动时为其提供重映射规则。动态重新映射是在节点运行时重新映射名称的功能。对于已经启动节点并希望将其连接到不同源的开发人员来说，这可能很有用。因为用户将在名称被静态规则重新映射后看到该名称，所以应该在静态规则之后应用动态规则。这样，新规则将与用户使用内省工具看到的名称相匹配，而不是与代码中使用的原始名称相匹配。
