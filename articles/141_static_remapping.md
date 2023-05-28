---
tip: translate by openai@2023-05-28 11:39:41
...
---
    layout: default
    title: Remapping Names
    permalink: articles/static_remapping.html
    abstract:
      Topics, parameters, and services are identified by [Names](http://wiki.ros.org/Names). Names are hard coded in ROS nodes, but they can be changed at runtime through remapping. Without remapping every instance of a node would require changes in code. This article describes the requirements, rationale, and mechanisms for remapping names in ROS 2.
    author: '[Shane Loretz](https://github.com/sloretz)'
    date_written: 2017-03
    last_modified: 2020-03
    published: true
    Authors: {{ page.author }}
    Date Written: {{ page.date_written }}
    Last Modified: {% if page.last_modified %}{{ page.last_modified }}{% else %}{{ page.date_written }}{% endif %}
---

## Why remap names


Remapping names allows reusing the same node executable in different parts of the system.

> 重新映射名称可以让同一个节点可执行文件在系统的不同部分重复使用。

A robot that has multiple sensors of the same type could launch multiple instances of the same node with outputs remapped to different topics.

> 一个拥有多个相同类型传感器的机器人可以启动多个同一节点的实例，并将输出重新映射到不同的主题上。

```
                +-------------+
     (Lidar 1)--> node inst 1 +-->/head_scan
                +-------------+
    
                +-------------+
     (Lidar 2)--> node inst 2 +-->/base_scan
                +-------------+
```

## Structure of a Name


The complete definition of a name is [here](http://design.ros2.org/articles/topic_and_service_names.html).

> 完整的名称定义在[这里](http://design.ros2.org/articles/topic_and_service_names.html)。

It should be read before reading this article.

> 应该在阅读这篇文章之前先去阅读它。

### Quick Summary


If a name begins with `/` it is called a **Fully Qualified Name** (FQN) otherwise it is called a **relative name**.

> 如果一个名字以"/"开头，它就被称为**完全限定名**(FQN)，否则就被称为**相对名**。

The strings between slashes are called **tokens**.

> 在斜杠之间的字符串称为**令牌**。

Names are conceptually divided into two pieces: **namespace** and **basename**.

> 名称概念上分为两部分：**命名空间**和**基本名称**。

The basename is the last token in a name.

> 基本名称是名称中的最后一个标记。

The namespace is everything prior to the basename.

> 命名空间是基本名称之前的一切。

### Example names


- `/foo`

> - `/foo`（/foo）

- `/foo/bar`

> `-`/foo/bar` 的简体中文翻译是：-`/foo/bar`

- `~/foo/bar`

> ~/foo/bar  的简体中文是：~/foo/bar

- `{node}/bar`

> `-`{节点}/bar`

- `bar`

> - 酒吧

## Structure of a Remapping Rule


**Remapping rules** are the instructions describing how a node should change the names it uses.

> **重映射规则**是描述节点应该如何更改其使用的名称的指令。

Remapping rules have two parts.

> 规则重映射有两个部分。

The first part is used to determine if the rule applies to a name.

> 第一部分用于确定规则是否适用于名称。

The second part is the replacement for a matched name.

> 第二部分是用于匹配名称的替换。

The act of replacing one name with another is **remapping**.

> 把一个名字替换成另一个名字的行为叫做重映射。

## ROS 2 Remapping Use cases


These use cases are being considered for remapping in ROS 2:

> 这些用例正在考虑用于ROS 2的重映射：


- Remap One Node in a Process

> 重新映射一个流程中的一个节点

- Change a Namespace

> 更改命名空间

- Change a Basename

> 更改基本名

- Change a Token

> 更改令牌

- Pre-FQN Remapping

> - 预先FQN重映射

- Exact FQN Replacement

> 精确的全限定名替换

- Exact Relative Name Replacement

> 精确的亲属姓名替换

- Remap via Command Line

> 通过命令行重新映射

- Change the Default Namespace

> 更改默认命名空间

- Change the Node Name

> 更改节点名称

- Remap Topic and Service Names Separately

> - 分别重新映射主题和服务名称

### Remap One Node in a Process


This is the ability to apply remap rules to one node in a process without affecting the other nodes.

> 这是指在一个流程中对一个节点应用重映射规则而不影响其他节点的能力。

Because processes in ROS 2 can contain multiple nodes, it is possible multiple nodes in a process may use the same name for different purposes.

> 因为ROS 2中的进程可以包含多个节点，因此一个进程中的多个节点可能会为不同的目的使用相同的名称。

A user may want to change a name used in one node without affecting the rest.

> 一个用户可能想要更改一个节点中使用的名称而不影响其他节点。

### Change a Namespace


Nodes are said to be in a namespace or have a **default namespace**.

> 节点通常被称为处于一个命名空间中或具有**默认命名空间**。

This namespace gets prepended to all relative names used by the node.

> 这个命名空间会被附加到节点使用的所有相对名称上。

This use case is the ability to change the namespace of multiple names with one rule.

> 这个用例是用一条规则更改多个名称的命名空间的能力。


A popular ROS 1 package [actionlib](http://wiki.ros.org/actionlib) creates 5 topics with the same namespace.

> 一个流行的ROS 1包[actionlib](http://wiki.ros.org/actionlib)创建了5个具有相同命名空间的主题。

In ROS 1 remapping an actionlib client or server means creating 5 remapping rules.

> 在ROS 1中，对actionlib客户端或服务器进行重映射意味着创建5条重映射规则。

In ROS 2 just one rule could remap them all.

> 在ROS 2中，只有一条规则可以重映射它们全部。


*Example:*

> 例子：


- Node provides an actionlib server `move_head` and checks a parameter called `move_head`

> 节点提供一个actionlib服务器`move_head`，并检查一个叫做`move_head`的参数。

- A rule remaps namespace `move_head` to `move_head_check_collision`

> 规则将命名空间'move_head'重新映射为'move_head_check_collision'

- All 5 actionlib topics are remapped to `/move_head_check_collision`, but the parameter name remains unchanged

> 所有5个actionlib主题都被重新映射到'/move_head_check_collision'，但参数名称保持不变。

### Change a Basename


This is the ability to change the basename of multiple names with one rule.

> 这是一项能够用一条规则改变多个名字的基本名称的能力。

It's possible a user may want to change multiple instances of a basename to another token.

> 可能有用户希望将多个基本名称更改为另一个令牌。


*Example:*

> 例子：


- A node uses names `/scan/head/scan`, `/base/scan`

> - 一个节点使用名称`/scan/head/scan`，`/base/scan`

- A user wants the node to subscribe to the same data after some processing

> 用户希望在经过一些处理之后，节点能够订阅相同的数据。

- The user remaps basename `scan` to `scan_filtered`

> 用户将基本名称`scan`重新映射为`scan_filtered`。

- The final topics are `/scan/head/scan_filtered`, `/base/scan_filtered`

> 最终的主题是/scan/head/scan_filtered，/base/scan_filtered。

### Change a Token


This is the ability to change a token in multiple names regardless of where it appears.

> 这是一种能力，可以在任何地方将一个令牌更改为多个名称。

It is possible a token is used throughout an interface, but is undesirable to the end user.

> 可能会在界面中使用令牌，但不受最终用户欢迎。

This means it should be possible to make a rule that replaces all uses of this token.

> 这意味着应该有可能制定一个规则，用来替换所有使用这个令牌的地方。


*Example:*

> 例子：


- A company sells a generic mobile robot base with a ROS 2 driver

> 一家公司出售带有ROS 2驱动程序的通用移动机器人底座。

- The driver uses lots of names with the company's name in it: `UmbrellaCorp`

> 司机经常使用包含公司名称`UmbrellaCorp`的名字。

- Another company incorporates the base into their product, and their customers want a ROS 2 interface

> 另一家公司将基础集成到他们的产品中，他们的客户希望有一个ROS 2接口。

- The second company doesn't want their interface to contain `UmbrellaCorp`, so they remap the token to `mobile_base` when launching the driver

> 第二家公司不希望他们的界面包含“UmbrellaCorp”，因此在启动驱动程序时，他们将令牌重新映射为“mobile_base”。

### Pre-FQN Remapping


This is the ability to match a name by how it is used in code.

> 这是按照代码中使用的方式匹配名称的能力。

Doing so requires matching prior to FQN expansion.

> 这样做需要在FQN展开之前进行匹配。

This could be useful when two different names expand to the same FQN.

> 这在两个不同的名称扩展到同一个完全限定名称时可能很有用。


*Example:*

> 例子：


- A node uses two names `cat` and `/ns/cat`

> - 一个节点使用两个名称`cat`和`/ns/cat`

- The node is run in namespace `/ns/`, so the FQN of both names is `/ns/cat`

> 节点运行在名称空间'/ns/'中，因此两个名称的完全限定名称都是'/ns/cat'。

- A rule remaps Pre-FQN expansion `cat` to `lion`

> 规则将预先定义的完整名称展开`cat`重新映射为`lion`。

- The final names are `/ns/lion` and `/ns/cat`

> 最终的名字是/ns/lion和/ns/cat

### Exact FQN Replacement


This is the ability to replace a name by exactly matching it.

> 这是一种能够通过完全匹配来替换名称的能力。

This is part of the behavior of ROS 1 remapping, so it has proven useful and including it will ease the transition to ROS 2.

> 这是ROS 1重映射行为的一部分，因此它已被证明是有用的，包括它将有助于过渡到ROS 2。


*Example:*

> 例子：


- A node uses names `/ns/bar` and `/ns/barista`

> - 一个节点使用名称'/ns/bar'和'/ns/barista'

- A rule is created to remap `/ns/bar` to `/ns/foo`

> 一条规则被创建来重新映射/ns/bar到/ns/foo

- The final names are `/ns/foo` and `/ns/barista`

> 最终的名字是/ns/foo和/ns/barista

### Exact Relative Name Replacement


This allows a user to remap a relative name to another name.

> 这允许用户将相对名称重新映射到另一个名称。

It works by first expanding the relative name and then doing FQN replacement.

> 它首先扩展相对名称，然后进行FQN替换，从而实现工作。

This is also part of ROS 1 remapping.

> 这也是ROS 1重映射的一部分。


*Example:*

> 例子：


- A node is in namespace `/ns/` and uses name `bar`

> 节点位于命名空间/ns/下，使用名称bar

- A rule is created to remap `bar` to `foo`

> 规则被创建以将“bar”重新映射为“foo”。

- The node's use of `bar` is expanded to `/ns/bar`

> 节点对 `bar` 的使用扩展到 `/ns/bar`

- Both sides of the remap rule are expanded to `/ns/bar` and `/ns/foo`

> 两边的重映射规则都扩展到/ns/bar和/ns/foo

- Because the FQN match, the final name is `/ns/foo`

> 因为FQN匹配，最终名称是`/ns/foo`

### Remap via Command Line


A user can supply node specific remapping arguments via the command line.

> 用户可以通过命令行提供特定节点的重新映射参数。

Because a process can contain multiple nodes, there must be a way to uniquely identify a node in a process.

> 因为一个过程可以包含多个节点，所以必须有一种方法来唯一标识一个过程中的节点。

This is a feature of ROS 1 remapping.

> 这是ROS 1重映射的一个功能。


*Example:*

> 例子：


- Node is an executable at `/bin/my_node`

> 节点是一个可执行文件，位于/bin/my_node中。

- User wants to change `/cat` to `/dog`

> 用户想把'/cat'改成'/dog'

- User types `/bin/my_node /cat:=/dog`

> 用户输入`/bin/my_node /cat:=/dog`

### Change the Default Namespace


The default namespace is the one in which relative names get expanded to.

> 默认命名空间是将相对名称扩展为的命名空间。

This should be changeable without affecting FQN.

> 这应该可以在不影响FQN的情况下进行更改。

ROS 1 has this feature using either the environment variable `ROS_NAMESPACE` or the argument `__ns`.

> ROS 1可以使用环境变量ROS_NAMESPACE或参数__ns来使用此功能。


*Example:*

> 例子：


- Node is in namespace `/ns` and uses relative name `bar`

> 节点位于命名空间/ns中，使用相对名称bar

- User changes the default namespace to `/foo`

> 用户将默认命名空间更改为'/foo'

- The final name is `/foo/bar`

> 最终的名字是 "/foo/bar"

### Change the Node Name


The node name is used in log messages and to create private names.

> 节点名称用于日志消息和创建私有名称。

ROS 1 has this feature using the argument `__name`.

> ROS 1 有这个功能，使用参数`__name`。


*Example:*

> 例子：


- Node is named `camera_driver` and uses a private name `camera_info`

> 节点名称为“camera_driver”，使用私有名称“camera_info”。

- User changes the node name to `left_camera_driver`

> 用户将节点名称更改为`left_camera_driver`

- The final name is `/ns/left_camera_driver/camera_info` and log messages use `left_camera_driver`

> 最终的名称是`/ns/left_camera_driver/camera_info`，日志消息使用`left_camera_driver`。

### Remap Topic and Service Names Separately


This is the ability to create a rule that will remap only topics or only services.

> 这是一种能力，可以创建一条规则，仅重新映射主题或仅服务。


*Example:*

> 例子：


- Node subscribes to a topic `/map` and offers a service `/map`

> 节点订阅主题/map，并提供服务/map。

- User changes the topic name to `/map_stream`

> 用户将主题名称更改为“/map_stream”

- The node is subscribed to topic `/map_stream` and offers a service `/map`

> 节点订阅了主题“/map_stream”，并提供服务“/map”

## Remapping Names in ROS 1


Remapping is a feature that also exists in ROS 1.

> 重映射是ROS 1中也存在的一项功能。

In ROS 1 remapping works by passing in [arguments](http://wiki.ros.org/Remapping%20Arguments) to each node.

> 在ROS 1中，重映射通过向每个节点传递[参数](http://wiki.ros.org/Remapping%20Arguments)来实现。

Client libraries also have APIs in code to pass remapping rules when the node is initialized.

> 客户端库也有代码中的API，以在节点初始化时传递重映射规则。

A remap rule consists of two names: one that should be replaced with another.

> 一个重映射规则由两个名称组成：一个应该被另一个替换。


ROS 1 remapping works on **Fully Qualified Names** (FQN).

> ROS 1 的重映射工作是基于**完全限定名称**（FQN）的。

Both sides of a rule are [expanded to FQN](http://docs.ros.org/kinetic/api/roscpp/html/namespaceros_1_1names.html#a377ff8fede7b95398fd5d1c5cd49246b).

> .

两边的规则都被扩展到了完整的限定名（FQN）。

Before a name is remapped it is also [expanded to FQN](http://docs.ros.org/kinetic/api/roscpp/html/namespaceros_1_1names.html#ab2eebaf734abfbdccb4122f8e24f547f).

> 在重新映射一个名字之前，它也会被[扩展为FQN](http://docs.ros.org/kinetic/api/roscpp/html/namespaceros_1_1names.html#ab2eebaf734abfbdccb4122f8e24f547f)。

The name is remapped to the right side only if it exactly matches the left side of a rule.

> 只有当名称与规则的左侧完全匹配时，才会将其重新映射到右侧。

## Remapping rule syntax


This is a proposal for the ROS 2 remapping rule syntax.

> 这是一个关于ROS 2重映射规则语法的提案。

It attempts to be the same as ROS 1 syntax when possible.

> 它尽可能尝试与ROS 1语法保持一致。


Use cases supported by this syntax:

> 支持此语法的用例：


- Remap One Node in a Process

> 重新映射一个流程中的一个节点

- Change a Namespace

> 更改命名空间

- Change a Basename

> 更改基本名

- Exact FQN Replacement

> 精确的全限定名替换

- Exact Relative Name Replacement

> 精确的亲属姓名替换

- Remap via Command Line

> 通过命令行重新映射

- Change the Default Namespace

> .

更改默认命名空间

- Change the Node Name

> 更改节点名称

- Remap Topic and Service Names Separately

> 重新映射主题和服务名称分开。


Not supported:

> 不支持


- Change a Token

> 更改令牌

- Pre-FQN Remapping

> - 预先FQN重映射

### How the Syntax Works


The structure of a remapping rule is `match:=replacement`.

> 翻译规则的结构是：“匹配：=替换”。
`match` tests if a name should be remapped.
`replacement` says what the new name will be.
`:=` behaves the same as it does in ROS 1.


Example rules are:

> 例如规则是：


- `foo:=bar`

> foo := bar  定义为等于

- `/foo/bar:=fiz/buzz`

> - `/foo/bar:=fiz/buzz` 等于翻譯為中文：`/foo/bar := fiz/buzz`

- `nodename:~/foo:=foo`

> nodename：~/foo：=foo

- `**/foo:=\1/bar`

> /- foo:=\1/bar-/ 等于 \1/bar

#### Match Part of a Rule


The match part of a rule uses these operators:

> 规则的匹配部分使用这些运算符：


- `*` matches a single token

> .

`- `*` 匹配单个令牌

- `**` matches zero or more tokens delimited by slashes

> - 匹配由斜杠分隔的零个或多个令牌

- `rosservice://` prefixed to the match makes the rule apply to only services

> `- 加上前缀rosservice://使规则仅适用于服务。`

- `rostopic://` prefixed to the match makes the rule apply to only topics

> `- 加上以“rostopic://”开头的匹配，可以使规则仅适用于主题。`

- `nodename:` prefixed to the match makes it apply only to a node with that name

> `-`节点名称：加上匹配前缀，只能应用于具有该名称的节点。


The operators `*` and `**` are similar to the globbing behavior in bash.

> 运算符 `*` 和 `**` 类似于 bash 中的 globbing 行为。
`**` behaves similar to its use in bash>=4.0 with the globstar option set.


The URL schemes `rosservice://` and `rostopic://` may only be given to topic or service name rules.

> URL方案`rosservice://`和`rostopic://`只能应用于主题或服务名规则。

They may not be prefixed to a node name or namespace replacement rule (`__name`, `__node`, or `__ns`).

> 他们不能被添加到节点名称或命名空间替换规则（`__name`、`__node`或`__ns`）之前。

If both a node name prefix and URL scheme are given, the node name prefix must come first.

> 如果同时提供节点名称前缀和URL方案，则必须先使用节点名称前缀。

`*`, and `**` match whole tokens only.
`*bar` looks like it would match `foobar`, but that would mean matching a partial token.

To avoid confusion they are required to be separated from tokens, substitutions, and each other by a `/`.

> 为了避免混淆，它们需要用斜杠（/）将令牌、替换和彼此分开。

For example `*/bar` `**/*` `~/*` are allowed, but `*bar` `***` `~*` are invalid.

> 例如，允许使用`*/bar` `**/*` `~/*`，但`*bar` `***` `~*`是无效的。


Matching works on FQN only.

> 匹配只针对完全限定名称。

When a name is to be tested the substitution operators (`~` and `{}`) in the name and in the rule are replaced with the content they stand for.

> 当要测试一个名称时，名称中和规则中的替换运算符（`~`和`{}`）将被它们所代表的内容所取代。

Then the name is expanded to a FQN.

> 然后名称被扩展为完全限定名（FQN）。

If the match part of a rule does not begin with `/`, `*`, or `**` it is prefixed with `/namespace/` to make it a FQN.

> 如果规则的匹配部分不以`/`、`*`或`**`开头，则会在其前面添加`/namespace/`以使其成为一个FQN。

Finally the name is compared against the match part of the rule.

> 最后，将名称与规则的匹配部分进行比较。

If the name matches it is remapped.

> 如果名称匹配，它就会被重新映射。

#### Replacement Part of a rule


These special operators are unique to the replacement part of a rule:

> 这些特殊运算符只用于规则的替换部分：


- `\1` - `\9` are replaced with the matched content of a `*` or `**`

> `- \1` - \9` 被用來取代 `*` 或 `**` 的匹配內容


The syntax for `\1` through `\9` was taken from backreferences in POSIX BRE.

> 语法`\1`到`\9`是根据POSIX BRE中的反向引用而采用的。

However, parenthesis are not used; the wild cards always capture.

> 然而，不使用括号；通配符总是能捕获。


These references are required to be separated from tokens by a `/`.

> 这些参考需要用“/”与令牌分开。

When this creates a name with `//` one slash is automatically deleted.

> 当使用'//'创建一个名称时，会自动删除一个斜杠。

For example `**/bar:=/bar/\1` matches the name `/foo/bar` with `**` capturing `/foo`, but the new name is `/bar/foo`.

> 例如，/bar:=/bar/\1 匹配名称/foo/bar，其中\1捕获/foo，但新名称为/bar/foo。


The replacement part of a rule may not have a URL scheme.

> 替换规则的部分不能有URL方案。

This is to avoid a mismatch between the scheme type of the match side and of the replacement side.

> 这是为了避免匹配侧和替换侧的方案类型不匹配。


The substitution operators (`~` and `{}`) are replaced first.

> 替换运算符（`~`和`{}`）会被优先替换。

Afterwards the reference operators are replaced with the matched content.

> 之后，参考操作符将被匹配内容替换。

Then if the replacment name does not begin with `/` it is automatically prefixed with the node's default namespace to make it a FQN.

> 如果替换的名称不以“/”开头，则会自动为该节点的默认命名空间添加前缀，以使其成为完整限定名称（FQN）。

Finally the name is replaced with the replacement.

> 最后，名称被替换了。

For example, `/bar/*:=\1/bar` matches the name `/bar/foo` use by a node with default namespace `/ns` with `*` capturing `foo` and replacement name `/ns/foo/bar`.

> 例如，`/bar/*:=\1/bar` 匹配使用默认命名空间 `/ns` 的节点的名称 `/bar/foo`，其中 `*` 捕获 `foo`，替换名称为 `/ns/foo/bar`。

#### Special Rule for Changing the Default Namespace


The string `__ns` can be given on the match part of a rule to signal a change of the default namespace.

> `__ns` 字符串可以在规则的匹配部分指定，以指示更改默认的命名空间。

On the match side `__ns` must be used by itself or with a `nodename:` prefix.

> 在匹配端，必须单独使用`ns`或者加上`nodename:`前缀。

The replacement side of a rule must have a FQN which will become the new default namespace.

> 替换规则的一边必须具有FQN，这将成为新的默认命名空间。

#### Special Rule for Changing the Node Name


The strings `__name` or  `__node` can be given on the match part of a rule to signal a change of the node's name.

> `__name` 或 `__node` 这些字符串可以在规则的匹配部分提供，以表明节点名称的变更。

On the match side it may be used by itself or with a `nodename:` prefix.

> 在比赛的一边，它可以单独使用，也可以用`nodename：`前缀。

The replacement must be a single token which will become the node's new name.

> 替换必须是一个单独的令牌，它将成为节点的新名称。

#### Order of Applying Remapping Rules


Remapping rules are applied in the following order:

> 重新映射规则按照以下顺序应用：


1. Node name remapping

> 1. 节点名称重新映射

1. Namespace remapping

> 1. 命名空间重映射

1. All other rules

> 1. 所有其他规则


Within each category, the rules are applied in the order in which the user gave them.

> 在每个类别中，规则按用户给出的顺序执行。


**Example of topic/service remapping order:**

> **示例主题/服务重新映射订单：**


- a node uses a name `/foo/bar`

> 一个节点使用名称'/foo/bar'

- a user gives the node a rule `/*/*:=/asdf` and then `/foo/bar:=fizzbuzz`

> 用户给节点一条规则：/*/*:=/asdf，然后/foo/bar:=fizzbuzz。

- the final name is `/asdf` because the name did not match the second rule after being remapped by the first rule

> 最终的名称是`/asdf`，因为在经过第一条规则映射后，名称与第二条规则不匹配。


**Example of node/namespace remapping order:**

> **節點/命名空間重新對應的示例：**


- A node has name `talker`

> 一个节点的名字叫做“talker”。

- A user gives the rules `talker:__ns:=/my_namespace` then `talker:__node:=foo`

> 用户给出规则`talker:__ns:=/my_namespace`然后`talker:__node:=foo`

- The final namespace is the default `/` because the node name remap is applied before the namespace remap

> 最终的命名空间是默认的`/`，因为节点名称重映射在命名空间重映射之前已经应用。


**Example of a default and node specific namespace remap:**

> .

**示例默认和节点特定的命名空间重映射：**


- A node has name `talker`

> 一个节点的名字叫做“talker”

- A user gives the rules `talker:__ns:=/foo` then `__ns:=/bar`

> 用户给出规则`talker:__ns:=/foo` 然后 `__ns:=/bar`

- talker's namespace is `/foo` because that rule was given first

> 说话者的名字空间是/foo，因为首先给出了这个规则。

### Applications of the syntax


The following sections explain how the syntax enables the use cases above.

> 以下部分将解释语法如何实现上述用例。

#### Supporting: Remap One Node in a Process


Remapping a node in a process requires a way to uniquely identify a node.

> 重新映射一个流程中的节点需要一种方式来唯一地标识一个节点。

Assuming the node's name is unique in a process, a rule can be prefixed with the name of the target node and a `:`.

> 假设节点的名称在过程中是唯一的，可以在目标节点的名称前加上一个':'作为前缀。

If the node name is not prefixed, the rule will be applied to all nodes in the process.

> 如果节点名称没有前缀，该规则将应用于流程中的所有节点。


*Example:*

> 例子：


- Multiple nodes in a process use the name `scan`

> 多个节点在一个过程中使用名称“扫描”

- A rule is given to one node `node1:scan:=scan_filtered`

> 给节点node1规定一条规则：scan:=scan_filtered

- Only the node named `node1` uses the rule

> 只有名为node1的节点使用该规则

#### Supporting: Change a Namespace


There are two cases: changing part of a namespace, and changing the entire namespace.

> 有两种情况：更改命名空间的一部分，以及更改整个命名空间。

The first case requires a wildcard to match the rest of a namespace.

> 第一种情况需要使用通配符来匹配命名空间的其余部分。

The second requires a wildcard to match the basename at the end.

> 第二个需要一个通配符来匹配结尾的基本名称。


*Example of partial namespace replacement:*

> *部分命名空间替换的示例：*


- Node uses names `/foo`, `/foo/bar`, `/foo/bar/baz`

> - 节点使用名称`/foo`, `/foo/bar`, `/foo/bar/baz`

- Node given rule `/foo/**:=/fizz/\1`

> -给定规则“/foo/**:=/fizz/\1”的节点

- Resulting names `/foo`, `/fizz/bar`, `/fizz/bar/baz`

> 结果名称：/foo、/fizz/bar、/fizz/bar/baz


*Example of full namespace replacement:*

> *示例完整的命名空间替换：*


- Node uses names `/foo/bar/baz`, `/foo/bar/fee/biz`

> - 节点使用名称'/foo/bar/baz'、'/foo/bar/fee/biz'

- Node given rule `/foo/bar/*:=/bar/foo/\1`

> - 给定规则`/foo/bar/*:=/bar/foo/\1` 的节点

- Resulting names `/bar/foo/baz`, `/foo/bar/fee/biz`

> 结果名称：/bar/foo/baz，/foo/bar/fee/biz

#### Supporting: Change a Basename


Changing a basename requires a wildcard which matches the entire namespace.

> 更改基本名称需要一个与整个命名空间匹配的通配符。

The wildcard `**` is useful because it matches every possible namespace when combined with a slash.

> "通配符'**'很有用，因为它与斜杠结合时可以匹配所有可能的命名空间。"


*Example:*

> 例子：


- Node uses names `/foo`, `/buz/foo`, `/biz/buz/foo`

> -节点使用名称'/foo'、'/buz/foo'、'/biz/buz/foo'

- Node given rule `**/foo:=\1/bar`

> -给定规则 `/foo:=\1/bar`

- Resulting names `/bar`, `/buz/bar`, `/biz/buz/bar`

> 结果名称：/bar、/buz/bar、/biz/buz/bar

#### Supporting: Exact FQN Replacement


Exact FQN replacement requires no wildcards.

> 精确的全限定名替换不需要通配符。

This syntax is identical to ROS 1.

> 这种语法与ROS 1完全相同。


*Example rules:*

> *示例规则：*


- `/foo/bar:=/fiz/buz`

> - `/foo/bar:=fiz/buz` 的简体中文翻译是：/foo/bar:=fiz/buz

- `/foo:=/foo/bar`

> - `/foo:=/foo/bar` 的简体中文翻译为：/foo:=/foo/bar

#### Supporting: Exact Relative Name Replacement


Exact relative replacement also requires no wildcards.

> .

精确的相对替换也不需要通配符。

It means relative names are first expanded to FQN, and then processed as during exact FQN replacement.

> 这意味着相对名称会首先被扩展为完全限定名称（FQN），然后按照完全限定名称（FQN）替换的方式进行处理。

This syntax is identical to ROS 1.

> 这种语法与ROS 1完全相同。


*Example rules:*

> *示例规则：*


- `foo:=/foo/bar` in namespace `/ns` is identical to `/ns/foo:=/foo/bar`

> - 在命名空間`/ns`中，`foo:=/foo/bar`等同於`/ns/foo:=/foo/bar`

- `foo:=bar` in namespace `/ns` is identical to `/ns/foo:=/ns/bar`

> '- `foo:=bar` 在命名空間`/ns`中等同於 `/ns/foo:=/ns/bar`

- `/foo/bar:=foo` in namespace `/ns` is identical to `/foo/bar:=/ns/foo`

> `- `/foo/bar:=foo` 在命名空间`/ns`中等同于 `/foo/bar:=/ns/foo`

#### Supporting: Remap via Command Line


The syntax here can be passed to a node via the command line.

> 这里的语法可以通过命令行传递给一个节点。

The syntax has been chosen to not conflict with special shell characters in bash.

> 语法已选择不与bash中的特殊shell字符发生冲突。

For example in bash, the character `*` only has special behavior if it is surrounded by whitespace, but remap rules don't contain whitespace.

> 例如在bash中，字符`*`只有在被空格包围时才会有特殊的行为，但重映射规则中不包含空格。

This character may still be difficult on other shells, like zsh.

> 这个字符可能在其他shell，比如zsh上仍然很难。

#### Supporting: Change the Default Namespace


This isn't really a remapping rule, but the syntax is similar.

> 这不是一个真正的重映射规则，但语法是相似的。

In ROS 1 the argument `__ns:=` could change the default namespace.

> 在ROS 1中，参数`__ns:=`可以更改默认的命名空间。

Here the syntax is the same, and additionally it can be prefixed with a node's name.

> 在这里，语法相同，此外可以使用节点名称作为前缀。

The replacement side must have a FQN with no special operators.

> 替换方必须有一个不带特殊运算符的FQN（全限定名）。

All relative names are expanded to the new namespace before any remapping rules are applied to them.

> 所有相关名称在应用任何重映射规则之前，都会扩展到新的命名空间。


*Examples:*

> *例子：*


- `__ns:=/new/namespace`

> - `__ns:=/新/命名空间`

- `node1:__ns:=/node1s/new/namespace`

> - node1:__ns:=/node1s/新的/命名空间

#### Supporting: Change the Node Name


This also isn't a true remapping rule, but the syntax is similar.

> 这也不是一个真正的重新映射规则，但语法相似。

In ROS 1 the argument `__name:=` could change the node's name.

> 在ROS 1中，参数`__name:=`可以改变节点的名称。

Here the syntax is the same, and additionally it can be prefixed with a node's current name.

> 在这里，语法相同，此外可以用节点当前名称作为前缀。

The argument `__node:=` has the same effect.

> 参数`__node:=`有同样的效果。

The replacement side must have a single token.

> 替换方必须有一个唯一的令牌。

Log messages use the new name immediately.

> 立即使用新名称来记录消息。

All private names are expanded to the new name before any remapping rules are applied to them.

> .

所有私有名称在应用任何重映射规则之前都会被扩展为新名称。


*Examples:*

> *例子：*


- `__name:=left_camera_driver`

> 名称：左摄像头驱动程序

- `__node:=left_camera_driver`

> `node:=左侧摄像头驱动程序`

- `camera_driver:__name:=left_camera_driver`

> - camera_driver: 名称:=左摄像头驱动程序

#### Supporting: Remap Topic and Service Names Separately

Specifying a URL scheme on the match side of the rule makes it exclusive to one type of name.

> 指定一个URL方案在规则的匹配侧使其专属于一种名称。

If no URL scheme is given then the rule applies to both topics and services.

> 如果没有给出URL方案，那么该规则适用于主题和服务。


*Examples:*

> *例子：*


- `rosservice:///foo/bar:=/bar/foo`

> rosservice:///foo/bar:=bar/foo

- `rostopic://foo/bar:=bar/foo`

> rostopic://foo/bar:=bar/foo 的简体中文翻译是：rostopic://foo/bar:=bar/foo

- `nodename:rosservice://~/left:=~/right`

> - nodename:rosservice://~/左边:=~/右边

#### Not Supporting: Change a Token


The syntax can't change all uses of a token with one rule.

> 不能用一条规则就改变所有使用的令牌的语法。

Supporting this use case with a single rule is not a priority.

> 这种用例只用一条规则支持不是重点。


*Workaround using two rules*

> 使用两条规则的应急方案


- First rule remaps token used in namespace `**/foobar/**:=\1/fizzbuz/\2`

> 第一条规则重新映射在命名空间**/foobar/**中使用的令牌:=\1/fizzbuz/\2

- Second rule remaps token used as basename `**/foobar:=\1/fizzbuz`

> 第二条规则将令牌映射为基本名称`/foobar:=\1/fizzbuz`

#### Not Supporting: Pre-FQN Remapping


The syntax doesn't have a way to specify that a rule should be applied Prior to FQN expansion.

> 语法没有指定规则在FQN展开之前应用的方法。

There is no workaround.

> 没有解决办法。

### Fnmatch Syntax


A syntax like fnmatch is being considered.

> 正在考虑一种类似fnmatch的语法。

The character for the wild card `*` was chosen to match fnmatch.

> 字符"*"被选为通配符，以匹配fnmatch。

Because remapping needs to capture text to use during replacement, the C function `fnmatch()` cannot be used as the implementation.

> 由于重新映射需要捕获用于替换的文本，所以C函数`fnmatch()`无法用作实现。

The extra wildcards `?` and `[]` don't appear to enable more uses cases above.

> 额外的通配符`?`和`[]`似乎没有使上述用例增加更多用途。

Fnmatch syntax may or may not match text with slashes depending on the option `FNM_PATHNAME`.

> 根据选项FNM_PATHNAME，Fnmatch语法可能会或可能不会匹配带斜杠的文本。

## Static Versus Dynamic Remapping


Static remapping is giving a node remapping rules at the time it is launched.

> 静态重映射是在节点启动时给它提供重映射规则。

Dynamic remapping is the ability to remap a name while a node is running.

> 动态重映射是指在节点运行时能够重新映射名称的能力。

It may be useful for a developer who has started a node and wants to connect it to a different source.

> 它可能对一个已经启动了节点并想要将其连接到不同源的开发者有用。

Because the user will see the name after it has been remapped by static rules, dynamic rules should be applied after static ones.

> 由于用户在静态规则重映射后才能看到名称，因此应该先应用静态规则，然后再应用动态规则。

This way the new rule matches against the name the user sees with introspection tools rather than the original name used in code.

> 这样，新规则与用户通过内省工具看到的名称匹配，而不是在代码中使用的原始名称。
