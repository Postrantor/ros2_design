---
tip: translate by openai@2023-05-29 22:53:21
...
---
    layout: default
    title: ROS Command Line Arguments
    permalink: articles/ros_command_line_arguments.html
    abstract: This article describes ROS 2 nodes command line arguments and their syntax.
    author: "[Michel Hidalgo](https://github.com/hidmic)"
    date_written: 2019-09
    last_modified: 2021-08
    published: true
    Authors: {{ page.author }}
    Date Written: {{ page.date_written }}
    Last Modified: {% if page.last_modified %}{{ page.last_modified }}{% else %}{{ page.date_written }}{% endif %}
---

## Overview


As it was the case in ROS 1, ROS 2 nodes allow configuration via command line arguments to a certain degree. In ROS 2, this interface had to become more complex to cope with a larger set of configuration options, an ambiguity in remapping rules and parameter assignment syntax (as a result of the leading underscore name convention for hidden resources), a one-to-many relationship between executables and nodes, to name a few.

> 在ROS 1中，ROS 2节点允许通过命令行参数进行配置，在一定程度上。在ROS 2中，这个接口必须变得更加复杂，以应对更大范围的配置选项，重映射规则和参数分配语法（由于隐藏资源的前导下划线命名约定）之间的歧义，可执行文件与节点之间的一对多关系，等等。


Because of this, increasingly precise addressing mechanisms as well as leading double underscores (`__`) in some positional arguments, both natural extensions of existing ROS 1 command line features, are combined with ROS 2 specific command line flags. Flags, in contrast with other custom syntax alternatives, are:

> 因此，将越来越精确的地址机制以及一些位置参数中的双下划线（`__`）（ROS 1命令行功能的自然扩展）与ROS 2特定的命令行标志结合起来。与其他自定义语法替代方案相比，标志是：

- Widely known and used.
- Explicit and less error prone.
- Suitable for tab completion.


Unfortunately, since these flags coexist with user-defined ones, additional guarding and extraction devices must be put in place -- one of the reasons why these were avoided entirely in ROS 1 command lines.

> 不幸的是，由于这些标志与用户定义的标志共存，因此必须采取额外的保护和提取设备——这也是ROS 1命令行完全避免使用这些标志的原因之一。

## Features

### Namespacing


To prevent ROS specific command line flags from colliding with user-defined ones, the former are scoped using the `--ros-args` flag and a trailing double dash token (`--`):

> 为了防止ROS特定的命令行标志与用户定义的标志发生冲突，前者使用`--ros-args`标志和一个尾随双短划线令牌（`--`）进行范围限定：

```sh
    ros2 run some_package some_node [<user-defined-arg-0>...<user-defined-arg-N>] \
      --ros-args [<ros-specific-arg-0>...] -- [<user-defined-arg-N+1>...]
```


> [NOTE]:

If no user defined arguments are provided after ROS specific arguments are, the double dash token (`--`) may be elided:

> 如果在ROS特定参数之后没有提供用户定义参数，可以省略双破折号（`--`）。

```sh
    ros2 run some_package some_node [<user-defined-arg-0>...] --ros-args [<ros-specific-arg-0>...]
```


> [NOTE]:

More than one set of ROS specific flags may appear in the same command line:

> 在同一条命令行中可以出现多组ROS特定标志：

```sh
    ros2 run some_package some_node --ros-args [<ros-specific-arg-0>...<ros-specific-arg-N>] -- \
      [<user-defined-arg-0>...] --ros-args [<ros-specific-arg-N+1>...]
```


This way, multiple sources, potentially unaware of each other, can append flags to the command line with no regard for previous sets.

> 这样，多个潜在不知道彼此的源可以在不考虑以前设置的情况下向命令行添加标志。

### Capabilities

#### Summary


As a quick summary of ROS command line capabilities:

> 作为ROS命令行功能的快速摘要：

- For name remapping, use either `--remap from:=to` or `-r from:=to`.
- For single parameter assignment, use either `--param name:=value` or `-p name:=value` where value is in YAML format.
- For multiple parameter assignments, use `--params-file path/to/file.yaml` and a parameters YAML file.
- For setting logging (minimum) level, use `--log-level LEVEL_NAME`.
- For external logging configuration, use `--log-config-file path/to/file.config` and a log configuration file.
- For enabling/disabling logging:
  - to `rosout`, use `--enable-rosout-logs` or `--disable-rosout-logs`
  - to `stdout`, use `--enable-stdout-logs` or `--disable-stdout-logs`
  - to a external logging library, use `--enable-external-lib-logs` or `--disable-external-lib-logs`
- For enclave assignment, use either `--enclave value` or `-e value` where value is fully qualified enclave path.


For name remapping and parameter assignment, specific nodes can be targeted by prepending the option value with the node name followed by a colon `:`, as in `--remap my_node:from:=to` and `--param my_node:name:=value`.

> 为了重新映射名称和分配参数，可以在选项值前面加上节点名称，后面跟一个冒号（:），如 `--remap my_node:from:=to` 和 `--param my_node:name:=value`。

#### Name remapping rules


Remapping rules may be introduced using the `--remap`/`-r` option. This option takes a single `from:=to` remapping rule.

> 使用`--remap`/`-r`选项可以引入重映射规则。此选项需要单个`from:=to`重映射规则。


As an example, to remap from `foo` to `bar` for `some_ros_executable`, one may execute:

> 举个例子，要把`foo`重新映射到`bar`，用于`some_ros_executable`，可以执行：

```sh
    ros2 run some_package some_ros_executable --ros-args --remap foo:=bar
```


or its shorter equivalent:

> .

或者它的简短的等价物：

```sh
    ros2 run some_package some_ros_executable --ros-args -r foo:=bar
```


As is, this remapping rule applies to each and every node that `some_ros_executable` spawns unless explicitly ignored in code. To limit it to `some_node`, one may execute:

> 这个重新映射规则默认适用于`some_ros_executable`生成的每个节点，除非在代码中明确忽略。要将其限制为`some_node`，可以执行：

```sh
    ros2 run some_package some_ros_executable --ros-args -r some_node:foo:=bar
```

#### Single parameter assignments


Parameter assignment may be achieved using the `--param`/`-p` option. This option takes a single `name:=value` assignment statement, where `value` is in [YAML format](https://yaml.org/spec/) and thus YAML type inference rules apply.

> 可以使用`--param`/`-p`选项来实现参数赋值。此选项接受单个`name:=value`赋值语句，其中`value`的格式为[YAML格式](https://yaml.org/spec/)，因此应用YAML类型推断规则。


As an example, to assign a string value `test` to a parameter `string_param` for `some_ros_executable`, one may execute:

> 举个例子，为`some_ros_executable`的参数`string_param`分配字符串值`test`，可以执行：

```sh
    ros2 run some_package some_ros_executable --ros-args --param string_param:=test
```


or its shorter equivalent:

> .

或其简短的等效版本：

```sh
    ros2 run some_package some_ros_executable --ros-args -p string_param:=test
```


As is, this parameter assignment applies to each and every node that `some_ros_executable` spawns unless explicitly ignored in code. To limit it to `some_node`, one may execute:

> 如果没有在代码中明确忽略，此参数赋值将应用于`some_ros_executable`生成的每个节点。要将其限制在`some_node`上，可以执行：

```sh
    ros2 run some_package some_ros_executable --ros-args -p some_node:string_param:=test
```

#### Multiple parameter assignments


Multiple parameter assignments can be performed at once using the `--params-file` option. This option takes a [YAML](https://yaml.org/spec/) file with the following structure:

> 可以使用`--params-file`选项一次执行多个参数分配。此选项采用具有以下结构的[YAML](https://yaml.org/spec/)文件：

```yaml
    node0_name:
      ros__parameters:
         param0_name: param0_value
         ...
         paramN_name: paramN_value
    ...
    nodeM_name:
      ros__parameters:
         ...
```


Multiple nodes in a single executable can be targeted this way. Note that YAML type inference rules for parameter values apply.

> 多个节点可以通过单个可执行文件进行定位。注意，YAML类型推断规则适用于参数值。


As an example, to assign a string value `foo` to a parameter `string_param` for `some_node` and a string value `bar` to that same parameter `string_param` but for `another_node` upon running `some_ros_executable` that contains both, one may execute:

> 举个例子，为`some_node`的参数`string_param`分配一个字符串值`foo`，为`another_node`的同一个参数`string_param`分配一个字符串值`bar`，在运行包含这两个的`some_ros_executable`时，可以执行：

```sh
    ros2 run some_package some_ros_executable --ros-args --params-file params_file.yaml
```


where `params_file.yaml` reads:

> 在params_file.yaml中读取：

```yaml
    some_node:
      ros__parameters:
        string_param: foo
    another_node:
      ros__parameters:
        string_param: bar
```


Wildcards can be used for node names and namespaces as described in [Remapping Names](static_remapping.html#match-part-of-a-rule). `*` matches a single token delimeted by slashes (`/`). `**` matches zero or more tokens delimeted by slashes. Partial matches are not allowed (e.g. `foo*`).

> 通配符可以用于节点名称和命名空间，如[重映射名称](static_remapping.html#match-part-of-a-rule)所述。`*`匹配由斜杠（`/`）分隔的单个标记。`**`匹配由斜杠分隔的零个或多个标记。不允许部分匹配（例如`foo*`）。


For example,

> 例如，

```yaml
    /**:
      ros__parameters:
        string_param: foo
```


will set the parameter `string_param` on all nodes,

> 在所有节点上设置参数'string_param'

```yaml
    /**/some_node:
      ros__parameters:
        string_param: foo
```


will set the parameter `string_param` on nodes named `some_node` in any namespace,

> 在任何命名空间中，将参数`string_param`设置在名为`some_node`的节点上。

```yaml
    /foo/*:
      ros__parameters:
        string_param: foo
```


will set the parameter `string_param` on any node in the namespace `/foo`.

> 將參數`string_param`設置在命名空間`/foo`中的任何節點上。

#### Logging level assignments


Minimum logging level can be externally set either globally or per logger using the `--log-level` option.

> 最低日志级别可以使用`--log-level`选项全局或按日志记录器外部设置。


As an example, to set a global logging level to `DEBUG` for `some_ros_executable`, one may execute:

> 举个例子，要将全局日志级别设置为“DEBUG”，可以执行以下操作：`some_ros_executable`

```sh
    ros2 run some_package some_ros_executable --ros-args --log-level DEBUG
```


Loggers can be set using the `--log-level` option as well:

> 可以使用 `--log-level` 选项来设置日志记录器：

```sh
    ros2 run some_package some_ros_executable --ros-args --log-level talker1:=DEBUG --log-level talker2:=WARN --log-level rclcpp:=DEBUG
```


The minimum logging level of a specific logger will override the globally specified minimum logger level. If a logging level is specified more than once in the passed command line arguments, the last one prevails.

> 特定的日志记录级别将覆盖全局指定的最低日志记录级别。如果在传入的命令行参数中指定了多次日志级别，则最后一次的日志级别起作用。


See `rcutils` and `rcl` logging documentation for reference on existing logging levels.

> 参考现有的日志级别，请查看`rcutils`和`rcl`日志文档。

#### External logging configuration


External logging may be configured using the `--log-config-file` option. This option takes a single configuration file, whose format depends on the actual external logging library being used.

> 外部日志可以使用`--log-config-file`选项进行配置。 该选项需要一个配置文件，其格式取决于实际使用的外部日志库。


As an example, to pass `some_log.config` configuration file to `some_ros_executable`, one may execute:

> 举个例子，要将`some_log.config`配置文件传递给`some_ros_executable`，可以执行：

```sh
    ros2 run some_package some_ros_executable --ros-args --log-config-file some_log.config
```

#### Enabling and disabling logging


Logging to `rosout`, `stdout` and an external logging library can be independently enabled or disabled.

> 登录到rosout、stdout和外部日志库可以独立地启用或禁用。


As an example, to disable logging to `rosout` and `stdout` but not to an external logging library for `some_ros_executable`, one may execute:

> 举个例子，如果要禁用对`rosout`和`stdout`的日志记录，但不禁用对`some_ros_executable`的外部日志库的记录，可以执行：

```sh
    ros2 run some_package some_ros_executable --ros-args --disable-rosout-logs --disable-stdout-logs --enable-external-lib-logs
```


Logging is fully enabled by default, thus `--enable-*` options are usually redundant unless a `--disable-*` option found earlier in the command line is being overridden.

> 日志默认已完全启用，因此除非在命令行中更早发现了`--disable-*`选项，否则`--enable-*`选项通常是多余的。

#### Enclave assignments


Enclave assignment may be achieved using the `--enclave`/`-e` option. This option takes a single string `value` assignment statement, where `value` is a fully qualified enclave path used to locate the respective security artifacts within the configured keystore.

> 可以使用`--enclave`/`-e`选项来完成隔离区分配。此选项采用单个字符串`value`分配语句，其中`value`是用于在配置的密钥库中查找相应安全项的完全限定的隔离区路径。


As an example, to assign an enclave path `/foo/bar` one may execute:

> 举个例子，要指定一个飞地路径/foo/bar，可以执行：

```sh
    ros2 run some_package some_ros_executable --ros-args --enclave="/foo/bar"
```


or its shorter equivalent:

> .

或其简短的等效物。

```sh
    ros2 run some_package some_ros_executable --ros-args -e "/foo/bar"
```


As is, this enclave assignment applies to each and every Domain Participant that `some_ros_executable` spawns unless explicitly ignored in code or overridden via security environment variables.

> 此前，除非在代码中明确忽略或通过安全环境变量覆盖，否则此私密空间分配适用于`some_ros_executable`产生的每个域参与者。

## Implementation

### Extraction


Command line argument extraction happens within `rcl`. When an instance of the `--ros-args` flag is found in `argv`, until either a double dash token (`--`) is found or the end of the argument array is reached, all arguments that follow are taken as ROS specific arguments to be parsed as such. Remaining arguments can still be accessed by the user via `rcl` API.

> 当在argv中发现`--ros-args`标志的实例时，命令行参数提取发生在rcl中。直到发现双破折号标记（`--`）或到达参数数组的末尾，所有随后的参数都被视为ROS特定参数，并按此解析。用户仍可通过rcl API访问剩余参数。

### Parsing


At the time of writing, most ROS specific arguments target and are thus parsed by `rcl`. This is the case for name remapping rules or parameter assignments flags, to name a few. However, to support ROS specific arguments that target upper ROS layers e.g. a ROS client library like `rclcpp`, arguments unknown to `rcl` are left unparsed but accessible by these layers, which in turn can continue parsing or eventually warn the user if unknown arguments remain.

> 在写作的时候，大多数ROS特定的参数都是针对`rcl`的，因此由其解析。例如，名称重映射规则或参数分配标志等。但是，为了支持针对更高层的ROS层（例如ROS客户端库`rclcpp`）的ROS特定参数，`rcl`未知的参数将保持未解析，但这些层可以访问它们，然后继续解析，或者最终如果未知参数仍然存在则警告用户。

## Alternative designs


Other, alternative designs were under discussion.

> 其他替代设计正在讨论中。

### Additional operators


Stop using the same `:=` operator for parameter assignments and name remapping rules and introduce additional operators e.g. `:=` for parameter assignment and `~=` for name remapping. This keeps the command line verbosity at a minimum and avoids the need for flags, but is error prone.

> 停止使用相同的`:=`操作符来进行参数赋值和名称重映射规则，并引入额外的操作符，例如`:=`用于参数赋值，`~=`用于名称重映射。这样可以将命令行的冗长性降到最低，避免使用标志，但容易出错。

### Full name addressing


Rely on full name addressing to disambiguate operator significance e.g. `rosparam://this:=that` would result in a `that` string value being assigned to parameter `this` while `rosremap://this:=that` would result in name `this` being remapped to name `that`. Other URL schemes, specific to each interface type e.g. `rostopic` and `rosservice`, may also be used to further scope remapping rules. This signficantly increases command line verbosity, but still avoids the need for flags.

> 依靠全名地址来消除操作符的意义，例如`rosparam://this:=that`将会导致`that`字符串值被赋值给参数`this`，而`rosremap://this:=that`则会导致`this`名称被重新映射到`that`名称。其他专属于每种接口类型的URL方案，如`rostopic`和`rosservice`，也可以用于进一步范围重新映射规则。这显著增加了命令行的冗长性，但仍然避免了使用标志的需要。

### Prefixed option names


Remove the need for double dash tokens (`--`), conventionally used to signify the end of CLI options for a command, by adding the `--ros-` prefix to all ROS specific command line flags e.g. `--ros-remap`, `--ros-param`, etc. In exchange, it makes argument extraction slightly more difficult as all options must be known ahead of time, whereas `--ros-args`-based namespacing can achieve the same with a couple rules. It also increases command line verbosity.

> 移除对双破折号令牌（`--`）的需求，这种令牌通常用于表示命令行选项的结束，通过在所有ROS特定的命令行标志前加上`--ros-`前缀，例如`--ros-remap`，`--ros-param`等。作为交换，它使参数提取变得更加困难，因为所有选项必须提前知道，而基于`--ros-args`的命名空间可以通过几条规则达到同样的效果。它也会增加命令行的冗长度。
