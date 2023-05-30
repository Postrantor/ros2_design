---
tip: translate by openai@2023-05-30 23:02:16
layout: default
title: ROS 2 Launch XML Format
permalink: articles/roslaunch_xml.html
abstract:
  The XML format for declarative launch descriptions in the ROS 2 launch system.
author: '[Michel Hidalgo](https://github.com/hidmic)'
date_written: 2019-09
last_modified: 2021-06
published: true
Authors: {{ page.author }}
Date Written: {{ page.date_written }}
Last Modified: {% if page.last_modified %}{{ page.last_modified }}{% else %}{{ page.date_written }}{% endif %}
---

# ROS 2 Launch XML Format v0.1.0

## Rationale

As an alternative to a programmatic approach to the ROS 2 launch system's API, a declarative description features a WYSIWYG approach, easier to read, audit and maintain. This is the preferred approach for ROS 1 `roslaunch` launch files, thus some degree of familiarity is expected (and relied upon).

> 作为 ROS 2 启动系统 API 的编程方法的替代方案，声明式描述具有 WYSIWYG 方法，更容易阅读，审计和维护。这是 ROS 1 `roslaunch`启动文件的首选方法，因此有一定程度的熟悉度是期望的（并依赖于）。

The body of such a description is mainly comprised of statically declared launch actions with a prescribed configuration. To that, runtime value substitution is added in order to fullfill common dynamic (re)configuration needs like conditional execution, resource lookups, etc. It is intended for these entities to map to those of the underlying implementation, reducing support to file parsing.

> 这种描述的主体主要由具有规定配置的静态声明启动操作组成。为此，添加了运行时值替换，以满足条件执行、资源查找等常见的动态(重)配置需求。其目的是将这些实体映射到底层实现，减少对文件解析的支持。

This article describes XML aiming to ease the bridge between ROS and ROS 2 launch files. YAML is currently supported too, and other markup languages could be added.

> 本文介绍了 XML，旨在简化 ROS 和 ROS2 启动文件之间的桥梁。目前也支持 YAML，还可以添加其他标记语言。

## Static Description

### Schema Definition

```xml
    {% include_relative specs/launch.0.1.1.xsd %}
```

### Tags Semantics

#### All Tags

Every tag's execution can be conditioned on a boolean predicate via `if` or `unless` attributes.

> 每个标记的执行都可以通过“if”或“除非”属性以布尔谓词为条件。

#### `<launch>` Tag

Root tag of any launch file. There must only be one `<launch>` tag on a given launch file.

> 任何启动文件的根标记。给定的启动文件上只能有一个“＜ launch ＞”标记。

#### `<include>` Tag

##### Description

The `<include>` tag allows for bringing a launch file description into another, enabling re-use of hierarchical launch layouts. The included launch file description has its own scope for launch configurations. The included launch file description is not necessarily written in this format nor a declarative one (i.e. a programmatic description).

> “＜ include ＞”标记允许将启动文件描述引入另一个文件中，从而实现分层启动布局的重用。所包含的启动文件描述有其自己的启动配置范围。所包含的启动文件描述不一定是以这种格式编写的，也不一定是声明性的（即编程描述）。

##### Examples

```xml
    <include file="/opt/my_launch_file.py"/>
    <include file="$(find-pkg-share my_pkg)/launch/some_launch_file.xml"/>
    <include file="/opt/my_other_launch_file.xml">
      <arg name="some_argument" value="dummy_value"/>
    </include>
```

#### `<group>` Tag

##### Description

The `<group>` tag allows for launch actions' grouping as well as optional launch file configuration scoping.

> “＜ group ＞”标记允许启动操作分组以及可选的启动文件配置范围。

##### Examples

```xml
    <group scoped="true">
      <node pkg="a_ros_package" name="dummy0" namespace="my_ns" exec="dummy_node"/>
      <node pkg="a_ros_package" name="dummy1" exec="dummy_node"/>
    </group>
```

#### `<let>` Tag

##### Description

The `<let>` tags allows for definition of scoped launch file configuration variables.

> “＜ let ＞”标记允许定义作用域启动文件配置变量。

##### Examples

```xml
    <let name="foo" value="$(env BAR)"/>
    <let name="baz" value="false"/>
```

#### `<arg>` Tag

##### Description

The `<arg>` tag allows for launch file configuration via the command-line or when including it via an `<include>` tag. Arguments are launch configuration variables just like the ones defined by `<let>` tags. Arguments are limited to the scope of their definition and thus have to be explictly passed to included files if any.

> “＜ arg ＞”标记允许通过命令行或通过“＜ include ＞”标签包含启动文件配置。参数是启动配置变量，与“＜ let ＞”标记定义的变量一样。参数仅限于其定义的范围，因此必须明确地传递给包含的文件（如果有的话）。

##### Examples

```xml
    <arg name="publish_frequency" default="10"/>
    <arg name="output_path" description="Output path for some processing pipeline"/>
```

#### `<executable>` Tag

##### Description

The `<executable>` tag allows for executing any executable as a local OS process.

> “＜ executable ＞”标记允许将任何可执行文件作为本地操作系统进程执行。

##### Examples

```xml
    <executable cmd="ls -las" cwd="/home" launch-prefix="time" output="screen"/>
```

#### `<node>` Tag

##### Description

The `<node>` tag allows for executing a ROS node as a local OS process.

> “＜ node ＞”标记允许将 ROS 节点作为本地 OS 进程执行。

##### Examples

```xml
    <node pkg="ros_demos" exec="publisher">
      <param name="publish_frequency" value="10"/>
      <remap from="generic_topic_name" to="my_topic"/>
    </node>
```

#### `<param>` Tag

##### Description

The `<param>` tag allows for setting ROS parameters of a ROS node. These can be scalar values or sequences of scalar values, defined directly or either nested or brought from a YAML file to make a map.

> “＜ param ＞”标签允许设置 ROS 节点的 ROS 参数。这些可以是标量值或标量值序列，可以直接定义，也可以嵌套或从 YAML 文件中引入以生成映射。

##### Examples

```xml
<node pkg="my_pkg" exec="my_node">
  <param name="some_numeric_param" value="100.2"/>
  <param name="some_list_of_bools" value="[true, false, true, false]"/>
  <param name="some_list_of_strings" value="Some phrase,'100.0','true'" value-sep=","/>
  <param name="some_list_of_ints" value="5, 3, 2" value-sep=", "/>
  <param name="some_list_of_floats" value="5.0, 3.0, 2.0" value-sep=", "/>
  <param name="some_param_group">
    <param name="some_integer_param" value="10"/>
  </param>
  <param from="path/to/param/file.yml"/>
</node>
```

#### `<remap>` Tag

##### Description

The `<remap>` tag allows for ROS name remapping of a `<node>` instance.

> “＜ remap ＞”标记允许对“＜ node ＞”实例进行 ROS 名称重新映射。

##### Examples

```xml
    <node pkg="my_pkg" exec="my_node">
      <remap from="chatter" to="/my_chatter"/>
      <remap from="*/stuff" to="private_\1/stuff"/>
    </node>
```

#### `<env>` Tag

##### Description

The `<env>` tag allows for modifying an OS process environment. It can be used nested in `node` or `executable` tags. It doesn't allow conditional execution.

> `<env>`标记允许修改操作系统进程环境。它可以嵌套在“node”或“executable”标记中使用。它不允许有条件执行。

##### Examples

```xml
    <node pkg="my_pkg" exec="my_node">
      <env name="RMW_IMPLEMENTATION" value="rmw_fastrtps_cpp"/>
    </node>

    <executable cmd="ls" cwd="/var/log">
      <env name="LD_LIBRARY" value="/lib/some.so"/>
    </executable>
```

#### `<set_env>` Tag

##### Description

The `<set_env>` tag allows for modifying an OS process environment. It can be used nested in `launch` or `group` tags. It allows conditional execution.

> “＜ set_env ＞”标记允许修改操作系统进程环境。它可以嵌套在“启动”或“组”标记中使用。它允许有条件执行。

##### Examples

```xml
    <group>
      <set_env name="RMW_IMPLEMENTATION" value="rmw_fastrtps_cpp"/>
    </group>
```

#### `<unset_env>` Tag

##### Description

The `<unset_env>` tag allows for deleting an OS process environment variable. It can be used nested in `launch` or `group` tags. It allows conditional execution.

> “＜ unset_env ＞”标记允许删除操作系统进程环境变量。它可以嵌套在“启动”或“组”标记中使用。它允许有条件执行。

##### Examples

```xml
    <group>
      <unset_env name="MY_ENV_VAR"/>
    </group>
```

## Dynamic Configuration

### Substitution Syntax

All substitutions are enclosed by `$(...)`.

> 所有替换都用“$（…）”括起来。

### Built-in Substitutions

`$(find-pkg-prefix <pkg-name>)` : Substituted by the install prefix path of the given package. Forward and backwards slashes will be resolved to the local filesystem convention. Substitution will fail if the package cannot be found.

`$(find-pkg-share <pkg-name>)` : Substituted by the share directory path of the given package. The share directory includes the package's name, e.g. `<prefix>/share/<pkg-name>`. Forward and backwards slashes will be resolved to the local filesystem convention. Substitution will fail if the package cannot be found.

`$(find-exec <exec-name>)` : Substituted by the path to the executable in the local filesystem. Executables are looked up in the PATH environment variable. Forward and backwards slashes will be resolved to the local filesystem convention. Substitution will fail if the executable cannot be found.

`$(exec-in-package <exec-name> <package-name>)` : Substituted by the path to the executable in the local filesystem. Executables are looked up in the `lib` directory of the package. Substitution will fail if the executable cannot be found.

`$(var <name>)` : Substituted by the value of the launch configuration variable. Substitution will fail if the named argument does not exist.

`$(env <env-var> [default-value])` : Substituted by the value of the given environment variable Substitution will fail if the variable is not set, unless a default value is provided.

`$(eval <python-expression>)` : Substituted by the evaluated python expression. Substitution will fail if python fails to evaluate the expression.

`$(dirname)` : Substituted by the current launch file directory name. Substitution will always succeed.

#### User-defined Substitutions

TBD

> 待定
