---
tip: translate by openai@2023-05-30 08:24:58
layout: default
title: Interface definition using .msg / .srv / .action files
permalink: articles/legacy_interface_definition.html
abstract:
  This article specifies the file format coming from ROS 1 describing the data structures exchanged by ROS components to interact with each other.
published: true
author: '[Dirk Thomas](https://github.com/dirk-thomas)'
date_written: 2019-03
last_modified: 2021-02
categories: Interfaces
Authors: {{ page.author }}
Date Written: {{ page.date_written }}
Last Modified: {% if page.last_modified %}{{ page.last_modified }}{% else %}{{ page.date_written }}{% endif %}
---
## Scope

This article specifies the file format coming from ROS 1 describing the data structures which are being used to exchange information between components. The data structures are defined in a programming language agnostic way. The format is based on the [<code>.msg</code> format definition](http://wiki.ros.org/msg#Message_Description_Specification) from ROS 1.

> 这篇文章指定了来自 ROS 1 的文件格式，用于描述用于在组件之间交换信息的数据结构。这些数据结构以与编程语言无关的方式定义。该格式基于 ROS 1 中的[<code>.msg</code>格式定义](http://wiki.ros.org/msg#Message_Description_Specification)。

Below only the mapping to IDL types is described. Please see the [Interface Definition and Language Mapping](idl_interface_definition.html) article for the mappings to programming language specific types and API.

> 以下只描述了映射到 IDL 类型。请参阅[接口定义和语言映射](idl_interface_definition.html)文章，了解映射到特定编程语言类型和 API 的内容。

## Overview

A data structure is defined by a set of _fields_. The order of the fields is irrelevant. Each field is described by a _type_ and a _name_.

> 数据结构由一组字段定义。字段的顺序无关紧要。每个字段由类型和名称描述。

### Messages

A single data structure is called _message_. Each message has a _name_. Together with the name of the _package_ a message can be uniquely identified.

> 一个单独的数据结构称为*消息*。每条消息都有一个*名字*。通过包的名字，可以唯一地标识一条消息。

### Services

For request / reply style communication the two exchanged data structures are related. These pairs of data structures are called _services_. A service is identified by its _name_ and the _package_ it is in. Each service describes two messages, one for the request data structure, one for the reply data structure.

> 对于请求/回复式的通信，交换的数据结构是相关的。这些数据结构对被称为*服务*。服务通过其*名称*和*包*来标识。每个服务描述了两个消息，一个用于请求数据结构，一个用于回复数据结构。

### Actions

For longer running request / reply style communication with feedback about the progress the exchanged data structures are related. These triplets of data structures are called _actions_. An action is identified by its _name_ and the _package_ it is in. Each action describes three messages, one for the goal data structure, one for the result data structure, and one for the feedback data structure.

> 对于需要较长时间运行的请求/回复式通信，交换的数据结构相关联，并反馈关于进度的信息。这些三元数据结构被称为*动作*。动作由其*名称*和*包*来标识。每个动作描述三个消息，一个用于目标数据结构，一个用于结果数据结构，一个用于反馈数据结构。

### Field types

The type of a field can be either a primitive type or another data structure. Each of these can optionally be a dynamically or statically sized array.

> 字段的类型可以是基本类型或其他数据结构。这些类型可以选择性地是动态大小或静态大小的数组。

#### Primitive field types

The following primitive types are defined:

> 以下定义了原始类型：

- `bool`
- `byte`
- `char`
- `float32`, `float64`
- `int8`, `uint8`
- `int16`, `uint16`
- `int32`, `uint32`
- `int64`, `uint64`
- `string`

<div class="alert alert-info" markdown="1">

While <code>byte</code> and <code>char</code> are deprecated in ROS 1 they are still part of this definition to ease migration.

> 虽然字节和字符在 ROS 1 中已经被弃用，但仍然是这个定义的一部分，以便于迁移。

</div>

<div class="alert alert-warning" markdown="1">

In ROS 1 <code>string</code> does not specify any encoding and the transport is agnostic to it. This means commonly it can only contain ASCII. For explicit support of wide character strings please consider migrating to [.idl files](http://design.ros2.org/articles/idl_interface_definition.html) which defines explicit types for that.

> 在 ROS 1 中，<code>string</code>不指定任何编码，传输对其无感知。这意味着通常它只能包含 ASCII。为了明确支持宽字符串，请考虑迁移到[.idl 文件](http://design.ros2.org/articles/idl_interface_definition.html)，它为此定义了明确的类型。

</div>

#### Non-primitive field types

Beside the primitive types other messages can be referenced to describe the type of a "complex" field. A complex field type is identified by a package and a message name.

> 除原始类型外，还可以引用其他消息来描述“复杂”字段的类型。复杂字段类型由包和消息名称标识。

#### Arrays with static size

A static array has exactly `N` elements of the specified type. `N` must be greater than `0`.

> 一个静态数组有指定类型的 `N` 个元素，`N` 必须大于 `0`。

#### Arrays with dynamic size

A dynamic array can have between `0` and `N` elements of the specified type. `N` might not have an upper bound and may only be limited by the memory or other system specific limitations.

> 一个动态数组可以有 0 到 N 个指定类型的元素。N 可能没有上限，可能只受内存或其他系统特定限制的限制。

#### Upper boundaries

<div class="alert alert-info" markdown="1">

This feature is not available in ROS 1.

> 这个功能在 ROS 1 中不可用。

</div>

The size of _strings_ as well as _dynamic arrays_ can be limited with an _upper boundary_. This enables the preallocation of memory for data structures which use dynamically sized data.

> 字符串和动态数组的大小可以通过上限进行限制。这使得可以为使用动态大小数据的数据结构预先分配内存。

### Default values

<div class="alert alert-info" markdown="1">

This feature is not available in ROS 1.

> 这个功能在 ROS 1 中不可用。

</div>

A field can optionally specify a default value. If no default value is specified a common default value is used:

> 可以可选择地为字段指定默认值。如果没有指定默认值，将使用常用的默认值。

- for `bool` it is `false`
- for _numeric_ types it is a `0` value
- for `string` it is an _empty_ string
- for _static size arrays_ it is an array of N elements with its fields zero-initialized
- for _bounded size arrays_ and _dynamic size arrays_ it is an empty array `[]`

#### Array default values

A field of type `array` can optionally specify a default value.

> 一个类型为 `array` 的字段可以选择性地指定默认值。

- default values for an array must start with an opening square bracket (`[`) and end with a closing square bracket (`]`)
- each value within the array must be separated with a comma (`,`)
- all values in the array must be of the same type as the field
- there cannot be a comma `,` before the first value of the array
- a trailing comma after the last element of the array is ignored

Additional rule for `string` arrays:

> 对于 `string` 数组的额外规则：

- string arrays must contain only `string` s respecting the following rules:
  - a string value which can optionally be quoted with either single quotes (`'`) or double quotes (`"`)
  - a double-quoted (`"`) string (respectively single-quoted (`'`)) should have any inner double quotes (respectively single quotes) escaped

<div class="alert alert-warning" markdown="1">
  <b>TODO:</b> default values are currently not supported for <i>complex</i> fields
</div>

### Constants

Constants are defined by a _primitive type_, a _name_, and a _fixed value_. If the _primitive type_ is an integer type, the _fixed value_ can be specified in one of the following ways:

> 常量由原始类型、名称和固定值定义。如果原始类型是整数类型，则可以以下列方式指定固定值：

- a decimal integer, e.g. 1
- a binary integer, e.g. 0b01 or 0B01
- an octal integer, e.g. 0o01 or 0O01
- a hexadecimal integer, e.g. 0x01 or 0X01

## Conventions

### Naming of messages and services

Each file contains a single message or service. Message files use the extension `.msg`, service files use the extension `.srv`.

> 每个文件包含一条消息或服务。消息文件使用 `.msg` 扩展名，服务文件使用 `.srv` 扩展名。

Both file names must use an upper camel case name and only consist of alphanumeric characters.

> 两个文件名必须使用大驼峰命名法，只能由字母和数字组成。

### Naming of fields

Field names must be lowercase alphanumeric characters with underscores for separating words. They must start with an alphabetic character, they must not end with an underscore and never have two consecutive underscores.

> 字段名称必须是小写字母数字字符，用下划线分隔单词。它们必须以字母开头，不能以下划线结尾，也不能连续两个下划线。

### Naming of constants

Constant names must be uppercase alphanumeric characters with underscores for separating words. They must start with an alphabetic character, they must not end with an underscore and never have two consecutive underscores.

> 常量名称必须是大写字母数字字符，用下划线分隔单词。它们必须以字母开头，不能以下划线结尾，也不能有连续的两个下划线。

## Syntax

The message and service definitions are text files.

> 这些消息和服务定义都是文本文件。

### Comments

The character `#` starts a comment, which terminates at the end of the line on which it occurs.

> 字符 `#` 开始一个注释，在其出现的行结束时终止。

### Message file format

A line can either contain a field definition or a constant definition. While a single space is mandatory to separate tokens additional spaces can be inserted optionally between tokens.

> 一行可以包含一个字段定义或一个常量定义。虽然在标记之间必须有一个单独的空格，但可以选择性地在标记之间插入额外的空格。

#### Field definition

A field definition has the following structure:

> 一个字段定义的结构如下：

```
<type> <name> <optional_default_value>
```

#### Constant definition

A constant definition has the following structure:

> 一个常量定义的结构如下：

```
<type> <name>=<value>
```

#### Types

A `<type>` is defined by its _base type_ and optional _array specifier_.

> 一个 < 类型 > 由它的基本类型和可选的数组规格说明来定义。

The _base type_ can be one of the following:

> 基本类型可以是以下之一：

- a primitive type from the list above: e.g. `int32`
- a string with an upper boundary: `string<=N` to limit the length of the string to `N` characters
- a complex type referencing another message:
  - an _absolute_ reference of a message: e.g. `some_package/SomeMessage`
  - a _relative_ reference of a message within the same package: e.g. `OtherMessage`

The _array specifier_ can be one of the following:

> 以下是可用的数组说明符：

- a static array is described by the suffix `[N]` where `N` is the fixed size of the array
- an unbounded dynamic array is described by the suffix `[]`
- a bounded dynamic array is described by the suffix `[<=N]` where `N` is the maximum size of the array

#### Values

Depending on the type the following values are valid:

> 根据类型，以下值有效：

- `bool`:

  - `true`, `1`
  - `false`, `0`
- `byte`:

  - an opaque 8-bit quantity with a numerical value in the following interval `[0, 255]`
- `char`:

  - an unsigned integer value in the following interval `[0, 255]`
- `float32` and `float64`:

  - a decimal number using a dot (`.`) as the separator between the integer-part and fractional-part.
- `int8`, `int16`, `int32` and `int64`:

  - an integer value in the following interval `[- 2 ^ (N - 1), 2 ^ (N - 1) - 1]` where `N` is the number of bits behind `int`
- `uint8`, `uint16`, `uint32` and `uint64`:

  - an unsigned integer value in the following interval `[0, 2 ^ N - 1]` where `N` is the number of bits behind `uint`
- `string`:

  - a string value which can optionally be quoted with either single quotes (`'`) or double quotes (`"`)
  - a double-quoted (`"`) string (respectively single-quoted (`'`)) should have any inner double quotes (respectively single quotes) escaped:

    - `string my_string "I heard \"Hello\""` is valid
    - `string my_string "I heard "Hello""` is **not** valid
    - `string my_string "I heard 'Hello'"` is valid
    - `string my_string 'I heard \'Hello\''` is valid
    - `string my_string 'I heard 'Hello''` is **not** valid
    - `string my_string 'I heard "Hello"'` is valid

### Service file format

A service file contains two message definitions which are separated by a line which only contains three dashes:

> 一个服务文件包含两个消息定义，它们由只包含三个破折号的一行分隔开：

```
---
```

## Conversion to IDL

Code is generated for defined interfaces to be usable by different client libraries. Interfaces described using the above format are first converted to [IDL](idl_interface_definition.html). Code generation uses the generated file.

> 代码为定义的接口生成，以便不同的客户端库可以使用。使用上述格式描述的接口首先被转换为 [IDL](idl_interface_definition.html)。代码生成使用生成的文件。

<div class="alert alert-info" markdown="1">

A structure defined in the above format can be empty / contain no members. In that case the generated `.idl` structure will have a dummy member (`uint8 structure_needs_at_least_one_member`) to satisfy the requirement from IDL of not being empty.

> 结构可以按照上述格式定义，也可以为空/不包含任何成员。在这种情况下，生成的 `.idl` 结构将有一个哑成员（`uint8 structure_needs_at_least_one_member`）来满足 IDL 的要求，不能为空。

</div>

### Mapping to IDL types

```
| ROS type | IDL type           |
| -------- | ------------------ |
| bool     | boolean            |
| byte     | octet              |
| char     | uint8              |
| float32  | float              |
| float64  | double             |
| int8     | int8               |
| uint8    | uint8              |
| int16    | short              |
| uint16   | unsigned short     |
| int32    | long               |
| uint32   | unsigned long      |
| int64    | long long          |
| uint64   | unsigned long long |
| string   | string             |
```

<div class="alert alert-info" markdown="1">

The mapping of <code>byte</code> uses a different type than in ROS 1 while still remaining an opaque 8-bit quantity. [Definition in ROS 1](http://wiki.ros.org/msg#Field_Types): deprecated alias for <code>int8</code>. Definition in IDL (7.4.1.4.4.2.6): an opaque 8-bit quantity.

> 在 ROS 1 中，<code>byte</code> 的映射使用的类型与 ROS 1 中的不同，但仍然是一个不透明的 8 位数量。[ROS 1 中的定义](http://wiki.ros.org/msg#Field_Types)：已弃用的<code>int8</code>的别名。IDL（7.4.1.4.4.2.6）中的定义：一个不透明的 8 位数量。

</div>

<div class="alert alert-info" markdown="1">

While the mapping of <code>char</code> is unintuitive it preserves compatibility with the [definition in ROS 1](http://wiki.ros.org/msg#Field_Types): "deprecated alias for <code>uint8</code>".

> 尽管<code>char</code>的映射不直观，但它与 [ROS 1 中的定义](http://wiki.ros.org/msg#Field_Types)保持兼容：“<code>uint8</code>的弃用别名”。

</div>

```
| ROS type                | IDL type         |
| ----------------------- | ---------------- |
| static array            | array            |
| unbounded dynamic array | sequence         |
| bounded dynamic array   | bounded sequence |
| bounded string          | bounded string   |
```
