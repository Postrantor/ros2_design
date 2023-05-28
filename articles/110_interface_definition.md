---
tip: translate by openai@2023-05-28 11:26:13
...
---
    layout: default
    title: Interface definition
    permalink: articles/interface_definition.html
    abstract:
      This article specifies the file format describing the data structures exchanged by ROS 2 components to interact with each other.
    published: true
    author: '[Dirk Thomas](https://github.com/dirk-thomas)'
    date_written: 2015-06
    last_modified: 2020-02
    categories: Interfaces

    <div class="alert alert-warning" markdown="1">
    With the transition to use ``IDL`` for specifying interfaces in ROS 2 Dashing this article has been superseded by the [Interface definition using .msg / .srv / .action files](legacy_interface_definition.html) article.
    </div>
    
    Authors: {{ page.author }}
    Date Written: {{ page.date_written }}
    Last Modified: {% if page.last_modified %}{{ page.last_modified }}{% else %}{{ page.date_written }}{% endif %}
---

## Scope


This article specifies the file format describing the data structures which are being used to exchange information between components.

> 这篇文章指定了描述用于在组件之间交换信息的数据结构的文件格式。

The data structures are defined in a programming language agnostic way.

> 数据结构以与特定编程语言无关的方式定义。

Please see other articles for the mappings to programming language specific types and API.

> 请参阅其他文章以查看与特定编程语言类型和API的映射。

## Overview


A data structure is defined by a set of *fields*.

> 数据结构由一组字段定义。

The order of the fields is irrelevant.

> 顺序无关紧要。

Each field is described by a *type* and a *name*.

> 每个字段都由一个类型和一个名称来描述。

### Messages


A single data structure is called *message*.

> 一个单独的数据结构被称为消息。

Each message has a *name*.

> 每条消息都有一个名字。

Together with the name of the *package* a message can be uniquely identified.

> 随着包裹的名字，一条消息可以被唯一地识别。

### Services


For request / reply style communication the two exchanged data structures are related.

> 对于请求/回复式通信，两者交换的数据结构是相关的。

These pairs of data structures are called *services*

> 这些数据结构对被称为*服务*。

A service is identified by its *name* and the *package* it is in.

> 服务通过其名称和所在的包来识别。

Each service describes two messages, one for the request data structure, one for the reply data structure.

> 每个服务描述了两条消息，一个是用于请求数据结构的，一个是用于回复数据结构的。

### Field types


The type of a field can be either a primitive type or another data structure.

> 字段的类型可以是基本类型，也可以是另一种数据结构。

Each of these can optionally be a dynamically or statically sized array.

> 这些每一个都可以选择性地是一个动态大小的数组或静态大小的数组。

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

    <div class="alert alert-warning" markdown="1">
      <b>TODO:</b> consider <code>wchar</code>, <code>wstring</code>, <code>u16string</code>, <code>u32string</code>
    </div>

    <div class="alert alert-warning" markdown="1">
      <b>TODO:</b> <code>string</code> does not specify any encoding yet and the transport is agnostic to it, this means commonly it can only contain ASCII but all endpoints can also "agree" on using a specific encoding
    </div>

    <div class="alert alert-warning" markdown="1">
      <b>TODO:</b> consider removing <code>byte</code>, <code>char</code> after specifying the mapping to C++ and Python
    </div>

#### Non-primitive field types


Beside the primitive types other messages can be referenced to describe the type of a "complex" field.

> 除了原始类型，还可以引用其他消息来描述“复杂”字段的类型。

A complex field type is identified by a package and a message name.

> 一个复杂的字段类型由一个包和一个消息名称来识别。

#### Arrays with static size


A static array has exactly `N` elements of the specified type.

> 静态数组拥有指定类型的正好N个元素。
`N` must be greater than `0`.

#### Arrays with dynamic size


A dynamic array can have between `0` and `N` elements of the specified type.

> 一个动态数组可以有0到N个指定类型的元素。
`N` might not have an upper bound and may only be limited by the memory or other system specific limitations.

#### Upper boundaries


The size of *strings* as well as *dynamic arrays* can be limited with an *upper boundary*.

> 字符串和动态数组的大小可以通过上限进行限制。

This enables the preallocation of memory for data structures which use dynamically sized data.

> 这使得可以为使用动态大小数据的数据结构预先分配内存。

### Default values


A field can optionally specify a default value.

> 一个字段可以可选地指定默认值。

If no default value is specified a common default value is used:

> 如果没有指定默认值，则使用一个常用的默认值：

    - for `bool` it is `false`
    - for *numeric* types it is a `0` value
    - for `string` it is an *empty* string
    - for *static size arrays* it is an array of N elements with its fields zero-initialized
    - for *bounded size arrays* and *dynamic size arrays* it is an empty array `[]`

#### Array default values


A field of type `array` can optionally specify a default value.

> 一个类型为“数组”的字段可以可选地指定默认值。


- default values for an array must start with an opening square bracket (`[`) and end with a closing square bracket (`]`)

> 默认值数组必须以开放方括号（`[`）开头，以闭放方括号（`]`）结尾。

- each value within the array must be separated with a comma (`,`)

> 每个数组中的值必须用逗号（`,`）分隔。

- all values in the array must be of the same type as the field

> .

所有数组中的值必须与字段的类型相同。

- there cannot be a comma `,` before the first value of the array

> 不能在数组的第一个值之前加逗号。

- a trailing comma after the last element of the array is ignored

> 最后一个数组元素后的逗号被忽略。


Additional rule for `string` arrays:

> 对于`字符串`数组的附加规则：

- string arrays must contain only `string`s respecting the following rules:

> 字符串数组必须只包含符合以下规则的字符串：

  - a string value which can optionally be quoted with either single quotes (`'`) or double quotes (`"`)

> 一个字符串值，可以用单引号（'）或双引号（"）来选择性地引用。

  - a double-quoted (`"`) string (respectively single-quoted (`'`)) should have any inner double quotes (respectively single quotes) escaped

> 一个双引号（`"`）字符串（或单引号（`'`）字符串）应该对内部的双引号（或单引号）进行转义。

<div class="alert alert-warning" markdown="1">

  <b>TODO:</b> default values are currently not supported for <i>complex</i> fields

> <b>待办事项：</b>目前不支持<i>复杂</i>字段的默认值。
</div>

### Constants


Constants are defined by a *primitive type*, a *name* as well as a *fixed value*.

> 常量由原始类型、名称以及固定值定义。

## Conventions

### Naming of messages and services


Each file contains a single message or service.

> 每个文件包含一条消息或服务。

Message files use the extension `.msg`, service files use the extension `.srv`.

> 消息文件使用扩展名`.msg`，服务文件使用扩展名`.srv`。


Both file names must use an upper camel case name and only consist of alphanumeric characters.

> 两个文件名必须使用驼峰命名法，只能由字母和数字组成。

### Naming of fields


Field names must be lowercase alphanumeric characters with underscores for separating words.

> 字段名称必须是小写字母数字字符，用下划线分隔单词。

They must start with an alphabetic character, they must not end with an underscore and never have two consecutive underscores.

> 他们必须以字母开头，不能以下划线结尾，也不能连续出现两个下划线。

### Naming of constants


Constant names must be uppercase alphanumeric characters with underscores for separating words.

> 常量名称必须是大写字母数字字符，用下划线分隔单词。

They must start with an alphabetic character, they must not end with an underscore and never have two consecutive underscores.

> 他们必须以字母字符开头，不能以下划线结尾，也不能有两个连续的下划线。

## Syntax


The message and service definitions are text files.

> 消息和服务定义是文本文件。

### Comments


The character `#` starts a comment, which terminates at the end of the line on which it occurs.

> 字符`#`开始一个注释，它在出现的行的末尾终止。

### Message file format


A line can either contain a field definition or a constant definition.

> 一行可以包含字段定义或常量定义。

While a single space is mandatory to separate tokens additional spaces can be inserted optionally between tokens.

> 在分隔标记时，必须使用一个空格，可以可选地在标记之间插入额外的空格。

#### Field definition


A field definition has the following structure:

> 一个字段定义的结构如下：

    <type> <name> <optional_default_value>

#### Constant definition


A constant definition has the following structure:

> 一个常量定义的结构如下：

    <type> <name>=<value>

#### Types


A `<type>` is defined by its *base type* and optional *array specifier*.

> 一个<类型>由其基本类型和可选的数组规范定义。


The *base type* can be one of the following:

> 基本类型可以是以下之一：


- a primitive type from the list above: e.g. `int32`

> int32：整型


- a string with an upper boundary: `string<=N` to limit the length of the string to `N` characters

> 字符串长度不超过N个字符："string<=N"


- a complex type referencing another message:

> 一种复杂类型，引用另一条消息


  - an *absolute* reference of a message: e.g. `some_package/SomeMessage`

> 一条消息的*绝对*引用：例如`some_package/SomeMessage`

  - a *relative* reference of a message within the same package: e.g. `OtherMessage`

> 一个在同一个包中的消息的相对引用：例如，`OtherMessage`


The *array specifier* can be one of the following:

> 数组说明符可以是以下之一：


- a static array is described by the suffix `[N]` where `N` is the fixed size of the array

> 一个静态数组由后缀`[N]`描述，其中`N`是数组的固定大小。

- an unbounded dynamic array is described by the suffix `[]`

> 一个无限动态数组由后缀 `[]` 来描述。

- a bounded dynamic array is described by the suffix `[<=N]`  where `N` is the maximum size of the array

> 一个有界动态数组由后缀`[<=N]`描述，其中`N`是数组的最大大小。

#### Values


Depending on the type the following values are valid:

> 根据类型，以下值有效：


- `bool`:

> 布尔：


  - `true`, `1`

> 真的，1

  - `false`, `0`

> 假，零


- `byte`:

> 字节


  - an unsigned integer value in the following interval `[0, 255]`

> 一个无符号整数值在区间[0,255]内


- `char`:

> 字符：


  - an integer value in the following interval `[-128, 127]`

> 一个在以下区间[-128, 127]内的整数值


- `float32` and `float64`:

> float32和float64


  - a decimal number using a dot (`.`) as the separator between the integer-part and fractional-part.

> 一个带有小数点（`.`）分隔整数部分和小数部分的十进制数。


- `int8`, `int16`, `int32` and `int64`:

> int8：8 位整数
int16：16 位整数
int32：32 位整数
int64：64 位整数


  - an integer value in the following interval `[- 2 ^ (N - 1), 2 ^ (N - 1) - 1]` where `N` is the number of bits behind `int`

> 一个整数值在以下区间[-2^(N-1), 2^(N-1)-1]，其中N是int后面的位数。


- `uint8`, `uint16`, `uint32` and `uint64`:

> `uint8`、`uint16`、`uint32`和`uint64`：无符号8位整数、无符号16位整数、无符号32位整数和无符号64位整数。


  - an unsigned integer value in the following interval `[0, 2 ^ N - 1]` where `N` is the number of bits behind `uint`

> 一个无符号整数值在以下区间[0, 2 ^ N - 1]，其中N是uint后面的位数。


- `string`:

> - 字符串


  - a string value which can optionally be quoted with either single quotes (`'`) or double quotes (`"`)

> 可以用单引号（'）或双引号（"）括起来的字符串值。


  - a double-quoted (`"`) string (respectively single-quoted (`'`)) should have any inner double quotes (respectively single quotes) escaped:

> 一个双引号（“”）字符串（分别是单引号（''））应该有任何内部双引号（分别是单引号）被转义：

    - `string my_string "I heard \"Hello\""` is valid
    - `string my_string "I heard "Hello""` is **not** valid
    - `string my_string "I heard 'Hello'"` is valid
    - `string my_string 'I heard \'Hello\''` is valid
    - `string my_string 'I heard 'Hello''` is **not** valid
    - `string my_string 'I heard "Hello"'` is valid

### Service file format


A service file contains two message definitions which are separated by a line which only contains three dashes:

> 一个服务文件包含两个消息定义，它们由仅包含三个破折号的一行分隔：
