---
tip: translate by openai@2023-05-30 08:35:24
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

This article specifies the file format describing the data structures which are being used to exchange information between components. The data structures are defined in a programming language agnostic way. Please see other articles for the mappings to programming language specific types and API.

> 这篇文章指定了描述用于在组件之间交换信息的数据结构的文件格式。这些数据结构以与编程语言无关的方式定义。请参阅其他文章以获取与特定编程语言类型和 API 的映射。

## Overview

A data structure is defined by a set of _fields_. The order of the fields is irrelevant. Each field is described by a _type_ and a _name_.

> 数据结构由一组字段定义。字段的顺序无关紧要。每个字段由类型和名称描述。

### Messages

A single data structure is called _message_. Each message has a _name_. Together with the name of the _package_ a message can be uniquely identified.

> 一个单独的数据结构被称为消息。每条消息都有一个名称。通过包的名称，可以唯一地标识一条消息。

### Services

For request / reply style communication the two exchanged data structures are related. These pairs of data structures are called _services_ A service is identified by its _name_ and the _package_ it is in. Each service describes two messages, one for the request data structure, one for the reply data structure.

> 对于请求/回复式的通信，两个交换的数据结构是相关的。这些数据结构对被称为*服务*。服务由其*名称*和*包*来识别。每个服务描述了两条消息，一条是请求数据结构，一条是回复数据结构。

### Field types

The type of a field can be either a primitive type or another data structure. Each of these can optionally be a dynamically or statically sized array.

> 字段的类型可以是原始类型，也可以是另一种数据结构。这些类型可以选择性地是动态大小的数组或者静态大小的数组。

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

Beside the primitive types other messages can be referenced to describe the type of a "complex" field. A complex field type is identified by a package and a message name.

> 除了原始类型，还可以引用其他消息来描述“复杂”字段的类型。复杂字段类型由包和消息名称标识。

#### Arrays with static size

A static array has exactly `N` elements of the specified type. `N` must be greater than `0`.

> 静态数组必须包含指定类型的 N 个元素，N 必须大于 0。

#### Arrays with dynamic size

A dynamic array can have between `0` and `N` elements of the specified type. `N` might not have an upper bound and may only be limited by the memory or other system specific limitations.

> 一个动态数组可以有 0 到 N 个指定类型的元素。N 可能没有上限，可能只受内存或其他系统特定限制的限制。

#### Upper boundaries

The size of _strings_ as well as _dynamic arrays_ can be limited with an _upper boundary_. This enables the preallocation of memory for data structures which use dynamically sized data.

> 字符串和动态数组的大小可以被上限限制。这使得能够为使用动态大小数据的数据结构预先分配内存。

### Default values

A field can optionally specify a default value. If no default value is specified a common default value is used:

> 可以选择性地为字段指定默认值。如果没有指定默认值，则使用一个常用的默认值。

- for `bool` it is `false`
- for _numeric_ types it is a `0` value
- for `string` it is an _empty_ string
- for _static size arrays_ it is an array of N elements with its fields zero-initialized
- for _bounded size arrays_ and _dynamic size arrays_ it is an empty array `[]`

#### Array default values

A field of type `array` can optionally specify a default value.

> 一个类型为`数组`的字段可以选择性地指定默认值。

- default values for an array must start with an opening square bracket (`[`) and end with a closing square bracket (`]`)
- each value within the array must be separated with a comma (`,`)
- all values in the array must be of the same type as the field
- there cannot be a comma `,` before the first value of the array
- a trailing comma after the last element of the array is ignored

Additional rule for `string` arrays:

> 针对字符串数组的额外规则：

- string arrays must contain only `string`s respecting the following rules:
  - a string value which can optionally be quoted with either single quotes (`'`) or double quotes (`"`)
  - a double-quoted (`"`) string (respectively single-quoted (`'`)) should have any inner double quotes (respectively single quotes) escaped

<div class="alert alert-warning" markdown="1">
  <b>TODO:</b> default values are currently not supported for <i>complex</i> fields
</div>

### Constants

Constants are defined by a _primitive type_, a _name_ as well as a _fixed value_.

> 常量由原始类型、名称以及固定值定义。

## Conventions

### Naming of messages and services

Each file contains a single message or service. Message files use the extension `.msg`, service files use the extension `.srv`.

> 每个文件包含一条消息或服务。消息文件使用`.msg`扩展名，服务文件使用`.srv`扩展名。

Both file names must use an upper camel case name and only consist of alphanumeric characters.

> 两个文件名必须使用大驼峰命名法，只能由字母和数字组成。

### Naming of fields

Field names must be lowercase alphanumeric characters with underscores for separating words. They must start with an alphabetic character, they must not end with an underscore and never have two consecutive underscores.

> 字段名称必须是小写字母和数字组成的，用下划线分隔单词。它们必须以字母开头，不能以下划线结尾，也不能有两个连续的下划线。

### Naming of constants

Constant names must be uppercase alphanumeric characters with underscores for separating words. They must start with an alphabetic character, they must not end with an underscore and never have two consecutive underscores.

> 常量名称必须是大写字母数字字符，用下划线分隔单词。它们必须以字母开头，不能以下划线结尾，也不能有两个连续的下划线。

## Syntax

The message and service definitions are text files.

> 这些消息和服务定义是文本文件。

### Comments

The character `#` starts a comment, which terminates at the end of the line on which it occurs.

> 字符`#`开始一个注释，它在出现的行结束时终止。

### Message file format

A line can either contain a field definition or a constant definition. While a single space is mandatory to separate tokens additional spaces can be inserted optionally between tokens.

> 一行可以包含字段定义或常量定义。虽然在令牌之间必须有一个单独的空格，但可以选择性地在令牌之间插入额外的空格。

#### Field definition

A field definition has the following structure:

> 一个字段定义具有以下结构：

    <type> <name> <optional_default_value>

#### Constant definition

A constant definition has the following structure:

> 一个常量定义的结构如下：

    <type> <name>=<value>

#### Types

A `<type>` is defined by its _base type_ and optional _array specifier_.

> 一个<类型>由它的基本类型和可选的数组规范定义。

The _base type_ can be one of the following:

> 基本类型可以是以下之一：

- a primitive type from the list above: e.g. `int32`
- a string with an upper boundary: `string<=N` to limit the length of the string to `N` characters
- a complex type referencing another message:

  - an _absolute_ reference of a message: e.g. `some_package/SomeMessage`
  - a _relative_ reference of a message within the same package: e.g. `OtherMessage`

The _array specifier_ can be one of the following:

> 数组说明符可以是以下之一：

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

  - an unsigned integer value in the following interval `[0, 255]`

- `char`:

  - an integer value in the following interval `[-128, 127]`

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

> 服务文件包含两个消息定义，它们由仅包含三个破折号的一行分隔：
