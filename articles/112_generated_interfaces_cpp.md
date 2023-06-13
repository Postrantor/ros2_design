---
tip: translate by openai@2023-05-30 08:32:21

layout: default
title: Generated C++ interfaces
permalink: articles/generated_interfaces_cpp.html
abstract:
  This article describes the generated C++ code for ROS 2 interfaces.
published: true
author: '[Dirk Thomas](https://github.com/dirk-thomas)'
date_written: 2015-06
last_modified: 2019-03
categories: Interfaces
<div class="alert alert-warning" markdown="1">
With the transition to use ``IDL`` for specifying interfaces in ROS 2 Dashing this article has been superseded by the [Interface Definition and Language Mapping](idl_interface_definition.html) article.
</div>
Authors: 
Date Written: 
Last Modified:
---
## Scope

This article specifies the generated C++ code for ROS interface types defined in the [interface definition article](interface_definition.html).

> 这篇文章指定了[接口定义文章](interface_definition.html)中定义的 ROS 接口类型所生成的 C++ 代码。

## Namespacing

All code of a ROS package should be defined in a namespace named after the package. To separate the generated code from other code within the package it is defined in a sub namespace:

> 所有 ROS 包的代码都应该定义在一个以包名命名的命名空间中。为了将生成的代码与包中的其他代码分开，它被定义在一个子命名空间中。

- namespace for ROS messages: `<package_name>::msg`.
- namespace for ROS services: `<package_name>::srv`.

<div class="alert alert-info" markdown="1">
  <b>NOTE:</b> Using the additional sub namespace ensures that the symbols are different and don't overlap with the ROS 1 symbols. That allows to include both in a single compilation unit like the <code>ros1_bridge</code>.
</div>

## Generated files

Following the C++ style guide of ROS 2 the namespace hierarchy is mapped to a folder structure. The filenames use lowercase alphanumeric characters with underscores for separating words and end with either `.hpp` or `.cpp`.

> 在 ROS 2 的 C++ 样式指南中，命名空间层次结构映射到文件夹结构。文件名使用小写字母数字，用下划线分隔单词，以 `.hpp` 或 `.cpp` 结尾。

## Messages

For a message a templated `struct` with the same name followed by an underscore is generated. The single template argument is the allocator for the data structure.

> 对于消息，会生成一个带有相同名称后缀为下划线的模板 `struct`。单个模板参数是数据结构的分配器。

For ease of use there is a `typedef` with the same name as the message which uses a default allocator (e.g. `std::allocator`).

> 为了方便使用，有一个与消息同名的 `typedef`，它使用默认分配器(例如 `std::allocator`)。

For each message two files are being generated:

> 每条消息生成两个文件：

- `<my_message_name>.hpp` currently only includes `<my_message_name>__struct.hpp`
- `<my_message_name>__struct.hpp` containing the definition of the struct

This allows to add additional files besides the one with the suffix `__struct` to provide additional functionality. For each additional functionality it can be decided to include it from the first header file.

> 这允许添加除具有后缀“__struct”的文件之外的其他文件，以提供额外的功能。对于每个额外的功能，可以从第一个标头文件中进行选择性包含。

<div class="alert alert-warning" markdown="1">
  <b>TODO:</b> specify content of <code>&lt;my_message_name&gt;__traits.hpp</code> file
</div>

### Types

#### Mapping of primitive types

```
| ROS type | C++ type    |
| -------- | ----------- |
| bool     | bool        |
| byte     | uint8_t     |
| char     | char        |
| float32  | float       |
| float64  | double      |
| int8     | int8_t      |
| uint8    | uint8_t     |
| int16    | int16       |
| uint16   | uint16      |
| int32    | int32       |
| uint32   | uint32      |
| int64    | int64       |
| uint64   | uint64_t    |
| string   | std::string |
```

#### Mapping of arrays and bounded strings

```
| ROS type                | C++ type           |
| ----------------------- | ------------------ |
| static array            | std::array<T, N>   |
| unbounded dynamic array | std::vector<T>     |
| bounded dynamic array   | custom_class<T, N> |
| bounded string          | std::string        |
```

### Members

The struct has same-named public member variables for every field of the message. For each field a `typedef` is created which is named after the member with a leading underscore and a trailing `_type`.

> 结构体为每个消息字段都有公共成员变量同名。为每个字段创建一个 `typedef`，名称以下划线开头，以 `_type` 结尾。

### Constants

Numeric constants are defined as `enums` within the struct. All other constants are declared as `static const` members in the struct and their values are defined outside of the struct.

> 数值常量被定义为结构体内的枚举。所有其他常量被声明为结构体中的静态常量成员，它们的值定义在结构体之外。

### Constructors

In the following discussion, "member" refers to the class member in the C++ class while "field" refers to the field definition in the IDL file.

> 在以下讨论中，“成员”指的是 C++ 类中的类成员，而“字段”指的是 IDL 文件中的字段定义。

The _default constructor_ initializes all members with the default value specified in the IDL file, or otherwise with the common default for the field type [as defined in this article](http://design.ros2.org/articles/interface_definition.html#default-values) (note: `char` fields are considered numeric for C++). In some cases this may not be desirable, since these fields will often be immediately overwritten with user-provided values. Therefore, the constructor takes an optional directive of type `rosidl_generator_cpp::MessageInitialization` to control how initialization is done:

> 默认构造函数会使用 IDL 文件中指定的默认值，或者使用字段类型的常见默认值(参见本文[[http://design.ros2.org/articles/interface_definition.html#default-values](http://design.ros2.org/articles/interface_definition.html#default-values)]，注意：C++ 中的 char 字段被视为数值)来初始化所有成员。在某些情况下，这可能不是理想的，因为这些字段往往会被用户提供的值立即覆盖。因此，构造函数接受一个 `rosidl_generator_cpp::MessageInitialization` 类型的可选指令，以控制初始化的方式：

- `MessageInitialization::ALL` - Initialize each member with the field's default value specified in the IDL file, or otherwise with the common default for the field type [as defined in this article](http://design.ros2.org/articles/interface_definition.html#default-values) (note: `char` fields are considered numeric for C++).
  - The safest option, and also the default (used if not passing any argument to the constructor).
- `MessageInitialization::SKIP` - Don't initialize any members; it is the user's responsibility to ensure that all members get initialized with some value, otherwise undefined behavior may result
  - Used for maximum performance if the user is setting all of the members themselves.
- `MessageInitialization::ZERO` - Zero initialize all members; all members will be [value-initialized](http://en.cppreference.com/w/cpp/language/value_initialization) ([dynamic size](interface_definition.html#arrays-with-dynamic-size) or [upper boundary](interface_definition.html#upper-boundaries) arrays will have zero elements), and default values from the message definition will be ignored
  - Used when the user doesn't want the overhead of initializing potentially complex or large default values, but still wants to ensure that all variables are properly initialized.
- `MessageInitialization::DEFAULTS_ONLY` - Initialize only members that have field default values; all other members will be left uninitialized
  - Minimal initialization which ensures that existing code has correctly initialized members when a new field with a default value is added to the IDL later.

Optionally the constructor can be invoked with an allocator.

> 构造函数可以可选地使用分配器调用。

The struct has no constructor with positional arguments for the members. The short reason for this is that if code would rely on positional arguments to construct data objects changing a message definition would break existing code in subtle ways. Since this would discourage evolution of message definitions the data structures should be populated by setting the members separately, e.g. using the setter methods.

> 结构体没有构造函数来为成员提供位置参数。简短的原因是，如果代码依赖于位置参数来构造数据对象，更改消息定义将以微妙的方式破坏现有的代码。由于这会抑制消息定义的进化，因此应该通过单独设置成员来填充数据结构，例如使用设置器方法。

### Setters

For each field a _setter_ method is generated to enable [method chaining](https://isocpp.org/wiki/faq/ctors#named-parameter-idiom). They are named after the fields with a leading `set__`. The setter methods have a single argument to pass the value for the member variable. Each setter method returns the struct itself.

> 为每个字段生成一个*设置器*方法，以启用[方法链](https://isocpp.org/wiki/faq/ctors#named-parameter-idiom)。它们以字段名称为前缀，以 `set__` 命名。设置器方法有一个参数，用于传递成员变量的值。每个设置器方法都返回结构本身。

### Operators

The comparison operators `==` and `!=` perform the comparison on a per member basis.

> 比较运算符 `==` 和 `!=` 会按成员逐一进行比较。

### Pointer types

The struct contains `typedefs` for the four common pointer types `plain pointer`, `std::shared_ptr`, `std::unique_ptr`, `std::weak_ptr`. For each pointer type there a non-const and a const `typedef`:

> 这个结构包含四种常见指针类型的 `typedef`：`plain pointer`、`std::shared_ptr`、`std::unique_ptr`、`std::weak_ptr`。每种指针类型都有一个非常量和一个常量的 `typedef`。

- `RawPtr` and `ConstRawPtr`
- `SharedPtr` and `ConstSharedPtr`
- `UniquePtr` and `ConstUniquePtr`
- `WeakPtr` and `ConstWeakPtr`

For similarity to ROS 1 the `typedefs` `Ptr` and `ConstPtr` still exist but are deprecated. In contrast to ROS 1 they use `std::shared_ptr` instead of Boost.

> 对于与 ROS 1 相似，`typedefs` `Ptr` 和 `ConstPtr` 仍然存在，但已被弃用。与 ROS 1 相反，它们使用 `std::shared_ptr` 而不是 Boost。

## Services

For a service a `struct` with the same name followed by an underscore is generated.

> 为了提供服务，会生成一个以相同名称加下划线的 `struct`。

The struct contains only two `typedefs`:

> 这个结构只包含两个 `typedefs`:

- `Request` which is the type of the request part of the service
- `Response` which is the type of the request part of the service

The generated code is split across multiple files the same way as message are.

> 生成的代码和消息一样被分散到多个文件中。

### Request and response messages

For the request and response parts of a service separate messages are being generated. These messages are named after the service and have either a `_Request` or `_Response` suffix. They are are still defined in the `srv` sub namespace.

> 对于服务的请求和响应部分，会生成单独的消息。这些消息以服务命名，并以 `_Request` 或 `_Response` 后缀结尾。它们仍然定义在 `srv` 子命名空间中。
