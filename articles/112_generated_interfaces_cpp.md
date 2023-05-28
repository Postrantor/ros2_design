---
tip: translate by openai@2023-05-28 11:28:47
...
---
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
    
    Authors: {{ page.author }}
    Date Written: {{ page.date_written }}
    Last Modified: {% if page.last_modified %}{{ page.last_modified }}{% else %}{{ page.date_written }}{% endif %}
---

## Scope


This article specifies the generated C++ code for ROS interface types defined in the [interface definition article](interface_definition.html).

> 这篇文章指定了在[接口定义文章](interface_definition.html)中定义的ROS接口类型的生成的C++代码。

## Namespacing


All code of a ROS package should be defined in a namespace named after the package.

> 所有ROS包的代码都应定义在以包名命名的命名空间中。

To separate the generated code from other code within the package it is defined in a sub namespace:

> 将生成的代码与包中的其他代码分离开来，它定义在一个子命名空间中。


- namespace for ROS messages: `<package_name>::msg`.

> .

命名空间为ROS消息：`<package_name>::msg`。

- namespace for ROS services: `<package_name>::srv`.

> 命名空間為ROS服務：<package_name>::srv。

    <div class="alert alert-info" markdown="1">
      <b>NOTE:</b> Using the additional sub namespace ensures that the symbols are different and don't overlap with the ROS 1 symbols.
      That allows to include both in a single compilation unit like the <code>ros1_bridge</code>.
    </div>
    
## Generated files


Following the C++ style guide of ROS 2 the namespace hierarchy is mapped to a folder structure.

> 依照ROS 2的C++样式指南，命名空间层次结构映射到文件夹结构。

The filenames use lowercase alphanumeric characters with underscores for separating words and end with either `.hpp` or `.cpp`.

> 文件名使用小写字母数字字符，用下划线分隔单词，以`.hpp`或`.cpp`结尾。

## Messages


For a message a templated `struct` with the same name followed by an underscore is generated.

> 对于一条消息，会生成一个同名的`struct`模板，后面带有下划线。

The single template argument is the allocator for the data structure.

> 单个模板参数是数据结构的分配器。


For ease of use there is a `typedef` with the same name as the message which uses a default allocator (e.g. `std::allocator`).

> .

为了方便使用，有一个与消息同名的`typedef`，使用默认的分配器（例如`std::allocator`）。


For each message two files are being generated:

> 每条消息生成两个文件：


- `<my_message_name>.hpp` currently only includes `<my_message_name>__struct.hpp`

> - `<my_message_name>.hpp` 目前只包含 `<my_message_name>__struct.hpp`

- `<my_message_name>__struct.hpp` containing the definition of the struct

> - `<my_message_name>__struct.hpp`包含结构体的定义


This allows to add additional files besides the one with the suffix `__struct` to provide additional functionality.

> 这允许添加除带有后缀“__struct”的文件之外的其他文件，以提供额外的功能。

For each additional functionality it can be decided to include it from the first header file.

> 对于每个额外的功能，可以从第一个头文件中决定是否包含它。

    <div class="alert alert-warning" markdown="1">
      <b>TODO:</b> specify content of <code>&lt;my_message_name&gt;__traits.hpp</code> file
    </div>
    
### Types

#### Mapping of primitive types

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
    
#### Mapping of arrays and bounded strings

    | ROS type                | C++ type           |
    | ----------------------- | ------------------ |
    | static array            | std::array<T, N>   |
    | unbounded dynamic array | std::vector<T>     |
    | bounded dynamic array   | custom_class<T, N> |
    | bounded string          | std::string        |
    
### Members


The struct has same-named public member variables for every field of the message.

> 结构体为每个消息字段都有相同名称的公共成员变量。

For each field a `typedef` is created which is named after the member with a leading underscore and a trailing `_type`.

> 每个字段都会创建一个`typedef`，它以成员名称开头加下划线和以`_type`结尾命名。

### Constants


Numeric constants are defined as `enums` within the struct.

> 数值常量被定义为结构体中的枚举类型。


All other constants are declared as `static const` members in the struct and their values are defined outside of the struct.

> 所有其他常量都声明为结构体中的`static const`成员，其值定义在结构体之外。

### Constructors


In the following discussion, "member" refers to the class member in the C++ class while "field" refers to the field definition in the IDL file.

> 在下面的讨论中，“成员”是指C++类中的类成员，而“字段”是指IDL文件中的字段定义。


The *default constructor* initializes all members with the default value specified in the IDL file, or otherwise with the common default for the field type [as defined in this article](http://design.ros2.org/articles/interface_definition.html#default-values) (note: `char` fields are considered numeric for C++).

> 默认构造函数会使用IDL文件中指定的默认值来初始化所有成员，或者使用[本文中定义的](http://design.ros2.org/articles/interface_definition.html#default-values)该字段类型的通用默认值（注意：C++中的`char`字段被视为数值）。

In some cases this may not be desirable, since these fields will often be immediately overwritten with user-provided values.

> 在某些情况下，这可能不是理想的，因为这些字段往往会被用户提供的值立即覆盖。

Therefore, the constructor takes an optional directive of type `rosidl_generator_cpp::MessageInitialization` to control how initialization is done:

> 因此，构造函数接受一个可选的rosidl_generator_cpp::MessageInitialization类型的指令来控制初始化的方式：


- `MessageInitialization::ALL` - Initialize each member with the field's default value specified in the IDL file, or otherwise with the common default for the field type [as defined in this article](http://design.ros2.org/articles/interface_definition.html#default-values) (note: `char` fields are considered numeric for C++).

> - `MessageInitialization::ALL` - 使用IDL文件中指定的字段的默认值来初始化每个成员，或者根据[此文章中定义的](http://design.ros2.org/articles/interface_definition.html#default-values)该字段类型的常见默认值进行初始化（注意：C++中的`char`字段被视为数字）。

  - The safest option, and also the default (used if not passing any argument to the constructor).

> 最安全的选项，也是默认选项（如果不向构造函数传递任何参数时使用）。

- `MessageInitialization::SKIP` - Don't initialize any members; it is the user's responsibility to ensure that all members get initialized with some value, otherwise undefined behavior may result

> 不要初始化任何成员；用户有责任确保所有成员都被初始化为一些值，否则可能会导致未定义的行为。

  - Used for maximum performance if the user is setting all of the members themselves.

> 用于最大性能，如果用户自己设置所有成员。

- `MessageInitialization::ZERO` - Zero initialize all members; all members will be [value-initialized](http://en.cppreference.com/w/cpp/language/value_initialization) ([dynamic size](interface_definition.html#arrays-with-dynamic-size) or [upper boundary](interface_definition.html#upper-boundaries) arrays will have zero elements), and default values from the message definition will be ignored

> MessageInitialization::ZERO - 将所有成员都初始化为零；所有成员将被[值初始化](http://en.cppreference.com/w/cpp/language/value_initialization)（[动态大小](interface_definition.html#arrays-with-dynamic-size) 或[上限](interface_definition.html#upper-boundaries)数组将拥有零元素），而消息定义中的默认值将被忽略。

  - Used when the user doesn't want the overhead of initializing potentially complex or large default values, but still wants to ensure that all variables are properly initialized.

> 当用户不想初始化潜在复杂或大的默认值时使用，但仍希望确保所有变量都被正确初始化。

- `MessageInitialization::DEFAULTS_ONLY` - Initialize only members that have field default values; all other members will be left uninitialized

> MessageInitialization::DEFAULTS_ONLY - 仅初始化具有字段默认值的成员；所有其他成员将保持未初始化状态。

  - Minimal initialization which ensures that existing code has correctly initialized members when a new field with a default value is added to the IDL later.

> 最小化初始化，以确保在后续将默认值添加到IDL时，现有代码已正确初始化成员。


Optionally the constructor can be invoked with an allocator.

> 可选地，构造函数可以使用分配器调用。


The struct has no constructor with positional arguments for the members.

> 结构体没有带有成员的位置参数的构造函数。

The short reason for this is that if code would rely on positional arguments to construct data objects changing a message definition would break existing code in subtle ways.

> 这的简短原因是，如果代码依赖于位置参数来构造数据对象，更改消息定义会以微妙的方式破坏现有的代码。

Since this would discourage evolution of message definitions the data structures should be populated by setting the members separately, e.g. using the setter methods.

> 为了阻止消息定义的进化，应该通过单独设置成员来填充数据结构，例如使用setter方法。

### Setters


For each field a *setter* method is generated to enable [method chaining](https://isocpp.org/wiki/faq/ctors#named-parameter-idiom).

> .

为每个字段生成一个*设置器*方法，以启用[方法链接](https://isocpp.org/wiki/faq/ctors#named-parameter-idiom)。

They are named after the fields with a leading `set__`.

> 他们以一个以“set__”开头的字段命名。

The setter methods have a single argument to pass the value for the member variable.

> 设置方法有一个参数来传递成员变量的值。

Each setter method returns the struct itself.

> 每个设置方法都会返回结构本身。

### Operators


The comparison operators `==` and `!=` perform the comparison on a per member basis.

> 比较运算符`==`和`!=`是按成员进行比较的。

### Pointer types


The struct contains `typedefs` for the four common pointer types `plain pointer`, `std::shared_ptr`, `std::unique_ptr`, `std::weak_ptr`.

> 结构包含四种常见指针类型的`typedefs`：`普通指针`、`std::shared_ptr`、`std::unique_ptr`和`std::weak_ptr`。

For each pointer type there a non-const and a const `typedef`:

> 每种指针类型都有一个非常量和一个常量的`typedef`：


- `RawPtr` and `ConstRawPtr`

> - `RawPtr` 和 `ConstRawPtr`

- `SharedPtr` and `ConstSharedPtr`

> `SharedPtr` 和 `ConstSharedPtr`

- `UniquePtr` and `ConstUniquePtr`

> UniquePtr和ConstUniquePtr

- `WeakPtr` and `ConstWeakPtr`

> 弱指针和常量弱指针


For similarity to ROS 1 the `typedefs` `Ptr` and `ConstPtr` still exist but are deprecated.

> 为了与ROS 1相似，`typedefs` `Ptr` 和 `ConstPtr` 仍然存在，但已被弃用。

In contrast to ROS 1 they use `std::shared_ptr` instead of Boost.

> 与ROS 1相比，它们使用`std::shared_ptr`而不是Boost。

## Services


For a service a `struct` with the same name followed by an underscore is generated.

> 为了提供服务，会生成一个以相同名称加下划线的结构体。


The struct contains only two `typedefs`:

> 结构仅包含两个`typedef`：


- `Request` which is the type of the request part of the service

> 要求，这是服务的请求部分的类型。

- `Response` which is the type of the request part of the service

> 回应，这是服务请求部分的类型。


The generated code is split across multiple files the same way as message are.

> 生成的代码以与消息相同的方式被分割到多个文件中。

### Request and response messages


For the request and response parts of a service separate messages are being generated.

> 对于服务的请求和响应部分，正在生成单独的消息。

These messages are named after the service and have either a `_Request` or `_Response` suffix.

> 这些消息以服务的名称命名，并具有`_Request`或`_Response`后缀。

They are are still defined in the `srv` sub namespace.

> 他们仍然在`srv`子命名空间中定义。
