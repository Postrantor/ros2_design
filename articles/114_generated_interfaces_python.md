---
tip: translate by openai@2023-05-28 11:31:43
...
---
    layout: default
    title: Generated Python interfaces
    permalink: articles/generated_interfaces_python.html
    abstract:
      This article describes the generated Python code for ROS 2 interfaces.
    published: true
    author: '[Dirk Thomas](https://github.com/dirk-thomas)'
    date_written: 2016-01
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


This article specifies the generated Python code for ROS interface types defined in the [interface definition article](interface_definition.html).

> 这篇文章指定了在[接口定义文章](interface_definition.html)中定义的ROS接口类型生成的Python代码。

## Namespacing


All code of a ROS package should be defined in a Python package with the same name.

> 所有ROS包的代码都应该定义在同名的Python包中。

To separate the generated code from other code within the package it is defined in a sub module:

> 为了将生成的代码与定义在同一包中的其他代码分开，它被定义在一个子模块中。


- module for ROS messages: `<package_name>.msg`.

> - ROS消息模块：`<package_name>.msg`。

- module for ROS services: `<package_name>.srv`.

> - ROS服务的模块：<package_name>.srv

    <div class="alert alert-warning" markdown="1">
      <b>NOTE:</b> The names are currently identical to the ones used in ROS 1.
      Therefore it is not possible to import both in a Python application.
    </div>
    
## Generated files


Following the Python conventions the namespace hierarchy is mapped to a folder structure.

> 遵循Python约定，命名空间层次结构映射到文件夹结构。

The package and module names and therefore the folder and file names use lowercase alphanumeric characters with underscores for separating words (following PEP 8).

> 包和模块的名称以及文件夹和文件的名称使用小写字母数字字符，用下划线分隔单词（遵循PEP 8）。

Python files end with `.py`.

> Python文件以`.py`结尾。

## Messages


For a message a Python `class` with the same name as the message is generated in the file `_<my_message_name>.py`.

> 对于一条消息，Python中会生成一个与消息同名的`class`，它会被存储在文件`_<my_message_name>.py`中。

The Python module `<package_name>.msg` / `<package_name>.srv` exports all message / service classes without the message module name to shorten import statements, e.g. `import <package_name>.msg.<MyMessageName>`

> 模块`<package_name>.msg` / `<package_name>.srv`导出所有消息/服务类，而不带有消息模块名称，以简化导入语句，例如`import <package_name>.msg.<MyMessageName>`。

### Types

#### Mapping of primitive types

    | ROS type | Python type                  |
    | -------- | ---------------------------- |
    | bool     | builtins.bool                |
    | byte     | builtins.bytes with length 1 |
    | char     | builtins.str with length 1   |
    | float32  | builtins.float               |
    | float64  | builtins.float               |
    | int8     | builtins.int                 |
    | uint8    | builtins.int                 |
    | int16    | builtins.int                 |
    | uint16   | builtins.int                 |
    | int32    | builtins.int                 |
    | uint32   | builtins.int                 |
    | int64    | builtins.int                 |
    | uint64   | builtins.int                 |
    | string   | builtins.str                 |
    
#### Mapping of arrays and bounded strings

    | ROS type                | Python type   |
    | ----------------------- | ------------- |
    | static array            | builtins.list |
    | unbounded dynamic array | builtins.list |
    | bounded dynamic array   | builtins.list |
    | bounded string          | builtins.str  |
    
### Properties


The class has same-named properties for every field of the message.

> 类有为每个消息字段定义的同名属性。

The setter of a property will ensure that ROS type constraints beyond the Python type are enforced.

> 设置器的属性将确保ROS类型约束超出Python类型被强制执行。

### "Constants"


A constant is defined as a class variable with uppercase name.

> 常量被定义为具有大写名称的类变量。

The class variable is considered to be *read-only* which is ensured by overridding the setter or a magic method.

> 类变量被认为是只读的，这可以通过覆盖setter或魔术方法来确保。

### Constructors


The *__init__* function initializes all members with their default value.

> `__init__` 函数使用默认值初始化所有成员。


The class constructor supports only keyword arguments for all members.

> 类构造函数仅支持关键字参数来初始化所有成员。

The short reason for this is that if code would rely on positional arguments to construct data objects changing a message definition would break existing code in subtle ways.

> 这的简短原因是，如果代码依赖于位置参数来构造数据对象，改变消息定义会以微妙的方式破坏现有的代码。

Since this would discourage evolution of message definitions the data structures should be populated by setting the members separately, e.g. using the setter methods.

> 由于这会阻碍消息定义的进化，因此数据结构应该通过单独设置成员来填充，例如使用setter方法。

### Magic methods

    <div class="alert alert-warning" markdown="1">
      <b>TODO:</b> decide which magic methods should be provided
    </div>
    
## Services


For a service a `class` with the same name is generated.

> 为了提供服务，会生成一个同名的`类`。


The class contains only two `types`:

> 这个类只包含两种类型：


- `Request` which is the type of the request part of the service

> 要求，这是服务部分的请求类型

- `Response` which is the type of the request part of the service

> 响应，这是服务请求部分的类型。


The generated code is split across multiple files the same way as message are.

> 生成的代码以与消息一样的方式被分割到多个文件中。

### Request and response messages


For the request and response parts of a service separate messages are being generated.

> 对于服务的请求和响应部分，会生成单独的消息。

These messages are named after the service and have either a `_Request` or `_Response` suffix.

> 这些消息以服务命名，并以“_Request”或“_Response”为后缀。

They are are still defined in the `srv` sub namespace.

> 他们仍然定义在`srv`子命名空间中。
