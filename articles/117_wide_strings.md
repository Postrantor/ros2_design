---
tip: translate by openai@2023-05-30 08:12:28
layout: default
title: Unicode Support
permalink: articles/wide_strings.html
abstract:
  This article describes how ROS 2 will support sending multi-byte character data using the [Unicode](https://en.wikipedia.org/wiki/Unicode) standard.
  It also describes how such data will be sent over the ROS 1 bridge.
author: '[Chris Lalancette](https://github.com/clalancette)'
date_written: 2019-05
last_modified: 2020-07
published: true
Authors: {{ page.author }}
Date Written: {{ page.date_written }}
Last Modified: {% if page.last_modified %}{{ page.last_modified }}{% else %}{{ page.date_written }}{% endif %}
---

## Background

Some users would like to send text data in languages that cannot be represented by ASCII characters. Currently ROS 1 only supports ASCII data in the string field but [allows users to populate it with UTF-8](http://wiki.ros.org/msg).

> 一些用户想要发送无法用 ASCII 字符表示的语言的文本数据。目前 ROS 1 仅支持字符串字段中的 ASCII 数据，但允许用户使用 UTF-8 填充它。

> [NOTE]:

The following links have more information about multi-byte characters and the history of character encodings.

> 以下链接有更多关于多字节字符和字符编码的历史信息。

    * [http://kunststube.net/encoding/](http://kunststube.net/encoding/)
    * [http://stackoverflow.com/questions/4588302/why-isnt-wchar-t-widely-used-in-code-for-linux-related-platforms](http://stackoverflow.com/questions/4588302/why-isnt-wchar-t-widely-used-in-code-for-linux-related-platforms)
    * [http://www.diveintopython3.net/strings.html](http://www.diveintopython3.net/strings.html)
    * [http://stackoverflow.com/questions/402283/stdwstring-vs-stdstring](http://stackoverflow.com/questions/402283/stdwstring-vs-stdstring)
    * [https://utf8everywhere.org/](http://utf8everywhere.org/)

## Unicode Characters in Strings

Two goals for ROS 2 strings are to be compatible with ROS 1 strings, and compatible with the DDS wire format. ROS 1 says string fields are to contain ASCII encoded data, but allows UTF-8. DDS-XTYPES mandates UTF-8 be used as the encoding of IDL type `string`. To be compatibile with both, in ROS 2 the content of a `string` is expected to be UTF-8.

> ROS 2 的两个字符串目标是与 ROS 1 字符串兼容，以及与 DDS 线路格式兼容。ROS 1 表示字符串字段应包含 ASCII 编码的数据，但允许使用 UTF-8。DDS-XTYPES 强制使用 UTF-8 作为 IDL 类型`string`的编码。为了与两者兼容，在 ROS 2 中，`string`的内容预计为 UTF-8。

## Wide Strings

ROS 2 messages will have a new [primitive field type](/articles/interface_definition.html) `wstring`. The purpose is to allow ROS 2 nodes to communicate with non-ROS DDS entities using an IDL containing a `wstring` field. The encoding of data in this type should be UTF-16 to match DDS-XTYPES 1.2. Since both UTF-8 and UTF-16 can encode the same code points, new ROS 2 messages should prefer `string` over `wstring`.

> ROS 2 消息将有一个新的[原语字段类型](/articles/interface_definition.html)`wstring`。其目的是允许 ROS 2 节点与不使用 ROS DDS 实体的 IDL 进行通信，该 IDL 包含一个`wstring`字段。此类型的数据编码应为 UTF-16，以匹配 DDS-XTYPES 1.2。由于 UTF-8 和 UTF-16 可以编码相同的代码点，因此新的 ROS 2 消息应优先使用`string`而不是`wstring`。

## Encodings are Required but not Guaranteed to be Enforced

`string` and `wstring` are required to be UTF-8 and UTF-16, but the requirement may not be enforced. Since ROS 2 is targeting resource constrained systems, it is left to the rmw implementation to choose whether to enforce the encoding. Further, since many users will write code to check that a string contains valid data, checking again in lower layers may not be necessary in some cases.

> `字符串`和`wstring`必须是 UTF-8 和 UTF-16，但可能无法执行要求。由于 ROS 2 是针对资源约束系统的目标，因此 RMW 实现将选择是否执行编码。此外，由于许多用户会编写代码以检查字符串是否包含有效的数据，因此在某些情况下可能不需要在较低层进行检查。

If a `string` or `wstring` field is populated with the wrong encoding then the behavior is undefined. It is possible the rmw implementation may allow invalid strings to be passed through to subscribers. Each subscriber is responsible for detecting invalid strings and deciding how to handle them. For example, subscribers like `ros2 topic echo` may echo the bytes in hexadecimal.

> 如果一个`string`或`wstring`字段使用了错误的编码，那么行为是不确定的。有可能 rmw 实现允许传递无效的字符串给订阅者。每个订阅者都负责检测无效的字符串，并决定如何处理它们。例如，像`ros2 topic echo`这样的订阅者可以以十六进制的形式输出字节。

The IDL specification forbids `string` from containing `NULL` values. To be compatible, a ROS message `string` field must not contain zero bytes, and a `wstring` field must not contain zero words. This restriction will be enforced.

> IDL 规范禁止字符串包含 NULL 值。为了兼容，ROS 消息字符串字段不能包含零字节，而 wstring 字段不能包含零字。这一限制将被强制执行。

## Unicode Strings Across ROS 1 Bridge

Since ROS 1 and 2 both allow `string` to be UTF-8, the ROS 1 bridge will pass values unmodified between them. If a message with a string field fails to serialize because the content is not legal UTF-8 then the default behavior will be to drop the entire message. Other strategies like replacing invalid bytes could unintentionally change the meaning, so they will be opt-in if available at all.

> 由于 ROS 1 和 2 都允许使用 UTF-8 字符串，ROS 1 桥接器将在它们之间传递未修改的值。如果由于内容不是合法的 UTF-8 而无法序列化带有字符串字段的消息，则默认行为将是丢弃整个消息。其他像替换无效字节这样的策略可能会意外改变意义，因此如果可用，它们将是可选的。

**Note:** Bridging `wstring` fields is not yet implemented. See [ros2/ros1_bridge#203](https://github.com/ros2/ros1_bridge/issues/203).

> **注意：** 桥接`wstring`字段尚未实现。请参阅[ros2/ros1_bridge#203](https://github.com/ros2/ros1_bridge/issues/203)。

If a ROS 2 message has a field of type `wstring` then the bridge will attempt to convert it from UTF-16 to UTF-8. The resulting UTF-8 encoded string will be published as a `string` type. If the conversion fails then the bridge will by default not publish the message.

> 如果 ROS 2 消息具有`wstring`类型的字段，那么桥接器将尝试将其从 UTF-16 转换为 UTF-8。转换后的 UTF-8 编码字符串将以`string`类型发布。如果转换失败，则桥接器默认不会发布消息。

## Size of a Wide String

Both UTF-8 and UTF-16 are variable width encodings. To minimize the amount of memory used, the `string` and `wstring` types are to be stored in client libraries according to the smallest possible code point. This means `string` must be specified as a sequence of bytes, and `wstring` is to be specified as a sequence of words.

> 两种 UTF-8 和 UTF-16 都是可变宽度编码。为了最大限度地减少内存的使用，字符串和宽字符串类型应根据最小可能的代码点存储在客户端库中。这意味着字符串必须指定为字节序列，宽字符串必须指定为字序列。

Some DDS implementations currently use 32bit types to store wide strings values. This may be due to DDS-XTYPES 1.1 section 7.3.1.5 specifying `wchar` as a 32bit value. However this changes in DDS-XTYPES 1.2 section 7.3.1.4 to be a 16bit value. It is expected that most DDS implementations will switch to 16bit character storage in the future. ROS 2 will aim to be compatible with DDS-XTYPES 1.2 and use 16bit storage for wide characters. Generated code for ROS 2 messages will automatically handle the conversion when a message is serialized or deserialized.

> 一些 DDS 实现目前使用 32 位类型来存储宽字符串值。这可能是由于 DDS-XTYPES 1.1 第 7.3.1.5 节指定“wchar”为 32 位值。但是在 DDS-XTYPES 1.2 第 7.3.1.4 节中，它改为 16 位值。预计大多数 DDS 实现将来会改用 16 位字符存储。ROS 2 将符合 DDS-XTYPES 1.2，并使用 16 位字符存储宽字符。ROS 2 消息的生成代码将自动处理序列化或反序列化时的转换。

### Bounded wide strings

Message definitions may restrict the maximum size of a string. These are referred to as bounded strings. Their purpose is to restrict the amount of memory used, so the bounds must be specified as units of memory. If a `string` field is bounded then the size is given in bytes. Similarly the size of a bounded `wstring` is to be specified in words. It is the responsibility of whoever populates a bounded `string` or `wstring` to make sure it contains whole code points only. Partial code points are indistinuguishable from invalid code points, so a bounded string whose last code point is incomplete is not guaranteed to be published.

> 字符串的消息定义可能会限制最大尺寸。这些被称为有界字符串。它们的目的是限制使用的内存量，因此边界必须用内存单位指定。如果`string`字段是有界的，那么尺寸是以字节为单位给出的。同样，有界`wstring`的大小也应以字为单位指定。负责填充有界`string`或`wstring`的人有责任确保它只包含完整的代码点。部分代码点与无效代码点难以区分，因此最后一个代码点不完整的有界字符串不能保证发布。

## Runtime impact of wide string

Dealing with wide strings puts more strain on the software of a system, both in terms of speed and code size. UTF-8 and UTF-16 are both variable width encodings, meaning a code point can take 1 to 4 bytes depending on the encoding. It may take multiple code points to represent a single user perceived character. One of the goals of ROS 2 is to support microcontrollers that are constrained by both code size and processor speed. Some wide string operations like splitting a string on a user perceived character may not be possible on these devices.

> 处理宽字符串会给系统的软件带来更大的压力，无论是在速度还是代码大小方面。UTF-8 和 UTF-16 都是可变宽度编码，意味着代码点可以根据编码而占用 1 到 4 个字节。可能需要多个代码点才能表示一个用户感知的字符。ROS 2 的一个目标是支持受代码大小和处理器速度限制的微控制器。在这些设备上，一些宽字符串操作，如根据用户感知的字符拆分字符串，可能是不可能的。

However, whole string equality checking is the same whether using wide strings or not. Further splitting a UTF-8 string on an ASCII character is identical to splitting an ASCII character on an ASCII string. If code on a microcontroller must do string manipluation then it could assert that a `string` only contains ASCII data by ceasing to proces a string when it encounters a byte greater than 127.

> 然而，不管是否使用宽字符串，字符串完全相等的检查都是相同的。在 ASCII 字符上拆分 UTF-8 字符串与在 ASCII 字符串上拆分 ASCII 字符是完全相同的。如果微控制器上的代码必须进行字符串操作，那么它可以通过停止处理字符串，当它遇到大于 127 的字节时，断定一个“字符串”只包含 ASCII 数据。

## What does the API look like to a user of ROS 2?

### Python 3

In Python the `str` type will be used for both strings and wide strings. Bytes of a known encoding should be converted to a `str` using [bytes.decode](https://docs.python.org/3/library/stdtypes.html#bytes.decode) before being assigned to a field.

> 在 Python 中，`str`类型既用于字符串，也用于宽字符串。在赋值给字段之前，应该使用[bytes.decode](https://docs.python.org/3/library/stdtypes.html#bytes.decode)将已知编码的字节转换为`str`。

**Example**

```python
import rclpy
from test_msgs.msg import WStrings


if __name__ == '__main__':
    rclpy.init()

    node = rclpy.create_node('talker')

    chatter_pub = node.create_publisher(WStrings, 'chatter', 1)

    msg = WStrings()
    msg.wstring_value = 'Hello Wörld'
    print('Publishing: "{0}"'.format(msg.wstring_value))
    chatter_pub.publish(msg)
    node.destroy_node()
  rclpy.shutdown()
```

### C++

In C++ wstring `wchar_t` has different sizes on different platforms (2 bytes on Windows, 4 bytes on Linux). Instead ROS 2 will use `char16_t` for characters of wide strings, and `std::u16string` for wide strings themselves.

> 在 C++的 wstring 中，wchar_t 在不同的平台上有不同的大小（在 Windows 上是 2 个字节，在 Linux 上是 4 个字节）。ROS 2 将使用 char16_t 来表示宽字符串的字符，并使用 std::u16string 来表示宽字符串本身。

**Example**

```cpp
/*
* Note that C++ source files containing unicode characters must begin with a byte order mark: https://en.wikipedia.org/wiki/Byte_order_mark .
* Failure to do so can result in an incorrect encoding of the characters on Windows.
* For an example, see https://github.com/ros2/system_tests/pull/362#issue-277436162
*/
#include <cstdio>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "test_msgs/msg/w_strings.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("talker");

  auto chatter_pub = node->create_publisher<test_msgs::msg::WStrings>("chatter", 10);

  test_msgs::msg::WStrings msg;
  std::u16string hello(u"Hello Wörld");
  msg.wstring_value = hello;
  chatter_pub->publish(msg);
  rclcpp::spin_some(node);

  return 0;
}
```
