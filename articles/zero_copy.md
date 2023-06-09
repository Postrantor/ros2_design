---
tip: translate by openai@2023-05-28 10:46:40
    layout: default
    title: Zero Copy via Loaned Messages
    permalink: articles/zero_copy.html
    abstract:
    author: '[Karsten Knese](https://github.com/karsten1987) [William Woodall](https://github.com/wjwwood) [Michael Carroll](https://github.com/mjcarroll)'
    date_written: 2020-02
    last_modified: 2020-04
    published: true
    
    Authors: {{ page.author }}
    Date Written: {{ page.date_written }}
    Last Modified: {% if page.last_modified %}{{ page.last_modified }}{% else %}{{ page.date_written }}{% endif %}
---
## Overview

There is a need to eliminate unnecessary copies throughout the ROS 2 stack to maximize performance and determinism.

> 需要消除 ROS 2 堆栈中的不必要的副本，以最大限度地提高性能和确定性。

In order to eliminate copies, the user must have more advanced control over memory management in the client library and middleware.

> 为了消除副本，用户必须在客户端库和中间件中拥有更高级别的内存管理控制。

One method of eliminating copies is via message loaning, that is the middleware can loan messages that are populated by the end user.

> 一种消除副本的方法是通过消息贷款，也就是说中间件可以借用由最终用户填充的消息。

This document outlines desired changes in the middleware and client libraries required to support message loaning.

> 这份文件概述了支持消息贷款所需的中间件和客户端库的所需变更。

## Motivation

The motivation for message loaning is to increase performance and determinism in ROS 2.

> 助言贷款的动机是为了提高 ROS 2 的性能和决定性。

As additional motivation, there are specific middleware implementations that allow for zero-copy via shared memory mechanisms.

> 作为额外的动力，有特定的中间件实现，可以通过共享内存机制实现零拷贝。

These enhancements would allow ROS 2 to take advantage of the shared memory mechanisms exposed by these implementations.

> 这些增强功能将使 ROS 2 能够充分利用这些实现提供的共享内存机制。

Examples of zero-copy transfer are [RTI Connext DDS Micro](https://community.rti.com/static/documentation/connext-micro/3.0.0/doc/html/usersmanual/zerocopy.html) and [Eclipse iceoryx](https://github.com/eclipse/iceoryx).

> 例子中的零复制传输有 [RTI Connext DDS Micro](https://community.rti.com/static/documentation/connext-micro/3.0.0/doc/html/usersmanual/zerocopy.html) 和 [Eclipse iceoryx](https://github.com/eclipse/iceoryx)。

### Publisher Use Cases

There are two primary kinds of use cases when publishing data which are relevant in this article:

> 在本文中，发布数据存在两种主要用例：

1. The user creates and owns an instance of a message which they wish to publish and then reuse after publishing.

> 用户创建并拥有一条需要发布的消息的实例，并在发布后可以重复使用。

2. The user borrows a message instance from the middleware, copies data into the message, and returns the ownership of the message during publish.

> 用户从中间件借用一个消息实例，将数据复制到消息中，并在发布时归还消息所有权。

Currently, only the first case is possible with the `rclcpp` API.

> 目前，只有使用 `rclcpp` API 才可能实现第一种情况。

After calling publish, the user still owns the message and may reuse it immediately.

> 在发布之后，用户仍然拥有该消息，可以立即重复使用。

In order to support the second use case, we need a way for the user to get at least a single message from the middleware, which they may then populate, then return when publishing.

> 为了支持第二种用例，我们需要一种方式让用户从中间件获取至少一条消息，然后填充它，然后在发布时返回。

In the second case, the memory that is used for the loaned message should be optionally provided by the middleware.

> 在第二种情况下，可以选择由中间件提供用于借记消息的内存。

For example, the middleware may use this opportunity to use a preallocated pool of message instances, or it may return instances of messages allocated in shared memory.

> 例如，中间件可以利用这个机会使用预先分配的消息实例池，或者它可以返回在共享内存中分配的消息实例。

However, if the middleware does not have special memory handling or pre-allocations, it may refuse to loan memory to the client library, and in that case, a user provided allocator should be used.

> 如果中间件没有特殊的内存处理或预分配，它可能拒绝向客户端库借内存，在这种情况下，应该使用用户提供的分配器。

This allows the user to have control over the allocation of the message when the middleware might otherwise use the standard allocator (new/malloc).

> 这允许用户在中间件可能使用标准分配器（new/malloc）时对消息的分配进行控制。

#### Additional Publisher Use Case

One additional publishing use case is allowing the user to loan the message to the middleware during asynchronous publishing.

> 一个额外的发布使用案例是允许用户在异步发布时将消息借给中间件。

This use case comes up when all or part of the data being published is located in memory that the middleware cannot allocate from, e.g. a hardware buffer via memory mapped I/O or something similar, and the user wants to still have zero copy and asynchronous publishing.

> 这种用例出现时，当要发布的所有或部分数据位于中间件无法从中分配的内存中，例如通过内存映射 I/O 或类似方式的硬件缓冲区，用户仍希望实现零拷贝和异步发布。

In this case, the user needs to call publish, then keep the message instance immutable until the middleware lets the user know that it is done with the loaned data.

> 在这种情况下，用户需要调用发布，然后保持消息实例不可变，直到中间件通知用户完成借用数据为止。

This is a narrow use case and will require additional interfaces to support, therefore it will be out of scope for this document, but it is mentioned for completion.

> 这是一个狭义的用例，需要额外的接口支持，因此它将超出本文档的范围，但为了完整起见而提及。

### Subscription Use Cases

There are two ways the users may take message instances from a subscription when data is available:

> 用户可以有两种方式从订阅中获取消息实例，当数据可用时：

1. Taking directly from the `Subscription` after polling it for data availability or waiting via a wait set.

> 从订阅中直接获取数据，或者通过等待集来等待数据的可用性。

2. Using an `Executor`, which takes the data from the user and delivers it via a user-defined callback.

> 使用执行器，从用户处获取数据，并通过用户定义的回调进行传递。

> [NOTE]:

In the first case, the user could choose to either:

> 在第一种情况下，用户可以选择：

- Manage the memory for the message instance themselves, providing a reference to it, into which the middleware should fill the data.

> 管理消息实例本身的内存，提供一个引用，中间件应该填充数据。

- Take one or more loaned messages from the middleware and return the loans later.

> 从中间件借用一个或多个消息，然后稍后归还贷款。

In the second case, the user is delegating the memory management of the client library via the `Executor`.The `Executor` may or may not borrow data from the middleware, but the user callback does not care, so this can be considered an implementation detail.

> 在第二种情况下，用户通过“执行者”委托客户端库的内存管理。“执行者”可能会或不会从中间件借用数据，但用户回调不在乎，因此这可以被认为是一个实现细节。

The user should be able to influence what the `Executor` does, and in the case that memory needs to be allocated, the user should be able to provide an allocator or memory management strategy which would influence the `Executor`'s behavior.

> 用户应该能够影响执行者的行为，如果需要分配内存，用户应该能够提供一个分配器或内存管理策略，这将影响执行者的行为。

## Requirements

Based on the use cases above, the general requirements are as follows:

> 根据上述用例，一般要求如下：

- Users must be able to avoid all memory operations and copies in at least one configuration.

> 用户至少必须能够在一种配置中避免所有的内存操作和复制。

### Publisher Requirements

- The user must be able to publish from messages allocated in their stack or heap.

> 用户必须能够从他们的堆栈或堆中分配的消息发布。

- The user must be able to get a loaned message, use it, and return it during publication.

> 用户必须能够获取借来的信息，使用它，并在发表时归还它。

- When taking a loan, the middleware should not do anything "special", that is that the user must be able to influence the allocation.

> 当借款时，中间件不应该做任何“特殊”的事情，也就是说用户必须能够影响分配。

### Subscription Requirements

The following requirements should hold whether the user is polling or using a wait set.

> 以下要求无论用户是轮询还是使用等待集，都应该遵守。

- The user must be able to have the middleware fill data into messages allocated in the user's stack or heap.

> 用户必须能够让中间件将数据填充到用户堆栈或堆上分配的消息中。

- The user must be able to get a loaned message from the middleware when calling take.

> 用户调用 take 时必须能从中间件获得借用的消息。

- The user must be able to get a sequence of loaned messages from the middleware when calling take.

> 用户调用 take 时，必须能够从中间件获取一系列借用的消息。

- The loaned message or sequence must be returned by the user.

> 用户必须归还借用的消息或序列。

- In general, users should return loans as soon as feasibly possible, as the underlying mechanism has finite resources to loan messages from. Holding loans too long may cause messages to dropped or publications to stall.

> .

一般而言，用户应尽可能快地归还贷款，因为底层机制只有有限的资源可以借用消息。借贷过久可能会导致消息丢失或发布停止。

- When not taking a loan, the middleware should not do anything "special", that is that the user must be able to influence the allocation.

> 当不贷款时，中间件不应做任何“特殊”的事情，也就是说用户必须能够影响分配。

### Special Requirements

These requirements are driven by idiosyncrasies of various middleware implementations and some of their special operating modes, e.g. zero copy:

> 这些要求是由各种中间件实现的特殊特性和一些特殊操作模式驱动的，例如零复制。

#### RTI Connext DDS

- The Connext API (more generally the DDS API) requires that the user to use a sequence of messages when taking or reading.

> 使用 Connext API（更一般地说是 DDS API）时，用户需要使用一系列的消息来进行发送或读取。

- This means the ROS API needs to do the same, otherwise the middleware would be giving a "loan" to a message in a sequence, but it would also need to keep the sequence immutable.

> 这意味着 ROS API 也需要这样做，否则中间件将向消息序列提供"贷款"，但也需要保持序列不变。

- Needs to keep sample and sample info together, therefore `rclcpp` loaned message sequence needs to own both somehow.

> 需要把样本和样本信息放在一起，因此 `rclcpp` 借来的消息序列需要以某种方式拥有它们。

Connext Micro Specific (ZeroCopy):

> Connext Micro 特定（零拷贝）：

- The user needs to be able to check if data has been invalidated after reading it.

> 用户需要能够在读取数据后检查数据是否已失效。

### Nice to Haves

- When using loaned messages, the overhead should be minimized, so that it can be used as the general approach as often as possible.

> 当使用借用的消息时，应尽量减少开销，以便尽可能经常使用这种通用方法。

- Versus reusing a user owned message on the stack or heap.

> 对比重复使用用户拥有的堆栈或堆上的消息。

- We will still recommend reusing a message repeatedly from the stack or heap of the user.

> 我们仍然建议用户从堆栈或堆中重复使用消息。

- Memory operations and copies should be avoided anywhere possible.

> 尽可能避免内存操作和复制。

## Design Proposal

### RMW API

Introduce APIs for creating/destroying loaned messages, as well as structure for management:

> 引入 API 来创建/销毁借用的消息，以及管理结构：

```cpp
    void *
    rmw_borrow_loaned_message(
      const rmw_publisher_t * publisher,
      const rosidl_message_type_support_t * type_support,
      void ** ros_message
    );

    rmw_ret_t
    rmw_return_loaned_message(
      const rmw_publisher_t * publisher,
      void * loaned_message

  );

> 请帮助我翻译

```

> `简体中文`

Extend publisher API for loaned messages:

> 扩展发布者 API 以支持借出的消息

```cpp
    rmw_ret_t
    rmw_publish_loaned_message(
      const rmw_publisher_t * publisher,
      const void * ros_message,
      rmw_publisher_allocation_t * allocation
    );
```

Extend subscription API for taking loaned message.

> 扩展订阅 API 以贷款发送消息。

Because a subscription does not necessarily allocate new memory for the loaned message during a take, the message only needs to be released rather than returned.

> 因为订阅在取走消息时不一定需要为借出的消息分配新的内存，因此只需要释放消息而不需要返回它。

```cpp
    rmw_ret_t
    rmw_take_loaned_message(
      const rmw_subscription_t * subscription,
      void ** loaned_message,
      bool * taken,
      rmw_subscription_allocation_t * allocation
    );

    rmw_ret_t
    rmw_release_loaned_message(
      const rmw_subscription_t * subscription,
      void * loaned_message
    );
```

### RCL API

Introduce APIs for creating/destroying loaned messages, as well as structure for management:

> 引入 API 来创建/销毁借出的消息，以及管理结构：

```cpp
    void *
    rcl_borrow_loaned_message(
      const rcl_publisher_t * publisher,
      const rosidl_message_type_support_t * type_support,
      void ** ros_message
    );

    rcl_ret_t
    rcl_return_loaned_message(
      const rcl_publisher_t * publisher,
      rcl_loaned_message_t * loaned_message
    );
```

Extend publisher API for loaned messages:

> 扩展出版商 API 以支持借阅的消息

```cpp
    rcl_ret_t
    rcl_publish_loaned_message(
      const rcl_publisher_t * publisher,
      const void * ros_message,
      rmw_publisher_allocation_t * allocation
    );

    bool
    rcl_publisher_can_loan_messages(const rcl_publisher_t * publisher);
```

Extend subscription API for taking loaned message:

> 扩展订阅 API 以获取贷款信息：

```cpp
    rcl_ret_t
    rcl_take_loaned_message(
      const rcl_subscription_t * subscription,
      void ** loaned_message,
      rmw_message_info_t * message_info,
      rmw_subscription_allocation_t * allocation
    );

    rcl_ret_t
    rcl_release_loaned_message(
      const rcl_subscription_t * subscription,
      void * loaned_message
    );

    bool
    rcl_subscription_can_loan_messages(const rcl_subscription_t * subscription);
```

### RCLCPP LoanedMessage

In order to support loaned messages in `rclcpp`, we introduce the concept of a `LoanedMessage`.

> 为了支持 rclcpp 中的借用消息，我们引入了“借用消息”的概念。

A `LoanedMessage` provides a wrapper around the underlying loan mechanisms, and manages the loan's lifecycle.

> 一个 `LoanedMessage` 提供了一个包裹基础贷款机制的外壳，并管理贷款的生命周期。

```cpp
    template <class MsgT, typename Alloc = std::allocator<void>>
    class LoanedMessage
    {
    public:
      // Get the underlying message
      MsgT& get()

      // Check if underlying message is valid and consistent
      bool is_consistent();

      // Loan message from the middleware, if loaning is not available,
      // allocate memory using Alloc
      LoanedMessage(
        const rclcpp::Publisher<MsgT, Alloc> * pub);

    private:
      const rclcpp::Publisher<MsgT, Alloc> * pub_;

      // Holds details of the message loan
      rcl_loaned_message_t * loaned_msg_;

      // Will be initialized with memory from loaned_msg_, otherwise allocated.
      std::unique_ptr<MsgT> msg_;
    };
```

### RCLCPP Publisher

Extend RCLCPP API:

> 扩展 RCLCPP API：

```cpp
    // Return a loaned message from the middleware
    rclcpp::LoanedMessage<MsgT, Alloc>
    rclcpp::Publisher::loan_message()

    // Publish a loaned message, returning the loan
    void
    rclcpp::Publisher::publish(std::unique_ptr<rclcpp::LoanedMessage<MsgT, Alloc>> loaned_msg);

    // Test if the middleware supports loaning messages
    bool
    rclcpp::Publisher::can_loan_messages()
```

### RCLCPP Subscription

```cpp
    /// Test if the middleware supports loaning messages
    bool
    rclcpp::Subscription::can_loan_messages()

    /// Subscription path for middlewares supporting loaned messages.
    void
    rclcpp::Subscription::handle_loaned_message(void * loaned_message, const rmw_message_info_t & message_info)
```

### ROS_DISABLE_LOANED_MESSAGES

By default, `Loaned Messages` will try to borrow memory from the underlying middleware.

> 默认情况下，“借贷消息”将尝试从底层中间件借用内存。

The `ROS_DISABLE_LOANED_MESSAGES` environment variable is provided so the user can disable `Loaned Messages` and fallback to normal publisher / subscription without any code change or middleware configuration.

> 环境变量 ROS_DISABLE_LOANED_MESSAGES 提供给用户，以便用户可以禁用 Loaned Messages，而无需更改代码或中间件配置即可回退到正常的发布者/订阅者。

How to disable `Loaned Messages`:

> 如何禁用“借出的消息”：

```cpp
    export ROS_DISABLE_LOANED_MESSAGES=1
```

## Additional Considerations

### Loaning non-POD messages

This design document is limited to handling POD message types only.

> 这份设计文档仅限于处理 POD 消息类型。

When performing a loan on a non-POD message, there is the additional consideration of ensuring that the allocators match between the `rclcpp` implementation and the underlying middleware.

> 在对非 POD 消息执行贷款时，还需要考虑确保 `rclcpp` 实现与底层中间件之间的分配器匹配。

With the current message structures, the allocator itself may have an impact on the `sizeof()` the message type.

> 在当前的消息结构中，分配器本身可能会对消息类型的 `sizeof()` 产生影响。

This is best illustrated via the following example:

> 这最好通过下面的例子来说明：

```cpp
    #include <string>

    // Example message with two allocated fields a and b.
    template <typename AllocatorT>
    struct Msg
    {
      std::basic_string<char, std::char_traits<char>, AllocatorT> a;
      int c;
      std::basic_string<char, std::char_traits<char>, AllocatorT> b;
    };

    // Custom allocator with enough padding to change the sizeof(MyAllocator)
    template <class T>
    struct MyAllocator : public std::allocator<T>
    {
      char padding[16];
    };

    int main()
    {
      // Example message with the standard allocator.
      Msg<std::allocator<char>> foo;
      void * ptr = &foo;
      // Same message contents, but casted to type including custom allocator.
      auto & bar = *static_cast<Msg<MyAllocator<char>> *>(ptr);

      // size of foo and bar are different on macOS and Linux with clang
      // address of foo and bar should be the same
      // address of foo.c and bar.c are different on macOS and Linux with clang.
      printf("sizeof(foo) == %zu\n", sizeof(foo));
      printf("&foo == %p\n", static_cast<void *>(&foo));
      printf("&foo.c == %p\n", static_cast<void *>(&foo.c));
      printf("sizeof(bar) == %zu\n", sizeof(bar));
      printf("&bar == %p\n", static_cast<void *>(&bar));
      printf("&bar.c == %p\n", static_cast<void *>(&bar.c));
      return 0;
    }
```

Example output using `g++ --std=c++14`

> 示例输出使用 `g++ --std=c++14`

```cpp
    sizeof(foo) == 72
    &foo == 0x7ffde46e0680
    &foo.c == 0x7ffde46e06a0
    sizeof(bar) == 72
    &bar == 0x7ffde46e0680
    &bar.c == 0x7ffde46e06a0
```

Example output using `clang++-7 -stdlib=libc++`

> 示例输出使用 `clang++-7 -stdlib=libc++`

```cpp
    sizeof(foo) == 56
    &foo == 0x7ffc37971460
    &foo.c == 0x7ffc37971478
    sizeof(bar) == 88
    &bar == 0x7ffc37971460
    &bar.c == 0x7ffc37971488
```
