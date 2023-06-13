---
tip: translate by openai@2023-06-13 23:01:26
...
---
author: "[Karsten Knese](https://github.com/karsten1987) [William Woodall](https://github.com/wjwwood) [Michael Carroll](https://github.com/mjcarroll)"

> 作者：[Karsten Knese](https://github.com/karsten1987) [William Woodall](https://github.com/wjwwood) [Michael Carroll](https://github.com/mjcarroll)
> date_written: 2020-02
> last_modified: 2020-04
> layout: default

permalink: articles/zero_copy.html

> 链接：文章/零复制.html
> published: true
> title: Zero Copy via Loaned Messages

---

## Overview

There is a need to eliminate unnecessary copies throughout the ROS 2 stack to maximize performance and determinism. In order to eliminate copies, the user must have more advanced control over memory management in the client library and middleware. One method of eliminating copies is via message loaning, that is the middleware can loan messages that are populated by the end user. This document outlines desired changes in the middleware and client libraries required to support message loaning.

> 需要消除 ROS 2 堆栈中的不必要副本，以最大限度地提高性能和确定性。为了消除副本，用户必须在客户端库和中间件中拥有更高级别的内存管理控制。消除副本的一种方法是通过消息贷款，即中间件可以借用由最终用户填充的消息。本文概述了中间件和客户端库所需的支持消息贷款的所需变更。

## Motivation

The motivation for message loaning is to increase performance and determinism in ROS 2.

> 鼓励消息借用的目的是为了提高 ROS 2 的性能和决定性。

As additional motivation, there are specific middleware implementations that allow for zero-copy via shared memory mechanisms. These enhancements would allow ROS 2 to take advantage of the shared memory mechanisms exposed by these implementations. Examples of zero-copy transfer are [RTI Connext DDS Micro](https://community.rti.com/static/documentation/connext-micro/3.0.0/doc/html/usersmanual/zerocopy.html) and [Eclipse iceoryx](https://github.com/eclipse/iceoryx).

> 作为额外的动力，有特定的中间件实现可以通过共享内存机制实现零拷贝。这些增强功能可以让 ROS 2 利用这些实现暴露的共享内存机制。零拷贝传输的示例有 [RTI Connext DDS Micro](https://community.rti.com/static/documentation/connext-micro/3.0.0/doc/html/usersmanual/zerocopy.html) 和 [Eclipse iceoryx](https://github.com/eclipse/iceoryx)。

### Publisher Use Cases

There are two primary kinds of use cases when publishing data which are relevant in this article:

> 在本文中，发布数据的两种主要用例相关：

1. The user creates and owns an instance of a message which they wish to publish and then reuse after publishing.

> 用户创建并拥有一条消息的实例，他们希望在发布后能重复使用。

2. The user borrows a message instance from the middleware, copies data into the message, and returns the ownership of the message during publish.

> 用户从中间件借用一个消息实例，将数据复制到消息中，并在发布时返回消息的所有权。

Currently, only the first case is possible with the `rclcpp` API. After calling publish, the user still owns the message and may reuse it immediately.

> 目前，只有使用 `rclcpp` API 才能实现第一种情况。调用发布后，用户仍然拥有该消息，可以立即重复使用。

In order to support the second use case, we need a way for the user to get at least a single message from the middleware, which they may then populate, then return when publishing.

> 为了支持第二个用例，我们需要一种方法，让用户从中间件获取至少一条消息，然后填充它，在发布时再返回。

In the second case, the memory that is used for the loaned message should be optionally provided by the middleware. For example, the middleware may use this opportunity to use a preallocated pool of message instances, or it may return instances of messages allocated in shared memory. However, if the middleware does not have special memory handling or pre-allocations, it may refuse to loan memory to the client library, and in that case, a user provided allocator should be used. This allows the user to have control over the allocation of the message when the middleware might otherwise use the standard allocator (new/malloc).

> 在第二种情况下，用于贷款消息的内存可以由中间件提供。例如，中间件可以利用这个机会使用预分配的消息实例池，或者它可以返回在共享内存中分配的消息实例。但是，如果中间件没有特殊的内存处理或预分配，它可能会拒绝向客户端库提供内存，在这种情况下，应使用用户提供的分配器。这允许用户在中间件可能使用标准分配器（new/malloc）的情况下控制消息的分配。

#### Additional Publisher Use Case

One additional publishing use case is allowing the user to loan the message to the middleware during asynchronous publishing. This use case comes up when all or part of the data being published is located in memory that the middleware cannot allocate from, e.g. a hardware buffer via memory mapped I/O or something similar, and the user wants to still have zero copy and asynchronous publishing. In this case, the user needs to call publish, then keep the message instance immutable until the middleware lets the user know that it is done with the loaned data.

> 一种额外的发布用例是允许用户在异步发布期间将消息借给中间件。当所有或部分要发布的数据位于中间件无法分配的内存中时，例如通过内存映射 I/O 或类似的硬件缓冲区，用户希望仍然具有零复制和异步发布，此时就会出现此用例。在这种情况下，用户需要调用发布，然后保持消息实例不变，直到中间件告知用户它完成了借用的数据。

This is a narrow use case and will require additional interfaces to support, therefore it will be out of scope for this document, but it is mentioned for completion.

> 这是一个狭窄的用例，需要额外的接口来支持，因此它将超出本文档的范围，但为了完整起见，还是提及一下。

### Subscription Use Cases

There are two ways the users may take message instances from a subscription when data is available:

> 用户可以通过两种方式从订阅中获取消息实例，当数据可用时：

1. Taking directly from the `Subscription` after polling it for data availability or waiting via a wait set.

> 从订阅中直接轮询数据的可用性或通过等待集等待。

2. Using an `Executor`, which takes the data from the user and delivers it via a user-defined callback.

> 使用一个 Executor，从用户那里获取数据，并通过用户定义的回调函数进行传递。

Note: It is assumed that the user will be able to take multiple messages at a time if they are available.

In the first case, the user could choose to either:

> 在第一种情况下，用户可以选择：

- Manage the memory for the message instance themselves, providing a reference to it, into which the middleware should fill the data.

> 管理消息实例的内存，提供一个引用，中间件应该将数据填充到其中。

- Take one or more loaned messages from the middleware and return the loans later.

In the second case, the user is delegating the memory management of the client library via the `Executor`.The `Executor` may or may not borrow data from the middleware, but the user callback does not care, so this can be considered an implementation detail. The user should be able to influence what the `Executor` does, and in the case that memory needs to be allocated, the user should be able to provide an allocator or memory management strategy which would influence the `Executor`'s behavior.

> 在第二种情况下，用户通过“执行器”委托客户端库的内存管理。“执行器”可能会或可能不会从中间件借用数据，但用户回调不关心，因此可以将其视为实现细节。用户应该能够影响“执行器”的行为，并且在需要分配内存的情况下，用户应该能够提供分配器或内存管理策略，以影响“执行器”的行为。

## Requirements

Based on the use cases above, the general requirements are as follows:

> 根据上述用例，一般要求如下：

- Users must be able to avoid all memory operations and copies in at least one configuration.

### Publisher Requirements

- The user must be able to publish from messages allocated in their stack or heap.
- The user must be able to get a loaned message, use it, and return it during publication.
- When taking a loan, the middleware should not do anything "special", that is that the user must be able to influence the allocation.

> 当借贷时，中间件不应该做任何“特殊”的事情，也就是说用户必须能够影响分配。

### Subscription Requirements

The following requirements should hold whether the user is polling or using a wait set.

> 以下要求无论用户是轮询还是使用等待集，都应该遵守。

- The user must be able to have the middleware fill data into messages allocated in the user's stack or heap.

> 用户必须能够让中间件将数据填充到用户堆栈或堆上分配的消息中。

- The user must be able to get a loaned message from the middleware when calling take.
- The user must be able to get a sequence of loaned messages from the middleware when calling take.

> 用户调用 take 时，必须能够从中间件获取一系列借出的消息。

- The loaned message or sequence must be returned by the user.

  - In general, users should return loans as soon as feasibly possible, as the underlying mechanism has finite resources to loan messages from. Holding loans too long may cause messages to dropped or publications to stall.

> 一般来说，用户应尽可能快地归还贷款，因为底层机制有限的资源来借用消息。持有贷款太久可能会导致消息被丢弃或出版物停滞。

- When not taking a loan, the middleware should not do anything "special", that is that the user must be able to influence the allocation.

> 当不贷款时，中间件不应该做任何“特殊”的事情，也就是说用户必须能够影响分配。

### Special Requirements

These requirements are driven by idiosyncrasies of various middleware implementations and some of their special operating modes, e.g. zero copy:

> 这些要求是由各种中间件实现和其一些特殊操作模式（例如零复制）所驱动的。

#### RTI Connext DDS

- The Connext API (more generally the DDS API) requires that the user to use a sequence of messages when taking or reading.

> 使用 Connext API（更普遍的 DDS API）时，用户需要使用一系列消息来接收或读取。

- This means the ROS API needs to do the same, otherwise the middleware would be giving a "loan" to a message in a sequence, but it would also need to keep the sequence immutable.

> 这意味着 ROS API 也需要做同样的事情，否则中间件将会在序列中给予消息一笔“贷款”，但也需要保持序列不可变。

- Needs to keep sample and sample info together, therefore `rclcpp` loaned message sequence needs to own both somehow.

> 需要将样本和样本信息放在一起，因此 `rclcpp` 借来的消息序列需要以某种方式拥有它们。

Connext Micro Specific (ZeroCopy):

> 连接微小特定（零拷贝）：

- The user needs to be able to check if data has been invalidated after reading it.

### Nice to Haves

- When using loaned messages, the overhead should be minimized, so that it can be used as the general approach as often as possible.

> 当使用借用的消息时，应尽量减少开销，以便尽可能经常使用这种通用方法。

- Versus reusing a user owned message on the stack or heap.
- We will still recommend reusing a message repeatedly from the stack or heap of the user.
- Memory operations and copies should be avoided anywhere possible.

## Design Proposal

### RMW API

Introduce APIs for creating/destroying loaned messages, as well as structure for management:

> 为创建/销毁贷款消息引入 API，以及管理结构：

```
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
```

Extend publisher API for loaned messages:

> 扩展发布者 API 以支持借出的消息

```
    rmw_ret_t
    rmw_publish_loaned_message(
      const rmw_publisher_t * publisher,
      const void * ros_message,
      rmw_publisher_allocation_t * allocation
    );
```

Extend subscription API for taking loaned message. Because a subscription does not necessarily allocate new memory for the loaned message during a take, the message only needs to be released rather than returned.

> 扩展订阅 API 以接收借用的消息。由于订阅期间不一定需要为借用的消息分配新的内存，因此在接收时只需要释放消息，而不需要返回消息。

```
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

> 为创建/销毁借贷消息提供 API，以及管理结构：

```
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

> 扩展发布者 API 以支持借用的消息

```
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

> 扩展订阅 API 以获取借贷消息

```
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

In order to support loaned messages in `rclcpp`, we introduce the concept of a `LoanedMessage`. A `LoanedMessage` provides a wrapper around the underlying loan mechanisms, and manages the loan's lifecycle.

> 为了支持 `rclcpp` 中的借用消息，我们引入了 `LoanedMessage` 概念。`LoanedMessage` 提供了一个包装底层借用机制的外壳，并管理借用的生命周期。

```
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

```
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

```
    /// Test if the middleware supports loaning messages
    bool
    rclcpp::Subscription::can_loan_messages()

    /// Subscription path for middlewares supporting loaned messages.
    void
    rclcpp::Subscription::handle_loaned_message(void * loaned_message, const rmw_message_info_t & message_info)
```

### ROS_DISABLE_LOANED_MESSAGES

By default, `Loaned Messages` will try to borrow memory from the underlying middleware. The `ROS_DISABLE_LOANED_MESSAGES` environment variable is provided so the user can disable `Loaned Messages` and fallback to normal publisher / subscription without any code change or middleware configuration.

> 默认情况下，“借贷消息”将尝试从底层中间件借用内存。提供了 `ROS_DISABLE_LOANED_MESSAGES` 环境变量，以便用户可以禁用“借贷消息”并回退到正常发布者/订阅者，而无需更改代码或配置中间件。

How to disable `Loaned Messages`:

> 如何禁用“借来的消息”？

```
    export ROS_DISABLE_LOANED_MESSAGES=1
```

## Additional Considerations

### Loaning non-POD messages

This design document is limited to handling POD message types only. When performing a loan on a non-POD message, there is the additional consideration of ensuring that the allocators match between the `rclcpp` implementation and the underlying middleware. With the current message structures, the allocator itself may have an impact on the `sizeof()` the message type.

> 此设计文档仅限于处理 POD 消息类型。在对非 POD 消息执行贷款时，还需要考虑确保 `rclcpp` 实现与底层中间件之间的分配器匹配。使用当前的消息结构，分配器本身可能会对消息类型的 `sizeof()` 产生影响。

This is best illustrated via the following example:

> 这最好通过以下例子说明：

```
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

```
    sizeof(foo) == 72
    &foo == 0x7ffde46e0680
    &foo.c == 0x7ffde46e06a0
    sizeof(bar) == 72
    &bar == 0x7ffde46e0680
    &bar.c == 0x7ffde46e06a0
```

Example output using `clang++-7 -stdlib=libc++`

> 例如使用 `clang++-7 -stdlib=libc++` 的输出

```
    sizeof(foo) == 56
    &foo == 0x7ffc37971460
    &foo.c == 0x7ffc37971478
    sizeof(bar) == 88
    &bar == 0x7ffc37971460
    &bar.c == 0x7ffc37971488

```
