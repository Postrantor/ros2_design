---
tip: translate by openai@2023-06-13 22:55:06
...
---
layout: default
title: ROS 2 DDS-Security integration

permalink: articles/ros2_dds_security.html

> 链接：文章/ros2_dds_security.html
> abstract: >

Robotics is full of experimentation: evaluating different hardware and software, pushing ahead with what works, and culling what doesn't.

> 机器人技术充满了实验：评估不同的硬件和软件，推进有效的技术，并剔除无效的技术。

ROS was designed to be flexible to enable this experimentation; to allow existing components to easily be combined with new ones or swapped with others.

> ROS 被设计成灵活的，以便于进行实验；可以轻松地将现有组件与新组件结合在一起，或者将其与其他组件互换。

In ROS 1, this flexibility was valued above all else, at the cost of security.

> 在 ROS 1 中，这种灵活性被看作是最重要的，不惜牺牲安全性。

By virtue of being designed on top of DDS, ROS 2 is able to retain that flexibility while obtaining the ability to be secured by properly utilizing the DDS-Security specification.

> 通过在 DDS 之上设计，ROS 2 能够保留这种灵活性，同时通过正确使用 DDS-Security 规范获得安全能力。

This article describes how ROS 2 integrates with DDS-Security.

> 这篇文章描述了 ROS 2 如何与 DDS-Security 集成。
> author: >

[Kyle Fazzari](https://github.com/kyrofa)

> [Kyle Fazzari](https://github.com/kyrofa)（凯尔·法扎里）
> date_written: 2019-07
> last_modified: 2020-07
> published: true
> categories: Security

---

# 

<div class="abstract" markdown="1">

</div>

Authors:

Date Written:

Last Modified:

# DDS-Security overview

The [DDS-Security specification][dds_security] expands upon the [DDS specification][dds], adding security enhancements by defining a Service Plugin Interface (SPI) architecture, a set of builtin implementations of the SPIs, and the security model enforced by the SPIs.

> DDS-Security 规范扩展了 DDS 规范，通过定义服务插件接口（SPI）架构、一组内置的 SPI 实现以及 SPI 执行的安全模型，来增强安全性。

Specifically, there are five SPIs defined:

> 具体来说，有五个 SPI 定义：

- **Authentication**: Verify the identity of a given domain participant.
- **Access control**: Enforce restrictions on the DDS-related operations that can be performed by an authenticated domain participant.

> 控制访问：强制执行对已认证的域参与者可以执行的 DDS 相关操作的限制。

- **Cryptographic**: Handle all required encryption, signing, and hashing operations.
- **Logging**: Provide the ability to audit DDS-Security-related events.
- **Data tagging**: Provide the ability to add tags to data samples.

ROS 2's security features currently utilize only the first three.

> ROS 2 的安全特性目前只使用前三个。

This is due to the fact that neither **Logging** nor **Data Tagging** are required in order to be compliant with the [DDS-Security spec][dds_security] (see section 2.3), and thus not all DDS implementations support them.

> 由于根据 [DDS-Security spec][dds_security]（见第 2.3 节），不需要**日志记录**和**数据标记**就能符合要求，因此并不是所有的 DDS 实现都支持它们，这就是原因。

Let's delve a little further into those first three plugins.

> 让我们更深入地研究一下这前三个插件吧。

## Authentication

The **Authentication** plugin (see section 8.3 of the [DDS-Security spec][dds_security]) is central to the entire SPI architecture, as it provides the concept of a confirmed identity without which further enforcement would be impossible (e.g. it would be awfully hard to make sure a given ROS identity could only access specific topics if it was impossible to securely determine which identity it was).

> 插件**认证**（参见[DDS-Security spec] [dds_security] 的第 8.3 节）是整个 SPI 架构的核心，它提供了确认身份的概念，没有它，进一步的强制执行将是不可能的（例如，如果无法安全地确定其身份，就很难确保给定的 ROS 身份只能访问特定的主题）。

The SPI architecture allows for a number of potential authentication schemes, but ROS 2 uses the builtin authentication plugin (called "DDS:Auth:PKI-DH", see section 9.3 of the [DDS-Security spec][dds_security]), which uses the proven Public Key Infrastructure (PKI).

> SPI 架构允许使用多种潜在的认证方案，但 ROS 2 使用内置的认证插件（称为“DDS：Auth：PKI-DH”，参见 [DDS-Security spec][dds_security] 的第 9.3 节），该插件使用经过验证的公钥基础设施（PKI）。

It requires a public and private key per domain participant, as well as an x.509 certificate that binds the participant's public key to a specific name.

> 需要每个域参与者一个公钥和一个私钥，以及一个将参与者的公钥绑定到特定名称的 x.509 证书。

Each x.509 certificate must be signed by (or have a signature chain to) a specific Certificate Authority (CA) that the plugin is configured to trust.

> 每个 x.509 证书必须由（或具有对插件配置为信任的特定证书颁发机构（CA）的签名链）签名。

The rationale for using the builtin plugin as opposed to anything else is twofold:

> 使用内置插件而不是其他任何东西的理由有两个：

1. It's the only approach described in detail by the spec.

> 这是规范中唯一详细描述的方法。

2. It's mandatory for all compliant DDS implementations to interoperably support it (see section 2.3 of the [DDS-Security spec][dds_security]), which makes the ROS 2 security features work across vendors with minimal effort.

> 所有符合 DDS 规范的实现都必须互相支持（参见 [DDS-Security spec][dds_security] 中的 2.3 节），这使得 ROS 2 安全功能可以跨厂商轻松实现。

## Access control

The **Access control** plugin (see section 8.4 of the [DDS-Security spec][dds_security]) deals with defining and enforcing restrictions on the DDS-related capabilities of a given domain participant.

> 控制访问插件（参见 [DDS-Security spec][dds_security] 的第 8.4 节）处理定义和实施给定域参与者的 DDS 相关能力的限制。

For example, it allows a user to restrict a particular participant to a specific DDS domain, or only allow the participant to read from or write to specific DDS topics, etc.

> 例如，它允许用户将特定参与者限制在特定的 DDS 域中，或者仅允许参与者从特定的 DDS 主题中读取或写入等。

Again the SPI architecture allows for some flexibility in how the plugins accomplish this task, but ROS 2 uses the builtin access control plugin (called "DDS:Access:Permission", see section 9.4 of the [DDS-Security spec][dds_security]), which again uses PKI.

> 再次，SPI 架构允许插件以某种灵活的方式完成这项任务，但 ROS 2 使用内置的访问控制插件（称为“DDS：Access：Permission”，请参见 [DDS-Security规范][dds_security]的第 9.4 节），该插件又使用 PKI。

It requires two files per domain participant:

> 它每个域参与者需要两个文件。

- **Governance** file: A signed XML document specifying how the domain should be secured.
- **Permissions** file: A signed XML document containing the permissions of the domain participant, bound to the name of the participant as defined by the authentication plugin (which is done via an x.509 cert, as we just discussed).

> 文件：-**权限**文件：一个签署的 XML 文档，包含域参与者的权限，绑定到身份验证插件（通过 x.509 证书完成，正如我们刚刚讨论的）定义的参与者的名称。

Both of these files must be signed by a CA which the plugin is configured to trust.

> 这两个文件必须由插件配置为信任的 CA 签名。

This may be the same CA that the **Authentication** plugin trusts, but that isn't required.

> 这可能是认证插件信任的同一个 CA，但不是必须的。

The rationale for using the builtin plugin as opposed to anything else is the same as the **Authentication** plugin:

> 使用内置插件而不是其他任何东西的原因与**身份验证**插件相同：

1. It's the only approach described in detail by the spec.

> 这是规范中唯一详细描述的方法。

2. It's mandatory for all compliant DDS implementations to interoperably support it (see section 2.3 of the [DDS-Security spec][dds_security]), which makes the ROS 2 security features work across vendors with minimal effort.

> 2. 所有符合 DDS 规范的实现都必须互操作地支持它（参见 [DDS-Security规范][dds_security]的第 2.3 节），这使得 ROS 2 安全功能可以轻松跨厂商工作。

## Cryptographic

The **Cryptographic** plugin (see section 8.5 of the [DDS-Security spec][dds_security]) is where all the cryptography-related operations are handled: encryption, decryption, signing, hashing, etc.

> 技术加密插件（参见 DDS-Security 规范第 8.5 节）处理所有与加密相关的操作：加密、解密、签名、散列等。

Both the **Authentication** and **Access control** plugins utilize the capabilities of the **Cryptographic** plugin in order to verify signatures, etc.

> 两个认证插件和访问控制插件都利用加密插件的功能来验证签名等。

This is also where the functionality to encrypt DDS topic communication resides.

> 这也是加密 DDS 主题通信功能所在的地方。

While the SPI architecture again allows for a number of possibilities, ROS 2 uses the builtin cryptographic plugin (called "DDS:Crypto:AES-GCM-GMAC", see section 9.5 of the [DDS-Security spec][dds_security]), which provides authenticated encryption using Advanced Encryption Standard (AES) in Galois Counter Mode (AES-GCM).

> 虽然 SPI 架构又允许多种可能性，ROS 2 使用内置的加密插件（称为“DDS：Crypto：AES-GCM-GMAC”，参见 [DDS-Security spec][dds_security] 的第 9.5 节），该插件提供使用高级加密标准（AES）在 Galois 计数模式（AES-GCM）下的认证加密。

The rationale for using the builtin plugin as opposed to anything else is the same as the other plugins:

> 使用内置插件而不是其他任何东西的原因和其他插件是一样的：

1. It's the only approach described in detail by the spec.

> 这是规范中唯一详细描述的方法。

2. It's mandatory for all compliant DDS implementations to interoperably support it (see section 2.3 of the [DDS-Security spec][dds_security]), which makes the ROS 2 security features work across vendors with minimal effort.

> 所有符合 DDS 规范的实现都必须互相兼容地支持它（请参阅 DDS-Security 规范的第 2.3 节），这使得 ROS 2 安全功能可以轻松跨厂商访问。

# DDS-Security integration with ROS 2: SROS 2

Now that we have established some shared understanding of how security is supported in DDS, let's discuss how that support is exposed in ROS 2.

> 现在我们已经建立了一些关于如何在 DDS 中支持安全性的共同理解，让我们来讨论一下这种支持是如何在 ROS 2 中暴露出来的。

By default, none of the security features of DDS are enabled in ROS 2.

> 默认情况下，ROS 2 中没有启用 DDS 的安全功能。

The set of features and tools in ROS 2 that are used to enable them are collectively named "Secure ROS 2" (SROS 2).

> SROS 2 是 ROS 2 中用于启用安全性的一组功能和工具的总称。

## Features in the ROS client library (RCL)

Most of the user-facing runtime support for SROS 2 is contained within the [ROS Client Library](https://github.com/ros2/rcl).

> 大多数用户面向的 SROS 2 运行时支持都包含在 [ROS 客户端库](https://github.com/ros2/rcl)中。

Once its requirements are satisfied it takes care of configuring the middleware support for each supported DDS implementation.

> 一旦满足其需求，它就会为每个支持的 DDS 实现配置中间件支持。

RCL includes the following features for SROS 2:

> RCL 为 SROS 2 提供以下功能：

- Support for security files for each domain participant.
- Support for both permissive and strict enforcement of security.
- Support for a master "on/off" switch for all SROS 2 features.

Let's discuss each of these in turn.

> 让我们依次讨论这些。

### Security files for each domain participant

As stated earlier, the DDS-Security plugins require a set of security files (e.g. keys, governance and permissions files, etc.) per domain participant.

> 正如前面所述，DDS-Security 插件需要每个域参与者一组安全文件（例如密钥，治理和权限文件等）。

Domain participants map to a context within process in ROS 2, so each process requires a set of these files.

> 在 ROS 2 中，域参与者映射到过程的上下文，因此每个过程都需要一组这些文件。

RCL supports being pointed at a directory containing security files in two different ways:

> RCL 支持通过两种不同的方式指向包含安全文件的目录：

- Directory tree of all security files.
- Manual specification.

Let's delve further into these.

> 让我们更深入地研究一下这些。

#### Directory tree of all security files

RCL supports finding security files in one directory that is inside the reserved `enclaves` subfolder, within the root keystore, corresponding to the fully-qualified path of every enclave.

> RCL 支持在保留的 `enclaves` 子文件夹内的根密钥库中，根据每个飞地的完全限定路径，查找安全文件。

For example, for the `/front/camera` enclave, the directory structure would look like:

> 例如，对于“/front/camera”私密空间，目录结构如下：

```
<root>
├── enclaves
│   └── front
│       └── camera
│           ├── cert.pem
│           ├── key.pem
│           ├── ...
└── public
    ├── ...
```

The set of files expected within each enclave instance directory are:

> 每个飞地实例目录中预期的文件集合为：

- **identity_ca.cert.pem**: The x.509 certificate of the CA trusted by the **Authentication** plugin (the "Identity" CA).

> - **identity_ca.cert.pem**：被**身份验证**插件信任的 CA 的 x.509 证书（"身份"CA）。

- **cert.pem**: The x.509 certificate of this enclave instance (signed by the Identity CA).
- **key.pem**: The private key of this enclave instance.
- **permissions_ca.cert.pem**: The x.509 certificate of the CA trusted by the **Access control** plugin (the "Permissions" CA).

> - **permissions_ca.cert.pem**：被**访问控制**插件信任的 CA 的 x.509 证书（“权限”CA）。

- **governance.p7s**: The XML document that specifies to the **Access control** plugin how the domain should be secured (signed by the Permissions CA).

> - **governance.p7s**：XML 文档，指定给**访问控制**插件如何加固域（由权限 CA 签名）。

- **permissions.p7s**: The XML document that specifies the permissions of this particular enclave instance to the **Access control** plugin (also signed by the Permissions CA).

> - **permissions.p7s**：此特定保护环境实例对**访问控制插件**的权限的 XML 文档（由权限 CA 签名）。

This can be specified by setting the `ROS_SECURITY_KEYSTORE` environment variable to point to the root of the keystore directory tree, and then specifying the enclave path using the `--ros-args` runtime argument `-e`, `--enclave`, e.g.:

> 可以通过将 `ROS_SECURITY_KEYSTORE` 环境变量设置为指向密钥库目录树根的位置，然后使用 `--ros-args` 运行时参数 `-e`，`--enclave` 指定私人空间路径来指定，例如：

```shell
export ROS_SECURITY_KEYSTORE="/home/bob/.ros/sros2_keystore"
ros2 run <package> <executable> --ros-args --enclave="/front/camera"
```

#### Manual specification

RCL also supports specifying the enclave path for the process that needs to be launched using an overriding environmental variable.

> RCL 也支持使用覆盖环境变量来指定需要启动的进程的飞地路径。

This can be done by setting the `ROS_SECURITY_ENCLAVE_OVERRIDE` environment variable to an alternate enclave path within the keystore.

> 可以通过将 `ROS_SECURITY_ENCLAVE_OVERRIDE` 环境变量设置为密钥库中的其他私密空间路径来实现此操作。
> Note that this setting takes precedence over `ROS_SECURITY_KEYSTORE` with `--enclave`.

Note that the following two examples load from the same enclave path as demonstrated prior:

```shell
export ROS_SECURITY_KEYSTORE="/home/bob/.ros/sros2_keystore"
export ROS_SECURITY_ENCLAVE_OVERRIDE="/front/camera"
ros2 run <package> <executable>
```

```shell
export ROS_SECURITY_KEYSTORE="/home/bob/.ros/sros2_keystore"
export ROS_SECURITY_ENCLAVE_OVERRIDE="/front/camera"
ros2 run <package> <executable> --ros-args --enclave="/spam"
```

### Support for both permissive and strict enforcement of security

Participants with the security features enabled will not communicate with participants that don't, but what should RCL do if one tries to launch a participant that has no discernable enclave with keys/permissions/etc.? It has two options:

> 如果有人试图启动没有可辨认的保护特性的参与者，RCL 应该怎么做？它有两个选择：参与者启用安全特性的参与者将不会与没有启用安全特性的参与者进行通信，但是。

- **Permissive mode**: Try to find security files, and if they can't be found, launch the participant without enabling any security features.

> **宽容模式**：尝试查找安全文件，如果找不到，则在不启用任何安全功能的情况下启动参与者。

This is the default behavior.

> 这是默认行为。

- **Strict mode**: Try to find security files, and if they can't be found, fail to run the participant.

> **严格模式**：尝试查找安全文件，如果找不到，则无法运行参与者。

The type of mode desired can be specified by setting the `ROS_SECURITY_STRATEGY` environment variable to "Enforce" (case-sensitive) for strict mode, and anything else for permissive mode.

> 可以通过将 `ROS_SECURITY_STRATEGY` 环境变量设置为“Enforce”（区分大小写）以获得严格模式，而其他任何内容都可以获得宽容模式。

### Support for a master "on/off" switch for all SROS 2 features

In addition to the supported features just discussed, RCL also supports a master shutoff for security features for easy experimentation.

> 此外，RCL 还支持主断电功能，以便实现安全功能的容易实验。

If it's turned off (the default), none of the above security features will be enabled.

> 如果关闭（默认），以上所有安全功能都将不可用。

In order to enable SROS 2, set the `ROS_SECURITY_ENABLE` environment variable to "true" (case-sensitive).

> 要启用 SROS 2，请将环境变量 `ROS_SECURITY_ENABLE` 设置为“true”（区分大小写）。

To disable, set to any other value.

> 若要禁用，请将其设置为其他值。

## Features in the SROS 2 CLI

Configuring a ROS 2 system to be secure in RCL involves a lot of new technology (PKI, DDS governance and permissions files and their syntax, etc.).

> 配置 ROS 2 系统以在 RCL 中实现安全性涉及很多新技术（PKI，DDS 治理和权限文件及其语法等）。

If a user is comfortable with these technologies, the above information should be all that's necessary to properly lock things down.

> 如果用户熟悉这些技术，上述信息应该就足以正确地锁定东西了。

However, the [SROS 2 CLI](https://github.com/ros2/sros2) should include a tool `ros2 security` to help those who don't want to set it all up themselves, including the following capabilities:

> 然而，[SROS 2 CLI](https://github.com/ros2/sros2) 应该包括一个名为 `ros2 security` 的工具来帮助那些不想自己设置它的人，具有以下功能：

- Create Identity and Permissions CA.
- Create directory tree containing all security files.
- Create a new identity for a given enclave, generating a keypair and signing its x.509 certificate using the Identity CA.

> 为给定隔离环境创建一个新的身份，生成密钥对并使用身份 CA 签署其 x.509 证书。

- Create a governance file that will encrypt all DDS traffic by default.
- Support specifying enclave permissions [in familiar ROS terms](/articles/ros2_access_control_policies.html) which are then automatically converted into low-level DDS permissions.

> 支持以熟悉的 ROS 术语指定保留区权限[参见](/articles/ros2_access_control_policies.html)，然后将其自动转换为低级 DDS 权限。

- Support automatically discovering required permissions from a running ROS system.

[dds_security]: https://www.omg.org/spec/DDS-SECURITY/1.1/PDF
> 简体中文：[dds_security]: [https://www.omg.org/spec/DDS-SECURITY/1.1/PDF](https://www.omg.org/spec/DDS-SECURITY/1.1/PDF)
>
> [dds_security]: https://www.omg.org/spec/DDS-SECURITY/1.1/PDF

[dds]: https://www.omg.org/spec/DDS/1.4/PDF
> 答复：[DDS]: [https://www.omg.org/spec/DDS/1.4/PDF](https://www.omg.org/spec/DDS/1.4/PDF)
>
> [dds]: https://www.omg.org/spec/DDS/1.4/PDF
