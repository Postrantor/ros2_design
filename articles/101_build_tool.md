---
tip: translate by openai@2023-05-30 08:40:32
layout: default
title: A universal build tool
permalink: articles/build_tool.html
abstract:
  This article describes the rationale for a universal build tool.
published: true
author: '[Dirk Thomas](https://github.com/dirk-thomas)'
date_written: 2017-03
last_modified: 2021-01
categories: Overview
Authors: {{ page.author }}
Date Written: {{ page.date_written }}
Last Modified: {% if page.last_modified %}{{ page.last_modified }}{% else %}{{ page.date_written }}{% endif %}
---

## Preface

In the ROS ecosystem the software is separated into numerous packages. It is very common that a developer is working on multiple packages at the same time. This is in contrast to workflows where a developer only works on a single software package at a time, and all dependencies are provided once but not being iterated on.

> 在 ROS 生态系统中，软件被分割成许多包。开发人员同时工作在多个包上是很常见的。这与只在一个软件包上工作，并且一次性提供所有依赖项而不进行迭代的工作流形成了对比。

The "manual" approach to build a set of packages consists of building all packages in their topological order one by one. For each package the documentation usually describes what the dependencies are, how to setup the environment to build the package as well as how to setup the environment afterwards to use the package. Such a workflow is impracticable at scale without a tool that automates that process.

> "手动"构建一组软件包的方法包括按照其拓扑顺序依次构建所有软件包。对于每个软件包，文档通常描述了它的依赖关系、如何设置环境来构建软件包以及如何在使用软件包之后设置环境。如果没有一个能够自动完成这些步骤的工具，这样的工作流程在大规模使用时是不可行的。

A build tool performs the task of building a set of packages with a single invocation. For ROS 1 multiple different tools provide support for this, namely `catkin_make`, `catkin_make_isolated`, and `catkin_tools`. For ROS 2 up to the Ardent release the build tool providing this functionality is called `ament_tools`.

> 一个构建工具可以用一次调用来构建一组软件包。对于 ROS 1，有多种不同的工具提供支持，即`catkin_make`、`catkin_make_isolated`和`catkin_tools`。对于 ROS 2，到 Ardent 发布版本，提供此功能的构建工具称为`ament_tools`。

This article describes the steps to unify these build tools as well as extend the field of application.

> 这篇文章描述了统一这些构建工具以及扩展应用领域的步骤。

## Goal

The goal of a unified build tool is to build a set of packages with a single invocation. It should work with ROS 1 packages as well as ROS 2 packages which provide the necessary information in their manifest files. It should also work with packages that do not provide manifest files themselves, given that the necessary meta information can be inferred and/or is provided externally. This will allow the build tool to be utilized for non-ROS packages (e.g. Gazebo including its ignition dependencies, sdformat, etc.).

> 目标是使用单一的调用来构建一组软件包。它应该能够与 ROS 1 软件包以及提供必要信息的 ROS 2 软件包一起工作。它也应该能够与本身不提供清单文件的软件包一起工作，只要能够推断出必要的元信息并/或从外部提供即可。这将允许构建工具用于非 ROS 软件包（例如 Gazebo 及其 Ignition 依赖项、sdformat 等）。

In the ROS ecosystems several tools already exist which support this use case (see below). Each of the existing tools performs similar tasks and duplicates a significant amount of the logic. As a consequence of being developed separately certain features are only available in some of the tools while other tools lack those.

> 在 ROS 生态系统中，已经存在几种工具，可以支持这种用例（见下文）。现有的每个工具都执行类似的任务，并重复了大量的逻辑。由于被单独开发，某些功能只在某些工具中可用，而其他工具则缺少这些功能。

The reason to use a single universal build tool comes down to reducing the effort necessary for development and maintenance. Additionally this makes newly developed features available for all the use cases.

> 使用单一通用构建工具的原因归结为减少开发和维护所需的努力。此外，这使新开发的功能可用于所有用例。

### Build Tool vs. Build System

Since this article focuses on the build tool the distinction to a build system needs to be clarified.

> 本文重点讨论构建工具，因此需要澄清与构建系统的区别。

A build tool operates on a set of packages. It determines the dependency graph and invokes the specific build system for each package in topological order. The build tool itself should know as little as possible about the build system used for a specific package. Just enough in order to know how to setup the environment for it, invoke the build, and setup the environment to use the built package. The existing ROS build tools are: `catkin_make`, `catkin_make_isolated`, `catkin_tools`, and `ament_tools`.

> 一个构建工具操作一组软件包。它确定依赖关系图，并按拓扑顺序为每个软件包调用特定的构建系统。构建工具本身应该对特定软件包使用的构建系统知之甚少。只需要知道如何设置环境，调用构建，以及如何设置环境以使用已构建的软件包。现有的 ROS 构建工具有：`catkin_make`，`catkin_make_isolated`，`catkin_tools`和`ament_tools`。

The build system on the other hand operates on a single package. Examples are `Make`, `CMake`, `Python setuptools`, or `Autotools` (which isn't used in ROS atm). A CMake package is e.g. build by invoking these steps: `cmake`, `make`, `make install`. `catkin` as well as `ament_cmake` are based on CMake and offer some convenience functions described below.

> 系统构建另一方面操作单个包。例如`Make`、`CMake`、`Python setuptools`或`Autotools`（ROS 目前不使用）。例如，CMake 包可以通过调用以下步骤来构建：`cmake`、`make`、`make install`。`catkin`和`ament_cmake`均基于 CMake，并提供一些方便的功能，如下所述。

### Environment Setup

A very important part beside the actual build of a package is the environment setup. For example, in order for a CMake project to discover a dependency using the CMake function `find_package`, the CMake module (e.g. `FindFoo.cmake`) or the CMake config file (e.g. `FooConfig.cmake`) for that dependency must either be in a prefix that CMake searches implicitly (e.g. `/usr`) or the location must be provided through the environment variable `CMAKE_PREFIX_PATH` / `CMAKE_MODULE_PATH`.

> 一个软件包的实际构建之外，环境设置是一个非常重要的部分。例如，为了使 CMake 项目使用 CMake 函数`find_package`发现一个依赖项，该依赖项的 CMake 模块（例如`FindFoo.cmake`）或 CMake 配置文件（例如`FooConfig.cmake`）必须位于 CMake 隐式搜索的前缀（例如`/usr`）中，或者必须通过环境变量`CMAKE_PREFIX_PATH`/`CMAKE_MODULE_PATH`提供位置。

In addition to building a package on top of another package (using `find_package` in the case of CMake), you may need to adjust the environment in order to run an executable from a package. For example, when a package installs a shared library in a non-default location then the environment variable `LD_LIBRARY_PATH` (or `PATH` on Windows) needs to be extended to include the containing folder before trying to run executables that load that library at runtime.

> 除了在另一个包的顶部构建一个包（在 CMake 的情况下使用`find_package`）之外，您还可能需要调整环境以运行来自包的可执行文件。例如，当一个包在非默认位置安装共享库时，环境变量`LD_LIBRARY_PATH`（或 Windows 上的`PATH`）需要扩展到包含文件夹，然后再尝试运行加载该库的可执行文件。

The functionality to setup these environment variables can be provided by either the build tool or the build system. In the latter case the build tool only needs to know how the build system exposes the environment setup in order to reuse it.

> 功能来设置这些环境变量可以由构建工具或构建系统提供。在后一种情况下，构建工具只需要知道构建系统如何暴露环境设置，以便重用它。

Considering the use case that a user might want to invoke the build system of each package manually it is beneficial if the build system already provides as much of the environment setup as possible. That avoids forcing the user to manually take care of the environment setup when not using a build tool.

> 考虑到用户可能希望手动调用每个软件包的构建系统的使用情况，如果构建系统已经提供尽可能多的环境设置，那将是有益的。这样可以避免用户在不使用构建工具时手动处理环境设置。

### Out of Scope

To clarify the scope of this article a few related topics are explicitly enumerated even though they are not being considered.

> 为了澄清本文的范围，尽管没有考虑，但明确列举了一些相关的主题。

#### Build System

Any build system related functionality (which is not directly relevant for the build tool) is not considered in this article.

> 本文不考虑任何与构建系统相关的功能（与构建工具没有直接关联）。

##### Mixing Different Build Systems

The unified build tool will support different build systems in order to satisfy the described goals. If packages using different build system inter-operate with each other correctly depends also to a large degree on the build system. While the build tool should ensure that it doesn't prevent that use case this article will not cover the use case of mixing multiple build systems in a single workspace (e.g. ROS 1 packages using `catkin` with ROS 2 packages using `ament_cmake`).

> 统一的构建工具将支持不同的构建系统，以满足所描述的目标。如果使用不同构建系统的软件包之间能够正确互操作，也在很大程度上取决于构建系统。虽然构建工具应该确保它不会阻止该用例，但本文不涵盖在单个工作区中混合多个构建系统（例如，使用`catkin`的 ROS 1 软件包与使用`ament_cmake`的 ROS 2 软件包）的用例。

#### Fetch Source Code

The build tool does not cover the steps necessary to fetch the sources of the to-be-built packages. There are already tools to help with this. For example, the list of repositories that need to be fetched is provided either by a hand crafted `.rosinstall` or `.repos` file or by using [rosinstall_generator](http://wiki.ros.org/rosinstall_generator) to generate one. The list of repositories can then be fetched with one of several tools, like [rosinstall](http://wiki.ros.org/rosinstall) or [wstool](http://wiki.ros.org/wstool) in the case of a `.rosinstall` file, or [vcstool](https://github.com/dirk-thomas/vcstool) in the case of a `.repos` file.

> 构建工具不包括获取要构建的软件包源码所需的步骤。已经有工具可以帮助这一点。例如，需要获取的存储库列表可以由手工制作的`.rosinstall`或`.repos`文件提供，或者使用[rosinstall_generator](http://wiki.ros.org/rosinstall_generator)生成。然后可以使用几种工具之一，如`.rosinstall`文件的[rosinstall](http://wiki.ros.org/rosinstall)或[wstool](http://wiki.ros.org/wstool)，或`.repos`文件的[vcstool](https://github.com/dirk-thomas/vcstool)获取存储库列表。

#### Install Dependencies

The build tool also does not provide a mechanism to install any dependencies required to build the packages. In the ROS ecosystem [rosdep](http://wiki.ros.org/rosdep) can be used for this.

> 构建工具也不提供安装构建包所需的依赖项的机制。在 ROS 生态系统中，可以使用[rosdep](http://wiki.ros.org/rosdep)来实现这一点。

#### Create Binary Packages

The build tool also does not create binary packages (e.g. a Debian package). In the ROS ecosystem [bloom](http://wiki.ros.org/bloom) is used to generate the required metadata and then platform dependent tools like `dpkg-buildpackage` build binary packages.

> 构建工具也不会创建二进制包（例如 Debian 包）。在 ROS 生态系统中，使用[bloom](http://wiki.ros.org/bloom)生成所需的元数据，然后使用像`dpkg-buildpackage`这样的平台相关工具构建二进制包。

## Existing Build Systems

In the following the build systems being used in the ROS ecosystem are briefly described.

> 以下简要描述了 ROS 生态系统中使用的构建系统。

### CMake

[CMake](https://cmake.org/) is a cross-platform build system generator. Projects specify their build process with platform-independent `CMakeLists.txt` files. Users build a project by using CMake to generate a build system for a native tool on their platform, e.g. `Makefiles` or `Visual Studio projects`.

> CMake 是一个跨平台构建系统生成器。项目使用平台无关的 CMakeLists.txt 文件指定其构建过程。用户通过使用 CMake 为其平台上的本地工具（如 Makefiles 或 Visual Studio 项目）生成构建系统来构建项目。

### catkin

[catkin](http://wiki.ros.org/catkin) is based on CMake and provides a set of convenience functions to make writing CMake packages easier. It automates the generation of CMake config files as well as pkg-config files. It additionally provides functions to register different kinds of tests.

> Catkin 基于 CMake，提供一系列便利函数，使编写 CMake 包变得更容易。它自动生成 CMake 配置文件和 pkg-config 文件。此外，它还提供了用于注册不同类型测试的功能。

A package using `catkin` specifies its meta data in a manifest file named `packapecified in the [ROS REP 149](http://www.ros.org/reps/rep-0149.html).

> 一个使用`catkin`的包会在一个名为`package.xml`的清单文件中指定它的元数据，具体参见[ROS REP 149](http://www.ros.org/reps/rep-0149.html)。

### ament_cmake

[ament_cmake](https://github.com/ament/ament_cmake) is an evolution of `catkin` and is also based on CMake. The main difference between `ament_cmake` and `catkin` is described in [another article](http://design.ros2.org/articles/ament.html). In the context of the build tool the biggest difference is that `ament_cmake` generates package-specific files to setup the environment to use the package after it has been built and installed.

> [ament_cmake](https://github.com/ament/ament_cmake) 是一种演变的`catkin`，也基于 CMake。`ament_cmake`和`catkin`之间的主要区别在[另一篇文章](http://design.ros2.org/articles/ament.html)中有所描述。在构建工具的上下文中，最大的区别是`ament_cmake`会生成特定于包的文件，以在构建和安装后设置环境以使用该包。

A package using `ament_cmake` uses the same manifest file as `catkin` (except that it requires format version 2 or higher).

> 使用`ament_cmake`的一个包使用与`catkin`相同的清单文件（除了它需要格式版本 2 或更高版本）。

### Python setuptools

`setuptools` is the common tool to package Python packages. A Python package uses a `setup.py` file to describe the dependencies as well as how and what to build and install. In ROS 2 a package can be a "vanilla" Python package whereas in ROS 1 any Python functionality is triggered from a CMake file.

## Existing Build Tools

Several different build tools are already being used in the ROS ecosystem. Their method of operating is being described in the following subsections together with their advantages as well as disadvantages.

> 在 ROS 生态系统中已经使用了几种不同的构建工具。它们的操作方式将在下面的子节中描述，并列出它们的优点和缺点。

### catkin_make

`catkin_make` is provided by the ROS package `catkin` which contains the build system for ROS 1. It has been designed as the successor of `rosbuild` for ROS Fuerte.

The tool invokes CMake only a single time and uses CMake's `add_subdirectory` function to process all packages in a single context. While this is an efficient approach since all targets across all packages can be parallelized it comes with significant disadvantages. Due to the single context all function names, targets and tests share a single namespace across all packages and on a larger scale this easily leads to collisions. The single context is also subject to side effects between the packages and sometimes requires adding additional target dependencies across package boundaries.

> 工具只调用 CMake 一次，并使用 CMake 的`add_subdirectory`函数在单个上下文中处理所有软件包。虽然这是一种高效的方法，因为所有软件包中的所有目标都可以并行化，但它也带来了重大缺点。由于单个上下文，所有函数名称，目标和测试在所有软件包中共享单个命名空间，在更大范围内容易导致冲突。单个上下文也受到软件包之间的副作用的影响，有时需要在软件包边界添加额外的目标依赖关系。

`catkin_make` supports building the following packages:

- ROS 1 `catkin` package with a `package.xml` file.

### catkin_make_isolated

`catkin_make_isolated` is provided by the ROS package `catkin` which contains the build system for ROS 1. It was developed after `catkin_make` to address the problems involved with building multiple packages in a single CMake context.

The tool only supports CMake-based packages and builds each package in topological order using the command sequence common for CMake packages: `cmake`, `make`, `make install`. While each package can parallelize the build of its targets the packages are processed sequentially even if they are not (recursive) dependencies of each other.

> 这个工具只支持基于 CMake 的软件包，并使用常见于 CMake 软件包的命令序列（`cmake`、`make`、`make install`）按拓扑顺序构建每个软件包。虽然每个软件包可以并行构建它的目标，但即使它们不是（递归）依赖关系，也会按顺序处理软件包。

`catkin_make_isolated` supports building the following packages:

- ROS 1 `catkin` package with a `package.xml` file.
- Plain CMake packages with a `package.xml` file.

### catkin_tools

[catkin_tools](https://catkin-tools.readthedocs.io/) is provided by a standalone Python package used to build ROS 1 packages. It was developed after `catkin_make` / `catkin_make_isolated` to build packages in parallel as well as provide significant usability improvements. The tool supports building CMake packages and builds them in isolation as well as supports parallelizing the process across packages.

> [Catkin_tools](https://catkin-tools.readthedocs.io/)是一个独立的 Python 包，用于构建 ROS 1 包。它是在`catkin_make`/`catkin_make_isolated`之后开发的，可以并行构建包，并提供显著的可用性改进。该工具支持构建 CMake 包，并将其隔离构建，并支持跨包并行化处理。

`catkin_tools` supports building the following packages:

- ROS 1 `catkin` package with a `package.xml` file.
- Plain CMake packages with a `package.xml` file.

### ament_tools

`ament_tools` is provided by a standalone Python 3 package used to build ROS 2 packages. It was developed to bootstrap the ROS 2 project, is therefore only targeting Python 3, and works on Linux, MacOS and Windows. In addition to CMake packages it also supports building Python packages and can infer meta information without requiring an explicit package manifest (which is e.g. used for the FastRTPS package). The tool performs an "isolated" build like `catkin_make_isolated` and `catkin_tools` (one CMake invocation per package) and also parallelizes the build of packages which have no (recursive) dependencies on each other (like `catkin_tools`). While it covers more build systems and platforms than `catkin_tools` it doesn't have any of `catkin_tools`s usability features like profiles, output handling, etc.

> “ament_tools” 由用于构建 ROS 2 软件包的独立 Python 3 软件包提供。它的开发是为了引导 ROS 2 项目，因此仅针对 Python 3，并且可以在 Linux，MacOS 和 Windows 上使用。除了 CMAKE 软件包外，它还支持构建 Python 软件包，并可以推断元信息，而无需明确的软件包清单（例如用于 Fastrtps 软件包）。该工具执行一个“隔离”构建，例如`catkin_make_isolated`和`catkin_tools'（每个软件包的一个`cmake invocation`，并且还同时使没有（递归）相互依赖的软件包的构建（例如`catkin_tools`）。虽然它涵盖了比“ catkin_tools”更多的构建系统和平台，但它没有任何 catkin_tools 的可用性功能，例如配置文件，输出处理等。

`ament_tools` supports building the following packages:

- ROS 2 `ament_cmake` package with a `package.xml` file (only format 2).
- Plain CMake package with a `package.xml` file.
- Plain CMake package without a manifest file (extracting the package name and dependencies from the CMake files).
- Python package with a `package.xml` file.
- Python package without a manifest file (extracting the package name and dependencies from the `setup.py` file).

### colcon

When the first draft of this article was written the conclusion was to not to spend any resources towards a universal build tool. Meanwhile the author of this article went ahead and developed [colcon](https://github.com/colcon/) as a personal project. Therefore its feature set is closely aligned with the following requirements.

> 当这篇文章的第一稿写成时，结论是不要花费任何资源来构建通用工具。与此同时，本文的作者继续推进了[colcon](https://github.com/colcon/)作为个人项目。因此，它的功能集与以下要求密切相关。

## Naming

The existing build tools in ROS are all named by the build system they are supporting. In general it should be possible for a build tool to support multiple different build systems. Therefore a name for a build tool being derived from a single build system might mislead the users that the tool only works for that specific build system. To avoid confusion of the user the build tool should have a different unrelated name to avoid implying an undesired correlation.

> 现有的 ROS 构建工具都是以支持的构建系统命名的。一般来说，一个构建工具应该可以支持多个不同的构建系统。因此，从单个构建系统派生出来的构建工具的名字可能会误导用户，让他们以为这个工具只能用于特定的构建系统。为了避免用户的混淆，构建工具应该有一个与之无关的不同名字，以避免暗示不希望的相关性。

## Requirements

The unified build tool should provide a superset of the functionality provided by the existing tools. In the following a few use cases are described as well as desired software criteria.

> 统一的构建工具应提供现有工具提供的功能的超集。以下描述了几个用例以及期望的软件准则。

Other use cases which are not explicitly covered but are already supported by the existing tools (e.g. cross-compilation, `DESTDIR` support, building CMake packages without a manifest) should continue to work with the unified build tool.

> 其他尚未明确涵盖的用例，但已经由现有工具支持（例如交叉编译、`DESTDIR`支持、不使用清单构建 CMake 包），应该继续使用统一的构建工具。

### Use Cases

The following uses cases should be satisfied by the unified build tool.

> 以下用例应该由统一的构建工具满足。

#### Build ROS 1 workspaces

The tool needs to be able to build ROS 1 workspaces which can already be built using `catkin_make_isolated` / `catkin_tools`. It is up to the implementation to decide if it only supports the standard CMake workflow or also the _custom devel space concept_ of `catkin`.

> 该工具需要能够构建已经可以使用`catkin_make_isolated`/`catkin_tools`构建的 ROS 1 工作区。它的实现是否只支持标准 CMake 工作流程还是也支持`catkin`的*自定义开发空间概念*取决于实现。

In ROS 2 the concept of the _devel space_ has intentionally been removed. In the future it might be feasible to provide the concept of _symlinked installs_ in ROS 1 to provide a similar benefit without the downsides.

> 在 ROS 2 中，有意移除了*devel space*的概念。未来可能提供 ROS 1 中的*符号链接安装*概念，以提供类似的好处，而不带有不利的方面。

#### Build ROS 2 workspaces

The tool needs to be able to build ROS 2 workspaces which can already be built using `ament_tools`.

> 工具需要能够构建可以使用`ament_tools`构建的 ROS 2 工作区。

#### Build Gazebo including dependencies

After cloning the repositories containing Gazebo and all its dependencies (excluding system packages) the tool needs to be able to build the set of packages. Meta information not inferable from the sources can be provided externally without adding or modifying any files in the workspace. After the build a single file can be sourced / invoked to setup the environment to use Gazebo (e.g. `GAZEBO_MODEL_PATH`).

> 在克隆包含 Gazebo 及其所有依赖项（不包括系统包）的存储库后，该工具需要能够构建一组软件包。无法从源代码推断的元数据可以在不在工作区中添加或修改任何文件的情况下从外部提供。构建完成后，可以源/调用单个文件来设置使用 Gazebo 的环境（例如`GAZEBO_MODEL_PATH`）。

### Development Environment Setup

Invoking a build system for a package implies also setting up environment variables before the process, e.g. the `CMAKE_PREFIX_PATH`. It should be possible for developers to manually invoke the build system for one package. The environment variable might be partially different from the environment variables necessary to use a package after it has been built. To make that convenient the tool should provide an easy-to-use mechanism to setup the development environment necessary to manually invoke the build system.

> 调用一个软件包的构建系统，意味着在进程之前设置环境变量，例如`CMAKE_PREFIX_PATH`。开发人员应该能够手动调用一个软件包的构建系统。环境变量可能与使用构建后的软件包所需的环境变量部分不同。为了方便起见，该工具应该提供一种易于使用的机制，以设置手动调用构建系统所需的开发环境。

### Beyond Building

Building packages is only one task the build tool can perform on a set of packages. Additional tasks like e.g. running tests should also be covered by the build tool. The build tool must provide these abstract tasks and then map them to the steps necessary for each supported build system.

> 构建软件包只是构建工具可以在一组软件包上执行的一项任务。构建工具还应该涵盖除此之外的任务，例如运行测试。构建工具必须提供这些抽象任务，然后将它们映射到每个支持的构建系统所需的步骤。

### Software Criteria

The tool aims to support a variety of build systems, use cases, and platforms. The above mentioned ones are mainly driven by the needs in the ROS ecosystem but the tool should also be usable outside the ROS ecosystem (e.g. for Gazebo). Therefore it should be designed in a way which enables extending its functionality.

> 这个工具旨在支持各种构建系统、用例和平台。上述这些主要是受 ROS 生态系统的需求驱动，但该工具也应该可以在 ROS 生态系统之外使用（例如用于 Gazebo）。因此，它应该以一种可以扩展其功能的方式设计。

Assuming that the tool will be implemented in Python (since that is the case for all existing ROS build tools) the entry point mechanism provides a convenient way to make the software extensible. Extensions don't even have to be integrated into the Python package containing the core logic of the build tool but can easily be provided by additional Python packages. This approach will not only foster a modular design and promote clear interfaces but enable external contributions without requiring them to be integrated in a single monolithic package.

> 假设工具将使用 Python 实现（因为所有现有的 ROS 构建工具都是如此），入口点机制提供了一种方便的方式来使软件可扩展。扩展甚至不必集成到包含构建工具核心逻辑的 Python 包中，而可以通过额外的 Python 包轻松提供。这种方法不仅可以促进模块化设计并促进清晰的接口，而且可以在不需要将其集成到单个巨型包中的情况下实现外部贡献。

Several well known software principles apply:

> 几个众所周知的软件原则适用：

- Separation of concerns
- Single Responsibility principle
- Principle of Least Knowledge
- Don’t repeat yourself
- Keep it stupid simple
- "Not paying for what you don't use"

### Extension Points

The following items are possible extension points to provide custom functionality:

> 以下项目可能是提供自定义功能的扩展点：

- contribute `verbs` to the command line tool (e.g. `build`, `test`)
- contribute command line options for specific features (e.g. `build`, `test`)
- discovery of packages (e.g. recursively crawling a workspace)
- identification of packages and their meta information (e.g. from a `package.xml` file)
- process a package (e.g. build a CMake package, test a Python package)
- execution model (e.g. sequential processing, parallel processing)
- output handling (e.g. console output, logfiles, status messages, notifications)
- setup the environment (e.g. `sh`, `bash`, `bat`)
- completion (e.g. `bash`, `Powershell`)

## Possible Approaches

When the first draft of this article was written neither of the existing build tools supported the superset of features described in this article. There were multiple different paths possible to reach the goal of a universal build tool which fall into two categories:

> 当这篇文章的第一稿写作时，现有的构建工具都不支持本文中描述的超集功能。有多种不同的路径可以达到通用构建工具的目标，这些路径可以分为两类：

- One approach is to incrementally evolve one of the existing tools to satisfy the described goals.
- Another approach would be to start "from scratch".

Since then the new project `colcon` has been developed which covers most of the enumerated requirements and represents the second category.

> 自那时起，新项目`colcon`已经开发出来，它涵盖了大多数列出的要求，并代表了第二类。

### Evolve catkin_make, catkin_make_isolated, or ament_tools

Since neither of these three build tools has the feature richness of `catkin_tools` it is considered strictly less useful to starting building upon one of these build tools. Therefore neither of these are being considered as a foundation for a universal build tool.

> 由于这三个构建工具中没有一个拥有`catkin_tools`的功能丰富性，因此以其中一个构建工具为基础开始构建被认为是比较不利的。因此，这些都不被认为是一个通用构建工具的基础。

### Evolve catkin_tools

Since `catkin_tools` is in many aspects the most complete ROS build tool it should be the one being evolved. While `ament_tools` has a few features `catkin_tools` currently lacks (e.g. plain CMake support without a manifest, Windows support) the feature richness of `catkin_tools` makes it a better starting point.

> 由于`catkin_tools`在许多方面是最完整的 ROS 构建工具，因此它应该是被演化的工具。虽然`ament_tools`有一些`catkin_tools`目前缺少的特性（例如，无需清单的纯 CMake 支持，Windows 支持），但`catkin_tools`的功能丰富使其成为更好的起点。

### Start "from scratch" / colcon

Since the first draft of this article the `colcon` project has been developed with the goals and requirements of a universal build tool in mind. In its current form it is already able to build ROS 1 workspaces, ROS 2 workspaces, as well as Gazebo including its ignition dependencies. It uses Python 3.5+ and targets all platforms supported by ROS: Linux, macOS, and Windows.

> 自从这篇文章的第一稿发布以来，`colcon`项目一直在以实现通用构建工具的目标和要求为出发点进行开发。目前，它已经能够构建 ROS 1 工作区、ROS 2 工作区以及 Gazebo（包括其 Ignition 依赖项）。它使用 Python 3.5+，并针对 ROS 支持的所有平台：Linux、macOS 和 Windows。

Since it hasn't been used by many people yet more advanced features like cross compilation, `DESTDIR`, etc. hasn't been tested (and will therefore likely not work yet).

> 由于尚未有多少人使用，因此更高级的功能，如交叉编译，`DESTDIR`等尚未经过测试（因此可能尚未正常工作）。

## Decision process

For the decision process only the following two options are being considering based on the rationale described above:

> 根据上述理由，仅考虑以下两个选择进行决策过程：

- option **A)** Use `catkin_tools` as a starting point
- option **B)** Use `colcon` as a starting point

If this topic would have been addressed earlier some of the duplicate effort could have likely been avoided. When the work towards a universal build tool was suspended over a year ago it was a conscious decision based on available resources. Nevertheless moving forward with a decision now will at least avoid further uncertainty and effort duplication.

> 如果早些时候解决了这个话题，可能可以避免重复的努力。一年多以前，当工作朝着一个通用构建工具暂停时，这是一个有意识的决定，基于可用的资源。然而，现在做出决定至少可以避免进一步的不确定性和重复努力。

Both of the considered options have unique and valuable features and there are good arguments to build our future development on either of the two tools. Since both are written in Python either of the two tools could be "transformed" to cover the pros of the other one. So the two important criteria for the decision are:

> 两种考虑的选项都具有独特而有价值的特性，有充分的理由基于其中任一工具来构建未来的发展。由于两者都是用 Python 编写的，因此可以将其中任一工具“转换”以涵盖另一个工具的优势。因此，决定的两个重要标准是：

- the effort it takes to do (in the short term as well as in the long term) and
- the difference of the resulting code base after the "transformation" is completed.

### Immediate goals

A ROS 2 developer currently builds a steadily growing workspace with ROS 2 packages. The same is happening in the monolithic Jenkins jobs on [ci.ros2.org](https://ci.ros2.org) (with the advantage to test changes across repositories easily). Therefore features to easily filter the packages which need to be build are eagerly awaited to improve the development process.

> 一位 ROS 2 开发者目前正在构建一个不断增长的 ROS 2 软件包工作区。在[ci.ros2.org](https://ci.ros2.org)上的单体式 Jenkins 作业也是如此（具有跨存储库轻松测试更改的优势）。因此，期望有一些功能来轻松过滤需要构建的软件包，以改善开发过程。

For the last ROS 2 release _Ardent_ the buildfarm [build.ros.org](http://build.ros2.org) only provides jobs to generate Debian packages. Neither _devel_ jobs or _pull request_ jobs are available nor is it supported to build a local _prerelease_. For the coming ROS 2 release _Bouncy_ these job types should be available to support maintainers.

> 对于最后一个 ROS 2 发行版*Ardent*，buildfarm [build.ros.org](http://build.ros2.org)仅提供生成 Debian 软件包的工作。没有*devel*工作或*pull request*工作可用，也不支持构建本地*prerelease*。对于即将到来的 ROS 2 发行版*Bouncy*，这些工作类型应该可用，以支持维护者。

In ROS 2 _Bouncy_ the universal build tool will be the recommended option.

> 在 ROS 2 *Bouncy*中，通用构建工具将是推荐的选择。

#### Necessary work

For either option **A)** or **B)** the follow items would need to be addressed:

> 对于选项 A 或 B，需要解决以下问题：

- The jobs and scripts on _ci.ros2.org_ need to be updated to invoke the universal build tool instead of `ament_tools`.
- The `ros_buildfarm` package needs to be updated to invoke the universal build tool instead of `catkin_make_isolated`.
  The ROS 2 buildfarm would use this modification for the upcoming ROS 2 _Bouncy_ release.
  The ROS 1 buildfarm could use the same modification in the future.

For option **A)** the follow items would need to be addressed:

> 对于选项 A，需要解决以下问题：

- Support for setup files generated by `ament_cmake`.
- Support additional packages types: plain Python packages, CMake packages without a manifest.
- Support for Windows using `.bat` files.
- Support for the package manifest format version 3.

For option **B)** the follow items would need to be addressed:

> 对于选项**B）**，需要解决以下问题：

- Address user feedback when the tool is being used by a broader audience.

### Future

The long term goal is that the universal build tool will be used in ROS 1, in ROS 2 as well as other non-ROS projects. There is currently no time line when the tool will be used on the ROS 1 build or be recommended to ROS 1 users. This solely depends on the resources available for ROS 1.

> 长期目标是，通用构建工具将在 ROS 1，ROS 2 以及其他非 ROS 项目中使用。目前尚无时间表，何时将该工具用于 ROS 1 构建或推荐给 ROS 1 用户。这完全取决于 ROS 1 可用的资源。

Beside that for both options there is follow up work beyond the immediate goals. The following enumerates a few of them but is by no means exhaustive:

> 除此之外，对于这两个选项，还有超出当前目标的后续工作。以下列举了其中的一些，但远不能说明全部。

For option **A)** the follow items should be considered:

> 对于选项**A）**，应考虑以下项目：

- Support for Python packages using a `setup.cfg` file.
- Support for `PowerShell` to work around length limitations for environment variable on Windows.
- Support for `Pytest` to run Python tests (instead of using `nose`).
- Support to pass package specific argument.
- Update code base to Python 3.5+.
- Refactor code base to reduce coupling (e.g. separate [API](https://github.com/catkin/catkin_tools/blob/2cae17f8f32b0193384d2c7734afee1c60c4add2/catkin_tools/execution/controllers.py#L183-L205) for output handling).
- Additional functionality to build Gazebo including its dependencies.
- Whether or not to continue supporting the _devel space_ concept in ROS 1.

For option **B)** the follow items should be considered:

> 对于选项 **B）**，应考虑以下内容：

- Support `DESTDIR`.
- Support a feature similar to the `profile` verb of `catkin_tools`.
- Support for a shared GNU Make job sever.
- Support for `GNUInstallDirs`
  - Not sure about the status of this, it would be in `colcon`'s generated shell files if anywhere.
  - Should have a test for this case.
- Test for, and fix if necessary, correct topological order with dependencies across workspaces.
  - See: [https://github.com/ros/catkin/pull/590](https://github.com/ros/catkin/pull/590)

## Summary and Decision

Based on the above information a decision has been made to pick `colcon` as the universal build tool.

> 根据上述信息，已决定选择`colcon`作为通用构建工具。

The decision was made after considering the input of ROS 2 team members and some ROS 1 users. The decision was not easy, as it was not unanimous, but the vast majority of input was either pro `colcon` or ambivalent.

> 决定是在考虑 ROS 2 团队成员和一些 ROS 1 用户的输入后做出的。做出这个决定并不容易，因为没有达成共识，但绝大多数输入要么支持`colcon`，要么是持中立态度。

To elaborate on the rationale one significant advantage of `colcon` is that it is ready to be deployed for ROS 2 right now and it covers our current use cases. Another argument leaning towards `colcon` is the expected little effort to provide devel / PR / prerelease jobs on build.ros2.org across all targeted platforms for the upcoming _Bouncy_ release. While some additional feature and usability options are still missing they can be added in the future whenever there is time and/or demand for them.

> 在阐述理由时，`colcon`的一个重要优势是它现在已经准备好部署 ROS 2，并且满足我们当前的使用场景。另一个有利于`colcon`的论据是在即将发布的*Bouncy*版本上，在 build.ros2.org 上为所有目标平台提供 devel/PR/prerelease 工作时，预计几乎不需要任何努力。尽管目前仍缺少一些额外的功能和可用性选项，但在未来有时间和/或需求时，可以添加它们。

The necessary up front development effort for `catkin_tools` to achieve the goals described for _Bouncy_ would distract the ROS 2 team from spending their time on feature development and bug fixing of ROS 2 itself.

> 为了实现*Bouncy*所描述的目标，`catkin_tools`所需要的前期开发努力会分散 ROS 2 团队对 ROS 2 功能开发和错误修复的时间。

While the short term advantages are certainly a main reason for the decision in favor of `colcon` they are not the only ones. The cleaner architecture, modularity and extensibility as well as Python 3.5 code base will be valuable long term benefits when developing this tool in the future. The separation of the build tool name from the supported build systems as well as the separation from being a "ROS-only" tool will hopefully also help users to understand the difference and attract new users and potential contributors.

> 短期利益肯定是支持`colcon`的主要原因，但不是唯一的原因。清晰的架构、模块化和可扩展性以及 Python 3.5 代码库将是未来开发该工具时的宝贵长期利益。将构建工具名称与所支持的构建系统分离，以及从“仅 ROS”工具中分离出来，也将有助于用户理解差异，吸引新用户和潜在贡献者。

### Next steps

The following next steps will happen before the next ROS 2 release _Bouncy_.

> 下一步在下一个 ROS 2 发行版*Bouncy*发布之前将会发生的事情。

- The ROS 2 buildfarm(s) will be updated to use `colcon` and provide devel / PR / prerelease jobs for the _Bouncy_ release.
- The instructions in the ROS 2 wiki to build from source will be updated to use `colcon` instead.

After the Bouncy release the `ament_tools` repository will be archived, removed from the `ros2.repos` file, and won't be released into future ROS 2 distributions.

> 在 Bouncy 发布之后，`ament_tools`存储库将被归档，从`ros2.repos`文件中删除，不会被发布到未来的 ROS 2 发行版中。

### Implications

The following items briefly enumerate what this decision means for ROS developers and users:

> 以下项目简要列出了这一决定对 ROS 开发者和用户意味着什么：

- **No CMake code** of any ROS 2 (or ROS 1) package **needs to be changed** for this transition.
- When building and testing ROS 2 the command `colcon build` / `colcon test` will be used instead of `ament build` / `ament test`.

  Please see the [documentation](http://colcon.readthedocs.io/en/released/migration/ament_tools.html) how to map `ament_tools` command line arguments to `colcon` arguments.

> 请参阅[文档](http://colcon.readthedocs.io/en/released/migration/ament_tools.html)，了解如何将`ament_tools`命令行参数映射到`colcon`参数。

- For ROS 1 nothing is changing at this point in time.
- In the future `colcon` will replace `catkin_make_isolated` and `catkin_make` as the recommended build tool for ROS 1.
  - `colcon` will not support the _devel space_ and will require packages to have install rules
  - `catkin` will likely still support the _devel space_, though it might be removed at some point (that has not been decided yet)
  - Therefore, it is possible that the default build tool for ROS 1 may not support the _devel space_, though legacy tools will continue to support it.
  - Note that it is already the case that individual ROS 1 catkin packages may either not have installation rules but support the _devel space_, or they might have installation rules but not properly support the _devel space_.

### Progress

#### ROS 2

- ci.ros2.org has been [updated](https://github.com/ros2/ci/pull/132)
- build.ros2.org will use [this branch](https://github.com/ros-infrastructure/ros_buildfarm/pull/548) of `ros_buildfarm`

#### ROS 1

- `catkin` has been [updated](https://github.com/ros/catkin/pull/940) to avoid writting the same file concurrently in workspaces which use a merged install space.
- The changes to the `ros_buildfarm` used to provide devel / PR jobs for ROS 2 should be applicable to ROS 1 as-is but will need further testing.

### Outlook

- Since `colcon` can be used to build ROS 1, early adopters can try to use it to build ROS 1 from source.

  While there is documentation how to migrate from [catkin_make_isolated](http://colcon.readthedocs.io/en/released/migration/catkin_make_isolated.html) and [catkin_tools](http://colcon.readthedocs.io/en/released/migration/catkin_tools.html) `colcon` won't be the recommended build tool in ROS 1 for the foreseeable future.

> 尽管有文档描述如何从[catkin_make_isolated](http://colcon.readthedocs.io/en/released/migration/catkin_make_isolated.html)和[catkin_tools](http://colcon.readthedocs.io/en/released/migration/catkin_tools.html)迁移到`colcon`，但在可预见的未来，`colcon`不会成为 ROS 1 中推荐的构建工具。

- If a test buildfarm using `colcon` proofs to deliver the exact same results as the ROS 1 buildfarm using `catkin_make_isolated` it might be changed to use `colcon` in the future to benefit from the features `colcon` provides (like non-interleaved output per package when building in parallel, per package log files, etc.).
