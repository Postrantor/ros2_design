---
tip: translate by openai@2023-05-28 11:20:24
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

In the ROS ecosystem the software is separated into numerous packages.

> 在 ROS 生态系统中，软件被分割成许多包。

It is very common that a developer is working on multiple packages at the same time.

> 开发者同时工作在多个包上非常常见。

This is in contrast to workflows where a developer only works on a single software package at a time, and all dependencies are provided once but not being iterated on.

> 这与工作流程形成了对比，在工作流程中，开发人员一次只能处理一个软件包，所有的依赖项只提供一次，而不会不断迭代。

The "manual" approach to build a set of packages consists of building all packages in their topological order one by one.

> 手动构建一组软件包的方法包括按照拓扑顺序逐一构建所有软件包。

For each package the documentation usually describes what the dependencies are, how to setup the environment to build the package as well as how to setup the environment afterwards to use the package.

> 对于每个包，文档通常会描述它们的依赖关系，如何设置环境来构建包以及如何在之后设置环境来使用该包。

Such a workflow is impracticable at scale without a tool that automates that process.

> 这种工作流程在没有自动化工具的情况下，在大规模部署时是不切实际的。

A build tool performs the task of building a set of packages with a single invocation.

> 一个构建工具可以通过一次调用来构建一组软件包。

For ROS 1 multiple different tools provide support for this, namely `catkin_make`, `catkin_make_isolated`, and `catkin_tools`.

> 对于 ROS 1，有多种不同的工具可以支持它，即`catkin_make`、`catkin_make_isolated`和`catkin_tools`。

For ROS 2 up to the Ardent release the build tool providing this functionality is called `ament_tools`.

> 对于 ROS 2 直到 Ardent 发布，提供此功能的构建工具称为“ament_tools”。

This article describes the steps to unify these build tools as well as extend the field of application.

> 这篇文章描述了统一这些构建工具以及扩展应用领域的步骤。

## Goal

The goal of a unified build tool is to build a set of packages with a single invocation.

> 一个统一的构建工具的目标是用一次调用构建一组包。

It should work with ROS 1 packages as well as ROS 2 packages which provide the necessary information in their manifest files.

> 它应该可以与 ROS 1 软件包以及提供必要信息的 ROS 2 软件包一起工作。

It should also work with packages that do not provide manifest files themselves, given that the necessary meta information can be inferred and/or is provided externally.

> 它也应该可以与不提供清单文件的包一起工作，只要可以推断出必要的元信息并/或从外部提供。

This will allow the build tool to be utilized for non-ROS packages (e.g. Gazebo including its ignition dependencies, sdformat, etc.).

> 这将允许构建工具用于非 ROS 包（例如 Gazebo，包括其 Ignition 依赖项、sdformat 等）。

In the ROS ecosystems several tools already exist which support this use case (see below).

> 在 ROS 生态系统中，已经存在几种可以支持这种用例的工具（见下文）。

Each of the existing tools performs similar tasks and duplicates a significant amount of the logic.

> 每个现有的工具执行类似的任务，重复了大量的逻辑。

As a consequence of being developed separately certain features are only available in some of the tools while other tools lack those.

> 由于分别开发，某些功能只有在某些工具中才可用，而其他工具则缺少这些功能。

The reason to use a single universal build tool comes down to reducing the effort necessary for development and maintenance.

> 使用唯一的通用构建工具的原因可以归结为减少开发和维护所需的努力。

Additionally this makes newly developed features available for all the use cases.

> 此外，这使得新开发的功能可以用于所有的用例。

### Build Tool vs. Build System

Since this article focuses on the build tool the distinction to a build system needs to be clarified.

> 本文重点关注构建工具，因此需要澄清与构建系统的区别。

A build tool operates on a set of packages.

> 一个构建工具操作一组包。

It determines the dependency graph and invokes the specific build system for each package in topological order.

> 它确定依赖关系图，并按拓扑顺序为每个包调用特定的构建系统。

The build tool itself should know as little as possible about the build system used for a specific package.

> 构建工具本身应该尽可能少了解用于特定软件包的构建系统。

Just enough in order to know how to setup the environment for it, invoke the build, and setup the environment to use the built package.

> 只需要足够的知识来知道如何设置环境、调用构建和设置环境以使用所构建的包。

The existing ROS build tools are: `catkin_make`, `catkin_make_isolated`, `catkin_tools`, and `ament_tools`.

> 现有的 ROS 构建工具有：`catkin_make`、`catkin_make_isolated`、`catkin_tools`和`ament_tools`。

The build system on the other hand operates on a single package.

> 另一方面，构建系统只操作一个包。

Examples are `Make`, `CMake`, `Python setuptools`, or `Autotools` (which isn't used in ROS atm).

> 例如：`Make`、`CMake`、`Python setuptools` 或者 `Autotools`（目前 ROS 中没有使用）。

A CMake package is e.g. build by invoking these steps: `cmake`, `make`, `make install`.

> 一个 CMake 包可以通过执行以下步骤来构建：`cmake`、`make`、`make install`。

`catkin` as well as `ament_cmake` are based on CMake and offer some convenience functions described below.

### Environment Setup

A very important part beside the actual build of a package is the environment setup.

> 重要的部分除了实际构建一个包之外，还有环境设置。

For example, in order for a CMake project to discover a dependency using the CMake function `find_package`, the CMake module (e.g. `FindFoo.cmake`) or the CMake config file (e.g. `FooConfig.cmake`) for that dependency must either be in a prefix that CMake searches implicitly (e.g. `/usr`) or the location must be provided through the environment variable `CMAKE_PREFIX_PATH` / `CMAKE_MODULE_PATH`.

> 为了使 CMake 项目使用 CMake 函数`find_package`发现依赖项，该依赖项的 CMake 模块（例如`FindFoo.cmake`）或 CMake 配置文件（例如`FooConfig.cmake`）必须位于 CMake 隐式搜索的前缀（例如`/usr`）中，或者可以通过环境变量`CMAKE_PREFIX_PATH` / `CMAKE_MODULE_PATH`提供其位置。

In addition to building a package on top of another package (using `find_package` in the case of CMake), you may need to adjust the environment in order to run an executable from a package.

> 除了在另一个包上构建一个包（在 CMake 的情况下使用`find_package`）之外，您可能还需要调整环境才能从包中运行可执行文件。

For example, when a package installs a shared library in a non-default location then the environment variable `LD_LIBRARY_PATH` (or `PATH` on Windows) needs to be extended to include the containing folder before trying to run executables that load that library at runtime.

> 例如，当一个包在非默认位置安装共享库时，在尝试运行在运行时加载该库的可执行文件之前，需要扩展环境变量`LD_LIBRARY_PATH`（或 Windows 上的`PATH`）以包含包含文件夹。

The functionality to setup these environment variables can be provided by either the build tool or the build system.

> 这些环境变量的设置功能可以由构建工具或构建系统提供。

In the latter case the build tool only needs to know how the build system exposes the environment setup in order to reuse it.

> 在后一种情况下，构建工具只需要了解构建系统如何暴露环境设置，以便重用它。

Considering the use case that a user might want to invoke the build system of each package manually it is beneficial if the build system already provides as much of the environment setup as possible.

> 考虑到用户可能想要手动调用每个包的构建系统的用例，如果构建系统已经提供尽可能多的环境设置，将是有益的。

That avoids forcing the user to manually take care of the environment setup when not using a build tool.

> 这避免了当不使用构建工具时，用户不得不手动处理环境设置的情况。

### Out of Scope

To clarify the scope of this article a few related topics are explicitly enumerated even though they are not being considered.

> 为了澄清本文的范围，尽管没有考虑，但仍明确列出了一些相关的话题。

#### Build System

Any build system related functionality (which is not directly relevant for the build tool) is not considered in this article.

> 本文不考虑任何与构建系统相关的功能（与构建工具没有直接关联）。

##### Mixing Different Build Systems

The unified build tool will support different build systems in order to satisfy the described goals.

> 统一的构建工具将支持不同的构建系统，以满足所描述的目标。

If packages using different build system inter-operate with each other correctly depends also to a large degree on the build system.

> 如果使用不同构建系统的包彼此之间正确互操作，也很大程度上取决于构建系统。

While the build tool should ensure that it doesn't prevent that use case this article will not cover the use case of mixing multiple build systems in a single workspace (e.g. ROS 1 packages using `catkin` with ROS 2 packages using `ament_cmake`).

> 尽管构建工具应确保不会阻止该用例，但本文不会涵盖在单个工作区中混合多个构建系统（例如，使用`catkin`的 ROS 1 软件包与使用`ament_cmake`的 ROS 2 软件包）的用例。

#### Fetch Source Code

The build tool does not cover the steps necessary to fetch the sources of the to-be-built packages.

> 构建工具不包括获取要构建的软件包源代码所必需的步骤。

There are already tools to help with this.

> 已经有工具可以帮助解决这个问题了。

For example, the list of repositories that need to be fetched is provided either by a hand crafted `.rosinstall` or `.repos` file or by using [rosinstall_generator](http://wiki.ros.org/rosinstall_generator) to generate one.

> 例如，需要获取的存储库的列表可以通过手动创建的`.rosinstall`或`.repos`文件提供，或者使用[rosinstall_generator](http://wiki.ros.org/rosinstall_generator)生成一个。

The list of repositories can then be fetched with one of several tools, like [rosinstall](http://wiki.ros.org/rosinstall) or [wstool](http://wiki.ros.org/wstool) in the case of a `.rosinstall` file, or [vcstool](https://github.com/dirk-thomas/vcstool) in the case of a `.repos` file.

> 可以使用几种工具（如[rosinstall](http://wiki.ros.org/rosinstall)或[wstool](http://wiki.ros.org/wstool)（如果是`.rosinstall`文件）或[vcstool](https://github.com/dirk-thomas/vcstool)（如果是`.repos`文件））来获取仓库列表。

#### Install Dependencies

The build tool also does not provide a mechanism to install any dependencies required to build the packages.

> 构建工具也不提供安装构建包所需的任何依赖项的机制。

In the ROS ecosystem [rosdep](http://wiki.ros.org/rosdep) can be used for this.

> 在 ROS 生态系统中，可以使用[rosdep](http://wiki.ros.org/rosdep)来完成这项任务。

#### Create Binary Packages

The build tool also does not create binary packages (e.g. a Debian package).

> 这个构建工具也不会创建二进制包（例如 Debian 包）。

In the ROS ecosystem [bloom](http://wiki.ros.org/bloom) is used to generate the required metadata and then platform dependent tools like `dpkg-buildpackage` build binary packages.

> 在 ROS 生态系统中，使用[bloom](http://wiki.ros.org/bloom)来生成所需的元数据，然后使用诸如`dpkg-buildpackage`等平台相关的工具来构建二进制包。

## Existing Build Systems

In the following the build systems being used in the ROS ecosystem are briefly described.

> 以下简要描述了 ROS 生态系统中使用的构建系统。

### CMake

[CMake](https://cmake.org/) is a cross-platform build system generator.

> CMake 是一个跨平台的构建系统生成器。

Projects specify their build process with platform-independent `CMakeLists.txt` files.

> 项目使用与平台无关的`CMakeLists.txt`文件来指定它们的构建过程。

Users build a project by using CMake to generate a build system for a native tool on their platform, e.g. `Makefiles` or `Visual Studio projects`.

> 用户使用 CMake 构建项目，为他们的平台上的本机工具生成构建系统，例如`Makefiles`或`Visual Studio projects`。

### catkin

[catkin](http://wiki.ros.org/catkin) is based on CMake and provides a set of convenience functions to make writing CMake packages easier.

> catkin 基于 CMake，提供了一系列便利的功能，使编写 CMake 包变得更容易。

It automates the generation of CMake config files as well as pkg-config files.

> 它自动生成 CMake 配置文件和 pkg-config 文件。

It additionally provides functions to register different kinds of tests.

> 它还提供注册不同类型测试的功能。

A package using `catkin` specifies its meta data in a manifest file named `package.xml`.

> 一个使用`catkin`的包在一个名为`package.xml`的清单文件中指定它的元数据。

The latest format of the manifest file is specified in the [ROS REP 149](http://www.ros.org/reps/rep-0149.html).

> 最新的清单文件格式在[ROS REP 149](http://www.ros.org/reps/rep-0149.html)中指定。

### ament_cmake

[ament_cmake](https://github.com/ament/ament_cmake) is an evolution of `catkin` and is also based on CMake.

> [ament_cmake](https://github.com/ament/ament_cmake) 是 catkin 的演进，也是基于 CMake 的。

The main difference between `ament_cmake` and `catkin` is described in [another article](http://design.ros2.org/articles/ament.html).

> 主要区别在[另一篇文章](http://design.ros2.org/articles/ament.html)中描述了`ament_cmake`和`catkin`之间的区别。

In the context of the build tool the biggest difference is that `ament_cmake` generates package-specific files to setup the environment to use the package after it has been built and installed.

> 在构建工具的上下文中，最大的不同是`ament_cmake`生成特定于包的文件，以在构建和安装后设置使用该包的环境。

A package using `ament_cmake` uses the same manifest file as `catkin` (except that it requires format version 2 or higher).

> 一个使用`ament_cmake`的包使用和`catkin`相同的清单文件（除了需要格式版本 2 或更高版本）。

### Python setuptools

`setuptools` is the common tool to package Python packages.

A Python package uses a `setup.py` file to describe the dependencies as well as how and what to build and install.

> 一个 Python 包使用`setup.py`文件来描述依赖关系，以及如何以及如何构建和安装。

In ROS 2 a package can be a "vanilla" Python package whereas in ROS 1 any Python functionality is triggered from a CMake file.

> 在 ROS 2 中，一个包可以是一个“vanilla”的 Python 包，而在 ROS 1 中，任何 Python 功能都是从 CMake 文件触发的。

## Existing Build Tools

Several different build tools are already being used in the ROS ecosystem.

> 在 ROS 生态系统中已经使用了几种不同的构建工具。

Their method of operating is being described in the following subsections together with their advantages as well as disadvantages.

> 以下子节将描述他们的操作方法，以及它们的优点和缺点。

### catkin_make

`catkin_make` is provided by the ROS package `catkin` which contains the build system for ROS 1.

It has been designed as the successor of `rosbuild` for ROS Fuerte.

> 它被设计为 ROS Fuerte 的`rosbuild`的后继者。

The tool invokes CMake only a single time and uses CMake's `add_subdirectory` function to process all packages in a single context.

> 工具仅调用 CMake 一次，并使用 CMake 的`add_subdirectory`函数在单个上下文中处理所有包。

While this is an efficient approach since all targets across all packages can be parallelized it comes with significant disadvantages.

> 这是一种高效的方法，因为可以并行处理所有包中的所有目标，但也有明显的缺点。

Due to the single context all function names, targets and tests share a single namespace across all packages and on a larger scale this easily leads to collisions.

> 由于单个上下文，所有函数名称、目标和测试在所有包中共享单个命名空间，而且在更大范围内，这很容易导致冲突。

The single context is also subject to side effects between the packages and sometimes requires adding additional target dependencies across package boundaries.

> 单一上下文也受到包之间的副作用的影响，有时需要跨越包的边界添加额外的目标依赖。

`catkin_make` supports building the following packages:

- ROS 1 `catkin` package with a `package.xml` file.

> 一个具有`package.xml`文件的 ROS 1 `catkin` 包

### catkin_make_isolated

`catkin_make_isolated` is provided by the ROS package `catkin` which contains the build system for ROS 1.

It was developed after `catkin_make` to address the problems involved with building multiple packages in a single CMake context.

> 它是在`catkin_make`之后开发的，以解决在单个 CMake 上构建多个包所涉及的问题。

The tool only supports CMake-based packages and builds each package in topological order using the command sequence common for CMake packages: `cmake`, `make`, `make install`.

> 这个工具只支持基于 CMake 的包，并以拓扑顺序使用 CMake 包的常用命令序列：`cmake`、`make`、`make install`来构建每个包。

While each package can parallelize the build of its targets the packages are processed sequentially even if they are not (recursive) dependencies of each other.

> 尽管每个包可以并行构建其目标，即使它们不是彼此的（递归）依赖关系，也会按顺序处理这些包。

`catkin_make_isolated` supports building the following packages:

- ROS 1 `catkin` package with a `package.xml` file.

> 一个具有`package.xml`文件的 ROS 1 `catkin` 包。

- Plain CMake packages with a `package.xml` file.

> 简单的 CMake 包，带有一个 `package.xml` 文件。

### catkin_tools

[catkin_tools](https://catkin-tools.readthedocs.io/) is provided by a standalone Python package used to build ROS 1 packages.

> [Catkin_tools](https://catkin-tools.readthedocs.io/)是一个独立的 Python 包，用于构建 ROS 1 软件包。

It was developed after `catkin_make` / `catkin_make_isolated` to build packages in parallel as well as provide significant usability improvements.

> 它是在`catkin_make`/`catkin_make_isolated`之后开发的，可以并行构建包，并提供重要的可用性改进。

The tool supports building CMake packages and builds them in isolation as well as supports parallelizing the process across packages.

> 这个工具支持构建 CMake 包，并且可以在隔离环境中构建它们，也支持跨包并行处理过程。

`catkin_tools` supports building the following packages:

- ROS 1 `catkin` package with a `package.xml` file.

> 一个 ROS 1 的 catkin 包，带有一个 package.xml 文件。

- Plain CMake packages with a `package.xml` file.

> 简单的 CMake 软件包，带有一个`package.xml`文件。

### ament_tools

`ament_tools` is provided by a standalone Python 3 package used to build ROS 2 packages.

It was developed to bootstrap the ROS 2 project, is therefore only targeting Python 3, and works on Linux, MacOS and Windows.

> 它是为了引导 ROS 2 项目而开发的，因此只针对 Python 3，并且可以在 Linux、MacOS 和 Windows 上运行。

In addition to CMake packages it also supports building Python packages and can infer meta information without requiring an explicit package manifest (which is e.g. used for the FastRTPS package).

> 此外，它还支持构建 Python 软件包，并且可以在不需要显式软件包清单（例如用于 FastRTPS 软件包）的情况下推断元信息。

The tool performs an "isolated" build like `catkin_make_isolated` and `catkin_tools` (one CMake invocation per package) and also parallelizes the build of packages which have no (recursive) dependencies on each other (like `catkin_tools`).

> 这个工具像`catkin_make_isolated`和`catkin_tools`一样执行"隔离"构建（每个包一次 CMake 调用），并且可以并行构建那些彼此没有（递归）依赖关系的包（就像`catkin_tools`一样）。

While it covers more build systems and platforms than `catkin_tools` it doesn't have any of `catkin_tools`s usability features like profiles, output handling, etc.

> 虽然它比 catkin_tools 支持更多的构建系统和平台，但它没有 catkin_tools 的可用性特性，比如配置文件、输出处理等。

`ament_tools` supports building the following packages:

- ROS 2 `ament_cmake` package with a `package.xml` file (only format 2).

> - 使用`package.xml`文件（仅限格式 2）的 ROS 2 `ament_cmake`软件包。

- Plain CMake package with a `package.xml` file.

> 简单的 CMake 包，带有一个`package.xml`文件。

- Plain CMake package without a manifest file (extracting the package name and dependencies from the CMake files).

> - 一个没有清单文件的纯 CMake 包（从 CMake 文件中提取包名称和依赖关系）。

- Python package with a `package.xml` file.

> Python 包，带有一个 `package.xml` 文件。

- Python package without a manifest file (extracting the package name and dependencies from the `setup.py` file).

> 无清单文件的 Python 包（从`setup.py`文件中提取包名称和依赖项）。

### colcon

When the first draft of this article was written the conclusion was to not to spend any resources towards a universal build tool.

> 当这篇文章的第一稿被写下时，结论是不应该投入任何资源去开发一个通用的构建工具。

Meanwhile the author of this article went ahead and developed [colcon](https://github.com/colcon/) as a personal project.

> 与此同时，这篇文章的作者继续推进，并将[colcon](https://github.com/colcon/)作为个人项目开发起来。

Therefore its feature set is closely aligned with the following requirements.

> 因此，它的功能集与以下要求密切相关。

## Naming

The existing build tools in ROS are all named by the build system they are supporting.

> ROS 中现有的构建工具都按照它们所支持的构建系统命名。

In general it should be possible for a build tool to support multiple different build systems.

> 一般来说，构建工具应该能够支持多种不同的构建系统。

Therefore a name for a build tool being derived from a single build system might mislead the users that the tool only works for that specific build system.

> 因此，将构建工具的名称源自于单一的构建系统可能会误导用户，认为该工具仅适用于特定的构建系统。

To avoid confusion of the user the build tool should have a different unrelated name to avoid implying an undesired correlation.

> 为了避免用户混淆，构建工具应该有一个不相关的不同名称，以避免暗示不希望的相关性。

## Requirements

The unified build tool should provide a superset of the functionality provided by the existing tools.

> 统一的构建工具应该提供现有工具所提供的功能的超集。

In the following a few use cases are described as well as desired software criteria.

> 以下描述了几个用例以及所需的软件要求。

Other use cases which are not explicitly covered but are already supported by the existing tools (e.g. cross-compilation, `DESTDIR` support, building CMake packages without a manifest) should continue to work with the unified build tool.

> 其他尚未明确涵盖但已由现有工具支持的用例（例如跨编译、`DESTDIR`支持、不使用清单构建 CMake 包）应继续使用统一的构建工具。

### Use Cases

The following uses cases should be satisfied by the unified build tool.

> 以下用例应该由统一的构建工具满足。

#### Build ROS 1 workspaces

The tool needs to be able to build ROS 1 workspaces which can already be built using `catkin_make_isolated` / `catkin_tools`.

> 这个工具需要能够构建 ROS 1 工作空间，这些工作空间已经可以使用`catkin_make_isolated`/`catkin_tools`来构建。

It is up to the implementation to decide if it only supports the standard CMake workflow or also the _custom devel space concept_ of `catkin`.

> 由实现决定是否只支持标准 CMake 工作流程，或者也支持 catkin 的*自定义开发空间概念*。

In ROS 2 the concept of the _devel space_ has intentionally been removed.

> 在 ROS 2 中，已经有意移除了*开发空间*的概念。

In the future it might be feasible to provide the concept of _symlinked installs_ in ROS 1 to provide a similar benefit without the downsides.

> 在将来，可能可行的是在 ROS 1 中提供*符号链接安装*的概念，以提供类似的好处而无需降低性能。

#### Build ROS 2 workspaces

The tool needs to be able to build ROS 2 workspaces which can already be built using `ament_tools`.

> 需要能够使用`ament_tools`构建 ROS 2 工作区的工具。

#### Build Gazebo including dependencies

After cloning the repositories containing Gazebo and all its dependencies (excluding system packages) the tool needs to be able to build the set of packages.

> 在克隆包含 Gazebo 及其所有依赖项（不包括系统包）的仓库后，该工具需要能够构建这组软件包。

Meta information not inferable from the sources can be provided externally without adding or modifying any files in the workspace.

> 不能从源中推断出的元信息可以在不添加或修改工作区中任何文件的情况下从外部提供。

After the build a single file can be sourced / invoked to setup the environment to use Gazebo (e.g. `GAZEBO_MODEL_PATH`).

> 在构建之后，可以源/调用单个文件来设置使用 Gazebo 的环境（例如`GAZEBO_MODEL_PATH`）。

### Development Environment Setup

Invoking a build system for a package implies also setting up environment variables before the process, e.g. the `CMAKE_PREFIX_PATH`.

> 调用构建系统来构建一个包，还需要在进程开始前设置环境变量，例如`CMAKE_PREFIX_PATH`。

It should be possible for developers to manually invoke the build system for one package.

> 可以让开发者手动调用一个包的构建系统。

The environment variable might be partially different from the environment variables necessary to use a package after it has been built.

> 环境变量可能与在构建后使用该包所需的环境变量部分不同。

To make that convenient the tool should provide an easy-to-use mechanism to setup the development environment necessary to manually invoke the build system.

> 要使其方便，该工具应提供一种易于使用的机制来设置手动调用构建系统所需的开发环境。

### Beyond Building

Building packages is only one task the build tool can perform on a set of packages.

> 编译包只是构建工具在一组包上执行的一项任务。

Additional tasks like e.g. running tests should also be covered by the build tool.

> 构建工具也应该覆盖诸如运行测试之类的额外任务。

The build tool must provide these abstract tasks and then map them to the steps necessary for each supported build system.

> 构建工具必须提供这些抽象任务，然后将它们映射到每个支持的构建系统所需的步骤。

### Software Criteria

The tool aims to support a variety of build systems, use cases, and platforms.

> 这个工具旨在支持各种构建系统、用例和平台。

The above mentioned ones are mainly driven by the needs in the ROS ecosystem but the tool should also be usable outside the ROS ecosystem (e.g. for Gazebo).

> 以上提到的主要受 ROS 生态系统需求的驱动，但该工具也可以在 ROS 生态系统之外使用（例如 Gazebo）。

Therefore it should be designed in a way which enables extending its functionality.

> 因此，它应该以一种使其功能可扩展的方式设计。

Assuming that the tool will be implemented in Python (since that is the case for all existing ROS build tools) the entry point mechanism provides a convenient way to make the software extensible.

> 假设工具将用 Python 实现（因为所有现有的 ROS 构建工具都是这样），入口点机制提供了一种方便的方式来使软件可扩展。

Extensions don't even have to be integrated into the Python package containing the core logic of the build tool but can easily be provided by additional Python packages.

> 扩展甚至不必集成到包含构建工具核心逻辑的 Python 包中，而可以通过额外的 Python 包轻松提供。

This approach will not only foster a modular design and promote clear interfaces but enable external contributions without requiring them to be integrated in a single monolithic package.

> 这种方法不仅可以促进模块化设计和清晰的接口，而且可以在不需要将它们整合到单一的大型软件包中的情况下实现外部贡献。

Several well known software principles apply:

> 几个著名的软件原则适用：

- Separation of concerns

> 分离关注点

- Single Responsibility principle

> 单一职责原则

- Principle of Least Knowledge

> 原则最少知识

- Don’t repeat yourself

> 不要重复自己。

- Keep it stupid simple

> 保持简单傻瓜式。

- "Not paying for what you don't use"

> 不要为你不使用的东西付费。

### Extension Points

The following items are possible extension points to provide custom functionality:

> 以下项目可用于提供自定义功能：

- contribute `verbs` to the command line tool (e.g. `build`, `test`)

> 为命令行工具贡献动词（例如 `build`、`test`）

- contribute command line options for specific features (e.g. `build`, `test`)

> 为特定功能提供命令行选项（例如`build`，`test`）

- discovery of packages (e.g. recursively crawling a workspace)

> 发现包（例如递归爬取工作区）

- identification of packages and their meta information (e.g. from a `package.xml` file)

> - 识别包及其元信息（例如从`package.xml`文件）

- process a package (e.g. build a CMake package, test a Python package)

> 处理一个包（例如构建一个 CMake 包，测试一个 Python 包）

- execution model (e.g. sequential processing, parallel processing)

> 执行模型（例如顺序处理、并行处理）

- output handling (e.g. console output, logfiles, status messages, notifications)

> 处理输出（例如控制台输出、日志文件、状态消息和通知）

- setup the environment (e.g. `sh`, `bash`, `bat`)

> 設定環境（例如`sh`、`bash`、`bat`）

- completion (e.g. `bash`, `Powershell`)

> 完成（例如`bash`、`Powershell`）

## Possible Approaches

When the first draft of this article was written neither of the existing build tools supported the superset of features described in this article.

> 当这篇文章的第一稿写作时，现有的构建工具都不支持本文描述的超集功能。

There were multiple different paths possible to reach the goal of a universal build tool which fall into two categories:

> 有多种不同的路径可以达到通用构建工具的目标，这些路径可以分为两类：

- One approach is to incrementally evolve one of the existing tools to satisfy the described goals.

> 一种方法是逐步演化其中一个现有工具，以满足所描述的目标。

- Another approach would be to start "from scratch".

> 另一种方法是从零开始。

Since then the new project `colcon` has been developed which covers most of the enumerated requirements and represents the second category.

> 从那时起，新项目“colcon”已经开发出来，它涵盖了大多数列出的要求，属于第二类。

### Evolve catkin_make, catkin_make_isolated, or ament_tools

Since neither of these three build tools has the feature richness of `catkin_tools` it is considered strictly less useful to starting building upon one of these build tools.

> 由于这三个构建工具中没有一个具有`catkin_tools`的功能丰富性，因此认为基于其中一个构建工具开始构建严格不够有用。

Therefore neither of these are being considered as a foundation for a universal build tool.

> 因此，这两者都不被视为通用构建工具的基础。

### Evolve catkin_tools

Since `catkin_tools` is in many aspects the most complete ROS build tool it should be the one being evolved.

> 由于`catkin_tools`在许多方面是最完整的 ROS 构建工具，因此应该是要演变的一种。

While `ament_tools` has a few features `catkin_tools` currently lacks (e.g. plain CMake support without a manifest, Windows support) the feature richness of `catkin_tools` makes it a better starting point.

> 尽管`ament_tools`有一些`catkin_tools`目前缺少的功能（例如，不需要清单就支持普通 CMake，支持 Windows），但`catkin_tools`的功能丰富使其成为一个更好的起点。

### Start "from scratch" / colcon

Since the first draft of this article the `colcon` project has been developed with the goals and requirements of a universal build tool in mind.

> 自从这篇文章的第一稿发布以来，`colcon`项目一直在按照一个通用构建工具的目标和要求进行开发。

In its current form it is already able to build ROS 1 workspaces, ROS 2 workspaces, as well as Gazebo including its ignition dependencies.

> 它目前的形式已经能够构建 ROS 1 工作区、ROS 2 工作区以及 Gazebo，包括其点火依赖项。

It uses Python 3.5+ and targets all platforms supported by ROS: Linux, macOS, and Windows.

> 它使用 Python 3.5+，并针对 ROS 支持的所有平台：Linux、macOS 和 Windows。

Since it hasn't been used by many people yet more advanced features like cross compilation, `DESTDIR`, etc. hasn't been tested (and will therefore likely not work yet).

> 由于尚未有多少人使用，因此跨编译、DESTDIR 等更高级功能尚未经过测试（因此很可能尚未正常工作）。

## Decision process

For the decision process only the following two options are being considering based on the rationale described above:

> 根据上述理由，仅考虑以下两个选项来进行决策：

- option **A)** Use `catkin_tools` as a starting point

> **A)** 使用`catkin_tools`作为起点

- option **B)** Use `colcon` as a starting point

> 使用 colcon 作为起点，选项**B）**

If this topic would have been addressed earlier some of the duplicate effort could have likely been avoided.

> 如果这个话题早些时候被提出，可能会避免一些重复的努力。

When the work towards a universal build tool was suspended over a year ago it was a conscious decision based on available resources.

> 当一年前暂停通向通用构建工具的工作时，这是基于可用资源的有意识的决定。

Nevertheless moving forward with a decision now will at least avoid further uncertainty and effort duplication.

> 尽管如此，现在做出决定至少可以避免进一步的不确定性和重复努力。

Both of the considered options have unique and valuable features and there are good arguments to build our future development on either of the two tools.

> 两个考虑的选项都有独特而有价值的特性，有充分的理由基于这两种工具来构建我们未来的发展。

Since both are written in Python either of the two tools could be "transformed" to cover the pros of the other one.

> 两者都用 Python 编写，可以把其中一个工具“转换”来覆盖另一个工具的优势。

So the two important criteria for the decision are:

> 两个重要的决定标准是：

- the effort it takes to do (in the short term as well as in the long term) and

> 所需的努力（无论是短期还是长期）

- the difference of the resulting code base after the "transformation" is completed.

> 经过“转换”完成后，所得到的代码库的差异。

### Immediate goals

A ROS 2 developer currently builds a steadily growing workspace with ROS 2 packages.

> 一位 ROS 2 开发者目前正在构建一个不断增长的 ROS 2 包工作空间。

The same is happening in the monolithic Jenkins jobs on [ci.ros2.org](https://ci.ros2.org) (with the advantage to test changes across repositories easily).

> 在[ci.ros2.org](https://ci.ros2.org)上的单体 Jenkins 作业也是如此（有利于跨存储库轻松测试变更）。

Therefore features to easily filter the packages which need to be build are eagerly awaited to improve the development process.

> 因此，期望有一种容易过滤需要构建的软件包的功能，以改善开发过程。

For the last ROS 2 release _Ardent_ the buildfarm [build.ros.org](http://build.ros2.org) only provides jobs to generate Debian packages.

> 对于最新的 ROS 2 发行版*Ardent*，[build.ros.org](http://build.ros2.org) 上只提供生成 Debian 软件包的任务。

Neither _devel_ jobs or _pull request_ jobs are available nor is it supported to build a local _prerelease_.

> 没有开发工作或拉取请求工作可用，也不支持构建本地预发布版本。

For the coming ROS 2 release _Bouncy_ these job types should be available to support maintainers.

> 对于即将发布的 ROS 2 _Bouncy_ 版本，应提供这些工作类型来支持维护人员。

In ROS 2 _Bouncy_ the universal build tool will be the recommended option.

> 在 ROS 2 *Bouncy*中，通用构建工具将是推荐的选择。

#### Necessary work

For either option **A)** or **B)** the follow items would need to be addressed:

> 无论是选择 A)还是 B)，以下项目都需要解决：

- The jobs and scripts on _ci.ros2.org_ need to be updated to invoke the universal build tool instead of `ament_tools`.

> 在*ci.ros2.org*上的工作和脚本需要更新，以调用通用构建工具而不是`ament_tools`。

- The `ros_buildfarm` package needs to be updated to invoke the universal build tool instead of `catkin_make_isolated`.

> `ros_buildfarm` 包需要更新，以调用通用构建工具而不是 `catkin_make_isolated`。

The ROS 2 buildfarm would use this modification for the upcoming ROS 2 _Bouncy_ release.

> ROS 2 的构建农场将在即将发布的 ROS 2 _Bouncy_ 版本中使用这个修改。

The ROS 1 buildfarm could use the same modification in the future.

> ROS 1 的构建农场将来可以使用同样的修改。

For option **A)** the follow items would need to be addressed:

> 对于选项 A，以下项目需要解决：

- Support for setup files generated by `ament_cmake`.

> 支持由`ament_cmake`生成的安装文件。

- Support additional packages types: plain Python packages, CMake packages without a manifest.

> 支持额外的包类型：普通的 Python 包、没有清单的 CMake 包

- Support for Windows using `.bat` files.

> 支持使用`.bat`文件在 Windows 上运行。

- Support for the package manifest format version 3.

> 支持包清单格式版本 3.

For option **B)** the follow items would need to be addressed:

> 对于选项**B）**，需要解决以下问题：

- Address user feedback when the tool is being used by a broader audience.

> 回应用户反馈，当工具被更广泛的用户使用时。

### Future

The long term goal is that the universal build tool will be used in ROS 1, in ROS 2 as well as other non-ROS projects.

> 长期目标是，通用构建工具将在 ROS 1，ROS 2 以及其他非 ROS 项目中使用。

There is currently no time line when the tool will be used on the ROS 1 build or be recommended to ROS 1 users.

> 目前没有时间表可以告诉我们什么时候可以在 ROS 1 上使用这个工具，或者建议 ROS 1 用户使用它。

This solely depends on the resources available for ROS 1.

> 这完全取决于可用于 ROS 1 的资源。

Beside that for both options there is follow up work beyond the immediate goals.

> 除此之外，对于这两个选项，在达到立即目标之外，还有后续工作需要完成。

The following enumerates a few of them but is by no means exhaustive:

> 以下列举了其中一些，但绝不是全部。

For option **A)** the follow items should be considered:

> 对于选项 A，应该考虑以下几项：

- Support for Python packages using a `setup.cfg` file.

> 支持使用 `setup.cfg` 文件的 Python 包。

- Support for `PowerShell` to work around length limitations for environment variable on Windows.

> 支持 PowerShell 在 Windows 上围绕环境变量长度限制而工作。

- Support for `Pytest` to run Python tests (instead of using `nose`).

> 支持使用 Pytest 运行 Python 测试（而不是使用 nose）。

- Support to pass package specific argument.

> 支持传递特定包的参数。

- Update code base to Python 3.5+.

> 更新代码库到 Python 3.5+。

- Refactor code base to reduce coupling (e.g. separate [API](https://github.com/catkin/catkin_tools/blob/2cae17f8f32b0193384d2c7734afee1c60c4add2/catkin_tools/execution/controllers.py#L183-L205) for output handling).

> 重构代码以减少耦合（例如分离[API]（https://github.com/catkin/catkin_tools/blob/2cae17f8f32b0193384d2c7734afee1c60c4add2/catkin_tools/execution/controllers.py#L183-L205）用于输出处理）。

- Additional functionality to build Gazebo including its dependencies.

> 增加功能来构建 Gazebo，包括它的依赖。

- Whether or not to continue supporting the _devel space_ concept in ROS 1.

> 是否继续在 ROS 1 中支持*devel 空间*概念？

For option **B)** the follow items should be considered:

> 对于选项 B，应该考虑以下内容：

- Support `DESTDIR`.

> 支持 DESTDIR

- Support a feature similar to the `profile` verb of `catkin_tools`.

> 支持类似 catkin_tools 中的`profile`动词的功能。

- Support for a shared GNU Make job sever.

> 支持共享 GNU Make 作业服务器。

- Support for `GNUInstallDirs`

> 支持`GNUInstallDirs`

- Not sure about the status of this, it would be in `colcon`'s generated shell files if anywhere.

> 不确定这个状态，如果存在的话，它会出现在`colcon`生成的 shell 文件中。

- Should have a test for this case.

> 应该为这种情况进行测试。

- Test for, and fix if necessary, correct topological order with dependencies across workspaces.

> 测试并在必要时修复跨工作区的依赖关系的正确拓扑顺序。

- See: [https://github.com/ros/catkin/pull/590](https://github.com/ros/catkin/pull/590)

> 见：[https://github.com/ros/catkin/pull/590](https://github.com/ros/catkin/pull/590)

## Summary and Decision

Based on the above information a decision has been made to pick `colcon` as the universal build tool.

> 根据以上信息，已决定选择 colcon 作为通用构建工具。

The decision was made after considering the input of ROS 2 team members and some ROS 1 users.

> 决定是在考虑 ROS 2 团队成员和一些 ROS 1 用户的意见后做出的。

The decision was not easy, as it was not unanimous, but the vast majority of input was either pro `colcon` or ambivalent.

> 决定并不容易，因为并不是所有人都赞成，但是绝大多数的意见都是支持`colcon`或者中立的。

To elaborate on the rationale one significant advantage of `colcon` is that it is ready to be deployed for ROS 2 right now and it covers our current use cases.

> colcon 有一个重要的优势是它现在已经可以部署到 ROS 2 上，并且可以满足我们当前的使用场景。

Another argument leaning towards `colcon` is the expected little effort to provide devel / PR / prerelease jobs on build.ros2.org across all targeted platforms for the upcoming _Bouncy_ release.

> 另一个有利于`colcon`的论点是，预计在即将推出的*Bouncy*版本中，在 build.ros2.org 上为所有目标平台提供 devel/PR/预发布作业的努力会很小。

While some additional feature and usability options are still missing they can be added in the future whenever there is time and/or demand for them.

> 一些额外的功能和可用性选项仍然缺失，但可以在有时间和/或需求时添加。

The necessary up front development effort for `catkin_tools` to achieve the goals described for _Bouncy_ would distract the ROS 2 team from spending their time on feature development and bug fixing of ROS 2 itself.

> 开发 catkin_tools 来实现*Bouncy*所描述的目标所需的前期开发努力会分散 ROS 2 团队从 ROS 2 本身的功能开发和错误修复上花费的时间。

While the short term advantages are certainly a main reason for the decision in favor of `colcon` they are not the only ones.

> 虽然短期优势肯定是支持`colcon`的决定的主要原因，但不是唯一的原因。

The cleaner architecture, modularity and extensibility as well as Python 3.5 code base will be valuable long term benefits when developing this tool in the future.

> 清洁架构、模块化和可扩展性以及 Python 3.5 代码库将是未来开发此工具时的长期利益。

The separation of the build tool name from the supported build systems as well as the separation from being a "ROS-only" tool will hopefully also help users to understand the difference and attract new users and potential contributors.

> 将构建工具名称与支持的构建系统的分离，以及从“ROS-only”工具的分离，希望能帮助用户理解差异，吸引新用户和潜在贡献者。

### Next steps

The following next steps will happen before the next ROS 2 release _Bouncy_.

> 下一个 ROS 2 发行版*Bouncy*发布之前，将会发生以下下一步操作。

- The ROS 2 buildfarm(s) will be updated to use `colcon` and provide devel / PR / prerelease jobs for the _Bouncy_ release.

> ROS 2 的构建农场将被更新以使用`colcon`，并为*Bouncy*发行版提供开发/PR/预发布作业。

- The instructions in the ROS 2 wiki to build from source will be updated to use `colcon` instead.

> 按照 ROS 2 维基上的说明来源码编译将会更新为使用`colcon`代替。

After the Bouncy release the `ament_tools` repository will be archived, removed from the `ros2.repos` file, and won't be released into future ROS 2 distributions.

> 在 Bouncy 发布后，`ament_tools`存储库将被归档，从`ros2.repos`文件中移除，不会被发布到未来的 ROS 2 发行版中。

### Implications

The following items briefly enumerate what this decision means for ROS developers and users:

> 以下项目简要列举了这一决定对 ROS 开发者和用户意味着什么：

- **No CMake code** of any ROS 2 (or ROS 1) package **needs to be changed** for this transition.

> - 在这次过渡中，**不需要更改**任何 ROS 2（或 ROS 1）包的**CMake 代码**。

- When building and testing ROS 2 the command `colcon build` / `colcon test` will be used instead of `ament build` / `ament test`.

> 当构建和测试 ROS 2 时，将使用`colcon build`/`colcon test`命令而不是`ament build`/`ament test`命令。

Please see the [documentation](http://colcon.readthedocs.io/en/released/migration/ament_tools.html) how to map `ament_tools` command line arguments to `colcon` arguments.

> 请查看[文档](http://colcon.readthedocs.io/en/released/migration/ament_tools.html)，了解如何将`ament_tools`命令行参数映射到`colcon`参数。

- For ROS 1 nothing is changing at this point in time.

> 目前，对于 ROS 1 来说，一切都没有改变。

- In the future `colcon` will replace `catkin_make_isolated` and `catkin_make` as the recommended build tool for ROS 1.

> 在未来，colcon 将取代 catkin_make_isolated 和 catkin_make 作为 ROS 1 的推荐构建工具。

- `colcon` will not support the _devel space_ and will require packages to have install rules

> colcon 不支持*开发空间*，需要安装规则的包。

- `catkin` will likely still support the _devel space_, though it might be removed at some point (that has not been decided yet)

> catkin 可能仍然支持*devel 空间*，尽管它可能在某个时候被移除（尚未决定）。

- Therefore, it is possible that the default build tool for ROS 1 may not support the _devel space_, though legacy tools will continue to support it.

> 因此，ROS 1 的默认构建工具可能不支持*开发空间*，尽管传统工具仍将支持它。

- Note that it is already the case that individual ROS 1 catkin packages may either not have installation rules but support the _devel space_, or they might have installation rules but not properly support the _devel space_.

> 注意，单个 ROS 1 catkin 软件包可能没有安装规则，但支持*devel 空间*，或者它们可能有安装规则，但不能正确支持*devel 空间*。

### Progress

#### ROS 2

- ci.ros2.org has been [updated](https://github.com/ros2/ci/pull/132)

> - ci.ros2.org 已经[更新](https://github.com/ros2/ci/pull/132)了

- build.ros2.org will use [this branch](https://github.com/ros-infrastructure/ros_buildfarm/pull/548) of `ros_buildfarm`

> build.ros2.org 将使用 [此分支](https://github.com/ros-infrastructure/ros_buildfarm/pull/548) 的 `ros_buildfarm`

#### ROS 1

- `catkin` has been [updated](https://github.com/ros/catkin/pull/940) to avoid writting the same file concurrently in workspaces which use a merged install space.

> catkin 已经更新（https://github.com/ros/catkin/pull/940），以避免在使用合并安装空间的工作区中同时编写同一文件。

- The changes to the `ros_buildfarm` used to provide devel / PR jobs for ROS 2 should be applicable to ROS 1 as-is but will need further testing.

> 这些对`ros_buildfarm`的更改用于为 ROS 2 提供开发/ PR 工作，应该可以直接应用于 ROS 1，但需要进一步测试。

### Outlook

- Since `colcon` can be used to build ROS 1, early adopters can try to use it to build ROS 1 from source.

> 因为可以使用 colcon 来构建 ROS 1，早期采用者可以尝试使用它从源代码构建 ROS 1。

While there is documentation how to migrate from [catkin_make_isolated](http://colcon.readthedocs.io/en/released/migration/catkin_make_isolated.html) and [catkin_tools](http://colcon.readthedocs.io/en/released/migration/catkin_tools.html) `colcon` won't be the recommended build tool in ROS 1 for the foreseeable future.

> 尽管有文档介绍如何从[catkin_make_isolated](http://colcon.readthedocs.io/en/released/migration/catkin_make_isolated.html)和[catkin_tools](http://colcon.readthedocs.io/en/released/migration/catkin_tools.html)迁移到`colcon`，但在可预见的未来，`colcon`不会成为 ROS 1 中推荐的构建工具。

- If a test buildfarm using `colcon` proofs to deliver the exact same results as the ROS 1 buildfarm using `catkin_make_isolated` it might be changed to use `colcon` in the future to benefit from the features `colcon` provides (like non-interleaved output per package when building in parallel, per package log files, etc.).

> 如果一个使用`colcon`的测试构建农场能够提供与 ROS 1 使用`catkin_make_isolated`构建农场完全相同的结果，那么未来可能会改用`colcon`，以获得`colcon`提供的特性（如并行构建时每个包的非交错输出、每个包的日志文件等）。
