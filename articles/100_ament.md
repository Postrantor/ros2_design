---
tip: translate by openai@2023-05-30 08:48:33
layout: default
title: The build system "ament_cmake" and the meta build tool "ament_tools"
permalink: articles/ament.html
abstract:
  This article describes the build system "ament_cmake" and the meta build tool "ament_tools".
published: true
author: '[Dirk Thomas](https://github.com/dirk-thomas)'
date_written: 2015-07
last_modified: 2018-06
categories: Overview

Authors: {{ page.author }}
Date Written: {{ page.date_written }}
Last Modified: {% if page.last_modified %}{{ page.last_modified }}{% else %}{{ page.date_written }}{% endif %}
---
<div class="alert alert-warning" markdown="1">

When this article was originally written `ament_tools` was the ROS 2 specific build tool. The reason was that it needed to be side-by-side installable with existing ROS 1 packages which was a problem due to different targeted Python versions. In the meantime the problem due to different Python versions has been addressed in shared dependencies like `catkin_pkg`. As of ROS 2 Bouncy `ament_tools` has been superseded by `colcon` as described in the [universal build tool](http://design.ros2.org/articles/build_tool.html) article.

> 当这篇文章最初被写入时，`ament_tools` 是 ROS 2 特定的构建工具。原因是它需要与现有的 ROS 1 软件包并行安装，由于不同的目标 Python 版本，这是一个问题。与此同时，由于不同的 Python 版本而引起的问题已经在共享依赖项（如 `catkin_pkg`）中得到解决。从 ROS 2 Bouncy 开始，`ament_tools` 已经被 `colcon` 所取代，如[通用构建工具](http://design.ros2.org/articles/build_tool.html)文章所述。

</div>

## Preface

ROS is developed with a federated model. While this comes with a lot of advantages it makes the process of building several interdependent packages more complex.

> ROS 是用联邦模型开发的。虽然这带来了很多好处，但使构建几个相互依赖的软件包变得更加复杂。

In order to build a set of interdependent packages the user must build them in the correct topological order. The user must look at the documentation of each package to determine the dependencies, being careful to build those packages before hand, and to determine the instructions for building that package. Finally the user must build and install each package and do any environment configuration before proceeding to build the next package.

> 为了构建一组相互依赖的软件包，用户必须按正确的拓扑顺序构建它们。用户必须查看每个软件包的文档，以确定依赖关系，小心翼翼地在此之前构建这些软件包，并确定构建该软件包的说明。最后，用户必须构建和安装每个软件包，并在继续构建下一个软件包之前进行任何环境配置。

ROS mitigates that complexity with package and build system conventions as well as tools to automate the build process. Wherever possible ROS uses standard tools, e.g. CMake and Python setuptools. Where these tools lack built-in support for what ROS needs to do, the additional functionality is provided by ROS.

> ROS 通过包和构建系统约定以及用于自动构建过程的工具来减轻复杂性。在可能的地方，ROS 使用标准工具，例如 CMake 和 Python setuptools。在这些工具缺乏 ROS 所需要做的内置支持时，ROS 提供额外的功能。

### Origin of ament

ament is an evolution of [catkin](http://wiki.ros.org/catkin). The word _ament_ is actually a synonym of _catkin_. For more information about the differences please read [below](#how-is-ament-different-from-catkin).

> ament 是 [catkin](http://wiki.ros.org/catkin) 的进化。这个单词 ament 实际上是 catkin 的同义词。要了解更多关于它们之间的区别，请阅读下面的[内容](#how-is-ament-different-from-catkin)。

## ament

ament is a meta build system to improve building applications which are split into separate packages. It consists of two major parts:

> ament 是一个元构建系统，用于改善将应用程序分割成单独的包的构建。它由两个主要部分组成：

- a _build system_ (e.g. CMake, Python setuptools) to configure, build, and install a single package
- a _tool_ to invoke the build of individual packages in their topological order

The tool relies on meta information about the packages to determine their dependencies and their build type. This meta information is defined in a manifest file called `package.xml` which is specified in [REP 140](http://www.ros.org/reps/rep-0140.html).

> 该工具依赖包的元信息来确定它们的依赖关系和构建类型。这些元信息定义在一个称为 `package.xml` 的清单文件中，详见 [REP 140](http://www.ros.org/reps/rep-0140.html)。

Each package is built separately with its own build system. In order to make the output of one package available to other packages each package can extend the environment in a way that downstream packages can find and use its artifacts and resources. If the resulting artifacts are installed into `/usr`, for example, it might not be necessary to alter the environment at all since these folders are commonly being searched by various tools.

> 每个包都使用自己的构建系统单独构建。为了使一个包的输出可供其他包使用，每个包都可以以一种方式扩展环境，以便下游包可以找到并使用其工件和资源。如果生成的工件安装到 `/usr` 中，例如，可能根本不需要改变环境，因为这些文件夹通常会被各种工具搜索。

### The ament command line tool

[ament_tools](https://github.com/ament/ament_tools) is a Python package which provides the command line tool `ament` to build, test, install, and uninstall packages. It is similar to [catkin_tools](https://github.com/catkin/catkin_tools) and builds each package in a workspace in topological order. Support for different build systems is integrated through extension points which allows support for other build types to be added without changing the ament tool itself.

> `[ament_tools](https://github.com/ament/ament_tools) 是一个Python包，提供命令行工具` ament `来构建、测试、安装和卸载软件包。它类似于[catkin_tools](https://github.com/catkin/catkin_tools)，并且以拓扑顺序在工作区中构建每个软件包。通过扩展点集成了不同构建系统的支持，这允许添加其他构建类型的支持而不需要更改ament工具本身。`

While it currently does not build packages in parallel that feature will be added in the future to speed up the build process. The goal is to reuse common functionality from catkin_tools by making it available through a third package which can be used by both tools.

> 在目前，它不能并行构建包，但将来会添加此功能以加快构建过程。目标是通过第三个包提供 catkin_tools 的常用功能，两个工具都可以使用它。

### Integrate arbitrary build systems

Each package can utilize a different build system to perform the steps of configuring, building, and installing. The build type is defined in each package manifest using the [build_type](http://www.ros.org/reps/rep-0140.html#build-type) tag in the export section.

> 每个包都可以使用不同的构建系统来执行配置、构建和安装步骤。构建类型在每个包清单中使用 [build_type](http://www.ros.org/reps/rep-0140.html#build-type) 标记在导出部分中定义。

Currently supported are CMake and Python but support for others (e.g. autotools, plain Makefiles) will likely be added in the future. For each build system the native steps are being applied separately by the ament tool.

> 目前支持 CMake 和 Python，但将来可能会添加支持其他（如 autotools，普通 Makefile）的支持。每个构建系统由 ament 工具单独应用本机步骤。

#### Build type: ament_cmake

The CMake package `ament_cmake` provides several convenience functions to make it easier to write CMake-based packages:

> CMake 包 `ament_cmake` 提供了几个便利的功能，使得编写基于 CMake 的包变得更容易：

- It generates a CMake config file for the package. That allows passing information (e.g. about include directories and libraries) to downstream packages. Additionally it makes it easy to pass along information from recursive dependencies (and e.g. takes care about the ordering of include directories).
- It provides an easy interface to register tests and ensure that JUnit-compatible result files are generated for those. Currently it supports a few different testing frameworks like `nosetests`, `gtest`, and `gmock`.
- It allows a package to generate environment hooks to extend the environment e.g. by extending the `PATH`.
- It provides a CMake API to read and write [ament resource index](https://github.com/ament/ament_cmake/blob/master/ament_cmake_core/doc/resource_index.md) entries. The index is built at build time and provides efficient access to information like the available packages, messages, etc.
- It provides an uninstall target for convenience.

Most of these features are implemented in separate packages. The CMake code uses an extension point system to foster modularity of the code. This enables others to add further features without requiring to alter existing functionality in the core of ament_cmake.

> 大部分这些特性都在单独的包中实现。CMake 代码使用扩展点系统来促进 ament_cmake 的核心代码的模块化。这使得其他人可以在不需要改变现有功能的情况下添加更多功能。

#### Build type: cmake

The CMake build type uses plain CMake and gives all the flexibility as well as responsibility to the developer.

> CMake 的构建类型使用纯 CMake，并且给予开发者所有的灵活性和责任。

#### Build type: ament_python

The Python build type allows packages to use setuptools with only a `setup.py` file. It uses the standard Python work flow to build Python packages.

> Python 构建类型允许包使用只有 `setup.py` 文件的 setuptools。它使用标准的 Python 工作流来构建 Python 包。

The ament tool will copy the package manifest into the install location. It will also extend the environment variables `PYTHONPATH` and `PATH` to include the Python modules and executables provided by the package.

> 工具将会把包清单复制到安装位置。它也会扩展环境变量 `PYTHONPATH` 和 `PATH`，以包含包提供的 Python 模块和可执行文件。

### Environment creation

Commonly the install space should follow the [Filesystem Hierarchy Standard (FHS)](http://www.pathname.com/fhs/). But that is just a recommendation and not enforced by ament.

> 通常，安装空间应遵循文件系统层次标准（FHS）。但这只是一个建议，而不是由 ament 强制执行。

Depending on where the packages are being installed it might be necessary to setup the environment in order to find all resources. E.g. the location of installed executables should be on the `PATH`, installed Python code should be on the `PYTHONPATH` etc.

> 根據安裝包的位置，可能需要設置環境以找到所有資源。例如，已安裝的可執行文件應該位於 `PATH` 中，已安裝的 Python 代碼應該位於 `PYTHONPATH` 中等。

Therefore each package can provide a shell script to setup the environment to match its needs. These package specific scripts are provided in the folder `<prefix>/share/<pkg-name>`. The `local_setup.*` files will update environment variables as specified by the package. Even when the package is built without the ament tool these setup files are being generated.

> 因此，每个包都可以提供一个 shell 脚本来设置环境以满足其需求。这些特定于包的脚本位于文件夹 `<prefix>/share/<pkg-name>` 中。`local_setup.*` 文件将按照包指定的方式更新环境变量。即使在没有 ament 工具的情况下构建包，这些设置文件也会生成。

The different shell scripts in the root of the install space are generated by the ament tool. The `local_setup.*` files only iterate over all packages in the install space (by reading the list of `packages` in the ament index) and source their package specific setup files. The `setup.*` files also consider workspaces outside of this install space (by reading the list of `parent_prefixp_path` in the ament index) and source them before the `local_setup.*` files.

> 安装空间根目录中的不同 shell 脚本是由 ament 工具生成的。 `local_setup.*` 文件仅迭代安装空间中的所有软件包（通过阅读 ament 索引中的 `packages` 列表）并获取其特定设置文件。 `setup.*` 文件还考虑了该安装空间之外的工作区（通过阅读 ament 索引中的 `parent_prefixp_path` 列表），并在 `local_setup.*` 文件之前源它们。

### Optional symlinked install

It is very important to maximize the efficiency of the development cycle of changing code, building, and installing it and then run it to confirm the changes. Commonly the installation steps involve copying some resources from the source space to their final destination in the install location. ament provides an option to use symbolic links instead (if the platform supports that). This enables the developer to change the resources in the source space and skipping the installation step in many situations.

> 在更改代码、构建和安装并运行它来确认更改的开发周期中，最大化效率非常重要。通常，安装步骤涉及从源空间复制一些资源到安装位置的最终目的地。Ament 提供了一个使用符号链接的选项（如果平台支持）。这使得开发人员能够更改源空间中的资源，并在许多情况下跳过安装步骤。

For CMake packages this is achieved by optionally overriding the CMake `install()` function. For Python packages the [development mode](http://setuptools.readthedocs.io/en/latest/setuptools.html#development-mode) is used to install the package. The symlinked install is an optional feature and must be enabled explicitly by the developer using the command line option `--symlink-install`.

> 对于 CMake 软件包，可以通过可选地覆盖 CMake 的 `install（）` 函数来实现此目的。对于 Python 软件包，使用[开发模式](http://setuptools.readthedocs.io/en/latest/setuptools.html#development-mode)来安装软件包。符号链接安装是一个可选功能，必须由开发人员显式使用命令行选项 `--symlink-install` 来启用。

### ament linters

ament provides a set of linters to check that source code complies with the ROS 2 style guidelines. The usage of these linters is optional but it is very easy to integrate them as part of the automated tests of a package.

> ament 提供一组 linters 来检查源代码是否符合 ROS 2 风格指南。使用这些 linters 是可选的，但是很容易将它们集成到包的自动测试中。

Linters can also check non-style related criteria. E.g. [cppcheck](http://cppcheck.sourceforge.net/) is used to statically analyze C / C++ code and check for semantic bugs.

> 检查程序员也可以检查非样式相关的标准。例如，[cppcheck](http://cppcheck.sourceforge.net/) 用于静态分析 C / C++ 代码并检查语义错误。

### ament_auto

ament_auto is similar to catkin_simple. It aims to simplify writing CMake code for an ament package. It avoids repetition of dependencies by extracting them from the manifest. Additionally it automates several steps by relying on conventions, e.g.:

> ament_auto 和 catkin_simple 相似。它旨在简化为 ament 软件包编写 CMake 代码。它通过从清单中提取依赖项来避免重复。此外，它通过依赖约定来自动化几个步骤，例如：

- all information from dependencies are used for compiling and linking of all target
- if a package has an `include` folder it adds that folder to the include directories, installs all headers from this folder, and exports the include folder to downstream packages
- if the package contains interface definitions in the `msg` and / or `srv` subfolder they will be processed automatically
- all libraries and executables are being installed to default locations

For an example see [below](#how-is-ament-different-from-catkin).

> 例子参见下方（#how-is-ament-different-from-catkin）。

### Additional resources

The implementation of ament is spread across several repositories:

> 实施的实施跨越多个存储库:

- [ament_package](https://github.com/ament/ament_package) is a Python package providing an API to parse the files containing the meta information.
- [ament_cmake](https://github.com/ament/ament_cmake) contains a set of packages which provide CMake based functionality. The features are split across several packages in order to ensure that they are cleanly separated from each other.
- [ament_lint](https://github.com/ament/ament_lint) contains a set of packages which perform linting tasks. For a set of linters a ROS 2 specific configuration is provided which can be used via the command line as well as from within CMake (commonly as a CTest).
- [ament_tools](https://github.com/ament/ament_tools) is a Python package which provides command line tools to build, test, install, and uninstall packages.
- [ament_index](https://github.com/ament/ament_index) contains a set of packages which provide API to access the [ament resource index](https://github.com/ament/ament_cmake/blob/master/ament_cmake_core/doc/resource_index.md). Currently it only contains a Python implementation but other languages like C++ should follow.

## How is _ament_ different from _catkin_

catkin has been used for ROS 1 since ROS Groovy. It was developed as a replacement for [rosbuild](http://wiki.ros.org/rosbuild) which was used from the beginning of ROS. Therefore various features of catkin have been designed to be similar to rosbuild.

> catkin 自从 ROS Groovy 以来就一直用于 ROS 1。它是为 [rosbuild](http://wiki.ros.org/rosbuild) 开发的，从 ROS 开始就一直使用 rosbuild。因此，catkin 的各种功能都被设计成与 rosbuild 类似。

### Why not continue to use _catkin_

catkin has many advantages over rosbuild. Those cover out-of-source builds, automatic CMake config file generation, an installation target and many more.

> catkin 有很多优势比 rosbuild 更好。它们包括外部源代码构建，自动 CMake 配置文件生成，安装目标以及更多。

Over time, however, feedback about the shortcomings of catkin has been collected. Also additional tools have been developed (like catkin_simple and catkin_tools) to improve the user / developer experience.

> 随着时间的推移，关于 catkin 的缺点的反馈也被收集起来了。此外，还开发了一些附加工具（比如 catkin_simple 和 catkin_tools）来改善用户/开发人员的体验。

This feedback was used to develop the next iteration of catkin which was then called ament. A few of these cases are mentioned below and all addressed by ament:

> 这些反馈用于开发下一代 catkin，后来被称为 ament。下面提到了其中一些案例，并由 ament 解决：

#### CMake centric

catkin is based around CMake and even packages only containing Python code are being processed via CMake. Because of that a setup.py file in a catkin package can only utilize a small subset of features from setuptools. Common features like extension points are not supported which makes it more difficult to deploy a package on Windows.

> catkin 基于 CMake，即使只包含 Python 代码的包也会通过 CMake 进行处理。由于此，catkin 包中的 setup.py 文件只能使用 setuptools 的一小部分功能。常见的功能，如扩展点不受支持，这使得在 Windows 上部署包变得更加困难。

#### _Devel space_

catkin has the feature to provide a so called [devel space](http://wiki.ros.org/catkin/workspaces#Development_.28Devel.29_Space) after building a set of packages. That folder is providing a fully working ROS environment without the need to install the packages. Avoiding copying any files allows users to e.g. edit Python code and immediately try running the code.

> Catkin 有一个特性，可以在构建一组软件包之后提供所谓的 [devel space](http://wiki.ros.org/catkin/workspaces#Development_.28Devel.29_Space)。该文件夹提供了一个完整的 ROS 环境，而不需要安装软件包。避免复制任何文件，使用户可以编辑 Python 代码，并立即尝试运行代码。

While this is a very convenient feature and speeds up the development process it comes at a cost. The necessary logic in catkin increases its complexity significantly. Additionally the CMake code in every ROS package has to make sure to handle the _devel space_ correctly which puts an extra effort on every ROS developer.

> 虽然这是一个非常方便的功能，可以加快开发过程，但是也有代价。Catkin 中的必要逻辑会大大增加其复杂性。此外，每个 ROS 包中的 CMake 代码必须确保正确处理* devel 空间*，这给每个 ROS 开发人员带来了额外的工作。

ament provides the same advantage using the optional feature of _symlinked installs_ without the extra complexity for each ROS package.

> Ament 可以使用可选的*符号链接安装*功能提供同样的优势，而无需为每个 ROS 包增加额外的复杂性。

#### _CMAKE_PREFIX_PATH_

catkin relies directly on the `CMAKE_PREFIX_PATH` environment variable to store the prefixes of multiple workspaces. This has been considered not a good approach since it interferes with other values set in the variable and is a CMake specific build variable. Therefore ament uses a separate environment variable (`AMENT_PREFIX_PATH`) for that purpose which is used at runtime. At build time of CMake packages the CMake specific variable can be derived from the generic ament variable.

> catkin 直接依赖于 `CMAKE_PREFIX_PATH` 环境变量来存储多个工作空间的前缀。由于这个变量会干扰其他设置的值，并且是 CMake 特定的构建变量，因此这种方法被认为是不好的。因此，ament 使用单独的环境变量（`AMENT_PREFIX_PATH`）来实现这一目的，并在运行时使用。在 CMake 软件包的构建时，可以从通用的 ament 变量中获得 CMake 特定的变量。

#### catkin_simple

The ROS package [catkin_simple](https://github.com/catkin/catkin_simple) was the attempt to make the common cases of developing ROS packages easier. While it is able to reduce the complexity of the CMake code in some ROS packages it fails conceptually in other cases. Some of the limitations were due to core catkin design decisions like the order and position of calling certain CMake functions etc.

> 包 [catkin_simple](https://github.com/catkin/catkin_simple) 是为了使开发 ROS 包变得更简单而诞生的尝试。虽然它能够减少一些 ROS 包中 CMake 代码的复杂性，但在其他情况下却无法实现概念上的要求。一些局限性是由于核心 catkin 设计决定，比如调用某些 CMake 函数的顺序和位置等。

E.g. the function `catkin_package()` must be invoked _before_ any target in order to setup the appropriate location for the build targets in the devel space. But in order to automatically perform several tasks in the code generation step this functionality needs to happen _after_ all target have been defined.

> 例如，必须先调用 `catkin_package（）` 函数，以便在开发空间中为构建目标设置适当的位置。但为了自动执行代码生成步骤中的几项任务，这个功能需要在定义所有目标之后才能完成。

With the different design of ament it becomes possible to implement a package similar to catkin_simple which can actually work reliably in all the cases where catkin_simple fails.

> 通过不同的 ament 设计，可以实现一个类似 catkin_simple 的包，它可以在 catkin_simple 失败的所有情况下可靠地工作。

#### Building within a single CMake context

catkin allows users to build multiple packages within a single CMake context (using `catkin_make`). While this significantly speeds up the process it has several drawbacks. Since all packages within a workspace share the same CMake context all targets are within the same namespace and must therefore be unique across all packages. The same applies to global variables, functions, macros, tests, etc. Furthermore a package might need to declare target level dependencies to targets in other packages to avoid CMake targets being parallelized when they ought to be sequential (but only when being built in the same CMake context). This requires internal knowledge about the build structure of other packages and subverts the goal of decoupling packages. ament does not provide that feature due to these drawbacks.

> Catkin 允许用户在单个 CMake 上下文中构建多个包（使用 `catkin_make`）。虽然这大大加快了流程，但也有几个缺点。由于工作空间中的所有包都共享相同的 CMake 上下文，因此所有目标都处于相同的命名空间中，因此必须在所有包中唯一。同样适用于全局变量，函数，宏，测试等。此外，一个包可能需要声明目标级别依赖关系到其他包中的目标，以避免在同一 CMake 上下文中构建时将 CMake 目标并行化（但仅当在同一 CMake 上下文中构建时）。这需要了解其他包和子包的构建结构，从而阻碍了解耦包的目标。由于这些缺点，ament 不提供此功能。

### Additional improvements in ament over catkin

catkin is a single monolithic package providing various features and functionalities. E.g. it integrates support for `gtest` which makes it very difficult to also optionally support `gmock` since only one tool can be selected at a time.

> catkin 是一个单一的单体包，提供各种功能和功能。例如，它集成了对 `gtest` 的支持，这使得同时可选择的 `gmock` 也变得非常困难，因为一次只能选择一个工具。

ament on the other hand is designed to be very modular. Almost every feature is provided by a separate package and can be optionally used. The CMake part of ament also features an extension point system to extend it with further functionality (see above).

> ament 另一方面被设计为非常模块化的。几乎每个功能都由单独的包提供，可以根据需要使用。ament 的 CMake 部分还提供了一个扩展点系统，可以用来扩展其其他功能（见上文）。

### Why not evolve _catkin_ with the necessary features

One important goal of ROS 2 is to [_not_ disrupt ROS 1](http://design.ros2.org/articles/why_ros2.html) in any way. Any changes to catkin would potentially introduce regressions into ROS 1 which is undesired.

> ROS 2 的一个重要目标是以任何方式都不会破坏 ROS 1。任何对 catkin 的更改都可能会导致 ROS 1 出现回退，这是不希望看到的。

Additionally some necessary changes would require a different usage of catkin in every ROS package. Releasing such a change into ROS 1 in a future distribution would imply a significant effort for each developer which should also be avoided. Not to mention the effort to update the documentation and tutorials and making the users aware of the subtle changes.

> 此外，某些必要的更改需要在每个 ROS 包中使用不同的 catkin。将此类更改发布到未来的 ROS 1 版本中将意味着每个开发人员都需要付出巨大的努力，这也应该避免。更不用说更新文档和教程以及让用户意识到微妙变化的努力了。

### Why use a new name instead of continuing to call it _catkin_

For the use case of building a bridge between ROS 1 and ROS 2 it must be possible to access both code bases in a single project. With both build systems having the same name they would not be distinguishable on a CMake level (which of the two packages is found with `find_package(catkin)`?). Therefore the package names of both build system must be different.

> 为了建立 ROS 1 和 ROS 2 之间的桥梁，必须能够在一个项目中访问两个代码库。由于两个构建系统的名称相同，在 CMake 级别上无法区分（使用 `find_package（catkin）` 可以找到哪个包？）。因此，两个构建系统的包名必须不同。

The newly developed build system has similar CMake functions but they have partly different arguments and / or are being called in different locations. With both system called the same (or even very similar) a significant user confusion was expected. Naming the new build system `catkin2` was considered, but the similarity would still have led to user confusion. Therefore a different name has been selected to clarify the different context and behavior.

> 新开发的构建系统具有类似的 CMake 功能，但它们具有部分不同的参数，或者被调用在不同的位置。由于两个系统的名称相同（甚至非常相似），因此预计会出现显著的用户混淆。考虑到将新构建系统命名为 `catkin2`，但相似性仍然会导致用户混淆。因此，选择了一个不同的名称来澄清不同的上下文和行为。

### Why has _catkin_pkg_ been forked to _ament_package_

ament as well as ROS 2 in general is using Python 3 exclusively. `catkin_pkg` as well as many other ROS 1 Python packages cannot be side-by-side installed for Python 2 and Python 3 at the same time. So requiring python3-catkin-pkg for ROS 2 would collide with python-catkin-pkg for ROS 1 and would make both ROS versions not installable at the same time.

> ROS 2 和一般的 ament 都只使用 Python 3。catkin_pkg 以及其他许多 ROS 1 Python 软件包不能同时安装到 Python 2 和 Python 3 上。因此，为 ROS 2 要求安装 python3-catkin-pkg 会与为 ROS 1 要求安装 python-catkin-pkg 发生冲突，从而使两个 ROS 版本无法同时安装。

ament_package only implements the specification [REP 140](http://www.ros.org/reps/rep-0140.html). It does not support _package.xml_ files with an older format.

> ament_package 只实现了 [REP 140](http://www.ros.org/reps/rep-0140.html) 的规范。它不支持具有较旧格式的 *package.xml* 文件。

Additionally it does provide the templates for all the environment hooks so that each package can generate its own environment and does not rely on an external tool to setup the environment. Beside that the package provides none of the additional functionalities of catkin_pkg (no crawling, no topological ordering, etc.).

> 此外，它还提供了所有环境钩子的模板，以便每个包可以生成自己的环境，而不依赖外部工具来设置环境。除此之外，该软件包不提供 catkin_pkg 的任何其他功能（无爬行，无拓扑排序等）。

### Example CMakeLists.txt files

The following CMake files are examples showing the similarities and differences between catkin and ament:

> 以下 CMake 文件是示例，展示了 catkin 和 ament 之间的相似性和差异：

- [CMakeLists.txt](https://gist.github.com/dirk-thomas/596a53bbb922c4d6988a) using catkin
- [CMakeLists.txt](https://gist.github.com/dirk-thomas/83ae6bfbfc94e06cd519) using ament
- [CMakeLists.txt](https://gist.github.com/dirk-thomas/a76f952d05e7b21b0128) using ament_auto
