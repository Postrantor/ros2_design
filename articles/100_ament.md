---
tip: translate by openai@2023-05-28 11:18:32
...
---
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

When this article was originally written `ament_tools` was the ROS 2 specific build tool.

> 当这篇文章最初被写下时，`ament_tools`是ROS 2特定的构建工具。

The reason was that it needed to be side-by-side installable with existing ROS 1 packages which was a problem due to different targeted Python versions.

> 原因是它需要与现有的ROS 1软件包并排安装，由于针对不同的Python版本，这是一个问题。

In the meantime the problem due to different Python versions has been addressed in shared dependencies like `catkin_pkg`.

> 在此期间，由于不同的Python版本而引起的问题已经在像catkin_pkg这样的共享依赖项中得到解决。

As of ROS 2 Bouncy `ament_tools` has been superseded by `colcon` as described in the [universal build tool](http://design.ros2.org/articles/build_tool.html) article.

> 随着ROS 2 Bouncy，`ament_tools`已被`colcon`取代，如[通用构建工具](http://design.ros2.org/articles/build_tool.html)文章所述。
    </div>

## Preface


ROS is developed with a federated model.

> ROS是用联邦模型开发的。

While this comes with a lot of advantages it makes the process of building several interdependent packages more complex.

> 这带来了许多好处，但也使构建多个相互依赖的包变得更加复杂。


In order to build a set of interdependent packages the user must build them in the correct topological order.

> 要构建一组相互依赖的软件包，用户必须按正确的拓扑顺序来构建它们。

The user must look at the documentation of each package to determine the dependencies, being careful to build those packages before hand, and to determine the instructions for building that package.

> 用户必须查看每个软件包的文档，以确定依赖项，小心翼翼地先构建这些软件包，并确定构建该软件包的说明。

Finally the user must build and install each package and do any environment configuration before proceeding to build the next package.

> 最后，用户必须构建和安装每个包，然后在构建下一个包之前完成任何环境配置。


ROS mitigates that complexity with package and build system conventions as well as tools to automate the build process.

> ROS 通过包和构建系统约定以及自动构建过程的工具来减轻复杂性。

Wherever possible ROS uses standard tools, e.g. CMake and Python setuptools.

> ROS 尽可能使用标准工具，例如CMake和Python setuptools。

Where these tools lack built-in support for what ROS needs to do, the additional functionality is provided by ROS.

> 当这些工具缺乏ROS所需要做的内置支持时，额外的功能将由ROS提供。

### Origin of ament


ament is an evolution of [catkin](http://wiki.ros.org/catkin).

> ament是catkin的进化版本。

The word *ament* is actually a synonym of *catkin*.

> "词语ament实际上是catkin的同义词。"

For more information about the differences please read [below](#how-is-ament-different-from-catkin).

> 欲了解更多关于差异的信息，请阅读[下面](#how-is-ament-different-from-catkin)。

## ament


ament is a meta build system to improve building applications which are split into separate packages.

> ament是一个元构建系统，用于改善将应用程序拆分成单独的包进行构建的过程。

It consists of two major parts:

> 它由两个主要部分组成：


- a *build system* (e.g. CMake, Python setuptools) to configure, build, and install a single package

> 一个构建系统（例如CMake、Python setuptools）来配置、构建和安装单个软件包

- a *tool* to invoke the build of individual packages in their topological order

> 一个工具，用来按照拓扑顺序调用单个包的构建。


The tool relies on meta information about the packages to determine their dependencies and their build type.

> 该工具依靠关于包的元信息来确定它们的依赖关系和构建类型。

This meta information is defined in a manifest file called `package.xml` which is specified in [REP 140](http://www.ros.org/reps/rep-0140.html).

> 这些元信息定义在一个叫做`package.xml`的清单文件中，详见[REP 140](http://www.ros.org/reps/rep-0140.html)。


Each package is built separately with its own build system.

> 每个包都使用自己的构建系统单独构建。

In order to make the output of one package available to other packages each package can extend the environment in a way that downstream packages can find and use its artifacts and resources.

> 为了使一个包的输出可供其他包使用，每个包都可以以一种方式扩展环境，以便下游包可以找到并使用其工件和资源。

If the resulting artifacts are installed into `/usr`, for example, it might not be necessary to alter the environment at all since these folders are commonly being searched by various tools.

> 如果将生成的工件安装到`/usr`，例如，可能根本不需要更改环境，因为这些文件夹通常被各种工具搜索。

### The ament command line tool


[ament_tools](https://github.com/ament/ament_tools) is a Python package which provides the command line tool `ament` to build, test, install, and uninstall packages.

> [ament_tools](https://github.com/ament/ament_tools) 是一个 Python 包，提供命令行工具 `ament` 来构建、测试、安装和卸载软件包。

It is similar to [catkin_tools](https://github.com/catkin/catkin_tools) and builds each package in a workspace in topological order.

> 它类似于[catkin_tools](https://github.com/catkin/catkin_tools)，并以拓扑顺序在工作区中构建每个包。

Support for different build systems is integrated through extension points which allows support for other build types to be added without changing the ament tool itself.

> 支持不同的构建系统通过扩展点集成，这允许添加其他构建类型的支持而无需更改ament工具本身。


While it currently does not build packages in parallel that feature will be added in the future to speed up the build process.

> 本身不支持并行构建包，但将来会添加这个功能来加速构建过程。

The goal is to reuse common functionality from catkin_tools by making it available through a third package which can be used by both tools.

> 目标是通过第三方包来重用catkin_tools中的常用功能，以便两个工具都可以使用它。

### Integrate arbitrary build systems


Each package can utilize a different build system to perform the steps of configuring, building, and installing.

> 每个包都可以使用不同的构建系统来执行配置、构建和安装的步骤。

The build type is defined in each package manifest using the [build_type](http://www.ros.org/reps/rep-0140.html#build-type) tag in the export section.

> 每个包清单中都使用[build_type](http://www.ros.org/reps/rep-0140.html#build-type)标签在导出部分定义构建类型。


Currently supported are CMake and Python but support for others (e.g. autotools, plain Makefiles) will likely be added in the future.

> 目前支持的是CMake和Python，但将来可能会添加其他（如autotools、普通Makefiles）的支持。

For each build system the native steps are being applied separately by the ament tool.

> 每个构建系统的本机步骤由ament工具单独应用。

#### Build type: ament_cmake


The CMake package `ament_cmake` provides several convenience functions to make it easier to write CMake-based packages:

> CMake 包`ament_cmake`提供了几个便利的函数，使编写基于CMake的软件包变得更容易。


- It generates a CMake config file for the package.

> 它为该包生成一个CMake配置文件。

  That allows passing information (e.g. about include directories and libraries) to downstream packages.

> 这允许传递信息（例如关于包含目录和库）到下游包。

  Additionally it makes it easy to pass along information from recursive dependencies (and e.g. takes care about the ordering of include directories).

> 此外，它还可以轻松地传递来自递归依赖项（以及例如处理包含目录的顺序）的信息。


- It provides an easy interface to register tests and ensure that JUnit-compatible result files are generated for those.

> 它提供一个简单的界面来注册测试，并确保为这些测试生成兼容JUnit的结果文件。

  Currently it supports a few different testing frameworks like `nosetests`, `gtest`, and `gmock`.

> 目前它支持几种不同的测试框架，如`nosetests`、`gtest`和`gmock`。


- It allows a package to generate environment hooks to extend the environment e.g. by extending the `PATH`.

> 它允许一个包来生成环境钩子来扩展环境，例如通过扩展`PATH`。


- It provides a CMake API to read and write [ament resource index](https://github.com/ament/ament_cmake/blob/master/ament_cmake_core/doc/resource_index.md) entries.

> 它提供了一个 CMake API 来读取和写入[ament 资源索引](https://github.com/ament/ament_cmake/blob/master/ament_cmake_core/doc/resource_index.md)条目。

  The index is built at build time and provides efficient access to information like the available packages, messages, etc.

> 索引在构建时创建，可以高效地访问可用包、消息等信息。


- It provides an uninstall target for convenience.

> 它提供了一个方便的卸载目标。


Most of these features are implemented in separate packages.

> 大部分这些功能都在单独的软件包中实现。

The CMake code uses an extension point system to foster modularity of the code.

> CMake 代码使用扩展点系统来促进代码的模块化。

This enables others to add further features without requiring to alter existing functionality in the core of ament_cmake.

> 这使得其他人可以在ament_cmake的核心中添加更多功能，而无需更改现有功能。

#### Build type: cmake


The CMake build type uses plain CMake and gives all the flexibility as well as responsibility to the developer.

> CMake构建类型使用普通的CMake，并且给予开发者所有的灵活性和责任。

#### Build type: ament_python


The Python build type allows packages to use setuptools with only a `setup.py` file.

> Python构建类型允许包只使用`setup.py`文件就可以使用setuptools。

It uses the standard Python work flow to build Python packages.

> 它使用标准的Python工作流程来构建Python包。


The ament tool will copy the package manifest into the install location.

> 工具将把包清单复制到安装位置。

It will also extend the environment variables `PYTHONPATH` and `PATH` to include the Python modules and executables provided by the package.

> 它还会扩展环境变量`PYTHONPATH`和`PATH`，以包括该包提供的Python模块和可执行文件。

### Environment creation


Commonly the install space should follow the [Filesystem Hierarchy Standard (FHS)](http://www.pathname.com/fhs/).

> 一般来说，安装空间应遵循[文件系统层次标准（FHS）](http://www.pathname.com/fhs/)。

But that is just a recommendation and not enforced by ament.

> 但这只是一个建议，没有被法令强制执行。


Depending on where the packages are being installed it might be necessary to setup the environment in order to find all resources.

> 根据包安装的位置，可能需要设置环境以便找到所有资源。

E.g. the location of installed executables should be on the `PATH`, installed Python code should be on the `PYTHONPATH` etc.

> 例如，安装的可执行文件的位置应该在`PATH`上，安装的Python代码应该在`PYTHONPATH`上等。


Therefore each package can provide a shell script to setup the environment to match its needs.

> 因此，每个包都可以提供一个shell脚本来设置环境以满足其需求。

These package specific scripts are provided in the folder `<prefix>/share/<pkg-name>`.

> 这些特定于包的脚本放在文件夹`<prefix>/share/<pkg-name>`中提供。

The `local_setup.*` files will update environment variables as specified by the package.

> 本地_setup.*文件将根据包所指定的环境变量进行更新。

Even when the package is built without the ament tool these setup files are being generated.

> 即使在没有ament工具的情况下构建包，这些设置文件也会被生成。


The different shell scripts in the root of the install space are generated by the ament tool.

> 根安装空间中的不同shell脚本是由ament工具生成的。

The `local_setup.*` files only iterate over all packages in the install space (by reading the list of `packages` in the ament index) and source their package specific setup files.

> `local_setup.*`文件只会遍历安装空间中的所有软件包（通过阅读ament索引中的`软件包`列表）并调用它们的特定设置文件。

The `setup.*` files also consider workspaces outside of this install space (by reading the list of `parent_prefixp_path` in the ament index) and source them before the `local_setup.*` files.

> setup.*文件也可以考虑安装空间之外的工作空间（通过阅读ament索引中的parent_prefix_path列表），并在local_setup.*文件之前对它们进行源代码处理。

### Optional symlinked install


It is very important to maximize the efficiency of the development cycle of changing code, building, and installing it and then run it to confirm the changes.

> 很重要的是最大化改变代码、构建和安装的开发周期的效率，然后运行它来确认这些改变。

Commonly the installation steps involve copying some resources from the source space to their final destination in the install location.

> 一般来说，安装步骤包括从源空间复制一些资源到安装位置的最终目的地。

ament provides an option to use symbolic links instead (if the platform supports that).

> ament提供了一个使用符号链接的选项（如果平台支持的话）。

This enables the developer to change the resources in the source space and skipping the installation step in many situations.

> 这使开发人员能够更改源空间中的资源，在许多情况下跳过安装步骤。


For CMake packages this is achieved by optionally overriding the CMake `install()` function.

> 对于CMake软件包，可以通过可选地覆盖CMake的`install()`函数来实现。

For Python packages the [development mode](http://setuptools.readthedocs.io/en/latest/setuptools.html#development-mode) is used to install the package.

> 对于Python包，使用[开发模式](http://setuptools.readthedocs.io/en/latest/setuptools.html#development-mode)来安装包。

The symlinked install is an optional feature and must be enabled explicitly by the developer using the command line option `--symlink-install`.

> 可选的符号链接安装必须由开发人员使用命令行选项“--symlink-install”来显式启用。

### ament linters


ament provides a set of linters to check that source code complies with the ROS 2 style guidelines.

> ament提供一组linters来检查源代码是否符合ROS 2风格指南。

The usage of these linters is optional but it is very easy to integrate them as part of the automated tests of a package.

> 这些语法检查器的使用是可选的，但是很容易将它们集成到包的自动测试中。


Linters can also check non-style related criteria.

> Linters也可以检查与样式无关的标准。

E.g. [cppcheck](http://cppcheck.sourceforge.net/) is used to statically analyze C / C++ code and check for semantic bugs.

> 例如，[cppcheck](http://cppcheck.sourceforge.net/) 用于静态分析C/C++代码并检查语义错误。

### ament_auto


ament_auto is similar to catkin_simple.

> ament_auto类似于catkin_simple。

It aims to simplify writing CMake code for an ament package.

> 它旨在简化为ament包编写CMake代码的过程。

It avoids repetition of dependencies by extracting them from the manifest.

> 它通过从清单中提取依赖项来避免依赖项的重复。

Additionally it automates several steps by relying on conventions, e.g.:

> 此外，它通过依赖惯例来自动化多个步骤，例如：


- all information from dependencies are used for compiling and linking of all target

> 所有依赖的信息都用于编译和链接所有目标。

- if a package has an `include` folder it adds that folder to the include directories, installs all headers from this folder, and exports the include folder to downstream packages

> 如果一个包有一个`include`文件夹，它会将该文件夹添加到包含目录中，安装该文件夹中的所有头文件，并将include文件夹导出到下游包中。

- if the package contains interface definitions in the `msg` and / or `srv` subfolder they will be processed automatically

> 如果包含在`msg`和/或`srv`子文件夹中的接口定义，它们将被自动处理。

- all libraries and executables are being installed to default locations

> 所有的库和可执行文件都会安装到默认位置。


For an example see [below](#how-is-ament-different-from-catkin).

> 例子参见下方（#how-is-ament-different-from-catkin）。

### Additional resources


The implementation of ament is spread across several repositories:

> 实施的实施分布在几个存储库中：

- [ament_package](https://github.com/ament/ament_package) is a Python package providing an API to parse the files containing the meta information.

- [ament_cmake](https://github.com/ament/ament_cmake) contains a set of packages which provide CMake based functionality.

  The features are split across several packages in order to ensure that they are cleanly separated from each other.

> 这些功能被分散在几个包中，以确保它们彼此之间保持清晰的分离。

- [ament_lint](https://github.com/ament/ament_lint) contains a set of packages which perform linting tasks.

  For a set of linters a ROS 2 specific configuration is provided which can be used via the command line as well as from within CMake (commonly as a CTest).

> 为了一组linters，提供了一个ROS 2特定的配置，可以通过命令行以及CMake（通常作为CTest）使用。

- [ament_tools](https://github.com/ament/ament_tools) is a Python package which provides command line tools to build, test, install, and uninstall packages.

- [ament_index](https://github.com/ament/ament_index) contains a set of packages which provide API to access the [ament resource index](https://github.com/ament/ament_cmake/blob/master/ament_cmake_core/doc/resource_index.md).

  Currently it only contains a Python implementation but other languages like C++ should follow.

> 目前只包含了Python实现，但其他语言如C++也将会跟进。

## How is *ament* different from *catkin*


catkin has been used for ROS 1 since ROS Groovy.

> catkin自ROS Groovy以来一直用于ROS 1。

It was developed as a replacement for [rosbuild](http://wiki.ros.org/rosbuild) which was used from the beginning of ROS.

> 它是为了取代从ROS开始使用的[rosbuild](http://wiki.ros.org/rosbuild)而开发的。

Therefore various features of catkin have been designed to be similar to rosbuild.

> 因此，catkin的许多特性已被设计成与rosbuild相似。

### Why not continue to use *catkin*


catkin has many advantages over rosbuild.

> catkin有很多优势比rosbuild。

Those cover out-of-source builds, automatic CMake config file generation, an installation target and many more.

> 这些覆盖了跨源构建，自动CMake配置文件生成，安装目标以及更多功能。


Over time, however, feedback about the shortcomings of catkin has been collected.

> 随着时间的推移，人们收集了关于catkin的缺点的反馈。

Also additional tools have been developed (like catkin_simple and catkin_tools) to improve the user / developer experience.

> 也开发了额外的工具（比如catkin_simple和catkin_tools）来改善用户/开发者体验。


This feedback was used to develop the next iteration of catkin which was then called ament.

> 这些反馈用于开发下一个Catkin版本，然后称为Ament。

A few of these cases are mentioned below and all addressed by ament:

> 以下提到了其中一些案例，所有案例都由ament解决。

#### CMake centric


catkin is based around CMake and even packages only containing Python code are being processed via CMake.

> catkin是基于CMake的，即使只包含Python代码的包也会通过CMake处理。

Because of that a setup.py file in a catkin package can only utilize a small subset of features from setuptools.

> 由于这个原因，catkin包中的setup.py文件只能使用setuptools的一小部分功能。

Common features like extension points are not supported which makes it more difficult to deploy a package on Windows.

> 不支持像扩展点这样的通用功能，这使得在Windows上部署软件包更加困难。

#### *Devel space*


catkin has the feature to provide a so called [devel space](http://wiki.ros.org/catkin/workspaces#Development_.28Devel.29_Space) after building a set of packages.

> catkin有一个特性，在构建一组软件包后提供所谓的[devel空间](http://wiki.ros.org/catkin/workspaces#Development_.28Devel.29_Space)。

That folder is providing a fully working ROS environment without the need to install the packages.

> 那个文件夹提供一个完整的ROS环境，无需安装软件包。

Avoiding copying any files allows users to e.g. edit Python code and immediately try running the code.

> 避免复制任何文件可以让用户例如编辑Python代码，并立即尝试运行代码。


While this is a very convenient feature and speeds up the development process it comes at a cost.

> 这是一个非常方便的功能，可以加快开发过程，但是代价是不小的。

The necessary logic in catkin increases its complexity significantly.

> 在catkin中所需的逻辑大大增加了其复杂性。

Additionally the CMake code in every ROS package has to make sure to handle the *devel space* correctly which puts an extra effort on every ROS developer.

> 此外，每个ROS包中的CMake代码必须确保正确处理*开发空间*，这对每个ROS开发人员都是一种额外的努力。


ament provides the same advantage using the optional feature of *symlinked installs* without the extra complexity for each ROS package.

> ament提供了相同的优势，使用*符号链接安装*的可选功能，而不需要为每个ROS包增加额外的复杂性。

#### *CMAKE_PREFIX_PATH*


catkin relies directly on the `CMAKE_PREFIX_PATH` environment variable to store the prefixes of multiple workspaces.

> catkin直接依赖于`CMAKE_PREFIX_PATH`环境变量来存储多个工作区的前缀。

This has been considered not a good approach since it interferes with other values set in the variable and is a CMake specific build variable.

> 这被认为不是一个好的方法，因为它干扰了变量中设置的其他值，而且是CMake特定的构建变量。

Therefore ament uses a separate environment variable (`AMENT_PREFIX_PATH`) for that purpose which is used at runtime.

> 因此，ament使用一个单独的环境变量（`AMENT_PREFIX_PATH`）来实现这一目的，它在运行时被使用。

At build time of CMake packages the CMake specific variable can be derived from the generic ament variable.

> 在CMake包的构建时间，可以从通用的ament变量中导出CMake特定的变量。

#### catkin_simple


The ROS package [catkin_simple](https://github.com/catkin/catkin_simple) was the attempt to make the common cases of developing ROS packages easier.

> 包[catkin_simple](https://github.com/catkin/catkin_simple)是试图使开发ROS包的常见情况变得更容易的尝试。

While it is able to reduce the complexity of the CMake code in some ROS packages it fails conceptually in other cases.

> 在某些ROS包中，它能够降低CMake代码的复杂性，但在其他情况下，它在概念上却失败了。

Some of the limitations were due to core catkin design decisions like the order and position of calling certain CMake functions etc.

> 一些限制是由于核心catkin设计决策，比如调用某些CMake函数的顺序和位置等。


E.g. the function `catkin_package()` must be invoked *before* any target in order to setup the appropriate location for the build targets in the devel space.

> 例如，必须在任何目标之前调用函数`catkin_package()`，以便在开发空间中为构建目标设置适当的位置。

But in order to automatically perform several tasks in the code generation step this functionality needs to happen *after* all target have been defined.

> 为了在代码生成步骤中自动执行多个任务，这项功能需要在定义所有目标之后发生。


With the different design of ament it becomes possible to implement a package similar to catkin_simple which can actually work reliably in all the cases where catkin_simple fails.

> 随着安装设计的不同，可以实现一个与catkin_simple类似的包，它可以在catkin_simple失败的所有情况下可靠地工作。

#### Building within a single CMake context


catkin allows users to build multiple packages within a single CMake context (using `catkin_make`).

> catkin允许用户在单个CMake上下文中构建多个包（使用`catkin_make`）。

While this significantly speeds up the process it has several drawbacks.

> 这大大加快了进程，但也有一些缺点。

Since all packages within a workspace share the same CMake context all targets are within the same namespace and must therefore be unique across all packages.

> 所有工作区内的所有软件包都共享相同的CMake上下文，因此所有目标都在同一命名空间内，因此必须在所有软件包中唯一。

The same applies to global variables, functions, macros, tests, etc.

> 同样适用于全局变量、函数、宏、测试等等。

Furthermore a package might need to declare target level dependencies to targets in other packages to avoid CMake targets being parallelized when they ought to be sequential (but only when being built in the same CMake context).

> 此外，一个包可能需要声明目标级别的依赖关系，以避免在相同的CMake上下文中构建时，CMake目标被并行化而不是顺序化。

This requires internal knowledge about the build structure of other packages and subverts the goal of decoupling packages.

> 这需要对其他包和子包的构建结构有内部了解，这破坏了解耦包的目标。

ament does not provide that feature due to these drawbacks.

> 由于这些缺点，Ament不提供此功能。

### Additional improvements in ament over catkin


catkin is a single monolithic package providing various features and functionalities.

> catkin是一个单一的单体包，提供各种功能和功能。

E.g. it integrates support for `gtest` which makes it very difficult to also optionally support `gmock` since only one tool can be selected at a time.

> 例如，它集成了对`gtest`的支持，这使得在同一时间只能选择一个工具时，可选择性地支持`gmock`也变得非常困难。


ament on the other hand is designed to be very modular.

> ament 通常被设计成非常模块化的。

Almost every feature is provided by a separate package and can be optionally used.

> 几乎每个功能都由单独的包提供，可以根据需要使用。

The CMake part of ament also features an extension point system to extend it with further functionality (see above).

> ament的CMake部分还具有扩展点系统，可用于扩展其具有更多功能（见上文）。

### Why not evolve *catkin* with the necessary features


One important goal of ROS 2 is to [*not* disrupt ROS 1](http://design.ros2.org/articles/why_ros2.html) in any way.

> 一个重要的ROS 2的目标是不要以任何方式打扰ROS 1。

Any changes to catkin would potentially introduce regressions into ROS 1 which is undesired.

> 任何对catkin的更改可能会导致ROS 1出现回归，这是不希望的。


Additionally some necessary changes would require a different usage of catkin in every ROS package.

> 此外，一些必要的更改需要在每个ROS包中使用不同的catkin。

Releasing such a change into ROS 1 in a future distribution would imply a significant effort for each developer which should also be avoided.

> 发布这样一个变化到ROS 1在未来的发布中将意味着每个开发人员要付出巨大的努力，这也应该避免。

Not to mention the effort to update the documentation and tutorials and making the users aware of the subtle changes.

> 不用说，还要花费努力更新文档和教程，让用户意识到细微的变化。

### Why use a new name instead of continuing to call it *catkin*


For the use case of building a bridge between ROS 1 and ROS 2 it must be possible to access both code bases in a single project.

> 为了建立ROS 1和ROS 2之间的桥梁，必须能够在一个项目中访问两个代码库。

With both build systems having the same name they would not be distinguishable on a CMake level (which of the two packages is found with `find_package(catkin)`?).

> 两个构建系统都有相同的名字，在CMake层面上无法区分（使用`find_package(catkin)`找到哪个包？）？

Therefore the package names of both build system must be different.

> 因此，两个构建系统的包名必须不同。


The newly developed build system has similar CMake functions but they have partly different arguments and / or are being called in different locations.

> 新开发的构建系统具有类似的CMake功能，但它们的参数有所不同，或者在不同的位置被调用。

With both system called the same (or even very similar) a significant user confusion was expected.

> 有两个系统叫相同的（甚至非常相似），预计会有重大的用户混淆。

Naming the new build system `catkin2` was considered, but the similarity would still have led to user confusion.

> 考虑过将新构建系统命名为`catkin2`，但是由于名称相似，仍然会导致用户混淆。

Therefore a different name has been selected to clarify the different context and behavior.

> 因此，已选择一个不同的名称来澄清不同的上下文和行为。

### Why has *catkin_pkg* been forked to *ament_package*


ament as well as ROS 2 in general is using Python 3 exclusively.

> ROS 2和整体上的Ament都只使用Python 3。
`catkin_pkg` as well as many other ROS 1 Python packages cannot be side-by-side installed for Python 2 and Python 3 at the same time.

So requiring python3-catkin-pkg for ROS 2 would collide with python-catkin-pkg for ROS 1 and would make both ROS versions not installable at the same time.

> 如果要求ROS 2使用python3-catkin-pkg，这将与ROS 1的python-catkin-pkg发生冲突，使得两个ROS版本不能同时安装。


ament_package only implements the specification [REP 140](http://www.ros.org/reps/rep-0140.html).

> ament_package只实现了[REP 140](http://www.ros.org/reps/rep-0140.html)的规范。

It does not support *package.xml* files with an older format.

> 它不支持旧格式的 *package.xml* 文件。


Additionally it does provide the templates for all the environment hooks so that each package can generate its own environment and does not rely on an external tool to setup the environment.

> 此外，它还提供所有环境钩子的模板，以便每个包都可以生成自己的环境，而不依赖外部工具来设置环境。

Beside that the package provides none of the additional functionalities of catkin_pkg (no crawling, no topological ordering, etc.).

> 除此之外，该包不提供catkin_pkg的任何附加功能（无爬行，无拓扑排序等）。

### Example CMakeLists.txt files


The following CMake files are examples showing the similarities and differences between catkin and ament:

> 以下CMake文件是展示catkin和ament之间相似性和差异性的示例：

    - [CMakeLists.txt](https://gist.github.com/dirk-thomas/596a53bbb922c4d6988a) using catkin
    - [CMakeLists.txt](https://gist.github.com/dirk-thomas/83ae6bfbfc94e06cd519) using ament
    - [CMakeLists.txt](https://gist.github.com/dirk-thomas/a76f952d05e7b21b0128) using ament_auto
