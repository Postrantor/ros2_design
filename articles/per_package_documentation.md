---
tip: translate by openai@2023-06-13 23:03:54
abstract: This article describes the requirements and design of ROS 2's per-package documentation system.
> 本文描述了ROS 2每个软件包文档系统的要求和设计。
author: Marya Belanger
date_written: 2020-10
last_modified: 2020-10
layout: default
permalink: articles/per_package_documentation.html
published: true
title: Per-Package Documentation
---
## Background

ROS 2 is lacking a process for the conglomeration of package documentation. Discoverability for package documentation (and documentation in general) is one of the most frequent user complaints.

> ROS 2 缺乏一个用于整合包文档的流程。查找包文档(以及一般文档)是最常见的用户投诉之一。

The goal of this design is to make ROS 2 documentation as accessible as possible for all ROS users. To achieve this we are seeking to standardize a documentation process for ROS 2 packages that will encourage maintainers to robustly document their packages, in turn making ROS 2 package documentation more available. The results will be presented in a consistent way alongside the general ROS 2 documentation, making the documentation as a whole discoverable and predictable.

> 本设计的目标是尽可能让所有 ROS 用户访问 ROS 2 文档。为此，我们正在寻求为 ROS 2 软件包标准化文档流程，以鼓励维护者充分文档化他们的软件包，从而使 ROS 2 软件包文档更加可用。结果将以一致的方式与一般的 ROS 2 文档一起呈现，使文档作为整体可检索和可预测。

In general, the vision for the system is:

> 一般来说，系统的愿景是：

- Package maintainers document their packages in their repositories following some recommended guidelines or templates

> 包维护者根据一些推荐的指南或模板，在他们的存储库中文档他们的包。

- Package documentation (API documentation, other package related content) from the repositories is built on the ROS 2 infrastructure and deployed to the ROS 2 documentation site in an automatic process maintainers can opt in to by adding a `doc` block to the `distribution.yaml`

> 包文档(API 文档和其他与包相关的内容)来自存储库，建立在 ROS 2 基础架构上，并通过添加 `doc` 块到 `distribution.yaml` 中自动部署到 ROS 2 文档站点，维护者可以选择参与其中。

- See [REP 141](https://www.ros.org/reps/rep-0141.html) for more context.
- Package documentation is indexed alongside ROS 2's generic content on the docs site

## Context

The per-package documentation plan is an extension of the general ROS 2 documentation project, which introduced some requirements that affect the context of the per-package documentation:

> 这个按程序文档计划是 ROS 2 文档项目的延伸，它引入了一些影响按程序文档上下文的要求：

- All documentation will be hosted on `docs.ros.org`
- All documentation will be versioned by ROS 2 distribution names
- The URL structure will be `docs.ros.org/<lang>/<distro>/...` for generic documentation and `docs.ros.org/<lang>/<distro>/p/<package_name>/...` for package documentation

> URL 结构将为 `docs.ros.org/<lang>/<distro>/...`，用于通用文档，以及 `docs.ros.org/<lang>/<distro>/p/<package_name>/...` 用于包文档。

- See the diagram below for more context on where documentation will fall under this URL structure: ![Content placement in the new URL structure - flowchart](./per_package_documentation/Version_package_flowchart.png)

> 請參閱下面的圖表，以了解新的 URL 結構中文檔的位置：！[新 URL 結構中內容位置的流程圖](./per_package_documentation/Version_package_flowchart.png)

## Requirements

The [Primary requirements](#1-primary-requirements) are those that must be in place for the system to be functional and achieve its purpose. The [Secondary requirements](#2-secondary-requirements) are also absolute requirements, however they are not necessary to roll out the first stage of implementation. They will be carried out following the initial roll out.

> 首要要求(#1-primary-requirements)是必须为系统提供功能并实现其目的所必需的。次要要求(#2-secondary-requirements)也是绝对必需的，但不必在初始实施阶段就完成。它们将在初始实施后完成。

### 1. Primary requirements

**1.1 All package documentation must be available with the rest of the ROS 2 documentation under a single domain**

> **1.1 所有软件包文档必须与 ROS 2 文档一起在单个域下提供**

Package documentation will be treated the same as the generic ROS 2 documentation. Its presence as part of the docs site should be made well-known from the site's overview as well as from any entry point into the docs site, and be intuitive from the organization and layout of the site.

> 包文档将与通用 ROS 2 文档一样得到重视。它的存在应该在文档网站的概览中得到宣传，并从文档网站的任何入口处宣传，并且从网站的组织和布局中是直观的。

**1.2 Package documentation must have an easily navigable and intuitive URL structure**

> **1.2 包文档必须具有易于导航和直观的 URL 结构**

Every package's "home page" should be reachable with minimal effort by the scheme `docs.ros.org/<lang>/<distro>/p/<package_name>/`

> 每个软件包的“主页”都可以通过 `docs.ros.org/<lang>/<distro>/p/<package_name>/` 这一方案很容易访问到。

**1.3 Package documentation must be maintainable in its repository without going through a third party**

> 必须在其存储库中维护包文档，而无需通过第三方。

Maintainers and contributors will only have to work on their package's documentation within its repository. The details of building and hosting will not be a concern of package maintainers.

> 维护者和贡献者只需要在他们软件包的存储库中处理文档即可，构建和托管的细节不是软件包维护者的关注点。

Despite being hosted alongside the generic documentation, working on a package's documentation will not require any work on the repositories of the generic documentation (currently `ros2/ros2_documentation`) or the site repository (currently `ros-infrastructure/rosindex`).

> 尽管与通用文档一起托管，但是在一个包的文档上工作不需要在通用文档的存储库(目前为 `ros2/ros2_documentation`)或站点存储库(目前为 `ros-infrastructure/rosindex`)上进行任何工作。

**1.4 The system must support C++ and Python API docs generation**

> **1.4 系统必须支持 C++ 和 Python API 文档生成**

The system will automatically extract API docs for these languages from the source code in package repositories and build the output on the docs site.

> 系统将自动从包存储库中的源代码中提取 API 文档，并在文档站点上构建输出。

**1.5 Package documentation must be written and formatted in rst (reStructuredText)**

> **1.5 包文档必须用 rst(reStructuredText)格式编写和格式化**

Rst is the file format currently utilized by the ROS 2 documentation. Package documentation should continue to utilize rst for consistency.

> Rst 是 ROS 2 文档目前使用的文件格式。为了保持一致性，软件包文档应继续使用 rst。

**1.6 The system must allow package documentation to be versioned per ROS 2 distribution**

> 系统必须允许每个 ROS 2 发行版本的软件包文档进行版本控制。

The docs site and buildfarm will allow documentation for the latest version of a package corresponding to each ROS 2 distribution the docs site supports versioning for. The package repository can still maintain its own docs for previous versions.

> 文档站点和构建农场将允许每个 ROS 2 发行版本对应的最新版本的文档。文档站点支持版本控制。软件包存储库仍可以为以前的版本维护自己的文档。

While multiple package versions per distribution will not initially be supported, the URL structure will support the addition of this feature in the future (mentioned under [Secondary requirements](#2-secondary-requirements) below).

> 在最初的版本中不支持每个发行版本的多个包版本，但 URL 结构将支持在未来添加此功能(参见下面的[次要要求] (#2-secondary-requirements))。

**1.7 The buildfarm must automatically build package documentation**

> **1.7 构建农场必须自动构建包文档**

Changes to the documentation in a package repository shouldn't require the maintainer to manually trigger a build.

> 对包存储库中的文档的更改不应需要维护者手动触发构建。

**1.8 The system must automatically generate listing content for a package so it's listed on docs.ros.org even if the package maintainer does not explicitly set up anything in the package repository**

> **1.8 系统必须自动为包生成列表内容，以便即使包维护者没有明确在包存储库中设置任何内容，也可以将其列在 docs.ros.org 上。**

By default some package info will be listed for every package added to a rosdistro.

> 默认情况下，每次向 rosdistro 添加包时，将列出一些包信息。

**1.9 The systems must support cross-referencing between packages**

> 系统必须支持跨包之间的交叉引用。

A stable linking process will be in place to cross-reference between package docs despite their originating from separate repositories.

> 一个稳定的链接过程将会被安排以跨越参考不同仓库中的包文档。

**1.10 The system must prevent the possibility of package file names (created by maintainers) colliding with auto-generated content (from the system's side)**

> **1.10 系统必须防止维护者创建的包文件名与系统自动生成的内容发生冲突的可能性。**

When writing package documentation, maintainers should not have to concern themselves with auto-generated content, like the current `/changelogs` and `/symbols` directories in the current `docs.ros.org` API docs structure for ROS 1.

> 写软件包文档时，维护者不必担心自动生成的内容，比如当前 ROS 1 中 docs.ros.org API 文档结构中的 `/changelogs` 和 `/symbols` 目录。

### 2. Secondary requirements

**2.1 Package maintainers should have the ability to include free form documentation, like tutorials, design docs, etc., alongside generated API docs on docs.ros.org**

> **2.1 包维护者应该有能力在 docs.ros.org 上除了生成的 API 文档之外，还可以包括自由形式的文档，如教程、设计文档等。**

**2.2 The system should provide a standardized interface for generating an index page/landing page for packages, consistent across ROS 2**

> 系统应提供一个标准的接口，用于为 ROS 2 中的软件包生成一个索引页/入口页，以保持一致性。

**2.3 The system should provide a standardized method for generating package documentation locally, including build error alerts**

> 系统应提供一种标准化的方法来本地生成包文档，包括构建错误警报。

**2.4 The system should be extensible for more languages**

> 系统应该可以扩展支持更多的语言。

**2.5 Users should be able to switch between versions (distributions) while viewing a package's documentation**

> 用户应该能够在查看软件包文档时切换版本(发行版)。

**2.6 The system should support building documentation for more than one version of a package per ROS 2 distribution**

> 系统应该支持为每个 ROS 2 发行版构建多个版本的文档。

**2.7 Changes to documentation pushed in a package repository should be immediately deployed to the documentation site**

> 对包存储库中的文档进行的更改应立即部署到文档网站上。

**2.8 The system should support bi-directional cross-referencing ("backwards" to dependents)**

> 系统应支持双向交叉引用(“向后”引用依赖项)

## Design

The design covers the solution for satisfying the primary requirements. Some secondary requirements are also satisfied inherently through the primary design decisions.

> 设计涵盖了满足主要要求的解决方案。一些次要要求也通过主要设计决策内在地得到满足。

In summary, the design for the system is:

> 总之，系统的设计是：

- Utilize [Sphinx](https://www.sphinx-doc.org/en/master/), [Doxygen](https://www.doxygen.nl/index.html) and [Breathe](https://breathe.readthedocs.io/en/latest/) tools to satisfy both C++ and Python API documentation (1.4), cross-referencing across packages (1.9) and consistent use of RST (1.5)

> 使用 [Sphinx](https://www.sphinx-doc.org/en/master/)、[Doxygen](https://www.doxygen.nl/index.html) 和 [Breathe](https://breathe.readthedocs.io/en/latest/) 工具，满足 C++ 和 Python API 文档(1.4)、跨包交叉引用(1.9)和一致使用 RST(1.5)的要求。

- Develop a new doc tool integrated into the build system (1.7) for maintainers to run on their packages (1.3) that encompasses Sphinx, Doxygen and Breathe, and builds the docs locally (2.3).

> 开发一个新的文档工具，集成到构建系统(1.7)中，供维护者在他们的软件包(1.3)上运行，包括 Sphinx、Doxygen 和 Breathe，并在本地构建文档(2.3)。

- The output of the tool is uploaded to docs.ros.org (1.1) by the build farm, to the correct URL address (1.2, 1.10)

> 输出工具上传到 docs.ros.org(1.1)，由构建农场发送到正确的 URL 地址(1.2，1.10)。

- A package's inclusion in a rosdistro (as per normal release process\*) will version its documentation to that distribution (1.6) and list the package under that distribution's listing of packages on docs.ros.org (1.8)

> 一个软件包按照正常的发布流程被包含在 rosdistro 中，其文档的版本将会被设置为该发行版本(1.6)，并且该软件包将会在 docs.ros.org 上的发行版本列表(1.8)中列出。

- Via the inclusion of a doc block in a distribution.yaml in [github.com/ros/rosdistro](https://github.com/ros/rosdistro)

> 通过在 [github.com/ros/rosdistro](https://github.com/ros/rosdistro) 中的 distribution.yaml 中添加文档块。

The design description is laid out in the following subsections:

> 以下子节将详细描述设计：

- The [Documentation System](#documentation-system) section, which describes, at a high level, the functionality of the doc tool for both the user side of package documentation as well as how it interacts with the buildfarm.

> - [文档系统](#documentation-system) 部分，描述了文档工具在包文档的用户端以及与构建农场如何交互的功能，概述性描述。

- The [URL Structure](#url-structure) section, which shows how directories under `docs.ros.org` will be organized in the new system.

> - [URL 结构](#url-structure) 部分，展示了新系统下 `docs.ros.org` 下的目录如何组织。

- The [Considerations](#considerations) section, which addresses some other options that were considered for various aspects of the design, and why they were decided against.

> [考虑](#%E8%80%83%E8%99%91)部分讨论了设计的各个方面的其他选择，以及为什么最终没有采用这些选择。

### Documentation System

#### User process

The doc tool will invoke everything necessary to document a work space according to our requirements and vision for the new documentation site. It can be run on any ROS 2 package.

> 文档工具将按照我们对新文档站点的要求和愿景调用一切必要的文档工作空间。它可以在任何 ROS 2 软件包上运行。

The tool may be developed to run standalone, or it may be necessary for maintainers to add a one liner to their package.xml, rosdoc.yaml, etc. telling the tool whether it should be building C++ docs, Python docs, or both.

> 工具可以开发成独立运行，也可能需要维护者在他们的 package.xml、rosdoc.yaml 等中添加一行代码来告知工具是否应该构建 C++ 文档、Python 文档还是两者都构建。

The tool works by first building the package to ensure all the generated code is accounted for and able to be documented. Then, the tool checks for a `doxyfile` or C++ code. If either are present, the tool will run Doxygen.

> 工具首先构建包以确保所有生成的代码都被记录并可以被文档化。然后，工具会检查 `doxyfile` 或 C++ 代码。如果存在任一个，工具将运行 Doxygen。

If the package maintainer wants their C++ code documented, they should include a `doxyfile`. We can provide a standard template for the `doxyfile`, which maintainers can then customize the contents of, to the specifications of their package's needs.

> 如果包维护者希望对他们的 C++ 代码进行文档化，他们应该包含一个 `doxyfile`。我们可以为 `doxyfile` 提供一个标准模板，维护者可以根据他们包的需求自定义其内容。

Maintainers can also choose to not include a `doxyfile`, in which case the tool will substitute a default doxyfile if it finds C++ code, providing standard configuration for the package.

> 维护者也可以选择不包含 `doxyfile`，在这种情况下，如果工具发现 C++ 代码，它将提供标准配置的默认 doxyfile 给该包。

Doxygen will produce API docs for any C++ code it is run on, and the maintainer has the option to further elaborate on their API docs by including Doxygen comment blocks within their source code.

> Doxygen 可以在运行的 C++ 代码上产生 API 文档，维护者可以通过在源代码中包含 Doxygen 注释块来进一步详细说明他们的 API 文档。

Once Doxygen is completed, or if it never ran because the tool did not find any `doxyfile` or C++ code, the tool will proceed to running Breathe and Sphinx.

> 一旦 Doxygen 完成，或者如果它从未运行过，因为工具没有找到任何 `doxyfile` 或 C++ 代码，工具将继续运行 Breathe 和 Sphinx。

Breathe imports the symbols from Doxygen's output to Sphinx. It will allow C++ API docs to be built by Sphinx to maintain consistency across ROS 2 API docs and allow cross-referencing between all packages.

> Breathe 将 Doxygen 的输出符号导入到 Sphinx 中。它将允许用 Sphinx 构建 C++ API 文档，以保持 ROS 2 API 文档的一致性，并允许所有包之间进行交叉引用。

If there is no Doxygen output (no C++ documentation), Sphinx will run without Breathe. All packages will need a Sphinx configuration file. If a package doesn't have one, the tool can use a default one. Python packages will need Sphinx directives explicitly integrated into the source code to produce API documentation (standard for Sphinx API documentation).

> 如果没有 Doxygen 输出(没有 C++ 文档)，Sphinx 将不使用 Breathe 运行。所有软件包都需要一个 Sphinx 配置文件。如果软件包没有，该工具可以使用默认的。Python 软件包需要显式集成到源代码中的 Sphinx 指令来生成 API 文档(Sphinx API 文档的标准)。

Once complete, the tool will output the package documentation build locally. Users will be able to see any build errors as well as how their documentation will look once built to `docs.ros.org`.

> 一旦完成，这个工具将在本地构建软件包文档。用户将能够查看任何构建错误以及构建到 `docs.ros.org` 时文档的外观。

##### Support for other languages

This section addresses how the system will potentially satisfy requirement 2.4 in the future. In general, the goal is to treat additional languages the same as the system treats Python and Doxygen. That is, a library in any additional language should be able to utilize its native/standard documentation tool. There should then be a bridge between the output of that tool and Sphinx, like how Breathe bridges Doxygen and Sphinx. This will most likely not be an out-of-box solution for most languages, but the assumption is that extension will be possible.

> 这一节讨论系统如何有可能满足未来的要求 2.4。一般而言，目标是将额外的语言与系统对待 Python 和 Doxygen 一样。也就是说，任何额外的语言库都应该能够利用它们的本地/标准文档工具。然后，应该有一个桥接该工具输出和 Sphinx 的桥梁，就像 Breathe 桥接 Doxygen 和 Sphinx 一样。这很可能不是大多数语言的现成解决方案，但假设可以扩展。

Bridging with Sphinx allows the assumption that cross-referencing will be possible. It also maintains the standard landing page requirement; the top level entry point will be Sphinx regardless of a package's language. The landing page will provide access to the other generated documentation.

> 使用 Sphinx 进行桥接可以假定可以进行交叉引用。它还保持标准登录页面要求;无论包的语言如何，顶级入口点都将是 Sphinx。登录页面将提供访问其他生成文档的途径。

For packages with documentation for multiple languages, the landing pages will provide access to each generator (e.g. "Click here for Doxygen content, Click here for Javadoc content"). Supporting multiple documentation engines will require that each generator outputs into its own designated directory (see [URL Structure - /generated](#generated)).

> 对于拥有多种语言文档的包，入口页面将提供访问每个生成器的功能(例如，“点击这里访问 Doxygen 内容，点击这里访问 Javadoc 内容”)。支持多种文档引擎需要每个生成器输出到其自己指定的目录(参见 [URL 结构-/generated](#generated))。

#### Buildfarm process

To opt in to having documentation built on docs.ros.org, maintainers will have to add a `doc` block to their `distribution.yaml` file that indicates the rosdistro branch and a path to the repository. Opting-in this way will cause the tool to automatically generate an index landing page for the package.

> 要选择在 docs.ros.org 上构建文档，维护者必须在其 `distribution.yaml` 文件中添加一个 `doc` 块，以指示 rosdistro 分支和存储库的路径。以这种方式选择将导致工具自动生成该软件包的索引首页。

The buildfarm will invoke the same tool used in the user process, run on ***n*** packages, to generate the documentation for `docs.ros.org`. It will require some additional settings, for example the URL where content should generate into.

> 编译农场将调用用户过程中使用的相同工具，在***n***个包上运行，以为 docs.ros.org 生成文档。它需要一些额外的设置，例如内容应该生成到的 URL。

A webhook between GitHub and the buildfarm will indicate when there is a new commit to a repository (for the branches being tracked for documentation) so the documentation can be automatically regenerated and immediately deployed.

> 一个 GitHub 和构建农场之间的 Webhook 会指示当有新的提交到存储库(用于跟踪文档的分支)时，可以自动重新生成文档并立即部署。

Some files will be auto-generated during this process (changelogs, tag files, etc.). The auto-generated content won't be saved to the upstream repositories, but it will be built to `docs.ros.org`.

> 在此过程中会自动生成一些文件(更改日志、标签文件等)。自动生成的内容不会保存到上游存储库中，但会构建到 `docs.ros.org` 上。

There is a possibility of collision occurring between maintainer created files and generated files. To prevent this to the best of the system's ability, the system will place auto-generated content in a restricted directory (see [URL Structure - /generated](#generated)).

> 有可能发生维护者创建的文件和生成的文件之间的冲突。为了尽可能有效地防止这种情况，系统将把自动生成的内容放在一个受限的目录中(参见 [URL 结构-/generated](#generated))。

##### Cross-referencing between sibling packages

This section addresses requirement 2.8, cross-referencing between sibling packages. As packages integrate their documentation into the system, cross-referencing between packages will be inconsistent. It may require multiple builds of all the packages in the system to eventually reach a consistent state for cross-references. How exactly this will be executed will be determined.

> 这一节讨论了要求 2.8，即兄弟包之间的互参。随着包整合他们的文档到系统中，兄弟包之间的互参将会不一致。这可能需要对系统中的所有包进行多次构建，以最终达到一致的互参状态。如何执行这一点将会进一步确定。

### URL structure

![The URL structure of the new documentation system](./per_package_documentation/url-structure.png)

#### `index` pages

The index pages indicate the default page that is rendered when the directory level immediately before is visited without any specific `.html` page following.

> 索引页面表明，当没有任何特定的 `.html` 页面跟随时，访问立即之前的目录级别时所呈现的默认页面。

For example, visiting docs.ros.org/en/foxy would render `index1`, docs.ros.org/en/foxy/p would render `index2`, and docs.ros.org/en/foxy/p/`<package_name>`{=html} would render `index3`.

> 例如，访问 docs.ros.org/en/foxy 将呈现“index1”，docs.ros.org/en/foxy/p 将呈现“index2”，而 docs.ros.org/en/foxy/p/<package_name>{=html}将呈现“index3”。

`index1` is the general ROS 2 documentation overview page, what is currently seen when visiting index.ros.org/doc/ros2/

> index1 是 ROS 2 文档概览页面，访问 index.ros.org/doc/ros2/时当前所看到的内容。

`index2` will be the package listing page for each ROS 2 distribution.

> index2 将是每个 ROS 2 发行版的包列表页面。

`index3` will be each specific package's homepage. This will be generated by the tool, but a user-generated index page should also be able to be merged or augmented with the auto-generated page to prevent collision.

> `index3` 将是每个特定软件包的主页。这将由工具生成，但用户生成的索引页面也应该能够与自动生成的页面合并或增强以防止冲突。

#### Versioning at the `<package_name>` level

To support the goal of being able to support multiple versions of packages documented within the context of one distro (2.6) we plan to reserve URL space and have a plan to migrate forward but not to implement it at first.

> 为了支持在一个发行版(2.6)的上下文中文档化的多个版本的软件包，我们计划保留 URL 空间，并有一个向前迁移的计划，但不会一开始就实施它。

When functionality is added to the documentation system for more than one version of package documentation per rosdistro, those versions will be appended to the package name in the URL. For example, `/p/rclcpp` can have versioned content added to `/p/rclcpp-2.1.0` and `/p/rclcpp-3.0.0`. After enabling versioning, the unversioned URL will be a link to the version currently indexed in the rosdistro.

> 当为多个 rosdistro 的包文档系统添加功能时，这些版本将被附加到 URL 中的包名中。例如，`/p/rclcpp` 可以添加到 `/p/rclcpp-2.1.0` 和 `/p/rclcpp-3.0.0` 中。启用版本后，未版本化的 URL 将是当前在 rosdistro 中索引的版本的链接。

See [Considerations - Package version directory](#package-version-directory) for an explanation of the naming convention.

> 请参阅[考虑因素 - 包版本目录](#package-version-directory)以获得命名约定的解释。

#### `/generated`

The purpose of this directory is to allocate a place where automatically generated content can be placed without risk of collision with user-created content. The tool will throw warnings to ensure users do not write any manual content into `/generated`; it is ultimately up to maintainers to prevent collision. Manually created content will override auto-generated content it collides with.

> 此目录的目的是为自动生成的内容提供一个可以放置的地方，而不必担心与用户创建的内容发生冲突。该工具将发出警告，以确保用户不会将任何手动内容写入“/ generated”; 最终由维护者防止冲突。手动创建的内容将覆盖与其发生冲突的自动生成的内容。

The index page for each package will link to the generated API docs, changelogs, etc.

> 每个包的索引页面都会链接到生成的 API 文档、更新日志等。

#### `user generated`

Within a package's repo, the tool will generate a `/docs` directory where maintainers can place any manually created rst files they would like, like tutorials, architecture articles, etc.

> 在一个包的存储库中，工具将生成一个 `/docs` 目录，维护者可以在其中放置任何手动创建的 rst 文件，如教程、架构文章等。

The tool will direct Sphinx to generate the content in the `/docs` directory, which will output to the top level when deployed (sibling to `/generated` and `index3`).

> 工具将指导 Sphinx 生成 `/docs` 目录中的内容，部署后将输出到顶级(与 `/generated` 和 `index3` 兄弟)。

However, it may become evident during development of the system that it is necessary to add an additional subdirectory/subdirectories at this level to contain the manually created content. This decision will be confirmed before deployment of the new site. For now, the URL structure uses `user generated` as a placeholder for either case.

> 然而，在系统开发过程中，可能会显示出在此层级添加额外的子目录/子目录以包含手动创建的内容是必要的。在新网站部署之前，将确认此决定。目前，URL 结构使用 `用户生成` 作为两种情况的占位符。

#### ROS 1 documentation

Besides the addition of `/en`, the structure and functionality of ROS 1's API documentation will remain the same. Necessary redirects will be implemented.

> 除了添加 `/en` 之外，ROS 1 的 API 文档的结构和功能将保持不变。必要的重定向将被实施。

### Considerations

#### Other doc tools

There are many documentation tools available, but ultimately Sphinx, Doxygen and Breathe were the best choices to satisfy our requirements.

> 有很多文档工具可用，但最终 Sphinx、Doxygen 和 Breathe 是满足我们要求的最佳选择。

Sphinx and Doxygen are well-established and ubiquitous for documenting their respective languages This ensures the robustness of the system more so than any of the non-standard tools, and also that our users will have easy access to resources to aid them in documenting their packages with these tools.

> 斯芬克斯和 Doxygen 是用于记录其各自语言的成熟和普及的工具。这比任何非标准工具都能更有效地确保系统的稳健性，也可以让我们的用户轻松访问资源，以帮助他们使用这些工具来记录他们的软件包。

With Breathe, it is simple to satisfy the requirement of coordinating C++ and Python documentation, and basing the system off of Sphinx allows simple customization with Python modules.

> 通过 Breathe，可以轻松满足协调 C++ 和 Python 文档的要求，并且基于 Sphinx 的系统可以使用 Python 模块进行简单的定制。

Another tool, [Exhale](https://exhale.readthedocs.io/en/latest/) has been considered for future addition to the system. Exhale adds some extra features available in pure Doxygen that Breathe alone doesn't import into Sphinx.

> 另一个工具 [Exhale](https://exhale.readthedocs.io/en/latest/) 被考虑添加到系统中。Exhale 添加了一些纯 Doxygen 中 Breathe 单独无法导入 Sphinx 的额外功能。

#### Allowing pure Doxygen use

If a package maintainer insists on using Doxygen alone, they are of course welcome to. However, this is not a use case we plan to support.

> 如果包维护者坚持只使用 Doxygen，他们当然可以。但是，这不是我们计划支持的用例。

The standard entry point should be utilizing our tool, which will run Doxygen but then also run Breathe and Sphinx over it so the output can be uniform with the rest of our documentation. Using Doxygen by itself will sacrifice that consistency and also the ability to cross-reference to and from that package, both major driving points for this project. The tool should mostly mask the complexities of the Sphinx/Breathe/Doxygen system, making the difference between using "pure" Doxygen and using the new tool negligible for maintainers.

> 标准入口点应该是使用我们的工具，它将运行 Doxygen，然后还要运行 Breathe 和 Sphinx 来使输出与我们的其他文档保持一致。单独使用 Doxygen 会牺牲这种一致性，也会牺牲这个项目的跨引用能力。该工具应该主要掩盖 Sphinx/Breathe/Doxygen 系统的复杂性，使维护者在使用“纯”Doxygen 和使用新工具之间的区别变得可忽略不计。

#### Package version directory

To satisfy secondary requirement 2.6, the first obvious consideration for the structure was `.../p/<package_name>/<version>`, but, as indicated in the URL structure section above, the decision is to use `.../p/<package_name-version.number.n>`.

> 为满足次要要求 2.6，结构的第一个明显考虑是 `.../p/<package_name>/<version>`，但是，如上面 URL 结构部分所指示的，决定使用 `.../p/<package_name-version.number.n>`。

The chosen structure doesn't force another directory, which allows versioned package documentation to be added later on in the development of the system without leaving an unused directory level in the meantime. Adding package versions in the future will not disrupt the meaning of the URL structure, because `/<package_name>/` without any appended `-version.number.n` will always implicitly indicate the latest version.

> 所选的结构不会强制另一个目录，这使得在系统的开发过程中可以添加带有版本号的包文档，而不会在此期间留下未使用的目录级别。在未来添加包版本不会破坏 URL 结构的意义，因为没有任何附加的 `-version.number.n`，`/<package_name>/` 始终会隐式指示最新版本。

```{=html}
<!-- ## Reference -->
```
