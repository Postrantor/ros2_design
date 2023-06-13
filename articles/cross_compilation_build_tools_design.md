---
tip: translate by openai@2023-06-13 22:57:41
...
---
abstract: This article is a design proposal for developing a ROS 2 tool that sets up and manages sysroot environments for cross-compilation with the objective of being simple and extensible. Extending support for new cross compilation configurations using colcon mixins is also proposed.

> 本文是一个设计提案，旨在开发一个 ROS 2 工具，用于设置和管理 sysroot 环境，以实现跨编译，目标是简单易用且可扩展。还提出了使用 colcon mixins 扩展支持新的跨编译配置的方案。
> author: |

[Thomas Moulard](https://github.com/thomas-moulard), [Juan Rodriguez Hortala](https://github.com/juanrh), [Anas Abou Allaban](https://github.com/piraka9011)

> [Thomas Moulard](https://github.com/thomas-moulard)、[Juan Rodriguez Hortala](https://github.com/juanrh)、[Anas Abou Allaban](https://github.com/piraka9011)
> date_written: 2019-09
> last_modified: 2019-09
> layout: default

permalink: articles/cc_build_tools.html

> 链接：文章/cc_build_tools.html
> published: true
> title: Cross-compiling ROS 2 packages

---

## Background

The [cross-compilation tutorial](https://index.ros.org/doc/ros2/Tutorials/Cross-compilation) and the [esteve/ros2_raspbian_tools repository](https://github.com/esteve/ros2_raspbian_tools) contain instructions for cross-compiling ROS 2 to unsupported architectures, like ARM64 and ARM-HF, using toolchain files for CMake and Docker images. Other projects like [esteve/ros2_objc](https://github.com/esteve/ros2_objc) and [esteve/ros2_java](https://github.com/esteve/ros2_java) provide support for additional platforms like Android and iOS. Ideally, we should be able to cross-compile a ROS package for ROS 2 by launching a single command. That command should also be implemented using an extensible design that allows supporting new platforms with low effort. This design proposal is a push towards Tier-1 support of ARM-HF and ARM64 architectures.

> 教程[跨编译](https://index.ros.org/doc/ros2/Tutorials/Cross-compilation)和 [esteve/ros2_raspbian_tools 存储库](https://github.com/esteve/ros2_raspbian_tools)包含使用 CMake 工具链文件和 Docker 映像文件将 ROS 2 编译到不受支持的架构（如 ARM64 和 ARM-HF）的说明。其他项目，如 [esteve/ros2_objc](https://github.com/esteve/ros2_objc) 和 [esteve/ros2_java](https://github.com/esteve/ros2_java) 提供对 Android 和 iOS 等其他平台的支持。理想情况下，我们应该能够通过启动一个单独的命令来跨编译 ROS 包。该命令还应该使用可扩展的设计实现，允许以低成本支持新平台。此设计提案是朝着支持 ARM-HF 和 ARM64 架构的一级支持的努力。

## Proposed Approach

We propose continuing with the general approach of the cross-compilation tutorial. This involves building a sysroot for the target platform, using QEMU and Docker, and then using [colcon mixins](https://github.com/colcon/colcon-mixin) to compile with C and C++ cross-compilers.

> 我们建议继续使用跨编译教程的一般方法。这涉及使用 QEMU 和 Docker 为目标平台构建 sysroot，然后使用 [colcon mixins](https://github.com/colcon/colcon-mixin) 使用 C 和 C++ 跨编译器进行编译。

We propose adding a set of commands to wrap the instructions in the cross-compilation tutorial. These commands would use a new workspace directory for the target platform, determined by a convention based on the:

> 我们建议添加一组命令来包装交叉编译教程中的指令。这些命令将使用基于以下约定确定的目标平台的新工作空间目录：

- Target platform
- Operating system
- RMW implementation
- ROS distribution

Let us call this workspace directory the *cc-root*. Under this convention, the platform implies the architecture, but generic platforms like `generic_armhf` would also be available. An example *platform identifier* would be `generic_armhf-ubuntu_bionic-fastrtps-crystal` or `turtlebot_armhf-ubuntu_bionic-fastrtps-dashing`. To implement the proposal, we would add the following commands:

> 我们将这个工作区目录称为 *cc-root*。根据这个约定，平台暗示了架构，但是也会有像 `generic_armhf` 这样的通用平台可用。一个例子*平台标识符*会是 `generic_armhf-ubuntu_bionic-fastrtps-crystal` 或者 `turtlebot_armhf-ubuntu_bionic-fastrtps-dashing`。为了实现这个提案，我们会添加以下命令：

- A new `create-cc-sysroot` command that would use Docker to generate the sysroot for the target platform and store it in a `sysroot` subdirectory of the *cc-root*. This command would take arguments to specify the target platform, for example:

> 一个新的 `create-cc-sysroot` 命令，可以使用 Docker 来生成目标平台的 sysroot，并将其存储在 *cc-root* 的 `sysroot` 子目录中。此命令将接受参数来指定目标平台，例如：

```bash
create-cc-sysroot --arch generic_armhf --os ubuntu_bionic \
                  --rmw fastrtps --rosdistro crystal
```

- A new colcon mixin for each known platform, which adds options to the colcon build task for using a sysroot generated with `create-cc-sysroot`, by using the same path conventions. For example, from a ROS 2 overlay workspace on a developer workstation, the following command would cross-compile the packages in the workspace up to a package `performance_test` for the platform specified. This would create `build`, `install`, and `log` subdirectories of the cc-root `generic_armhf-ubuntu_bionic-fastrtps-crystal` with the cross-compiled binaries.

> 为每个已知平台提供一个新的 colcon 混合，它为 colcon 构建任务添加了使用使用 `create-cc-sysroot` 生成的 sysroot 的选项，使用相同的路径约定。例如，从开发者工作站上的 ROS 2 叠加工作区，可以执行以下命令来为指定平台跨编译工作区中的软件包，直到一个名为 `performance_test` 的软件包。这将创建 cc-root `generic_armhf-ubuntu_bionic-fastrtps-crystal` 的 `build`，`install` 和 `log` 子目录，其中包含编译后的二进制文件。

```bash
colcon build --mixin cc-generic_armhf-ubuntu_bionic-fastrtps-crystal \
  --packages-up-to performance_test # any other other `colcon build` arguments
```

- On a second iteration of this initiative we would:

  - Maintain a docker image for cross compilation, that has both the cross compilation toolchain and the aforementioned colcon mixins installed.

> 维护一个 docker 镜像，用于跨平台编译，其中包含跨平台编译工具链和上述的 colcon mixins 安装。

- Implement a new `cc-build` command that would ensure the sysroot is created using `create-cc-sysroot`, and then launch the cross compilation into a docker container using the corresponding Docker image.

> 实现一个新的 `cc-build` 命令，确保使用 `create-cc-sysroot` 创建 sysroot，然后使用相应的 Docker 映像在 Docker 容器中启动跨编译。

Under the hood, the `create-cc-sysroot` command would do the following:

> 在底层，`create-cc-sysroot` 命令会执行以下操作：

1. Downloads a base ROS 2 Docker image for the target platform.

> 下载目标平台的基础 ROS 2 Docker 镜像。

2. Builds a workspace dependent sysroot image by using a Dockerfile that starts from that base image and:

> 使用 Dockerfile 从基础镜像开始构建一个依赖工作区的系统根映像。

1. Runs `COPY` to get the contents of the workspace into the container.

> 运行 `COPY` 命令以将工作区的内容复制到容器中。

2. Uses `rosdep` to ensure no workspace system dependency is missing. Note we can run `rosdep` in Docker for architectures like ARM-HF thanks to [QEMU](https://www.qemu.org/).

> 使用 `rosdep` 确保没有工作区系统依赖性遗漏。注意，由于[QEMU]（[https://www.qemu.org/](https://www.qemu.org/)），我们可以在像 ARM-HF 这样的架构中在 Docker 中运行 `rosdep`。

3. Launches a container for that image and exports its file system into a `sysroot` subdirectory of the cc-root.

> 启动一个容器，使用该镜像，并将其文件系统导出到 cc-root 的 sysroot 子目录中。

4. During a build, the colcon mixin sets the appropriate CMake arguments for cross compilation and/or point to a toolchain file (ex. [generic_linux.cmake](https://github.com/ros2/cross_compile/blob/master/cmake-toolchains/generic_linux.cmake)) using the `CMAKE_TOOLCHAIN_FILE` argument.

> 在构建过程中，colcon mixin 使用 `CMAKE_TOOLCHAIN_FILE` 参数设置适当的 CMake 参数以进行交叉编译，或指向工具链文件（例如 [generic_linux.cmake](https://github.com/ros2/cross_compile/blob/master/cmake-toolchains/generic_linux.cmake)）。

The ROS 2 base images are variants of [sysroot/Dockerfile_ubuntu_arm](https://github.com/ros2/cross_compile/blob/master/sysroot/Dockerfile_ubuntu_arm) from ros2/cross_compile which use the [Docker Official Images](https://hub.docker.com/_/ros). OSRF would publish base ROS 2 images with ROS 2 prebuilt, that should be used for cross compiling workspaces with user packages. OSRF would also publish other base ROS 2 images with the system setup and basic tools for building ROS 2 preinstalled, that should be used for building ROS 2 from source.

> ROS 2 基础镜像是来自 ros2/cross_compile 的 [sysroot/Dockerfile_ubuntu_arm](https://github.com/ros2/cross_compile/blob/master/sysroot/Dockerfile_ubuntu_arm) 的变体，使用 [Docker 官方镜像](https://hub.docker.com/_/ros)。OSRF 将发布带有 ROS 2 预编译版本的基础 ROS 2 镜像，应用于用户包的跨编译工作空间。OSRF 还将发布其他基础 ROS 2 镜像，系统设置和用于构建 ROS 2 的基本工具预先安装，用于从源代码构建 ROS 2。

The `create-cc-sysroot` command will also support the following optional arguments:

> 命令 `create-cc-sysroot` 还将支持以下可选参数：

- `--sysroot-base-image`: Specifies a Docker image that will be used as the base image for the workspace dependent sysroot image. This should be a string that is a valid argument for `docker image pull` and that can be used in the `FROM` Dockerfile statement. If it is not specified, the command will deduce the value of `--sysroot-base-image` from the target platform and pull from a published image on [Docker's Official ROS Images](https://hub.docker.com/_/ros), the [arm64v8 Dockerhub repo](https://hub.docker.com/u/arm64v8) or from the [arm32v7 Dockerhub repo](https://hub.docker.com/u/arm32v7).

> `--sysroot-base-image`：指定用作工作空间相关 sysroot 镜像的基础镜像的 Docker 镜像。这应该是一个有效的 `docker image pull` 参数的字符串，并且可以在 `FROM` Dockerfile 语句中使用。如果未指定，则该命令将从 [Docker 的官方 ROS 镜像](https://hub.docker.com/_/ros)，[arm64v8 Dockerhub repo](https://hub.docker.com/u/arm64v8) 或 [arm32v7 Dockerhub repo](https://hub.docker.com/u/arm32v7) 中推断 `--sysroot-base-image` 的值并拉取。

- `--sysroot-image`: Specifies a Docker image for the workspace sysroot. The build will export the sysroot from that image, instead of creating a sysroot image from the workspace. No check is performed to ensure the image has all the dependencies required by the packages in the workspace.

> `--sysroot-image`：为工作区系统根指定一个 Docker 镜像。 该构建将从该图像导出系统根，而不是从工作区创建系统根映像。 不会检查以确保该映像具有工作区中的所有包所需的依赖项。

- `--use-base-image-sysroot`: Boolean flag, when true the base ROS 2 Docker image is used to build the sysroot, so the dependencies declared in packages in the current workspace are ignored. This is useful to speed up builds for packages without additional dependencies.

> `-`--use-base-image-sysroot`：布尔标记，当为 true 时，将使用基础 ROS 2 Docker 镜像来构建 sysroot，因此将忽略当前工作区中包中声明的依赖项。这对于没有额外依赖项的包来说可以加快构建速度。

Support for other platforms can be added by creating corresponding ROS 2 Docker images, and variants of the toolchain file if required. We could include support for generic versions of ARM-HF and ARM64, on Ubuntu Bionic with ROS 2 crystal and fastrtps, in the first release of the commands.

> 支持其他平台可以通过创建相应的 ROS 2 Docker 镜像和工具链文件的变体来添加。我们可以在第一个发布版本的命令中包括对 Ubuntu Bionic 上的 ARM-HF 和 ARM64 的通用版本以及 ROS 2 crystal 和 fastrtps 的支持。

### Frequently asked questions

- *How can we run the resulting binaries?* Packaging the resulting binaries is outside of the scope of this document. The cross-compiled binaries will be available in the workspace in the cc-root (e.g. `generic_armhf-ubuntu_bionic-fastrtps-crystal`). We make the assumption that the user will copy the whole workspace to the remote device (including `src`) and run `rosdep` on the target platform, to install the dependencies before running the binaries.

> 我们如何运行生成的二进制文件？封装生成的二进制文件不在本文档的范围之内。经过跨编译的二进制文件将会出现在 cc-root（例如 `generic_armhf-ubuntu_bionic-fastrtps-crystal`）的工作空间中。我们假设用户将整个工作空间（包括 `src`）复制到远程设备上，并在目标平台上运行 `rosdep`，在运行二进制文件之前安装依赖项。

### Maintenance and testing

Regarding maintenance, in order to support cross-compilation for a new platform we would need to:

> 关于维护，为了支持新平台的交叉编译，我们需要：

- Prepare and publish a base ROS 2 Docker image for the new platform, both fetching the dependencies using the ROS 2 source using `rosdep install`, and from the pre-built binary if available for that platform.

> 准备和发布一个基础 ROS 2 Docker 镜像，使用 ROS 2 源使用 `rosdep install` 获取依赖项，如果该平台可用，也可以从预构建的二进制文件获取。

- Adapt the toolchain file for the new platform, if required. For that the toolchains in [ruslo/polly](https://github.com/ruslo/polly) might be useful.

> 请按需要为新平台调整工具链文件。可以参考 [ruslo/polly](https://github.com/ruslo/polly) 中的工具链。

- Add a new colcon mixin to the `cross-compile` repository for the new platform and perform any modifications needed to use a suitable toolchain file.

> 在 `cross-compile` 存储库中添加一个新的 colcon mixin，并执行任何必要的修改以使用合适的工具链文件。

We can then test cross-compile support for the new platform by:

> 我们可以通过以下方式测试新平台的交叉编译支持：

1. Building the base ROS 2 Docker image.

> 构建基础 ROS 2 Docker 镜像。

2. Using `create-cc-sysroot` to build a custom sysroot image for some reference package, e.g. [ApexAI/performance_test](https://github.com/ApexAI/performance_test/), and export its file system into a sysroot.

> 使用 `create-cc-sysroot` 来构建某个参考包的自定义 sysroot 映像，例如 [ApexAI/performance_test](https://github.com/ApexAI/performance_test/)，并将其文件系统导出到 sysroot 中。

3. Cross-compiling that package using the sysroot.

> 编译该软件包时使用 sysroot。

4. Launching a Docker container for that sysroot image and running `colcon test` using the cross-compiled binaries. The container would be launched with a [bind mount](https://docs.docker.com/storage/bind-mounts/) of the workspace so they have access to the compiled binaries.

> 4. 使用交叉编译的二进制文件启动该 sysroot 映像的 Docker 容器，并运行 `colcon test`。该容器将使用工作空间的[绑定挂载](https://docs.docker.com/storage/bind-mounts/)启动，以便它们可以访问编译的二进制文件。

In order to simplify the development of base ROS 2 Docker images for new architecture-OS combinations, on a future iteration of this initiative we might add a new command `build-sysroot-base-image` that would:

> 为了简化新架构-操作系统组合的基础 ROS 2 Docker 映像的开发，在未来的迭代中，我们可能会添加一个新的命令 `build-sysroot-base-image`，它将：

1. Copy the required QEMU files and ROS 2 source files into a temporary location.

> 复制所需的 QEMU 文件和 ROS 2 源文件到临时位置。

2. Build a Docker image from a specified Dockerfile, with access to the those resources.

> 从指定的 Dockerfile 构建 Docker 镜像，并获取对这些资源的访问权限。

3. Optionally publish the image into a Docker registry, using a suitable naming convention that corresponds to the platform identifier.

> 可选择将图像发布到 Docker 注册表中，使用与平台标识符对应的合适的命名约定。

Dockerfiles compatible with that command would be kept on the `ros2/cross_compile` repository, so they can be used in CI/CD pipelines if needed.

> Dockerfiles 与该命令兼容的将保存在 ros2/cross_compile 存储库中，因此可以在需要时在 CI/CD 管道中使用它们。

### Advantages of the approach

The proposed design leads to a simple development experience, where cross compilation is triggered by two simple commands, or a single `cc-build` command for the second iteration. The design is extensible and new platforms can be supported easily.

> 所提出的设计导致了一种简单的开发体验，通过两个简单的命令或一个 `cc-build` 命令来触发跨编译，以实现第二次迭代。该设计是可扩展的，新平台可以轻松支持。

For simple packages with additional dependencies, there is no need to build the sysroot locally with Docker, and a readily available base Docker image can be used. For packages with custom dependencies, the base sysroot can be extended using `rosdep install` on a Docker image that uses QEMU through [`binfmt_misc`](https://en.wikipedia.org/wiki/Binfmt_misc).

> 对于具有附加依赖项的简单软件包，无需使用 Docker 本地构建 sysroot，可以使用可用的基础 Docker 映像。对于具有自定义依赖项的软件包，可以使用使用 QEMU 通过[`binfmt_misc`](https://en.wikipedia.org/wiki/Binfmt_misc)的 Docker 映像扩展基本 sysroot，使用 `rosdep install`。

## Limitations and open questions

This setup is focused on cross-compiling for Linux based platforms. For example we couldn't use this to cross-compile for Windows, because we cannot install the Visual Studio compiler using Apt. On the other hand, a Windows or Mac development workstation could use this build setup, running the build commands inside a Docker container, although that would imply some performance degradation due to the virtualization layer that Docker employs on those platforms.

> 这个设置专注于跨平台编译 Linux 平台。例如，我们不能使用它来跨平台编译 Windows，因为我们无法使用 Apt 安装 Visual Studio 编译器。另一方面，Windows 或 Mac 开发工作站可以使用这个构建设置，在 Docker 容器中运行构建命令，尽管这将意味着由于 Docker 在这些平台上部署的虚拟层而导致性能降低。

We should determine a process for building and publishing the base ROS 2 Docker images for each platform. Ideally a nightly job would publish an image built from source for the tip of master, but that might be not feasible depending on the available resources.

> 我们应该确定一个构建和发布 ROS 2 Docker 镜像的流程，对于每个平台而言。理想情况下，每天都会发布一个从源码构建的 master 最新版本的镜像，但这可能取决于可用资源而不可行。

## Alternative approaches

The [colcon bundle plugin](https://github.com/colcon/colcon-bundle) can be used to package a ROS workspace together with all its dependencies into a tarball. That could be used for cross-compiling a ROS 2 workspace by launching a Docker container for the target platform, building and bundling the workspace, and exporting the bundle, which can then be deployed to the target hardware. The main drawback of that approach is that using the default ARM compiler in the Docker image significantly slows down the compilation, which might be a burden for day-to-day development. By installing `qemu-user-static` in the present proposal, we are not only installing the QEMU binaries to simulate an ARM32/64 processor, but also registering [`binfmt_misc`](https://en.wikipedia.org/wiki/Binfmt_misc) hook in the local machine. That means that every time a program will start, if it is for a platform that QEMU supports, the binary will automatically run through QEMU. In practice, it lets you run ARM binaries on your X86 instance, as it is done implicitly through Docker when building the base image. The problem is that QEMU is \~10x slower than normal execution, but the present proposal limits the QEMU usage to building the base image (which is unavoidable), and uses compilers for the target platform running natively the rest of the time.

> Colcon Bundle 插件（[https://github.com/colcon/colcon-bundle](https://github.com/colcon/colcon-bundle)）可用于将 ROS 工作区及其所有依赖项打包成 tarball。这可用于通过为目标平台启动 Docker 容器，构建和打包工作区，并导出捆绑包来跨编译 ROS 2 工作区。该方法的主要缺点是，使用 Docker 映像中的默认 ARM 编译器会显着降低编译速度，这可能会给日常开发带来负担。在本提案中安装 `qemu-user-static`，不仅安装了 QEMU 二进制文件以模拟 ARM32/64 处理器，还在本地计算机中注册[`binfmt_misc`]（[https://en.wikipedia.org/wiki/Binfmt_misc](https://en.wikipedia.org/wiki/Binfmt_misc)）钩子。这意味着每次程序启动时，如果它是 QEMU 支持的平台，二进制文件将自动通过 QEMU 运行。实际上，它可以让您在 X86 实例上运行 ARM 二进制文件，就像 Docker 在构建基础映像时隐式执行的那样。问题是 QEMU 比正常执行慢\~10 倍，但本提案将 QEMU 的使用限制为构建基础映像（这是不可避免的），并在其余时间使用本机运行目标平台的编译器。

The colcon bundle approach would require adding support for ROS 2 to colcon bundle, that currently doesn't support Ament. That would provide a way to distribute cross-compiled system dependencies, which is convenient because `rosdep install` can take a lot of time on resource constrained hardware, and requires an Internet connection. The presented approach and colcon bundle are complementary approaches, and if colcon bundle eventually supports ROS 2, we could use `cc-build` for easy cross-compilation minimizing the usage of Docker, followed by `colcon bundle` to generate a deployable artifact.

> 采用 colcon bundle 方法需要为 colcon bundle 添加对 ROS 2 的支持，而 colcon bundle 目前不支持 Ament。这将提供一种分发跨编译系统依赖关系的方式，这很方便，因为 `rosdep install` 在资源受限的硬件上可能需要花费很多时间，而且需要互联网连接。所提出的方法和 colcon bundle 是互补的方法，如果 colcon bundle 最终支持 ROS 2，我们可以使用 `cc-build` 进行简单的跨编译，最大限度地减少使用 Docker，然后使用 `colcon bundle` 生成可部署的工件。

Regarding the workspace dependent sysroot image, we could avoid the Docker `COPY` of the workspace by not building a workspace sysroot image, and instead launching a container for the base ROS 2 image with a [bind mount](https://docs.docker.com/storage/bind-mounts/) for the workspace. The container's entry point would launch `rosdep` on the directory where the workspace is mounted, and we'd export the container's file system is then exported with `docker container export` as above. That avoids a `COPY`, but a disadvantage is that then we wouldn't have a sysroot image for our workspace, and we would have to run `rosdep` on the entry point of that image each time we launch a container. That image would be useful for running tests, as mentioned in the testing section above, and an eventual `cc-test` command could further simplify using that image.

> 关于工作区依赖的 sysroot 映像，我们可以通过不构建工作区 sysroot 映像，而是使用[绑定挂载](https://docs.docker.com/storage/bind-mounts/)启动基础 ROS 2 映像的容器，来避免 Docker 的 `COPY`。容器的入口点将在挂载工作区的目录上启动 `rosdep`，然后使用 `docker container export` 导出容器的文件系统。这样就可以避免 `COPY`，但缺点是我们就没有工作区的 sysroot 映像了，每次启动容器时都必须在该映像的入口点上运行 `rosdep`。该映像对于运行测试非常有用，如上文测试部分所述，最终的 `cc-test` 命令可以进一步简化使用该映像。

Finally, compared with the original approach in the [cross-compilation tutorial](https://index.ros.org/doc/ros2/Tutorials/Cross-compilation), the proposed commands basically organizes the commands that appear in the tutorial, and that are used in [cross_compile/entry_point.sh](https://github.com/ros2/cross_compile/blob/master/entry_point.sh), to allow cross-compiling a workspace with a one-liner command. So this is more an evolution than an alternative proposal.

> 最后，与[跨编译教程](https://index.ros.org/doc/ros2/Tutorials/Cross-compilation)中的原始方法相比，所提出的命令基本上整理了教程中出现的命令，以及 [cross_compile/entry_point.sh](https://github.com/ros2/cross_compile/blob/master/entry_point.sh) 中使用的命令，以允许一行命令跨编译工作区。因此，这更像是一种进化而不是替代性建议。
