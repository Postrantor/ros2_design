---
translate by baidu@2023-05-30 23:46:27
layout: default
title: ROS 2 Launch Static Descriptions - Implementation Considerations
permalink: articles/roslaunch_frontend.html
abstract: The launch system in ROS 2 aims to support extension of static descriptions, so as to easily allow both exposing new features of the underlying implementation, which may or may not be extensible itself, and introducing new markup languages. This document discusses several approaches for implementations to follow.
author: "[Michel Hidalgo](https://github.com/hidmic) [William Woodall](https://github.com/wjwwood)"
date_written: 2019-09
last_modified: 2020-07
published: true
Authors: {{ page.author }}
Date Written: {{ page.date_written }}
Last Modified: {% if page.last_modified %}{{ page.last_modified }}{% else %}{{ page.date_written }}{% endif %}
---
## Context

Static launch descriptions are an integral part to ROS 2 launch system, and the natural path to transition from predominant ROS 1 `roslaunch` XML description. This document describes parsing and integration approaches of different front ends i.e. different markup languages, with a focus on extensibility and scalability.

> 静态发射描述是 ROS 2 发射系统不可分割的一部分，也是从主要的 ROS 1“roslaunch”XML 描述过渡的自然路径。本文档描述了不同前端（即不同的标记语言）的解析和集成方法，重点是可扩展性和可扩展性。

## Proposed approaches

In the following, different approaches to parsing launch descriptions are described. This list is by no means exhaustive.

> 在下文中，将描述解析启动描述的不同方法。这份清单绝非详尽无遗。

It's worth noting some things they all have in common:

> 值得注意的是，它们都有一些共同点：

- All of them attempt to solve the problem in a general and extensible way to help the system scale with its community.
- All of them require a way to establish associations between entities, solved using unique reference id (usually, a human-readable name but that's not a requisite).
- All of them associate a markup language with a given substitution syntax.
- All of them supply a general, interpolating substitution to deal with embedded substitutions e.g. "rooted/$(subst ...)".
- All of them need some form of instantiation and/or parsing procedure registry for parsers to lookup.

How that registry is populated and provided to the parser may vary. For instance, in Python class decorators may populate a global dict or even its import mechanism may be used if suitable, while in C++ convenience macros may expand into demangled registration hooks that can later be looked up by a dynamic linker (assuming launch entities libraries are shared libraries).

> 注册表的填充方式和提供给解析器的方式可能会有所不同。例如，在 Python 类中，decorator 可以填充全局 dict，或者如果合适的话，甚至可以使用其导入机制，而在 C++ 中，方便宏可以扩展为非映射的注册挂钩，稍后可以由动态链接器查找（假设启动实体库是共享库）。

### Forward Description Mapping (FDM)

In FDM, the parser relies on a schema and well-known rules to map a static description (markup) to implementation specific instances (objects). The parser instantiates each launch entity by parsing and collecting the instantiations of the launch entities that make up the former description.

> 在 FDM 中，解析器依赖于模式和众所周知的规则来将静态描述（标记）映射到特定于实现的实例（对象）。解析器通过解析和收集组成前一个描述的启动实体的实例来实例化每个启动实体。

#### Description Markup

Some description samples in different markup languages are provided below:

> 下面提供了一些不同标记语言的描述示例：

_XML_

> _XML 语言_

```xml
    <launch version="x.1.0">
      <action name="some-action" type="actions.ExecuteProcess">
        <arg name="cmd" value="$(substitutions.FindExecutable name=my-process)"/>
        <arg name="env">
          <pair name="LD_LIBRARY_PATH"
                value="/opt/dir:$(substitutions.EnvironmentVariable name=LD_LIBRARY_PATH)"/>
        </arg>
        <arg name="prefix" value="$(substitutions.EnvironmentVariable name=LAUNCH_PREFIX)"/>
      </action>
      <on_event type="events.ProcessExit" target="some-action">
        <action type="actions.LogInfo">
          <arg name="message" value="I'm done"/>
        </action>
      </on_event>
    </launch>
```

_YAML_

> _亚马尔_

```yml
launch:
  version: x.1.0
  actions:
    - type: actions.ExecuteProcess
      name: some-action
      args:
        cmd: $(substitutions.FindExecutable name=my-process)
        env:
          LD_LIBRARY_PATH: "/opt/dir:$(substitutions.EnvironmentVariable name=LD_LIBRARY_PATH)"
        prefix: $(substitutions.EnvironmentVariable name=LAUNCH_PREFIX)
  event_handlers:
    - type: events.ProcessExit
      target: some-action
      actions:
        - type: actions.LogInfo
          args:
            message: "I'm done"
```

#### Advantages & Disadvantages

_+_ Straightforward to implement.

> *+*实施起来直截了当。

_+_ Launch implementations are completely unaware of the existence of the static description formats and their parsing process (to the extent that type agnostic instantiation mechanisms are available).

> *+*启动实现完全不知道静态描述格式及其解析过程的存在（在某种程度上，类型不可知的实例化机制是可用的）。

_-_ Statically typed launch system implementations may require variant objects to deal with actions.

_-_ Static descriptions are geared towards easing parsing, making them more uniform like a serialization format but also less user friendly.

_-_ Care must be exercised to avoid coupling static descriptions with a given implementation.

### Forward Description Mapping plus Markup Helpers (FDM+)

A variation on FDM that allows launch entities to supply markup language specific helpers to do their own parsing. The parser may thus delegate entire description sections to these helpers, which may or may not delegate back to the parser (e.g. for nested arbitrary launch entities).

> FDM 的一种变体，允许启动实体提供标记语言特定的帮助程序来进行自己的解析。因此，解析器可以将整个描述部分委托给这些助手，这些助手可以委托也可以不委托回解析器（例如，对于嵌套的任意启动实体）。

#### Description Markup

Some description samples in different markup languages are provided below:

> 下面提供了一些不同标记语言的描述示例：

_XML_

> _XML 语言_

```xml
    <launch version="x.1.0">
      <executable name="some-action" cmd="$(find-exec my-process)" prefix="$(env LAUNCH_PREFIX)">
         <env name="LD_LIBRARY_PATH" value="/opt/dir:$(env LD_LIBRARY_PATH)"/>
         <on_exit>
            <log message="I'm done"/>
         </on_exit>
      </executable>
    </launch>
```

_YAML_

> _亚马尔_

```yml
launch:
  version: x.1.0
  children:
    - executable:
        cmd: $(find-exec my-process)
        env:
          LD_LIBRARY_PATH: "/opt/dir:$(env LD_LIBRARY_PATH)"
        prefix: $(env LAUNCH_PREFIX)
        on_exit:
          - log: "I'm done"
```

#### Parsing Hooks

Some code samples in different programming languages are provided below:

> 下面提供了不同编程语言的一些代码示例：

_Python_

> _Python 语言_

```python
    @launch_markup.xml.handle_tag('process')
    def process_tag_helper(xml_element, parser):
        ...
        return launch.actions.ExecuteProcess(...)

    @launch_markup.handle_subst('env')
    def env_subst_helper(args, parser):
        ...
        return launch.substitutions.EnvironmentVariable(parser.parse(args[0]))
```

_C++_

> _C++_

```c++
    std::unique_ptr<launch::actions::ExecuteProcess>
    process_tag_helper(const XmlElement & xml_element, const launch_markup::xml::Parser & parser) {
        // ...
        return std::make_unique<launch::actions::ExecuteProcess>(/* ... */);
    };

    LAUNCH_XML_MARKUP_HANDLE_TAG("process", process_tag_helper);

    std::unique_ptr<launch::substitutions::EnvironmentVariable>
    env_subst_helper(const std::vector<std::string> & args, const launch_markup::Parser & parser) {
        // ...
        return std::make_unique<launch::substitutions::EnvironmentVariable>(parser.parse(args[0]));
    }

    LAUNCH_MARKUP_HANDLE_SUBST("env", env_subst_helper);
```

#### Advantanges & Disadvantages

_+_ Straightforward to implement.

> *+*实施起来直截了当。

_-_ Launch system implementations are aware of the parsing process, being completely involved with it if sugars are to be provided.

_+_ Allows leveraging the strengths of each markup language.

> *+*允许利用每种标记语言的优势。

_-_ Opens the door to big differences in the representation of launch entities across different front end implementations, and even within a given one by allowing the users to introduce multiple custom representations for the same concept (e.g. a list of numbers).

_-_ Care must be exercised to avoid coupling static descriptions with a given implementation.

### Abstract Description Parsing (ADP)

In ADP, the parser provides an abstract interface to the static description and delegates parsing and instantiation to hooks registered by the implementation. The parser does not attempt any form of description inference, traversing the description through of the provided hooks.

> 在 ADP 中，解析器为静态描述提供了一个抽象接口，并将解析和实例化委托给实现注册的钩子。解析器不尝试任何形式的描述推理，通过提供的钩子遍历描述。

#### Description Markup

Some description samples in different markup languages are provided below:

> 下面提供了一些不同标记语言的描述示例：

_XML_

> _XML 语言_

```xml
    <launch version="x.1.0">
      <executable name="some-action" cmd="$(find-exec my-process)" prefix="$(env LAUNCH_PREFIX)">
         <env name="LD_LIBRARY_PATH" value="/opt/dir:$(env LD_LIBRARY_PATH)"/>
         <on_exit>
          <log message="I'm done"/>
        </on_exit>
      </executable>
    </launch>
```

_YAML_

> _亚马尔_

```yaml
    launch:
      version: x.1.0
      children:
        - executable:
            cmd: $(find-exec my-process)
            env:
              LD_LIBRARY_PATH: "/opt/dir:$(env LD_LIBRARY_PATH)"
            prefix: $(env LAUNCH_PREFIX)
            on_exit:
              - log:
                  message: 'I'm done'
```

#### Abstract Description

To be able to abstract away launch descriptions written in conceptually different markup languages, the abstraction relies on the assumption that all launch system implementations are built as object hierarchies. If that holds, then ultimately all domain specific schemas and formats will just be different mappings of said hierarchy. Therefore, a hierarchical, object-oriented representation of the markup description can be built.

> 为了能够抽象出用概念上不同的标记语言编写的启动描述，抽象依赖于所有启动系统实现都构建为对象层次结构的假设。如果这种情况成立，那么最终所有特定于域的模式和格式都将是所述层次结构的不同映射。因此，可以构建标记描述的层次化、面向对象的表示。

Define an `Entity` object that has:

> 定义一个“实体”对象，该对象具有：

- a namespaced type;
- optionally a name, unique among the others;
- optionally a parent entity (i.e. unless it's the root entity);
- optionally one or more named attributes, whose values can be entities, ordered sequences of them or neither e.g. scalar values.

Some sample definitions in different programming languages are provided below:

> 下面提供了不同编程语言中的一些示例定义：

_Python_

> _Python 语言_

```python
    class Entity:

        @property
        def type_name(self):
            pass

        @property
        def parent(self):
            pass

        @property
        def children(self):
            pass

        def get_attr(
          self,
          name,
          *,
          data_type=str,
          optional=False
        ):
            pass
```

_C++_

> _C++_

```c++
    namespace parsing {
    class Entity {

      const std::string & type() const;

      const Entity & parent() const;

      const std::vector<Entity> & children() const;

      template<typename T>
      const T & get(const std::string& name) const { ... }

      const Entity & get(const std::string& name) const { ... }

      template<typename T>
      bool has(const std::string & name) const { ... }

      bool has(const std::string & name) const { ... }
    };
    }  // namespace parsing
```

It is up to each front end implementation to choose how to map these concepts to the markup language. For instance, one could map both of the following descriptions:

> 如何将这些概念映射到标记语言取决于每个前端实现。例如，可以映射以下两种描述：

_XML_

> _XML 语言_

```xml
    <node name="my-node" pkg="demos" exec="talker">
      <param name="a" value="100."/>
      <param name="b" value="stuff"/>
    </node>
```

_YAML_

> _亚马尔_

```yaml
- node:
    name: my-node
    pkg: demos
    exec: talker
    param:
      - name: a
        value: 100.
      - name: b
        value: stuff
```

such that their associated parsing entity `e` exposes its data as follows:

> 使得它们相关联的解析实体“e”如下公开其数据：

_Python_

> _Python 语言_

```python
    e.type_name == 'node'
    e.get_attr(name) == 'my-node'
    e.get_attr('pkg', optional=True) != None
    params = e.get_attr('param', data_type=List[Entity])
    params[0].get_attr('name') == 'a'
    params[1].get_attr('name') == 'b'
```

_C++_

> _C++_

```c++
    e.type() == "node"
    e.get<std::string>("name") == "my-node"
    e.has<std::string>("pkg") == true
    auto params = e.get<std::vector<parsing::Entity>>("param")
    params[0].get<std::string>("name") == "a"
    params[1].get<std::string>("name") == "b"
```

Inherent ambiguities will arise from the mapping described above, e.g. nested tags in an XML description may be understood either as children (as it'd be the case for a grouping/scoping action) or attributes of the enclosing tag associated entity (as it's the case in the example above). It is up to the parsing procedures to disambiguate them.

> 上述映射将产生固有的模糊性，例如，XML 描述中的嵌套标签可以被理解为子级（如分组/作用域动作的情况）或封闭标签相关实体的属性（如上例中的情况）。这取决于解析过程来消除它们的歧义。

#### Parsing Delegation

Each launch entity that is to be statically described must provide a parsing procedure.

> 要静态描述的每个启动实体都必须提供一个解析过程。

_Python_

> _Python 语言_

```python
    @launch.frontend.expose_action('some-action')
    class SomeAction(launch.Action):

        @classmethod
        def parse(
            cls,
            entity: launch.frontend.Entity,
            parser: launch.frontend.Parser
        ) -> SomeAction:
            return cls(
                scoped=parser.parse_substitution(entity.get_attr(scoped)), entities=[
                    parser.parse_action(child) for child in entity.children
                ]
            )

    @launch.frontend.expose_substitution('some-subst')
    class SomeSubstitution(launch.Substitution):

        @classmethod
        def parse(
            cls,
            data: Iterable[SomeSubstitutionsType]
        ) -> SomeSubstitution:
            return cls(*data)
```

_C++_

> _C++_

```c++
    class SomeAction : public launch::Action {
      static std::unique_ptr<SomeAction>
      parse(const parsing::Entity & e, const parsing::Parser & parser) {
        std::vector<launch::DescriptionEntity> entities;
        for (const auto & child : e.children()) {
          entities.push_back(parser.parse(child));
        }
        return std::make_unique<SomeAction>(
          parser.parse(e.get<parsing::Value>("scoped")), entities
        );
      }
    };

    LAUNCH_EXPOSE_ACTION("some-action", SomeAction);

    class SomeSubstitution : public launch::Substitution {
      static std::unique_ptr<SomeSubstitution>
      parse(const parsing::Value & v, const parsing::Parser & parser) {
        return std::make_unique<SomeSubstitution>(v.as<std::vector<std::string>>());
      }
    };

    LAUNCH_EXPOSE_SUBSTITUTION("some-subst", SomeSubst);
```

As can be seen above, procedures inspect the description through the given parsing entity, delegating further parsing to the parser recursively. To deal with substitutions, and variant values in general, the concept of a 'value' is introduced. Note that a value _may be_ an entity, but it isn't necessarily one.

> 如上所述，过程通过给定的解析实体检查描述，递归地将进一步的解析委托给解析器。为了处理替换和一般的变量值，引入了“值”的概念。请注意，值\_ma 可能是一个实体，但不一定是一个。

#### Procedure Provisioning

##### Manual Provisioning

In the simplest case, the user may explicitly provide their own parsing procedure for each launch entity.

> 在最简单的情况下，用户可以明确地为每个启动实体提供他们自己的解析过程。

##### Automatic Derivation

If accurate type information is somehow available (e.g.: type annotations in constructor), reflection mechanisms can aid derivation of a parsing procedure with no user intervention.

> 如果以某种方式可以获得准确的类型信息（例如：构造函数中的类型注释），则反射机制可以帮助推导解析过程，而无需用户干预。

#### Advantages & Disadvantages

The abstraction layer allows.

> 抽象层允许。

_-_ Launch system implementations are aware of the parsing process.

_+_ The static description abstraction effectively decouples launch frontends and backends, allowing for completely independent development and full feature availability at zero cost.

> *+*静态描述抽象有效地将发布前端和后端解耦，允许以零成本进行完全独立的开发和全功能可用性。

_-_ No markup language specific sugars are possible.

REVISIT(hidmic): IMHO explicitly disallowing this is a good thing, it makes for more homogeneus descriptions and avoids proliferation of multiple representation of the same concepts (e.g. a list of strings).

> 修订（隐藏）：IMHO 明确禁止这是一件好事，它有助于更同质的描述，并避免相同概念的多种表示形式（例如字符串列表）的扩散。

_+_ The transfer function nature of the parsing procedure precludes the need for a rooted object type hierarchy in statically typed launch system implementations.

> *+*解析过程的传递函数性质排除了在静态类型的启动系统实现中对根对象类型层次结构的需要。

_-_ Automatic parsing provisioning requires accurate type information, which may not be trivial to gather in some implementations.

_-_ Harder to implement.
