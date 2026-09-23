# gdext-lab

A laboratory for Godot GDExtension patterns.

Experiments with C++ ↔ GDScript ↔ C# interop, threading, packed data, and the
production tooling that makes GDExtensions actually usable in real projects.

This is a working reference — every class in `src/` demonstrates a real
technique, and every gotcha we hit is documented in [`docs/gotchas.md`](./docs/gotchas.md).

## What's Inside

- **Working GDExtension build pipeline** — `SConstruct` compiles `src/` into `addons/gdext_lab/bin/`
- **C++ ↔ GDScript interop** — properties, methods, signals, `PackedByteArray` batching
- **Threading with `WorkerThreadPool`** — safe parallel computation without touching the scene tree
- **C# bindings** — via the [C# GDExtension Bindgen](https://github.com/gilzoide/godot-csharp-gdextension-bindgen) plugin
- **Documented gotchas** — [`docs/gotchas.md`](./docs/gotchas.md) has every trap we hit, and how to fix it

## Demo Classes

| Class | Base | Demonstrates |
|---|---|---|
| `ExampleClass` | `RefCounted` | Minimal class with a bound method and a `Variant` parameter |
| `FrameData` | `Node` | `PackedByteArray` batching + `PackedFloat32Array`, plus `WorkerThreadPool` group tasks |

## Requirements

- **Godot** 4.4.x (Mono recommended if you want C# support)
- **Python** 3.8+
- **SCons** 4.5+ (`pip install scons` or `apt install scons`)
- **C++17 compiler** — Clang 14+, GCC 11+, or MSVC 2022+

## Quick Start

```bash
# 1. Clone with submodules
git clone --recursive https://github.com/cacheline-bitworks/gdext-lab.git
cd gdext-lab

# 2. Build the extension
scons

# 3. Link the extension into the demo project
cd demo
mkdir -p addons
ln -s ../../addons/gdext_lab addons/gdext_lab
cd ..

# 4. Open the demo in Godot
godot --editor --path demo --rendering-driver opengl3