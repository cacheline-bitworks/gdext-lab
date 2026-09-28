# GDExtension Gotchas

A running list of problems we hit while building GDExtensions, why they happen,
and how to fix them. If you're new to GDExtension, read this before you dive
into the code.

Each entry follows the same structure: **Symptom**, **Cause**, **Fix**.

---

## 1. Godot Silently Ignores the Extension

**Symptom**

You build successfully. The `.so` exists. Godot starts, but your classes never
appear in GDScript, and the editor's Output panel mentions nothing about the
extension. No error, no warning.

**Cause**

The `.gdextension` file is present but Godot never tried to load it. This
usually means one of:

- The file is not in a path Godot scans (it must live under `res://`)
- A stale `.godot/extension_list.cfg` does not reference it
- A `.gdignore` file in a parent folder excludes it from the scan

**Fix**

Confirm Godot knows about the file:

    cat project/.godot/extension_list.cfg

It should contain a line like `res://addons/your_extension/your_extension.gdextension`.

If empty, delete `.godot/` and reopen the project. Godot rescans on startup.

Check for a stray `.gdignore`:

    find . -name ".gdignore" -not -path "./godot-cpp/*"

Any `.gdignore` in a parent of your `.gdextension` file will exclude it.

---

## 2. No GDExtension library found for current OS and architecture

**Symptom**

    ERROR: No GDExtension library found for current OS and architecture (linux.x86_64)
           in configuration file: res://addons/your_extension/your_extension.gdextension
       at: parse_gdextension_file (core/extension/gdextension_library_loader.cpp:359)

**Cause**

The `[libraries]` section of the `.gdextension` file uses keys that do not match
Godot's expected feature tag format. Godot 4.4+ expects:

    <platform>.<arch>.<precision>.<target>

For example:

    linux.x86_64.single.debug
    linux.x86_64.single.release

Common mistakes:

- Wrong order: `linux.debug.x86_64.single` instead of `linux.x86_64.single.debug`
- Missing precision: `linux.debug.x86_64` instead of `linux.x86_64.single.debug`
- Wrong precision: using `double` when the `.so` was built single-precision
- Using `res://` paths: Godot resolves `[libraries]` paths relative to the
  `.gdextension` file, not the project root

**Fix**

Match the format of a working `.gdextension` file exactly. The template's
original file is the best reference:

    git show HEAD:project/bin/example.gdextension

Copy its key structure verbatim. Only replace the file names.

---

## 3. undefined symbol: _ZTVClassName

**Symptom**

Godot fails to load the extension with:

    Can't open dynamic library: ... undefined symbol: _ZTV11PlayerStats

The `_ZTV` prefix is the Itanium ABI's mangled name for a vtable.

**Cause**

A class with virtual functions was declared without an explicit constructor
and destructor. The compiler never emits the vtable because there is no
"key function" anchoring it to a translation unit.

**Fix**

Every Godot class needs:

    ClassName() = default;
    ~ClassName() override = default;

Even if they do nothing. The destructor declaration forces the vtable to be
emitted.

If that does not work, declare the destructor out-of-line:

    // In the header:
    ~ClassName() override;

    // In the .cpp:
    ClassName::~ClassName() {}

---

## 4. undefined symbol for a method you wrote

**Symptom**

Link error mentioning a method you know you declared.

**Cause**

You declared a method in the header but never defined it in the `.cpp`. C++
compiles each translation unit independently. A declaration alone is a
promise, not a fulfillment.

**Fix**

Grep your headers for all method declarations and verify each one has a
definition. Especially common with virtual overrides like `_ready`,
`_process`, `_physics_process`. Easy to declare in the header and forget
the body.

---

## 5. Callable Cannot Carry Raw Pointers

**Symptom**

    error: incomplete type 'godot::GetTypeInfo<float*, void>'
           used in nested name specifier

**Cause**

`Callable` marshals all arguments through `Variant`. `Variant` has no concept
of C++ pointers, so any bound argument that is a raw pointer (`float*`,
`MyStruct*`, etc.) fails to compile.

**Fix**

Do not pass pointers through `Callable`. Store them as member state for the
duration of the task:

    // Before submitting the task:
    m_parallel_buffer = reinterpret_cast<float*>(out.ptrw());
    m_parallel_time = m_time;

    // Submit -- the worker reads from members, not bound args:
    int task_id = pool->add_group_task(
        callable_mp(this, &Class::_parallel_worker),
        m_point_count
    );

    // Wait, then clear:
    pool->wait_for_group_task_completion(task_id);
    m_parallel_buffer = nullptr;

The worker's signature receives only `Variant`-compatible arguments (like
`int p_index`). Everything else comes from `this`.

---

## 6. GDCLASS Must Match the C++ Inheritance

**Symptom**

Cryptic runtime errors, silent registration failure, or crashes when Godot
tries to call methods on your object.

**Cause**

`GDCLASS(ClassName, ParentClass)` generates type-erased callbacks that assume
the class inherits from `ParentClass`. If they disagree, Godot calls methods
at the wrong offset in the vtable.

**Fix**

The macro must match the C++ declaration exactly:

    class PlayerStats : public Node {   // Node
        GDCLASS(PlayerStats, Node)       // must also be Node

If you change the inheritance, update `GDCLASS` in the same commit.

---

## 7. Version Alignment (Three-Way Match)

**Symptom**

Extension loads on one machine but not another. Or it loads in the editor but
crashes when the game runs. Or classes appear but methods return garbage.

**Cause**

Three versions must align:

| Component | Value |
|---|---|
| Godot editor | 4.4.1 Mono |
| godot-cpp branch | 4.4 |
| compatibility_minimum in .gdextension | "4.4" |

Mismatches cause silent ABI-level failures that do not produce useful error
messages.

**Fix**

- Pin `godot-cpp` to a specific branch via `.gitmodules`
- Set `compatibility_minimum` to match the `godot-cpp` branch
- Never mix debug extensions with release editors (or vice versa). This
  causes heap corruption that shows up as intermittent crashes

---

## 8. Threading: Never Touch the Scene Tree from a Worker

**Symptom**

Random segfaults after adding threading. Crashes that only occur on certain
machines. "Can't add child, already has a parent" errors that make no sense.

**Cause**

A `WorkerThreadPool` task called `add_child`, `queue_free`, `set_position`,
or any scene-tree method. The scene tree is not thread-safe and Godot does
not lock it for you.

**Fix**

Workers compute. Main thread applies.

- Workers: pure computation on C++ data structures
- Main thread: read results after `wait_for_group_task_completion`, then touch
  the scene tree
- If a worker must trigger a scene change, use `call_deferred` to queue it
  for the main thread's next idle frame

---

## 9. GPU Compatibility: Vulkan / Forward+ Crash on Old Hardware

**Symptom**

Godot 4 crashes on launch with a Vulkan device lost error:

    ERROR: Vulkan device was lost. This could be due to a driver issue,
           a hardware issue, or the driver resetting itself.

Or the editor opens but the game process crashes immediately.

**Cause**

Godot 4's Forward+ renderer uses Vulkan. Older Intel integrated GPUs (and
some older AMD/NVIDIA cards) cannot handle it reliably.

**Fix**

Run with the Compatibility renderer:

    godot --rendering-driver opengl3

And set it permanently in `project.godot`:

    [rendering]

    renderer/rendering_method="gl_compatibility"
    renderer/rendering_method.mobile="gl_compatibility"

The editor launch and the game launch must both use `opengl3`. Setting it
only for the editor is not enough, because the game process is a separate
launch that reads `project.godot`.

---

## 10. SCons Does Not Rebuild After a Header Change

**Symptom**

You edited a `.h` file, ran `scons`, and nothing recompiled. The change
does not take effect in the game.

**Cause**

SCons's dependency tracking occasionally misses header changes, especially
when the change is in a header included indirectly.

**Fix**

Force a clean rebuild of your own sources:

    rm -f src/*.os
    scons

This does not rebuild `godot-cpp` (which is slow). Only your files.

---

## 11. Special Character Corruption in Terminal Pastes

**Symptom**

Content pasted through terminal heredocs shows up with mojibake characters
instead of em-dashes or other Unicode punctuation.

**Cause**

Terminal encoding mismatches, especially when content contains Unicode
punctuation or emoji.

**Fix**

Use a file editor (VS Code, nano, vim) instead of terminal heredocs for
files containing non-ASCII characters. Or, better: keep the content
ASCII-only when possible.

---

---

## Adding New Entries

When you hit something that takes more than 15 minutes to figure out, add an
entry here. Structure:

**Symptom** -- the exact error, where it appeared, what you were doing.

**Cause** -- your best understanding, even if approximate.

**Fix** -- what actually resolved it.

The symptom is the most important part. That is what someone will search for.
