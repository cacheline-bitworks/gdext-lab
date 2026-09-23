


# GDExtension Gotchas

A running list of problems we hit while building GDExtensions, why they happen,
and how to fix them. If you're new to GDExtension, read this before you
dive into the code.

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

- The file isn't in the path Godot scans (it must live under `res://`)
- A stale `.godot/extension_list.cfg` doesn't reference it
- A `.gdignore` file in a parent folder excludes it from the scan

**Fix**

1. Confirm Godot knows about the file:

   ```bash
   cat project/.godot/extension_list.cfg