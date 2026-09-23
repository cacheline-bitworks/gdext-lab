#!/usr/bin/env python
"""
SConstruct for the gdext-lab GDExtension.

Builds C++ source from src/ into addons/gdext_lab/bin/<platform>/.
"""

import os
import sys

libname = "gdext_lab"

# ----------------------------------------------------------------------------
# 1. Load the godot-cpp build environment
# ----------------------------------------------------------------------------
env = SConscript("godot-cpp/SConstruct")

# ----------------------------------------------------------------------------
# 2. Include paths and source files
# ----------------------------------------------------------------------------
env.Append(CPPPATH=["src/"])
sources = Glob("src/*.cpp")

# Optional documentation XML (only in editor/debug builds)
if env["target"] in ["editor", "template_debug"]:
    try:
        doc_data = env.GodotCPPDocData("src/gen/doc_data.gen.cpp", source=Glob("doc_classes/*.xml"))
        sources.append(doc_data)
    except AttributeError:
        pass  # pre-4.3 baseline

# ----------------------------------------------------------------------------
# 3. Build the shared library, targeting the addons/ folder
# ----------------------------------------------------------------------------
suffix = env["suffix"].replace(".dev", "").replace(".universal", "")
lib_filename = "{}{}{}{}".format(
    env.subst("$SHLIBPREFIX"),
    libname,
    suffix,
    env.subst("$SHLIBSUFFIX"),
)

library = env.SharedLibrary(
    "addons/gdext_lab/bin/{}/{}".format(env["platform"], lib_filename),
    source=sources,
)

Default(library)
