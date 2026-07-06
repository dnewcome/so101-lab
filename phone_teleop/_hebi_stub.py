"""Make lerobot's Android/WebXR phone teleop importable without HEBI Core.

lerobot's `teleop_phone.py` does a module-level `import hebi` whenever the
`hebi-py` distribution is installed, and `hebi-py` needs a native "HEBI Core"
shared library that isn't present on a normal Linux box — yet only the *iOS*
phone path actually uses hebi. The Android/WebXR path (what the Quest browser
uses) doesn't touch it.

So: we keep `hebi-py` installed (its distribution metadata satisfies lerobot's
`require_package("hebi-py")` guard), and pre-load a lightweight stub `hebi`
module here so the real `import hebi` never loads the native lib.

Import this module BEFORE anything under `lerobot.teleoperators.phone`.
"""

import importlib.machinery
import sys
import types

if "hebi" not in sys.modules:
    _stub = types.ModuleType("hebi")
    _stub.__spec__ = importlib.machinery.ModuleSpec("hebi", loader=None)
    sys.modules["hebi"] = _stub
