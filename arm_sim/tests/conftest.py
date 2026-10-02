import os

# Headless rendering backend for any test that touches mujoco.Renderer (Linux CI). macOS uses CGL.
if os.uname().sysname == "Linux":
    os.environ.setdefault("MUJOCO_GL", "egl")
