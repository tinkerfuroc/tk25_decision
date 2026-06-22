# tk25_decision — Python Environment Setup

> Part of the tk25_ws workspace. Full safety doctrine, prerequisites, untracked-sidecar staging, and
> cross-module build order live in the canonical guide: `src/tk25_basic/docs/ENV_SETUP.md` (tk25_basic repo).

This module is the behavior-tree decision layer (`behavior_tree`, `py_trees`-based). It has exactly
one Python environment, `.venv_decision`, which is **CPU-only** (no torch, no CUDA at runtime).

## Safety doctrine (summary)

Never modify the system python (`/usr/bin/python3`), the user python (`~/.local`), or `pip --user`.
Every interpreter and every install routes through **`uv`** (which provisions its own managed CPython
and builds isolated venvs) or through an explicitly-named venv interpreter. Always
`export UV_PYTHON_PREFERENCE=only-managed` so uv never reuses the system python as a venv base, and
**never** run a bare `pip install` — use `uv pip install --python <venv>/bin/python <pkg>` or
`( source <venv>/bin/activate && pip install <pkg> )`. Full rationale and the
"nothing leaked into system python" audit live in the hub guide
(`src/tk25_basic/docs/ENV_SETUP.md` §1, §8).

## Prerequisites (this module)

- **uv** (source uses 0.10.8; any 0.10.x) on `PATH`, with `export UV_PYTHON_PREFERENCE=only-managed`.
- **ROS 2 Humble at exactly `/opt/ros/humble`** (apt). The `ros2_packages.pth` shim that
  `restore_venvs.sh` writes hardcodes this prefix + python3.10, and `py_trees_ros` / `rclpy` are
  supplied from there (they are deliberately **not** in this venv's lock).
- **System `/usr/bin/python3` == 3.10** — `tkbuild` aborts if the venv python-minor differs from the
  system python-minor. Ubuntu 22.04 + Humble = 3.10.
- **No GPU / CUDA needed.** `.venv_decision` is CPU-only; do not add torch here.

## Environment(s)

### `src/tk25_decision/.venv_decision` — Python 3.10.12 — CPU

- **Sidecar tracked on a fresh clone?** **No — UNTRACKED.** The sidecar dir
  `src/tk25_decision/.venv_decision.uv-project/` (`pyproject.toml` + `uv.lock` + `.python-version`) is
  git-untracked, so a fresh `git clone` does **not** contain it and `restore_venvs.sh` will silently
  `[skip]` this venv. **Stage it from the source machine first** (see hub guide §3, "Stage the 6
  untracked sidecars"):
  ```bash
  rsync -a tinker@SRC_HOST:/home/tinker/tk25_ws/src/tk25_decision/.venv_decision.uv-project/ \
           /home/tinker/tk25_ws/src/tk25_decision/.venv_decision.uv-project/
  ```
- **Handled by `restore_venvs.sh`?** **Yes** (once the sidecar is staged) — it is in the script's
  `VENVS` list.
- **Restore:**
  ```bash
  export UV_PYTHON_PREFERENCE=only-managed
  bash /home/tinker/tk25_ws/src/tk25_basic/tools/restore_venvs.sh .venv_decision
  ```
  Explicit equivalent (does the same `uv sync --frozen`, but does **not** write the ROS shim — re-run
  the script once afterward, it is idempotent):
  ```bash
  UV_PROJECT_ENVIRONMENT=/home/tinker/tk25_ws/src/tk25_decision/.venv_decision \
    uv sync --project /home/tinker/tk25_ws/src/tk25_decision/.venv_decision.uv-project \
            --no-install-project --frozen
  ```
- **Unlockables:** none. There is no `unlockable.txt`; `uv sync --frozen` installs the full set.
- **Editable/git sources needing a local tree:** none. The lock is all PyPI wheels (pure-python deps
  plus a numpy/scipy/matplotlib stack and transitive `nvidia-*`/`cuda-*` wheels that are never used at
  runtime — this venv is CPU-only). No directory or git dependencies.
- **Verify** (venv-scoped):
  ```bash
  /home/tinker/tk25_ws/src/tk25_decision/.venv_decision/bin/python \
    -c "import py_trees, numpy, openai; print('py_trees', py_trees.__version__, 'numpy', numpy.__version__)"
  # expect: py_trees 2.4.0  numpy 1.26.4
  ```
  `py_trees_ros` and `rclpy` are **not** in this venv — they resolve from `/opt/ros/humble` through the
  `ros2_packages.pth` shim, so verify them only after sourcing ROS Humble (or relying on the shim):
  ```bash
  ROS2_PTH_WARNED=1 /home/tinker/tk25_ws/src/tk25_decision/.venv_decision/bin/python \
    -c "import rclpy, py_trees_ros; print('ros bindings OK')"
  ```
- **Gotchas (module/venv-specific):**
  - Pure-Python decision layer — **no torch, no CUDA** at runtime. Do not add torch to this venv.
    Keep `numpy` at 1.26.4 and `py_trees` at 2.4.0.
  - The lock is linux/x86_64-only (`environments = ["sys_platform == 'linux' and platform_machine ==
    'x86_64'"]`); `--frozen` refuses to re-resolve elsewhere.
  - Non-mock operation needs the Tinker message packages (`tinker_vision_msgs_26`, `tinker_arm_msgs`,
    `tinker_audio_msgs`, `tinker_nav_msgs`) built/sourced from their own sub-workspaces. If absent, the
    behavior tree auto-falls back to **mock mode** (`BT_MOCK_MODE`; see `CLAUDE.md` §Mock Mode) — that
    is by design, not a venv failure.
  - A pre-existing import break is documented in `src/tk25_decision/CLAUDE.md` (§Known issues):
    `from tinker_audio_msgs.srv import TTSCnRequest, ...` can raise
    `ImportError: cannot import name 'TTSCnRequest'` when an older `tinker_audio_msgs` is installed.
    This is a message-package version mismatch, **unrelated to this venv's setup** — rebuild
    `tinker_audio_msgs` from newer source to clear it.

## Build

`.venv_decision` must be restored before building (`tkbuild` reads `<venv>/bin/python` and aborts if
it is missing). Then:

```bash
cd /home/tinker/tk25_ws
./tkbuild tk25_decision        # colcon build for behavior_tree under .venv_decision + venv-python shebangs
source /home/tinker/tk25_ws/install/setup.bash
```

`tkbuild` maps `[tk25_decision]=".venv_decision"`, activates that venv, strips `--symlink-install`, and
re-shebangs the install-tree entry points (`receptionist`, `GPSR`, `store-groceries`,
`help-me-carry`, etc.) at the venv python. **Never** substitute a raw
`colcon build --base-paths src/tk25_decision/src --symlink-install` — that writes `#!/usr/bin/python3`
shebangs that can't see venv-only deps (e.g. `openai`) plus orphaned entry points.

**COLCON_IGNORE mv-aside workaround (conditional).** A root `COLCON_IGNORE` (or a
`src/tk25_decision/COLCON_IGNORE`) makes `tkbuild` silently skip this sub-ws. Neither is present by
default, so a fresh clone builds fine. **Only** if a sentinel has been re-created, move it aside for
the build and restore it after:

```bash
mv src/tk25_decision/COLCON_IGNORE src/tk25_decision/COLCON_IGNORE.tkbuild-temp-aside
./tkbuild tk25_decision
mv src/tk25_decision/COLCON_IGNORE.tkbuild-temp-aside src/tk25_decision/COLCON_IGNORE
```

`tk25_decision` is also built in order by `./tkbuild -a` (build order:
`tk_24_audio  tk25_basic  tk25_decision  tk26_navigation  tk25_manipulation  tk26_vision  tk26_sim`).
