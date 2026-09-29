This folder defines a prebuilt dev container that gives every contributor the
complete litearm-pybullet simulation environment — Python, PyBullet, the test
tooling, and a browser view of the simulation window — with nothing to install
on the host except Docker and VS Code. Read this guide before you first open
the repository in a container.

## 1. Prerequisites

- Docker Desktop (Windows, macOS) or Docker Engine (Linux)
- VS Code with the Dev Containers extension (`ms-vscode-remote.remote-containers`)
- This repository cloned locally

Do not install an X server (VcXsrv, XQuartz) on the host. The container runs
its own virtual display and serves it to your browser over noVNC.

Note for Windows hosts: pybullet publishes no Windows wheels on PyPI, so a
native `pip install` builds it from source and needs a C++ toolchain. The
container installs the Linux wheel, which is why it is the easy path on
Windows.

## 2. Open the repository in the container

From VS Code: F1 → **Dev Containers: Reopen in Container**.

The first start builds the image, which installs the project dependencies and
the display stack — a few minutes on a typical machine. VS Code caches the
image afterwards, and `cacheFrom` in `devcontainer.json` reuses the image
published to GitHub Container Registry, so later builds only pick up changes.

When the terminal prompt appears, the package is already installed editable
from the mounted workspace. Confirm with:

```bash
cd /workspaces/litearm-pybullet
python -c "import litearm_pybullet; print(litearm_pybullet.__file__)"
```

The printed path starts with `/workspaces/litearm-pybullet/src/`.

## 3. See the simulation

The container starts a virtual display (Xvfb), a VNC server, and a noVNC proxy
on port 6080 — automatically, on every container start. VS Code forwards the
port and shows a notification.

Open <http://localhost:6080/vnc.html> and click **Connect**. Then, in the
VS Code terminal:

```bash
cd /workspaces/litearm-pybullet
python examples/01_hello_sim.py
```

The PyBullet window appears in the browser tab. Examples 02 and 03 work the
same way. If the page does not open, run
`bash .devcontainer/start-vnc.sh` once and retry; the script is idempotent.

## 4. Run the tests

All tests run headless (`render=False`) and need no display:

```bash
cd /workspaces/litearm-pybullet
python -m pytest tests/ -v
```

The same command is available as the VS Code task **test** (Terminal → Run
Task).

## 5. Mirror and dual-control mode

Modes 2 and 3 need `litearm-core`, which is not on PyPI, so the image does not
install it. Clone the SDK inside the container and install it editable:

```bash
cd /tmp
git clone https://github.com/nexform-tech/litearm-core.git
pip install --user -e /tmp/litearm-core
```

The real arm also needs its USB CDC port. Docker Desktop on Windows and macOS
cannot pass USB devices into containers, so the mirror and dual-control
examples only reach hardware from a Linux host — there, add the device to
`runArgs` in `devcontainer.json`:

```json
"runArgs": ["--shm-size=1g", "--device=/dev/ttyACM0"]
```

On Windows and macOS, run modes 2 and 3 on the host with a host-installed
`litearm-core`, or stay with the standalone simulation examples.

## 6. What the image contains

| Component | Installed |
|---|---|
| Python 3.11 | at image build |
| `pybullet`, `numpy`, `pytest`, `pytest-timeout` | at image build, mirroring `pyproject.toml` |
| PyBullet GUI libraries (Mesa, X11) | at image build |
| Xvfb, x11vnc, noVNC display stack | at image build |
| the `litearm_pybullet` package, editable | on container creation (`postCreateCommand`) |
| GitHub CLI (`gh`) | on container creation (devcontainer feature; the repository rules require it for PRs) |

If `pyproject.toml` changes dependencies, rebuild the container: F1 →
**Dev Containers: Rebuild Container**.

## 7. Prebuilt image on GitHub Container Registry

`.github/workflows/devcontainer.yml` builds this Dockerfile on every push to
`main` and publishes `ghcr.io/nexform-tech/litearm-pybullet-dev:latest`.
`devcontainer.json` uses that image as build cache. To pull the published
image instead of building locally, replace the `build` block in
`devcontainer.json` with:

```json
"image": "ghcr.io/nexform-tech/litearm-pybullet-dev:latest"
```

Do not do that while you are working on the container definition itself — an
`image` reference ignores local Dockerfile changes.

## 8. Use the image without VS Code

The image is plain Docker. From the repository root, on Linux or macOS:

```bash
docker run -it --rm -p 6080:6080 \
  -v "$PWD:/workspaces/litearm-pybullet" \
  ghcr.io/nexform-tech/litearm-pybullet-dev:latest bash -lc \
  "pip install --user --no-deps -e /workspaces/litearm-pybullet && \
   /workspaces/litearm-pybullet/.devcontainer/start-vnc.sh && bash"
```

On Windows PowerShell, write `${PWD}` instead of `$PWD`.

## Verification status

The image build was not verified on the machine that wrote this guide (no
Docker there); the configuration follows the standard devcontainer format,
and every external reference in it (base image tag, apt package names, the
gh CLI devcontainer feature) was checked against its registry. The test
suite passes outside the container on Python 3.11 with pybullet 3.2.5:
`python -m pytest tests/ -v` → 149 passed, 5 skipped.
