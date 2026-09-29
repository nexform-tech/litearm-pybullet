# Development image for litearm-pybullet. Everything the simulation needs is
# installed at build time (pre-built): Python 3.11, the project dependencies,
# the test tooling, the PyBullet GUI libraries, and a virtual display served
# over VNC/noVNC so render=True shows up in the host browser.
#
# Keep the Python version and the pip list below in sync with pyproject.toml
# (requires-python, [project].dependencies, [project].optional-dependencies.dev).

FROM mcr.microsoft.com/devcontainers/python:1-3.11-bookworm

# PyBullet's OpenGL window needs Mesa (software rendering via llvmpipe) and the
# X11 client libraries. xvfb + x11vnc + novnc/websockify serve that window to a
# browser; x11-utils provides xdpyinfo, used by start-vnc.sh.
#
# The base image ships a yarn apt repository without its signing key, which
# makes `apt-get update` fail (NO_PUBKEY 62D54FD4003F6525). This image does not
# use yarn, so drop the repo before updating.
RUN rm -f /etc/apt/sources.list.d/*yarn* \
    && apt-get update && export DEBIAN_FRONTEND=noninteractive \
    && apt-get install -y --no-install-recommends \
        libgl1 libgl1-mesa-dri libegl1 libglib2.0-0 \
        libx11-6 libxext6 libxrender1 libxrandr2 libxi6 \
        libxcursor1 libxinerama1 libsm6 libice6 libxkbcommon0 \
        libfreetype6 fonts-dejavu mesa-utils \
        xvfb x11vnc novnc websockify x11-utils \
    && rm -rf /var/lib/apt/lists/*

# Project dependencies at build time, so the container starts ready to run.
# This list mirrors [project].dependencies and the dev extra in pyproject.toml;
# the [mirror] extra is deliberately not installed (litearm-core is not on
# PyPI, see .devcontainer/README.md). The package itself is installed editable
# on container creation (postCreateCommand in devcontainer.json).
RUN pip install --no-cache-dir "pybullet>=3.2.5" "numpy>=1.21" pytest pytest-timeout

# The virtual display started by .devcontainer/start-vnc.sh. Force Mesa's
# software renderer: the image has no GPU, and this keeps rendering identical
# on every host.
ENV DISPLAY=:1 \
    LIBGL_ALWAYS_SOFTWARE=1
