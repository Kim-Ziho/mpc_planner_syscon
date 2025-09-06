#!/usr/bin/env bash
set -euo pipefail

# ============ Helpers ============
say() { printf "%b\n" "$*"; }
ok()  { say "✅ $*"; }
info(){ say "ℹ️  $*"; }
warn(){ say "⚠️  $*"; }
err() { say "❌ $*" >&2; }

# ============ Paths ============
PROJECT_ROOT="$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
ACADOS_DIR="${PROJECT_ROOT}/acados"
ACADOS_LIB="${ACADOS_DIR}/lib"
ACADOS_TEMPLATE_DIR="${PROJECT_ROOT}/acados/interfaces/acados_template" # editable install target
PY_VER="3.8.10"

say "🐍 Python/Acados setup started..."
info "Project root: ${PROJECT_ROOT}"

# ============ Install acados if missing ============
if [[ ! -d "${ACADOS_LIB}" ]]; then
  info "acados not found. Installing..."
  pushd "${PROJECT_ROOT}" >/dev/null
  git clone https://github.com/acados/acados.git
  cd acados
  git checkout c48cf1135b779fb4582358da89c119f7c481dcea  # pin
  git submodule update --recursive --init
  mkdir -p build && cd build
  cmake -DACADOS_SILENT=ON ..
  make install -j"$(nproc)"
  cd ../bin
  wget -q https://github.com/acados/tera_renderer/releases/download/v0.0.34/t_renderer-v0.0.34-linux -O t_renderer
  chmod +x t_renderer
  popd >/dev/null
  ok "acados installed."
else
  ok "acados already installed."
fi

# ============ Ensure pyenv ============
if command -v pyenv >/dev/null 2>&1; then
  ok "pyenv already installed."
else
  warn "pyenv not found."
  read -r -p "👉 Install pyenv now? [y/N]: " REPLY
  if [[ "${REPLY:-N}" =~ ^[Yy]$ ]]; then
    info "Installing build deps (sudo)..."
    sudo apt-get update -y
    sudo apt-get install -y \
      build-essential libssl-dev zlib1g-dev libbz2-dev libreadline-dev libsqlite3-dev curl \
      libncursesw5-dev xz-utils tk-dev libxml2-dev libxmlsec1-dev libffi-dev liblzma-dev git

    info "Installing pyenv..."
    curl -fsSL https://pyenv.run | bash

    # Add to ~/.bashrc if missing
    BASHRC="$HOME/.bashrc"
    if ! grep -q 'export PYENV_ROOT="$HOME/.pyenv"' "$BASHRC"; then
      info "Appending pyenv init to ~/.bashrc"
      {
        echo ""
        echo "# >>> pyenv init >>>"
        echo 'export PYENV_ROOT="$HOME/.pyenv"'
        echo '[[ -d $PYENV_ROOT/bin ]] && export PATH="$PYENV_ROOT/bin:$PATH"'
        echo 'eval "$(pyenv init -)"'
        echo "# <<< pyenv init <<<"
      } >> "$BASHRC"
    else
      info "pyenv init lines already present in ~/.bashrc"
    fi

    # Apply to current shell session
    export PYENV_ROOT="$HOME/.pyenv"
    [[ -d "$PYENV_ROOT/bin" ]] && export PATH="$PYENV_ROOT/bin:$PATH"
    eval "$(pyenv init -)"
    ok "pyenv installed."
  else
    err "pyenv is required for Python ${PY_VER}. Aborting."
    exit 1
  fi
fi

# Safety: ensure pyenv is usable now
if ! command -v pyenv >/dev/null 2>&1; then
  err "pyenv not available in current shell. Open a new terminal or 'source ~/.bashrc' and rerun."
  exit 1
fi

# ============ Ensure Python 3.8.10 via pyenv ============
if pyenv versions --bare | grep -qx "${PY_VER}"; then
  ok "Python ${PY_VER} already installed in pyenv."
else
  info "Installing Python ${PY_VER} (this may take a while)..."
  pyenv install "${PY_VER}"
  ok "Python ${PY_VER} installed."
fi

# ============ Apply local Python & venv ============
pushd "${PROJECT_ROOT}/src/mpc_planner" >/dev/null
info "Setting local Python to ${PY_VER}"
pyenv local "${PY_VER}"

PYTHON_BIN="$(pyenv which python)"
ok "Using interpreter: ${PYTHON_BIN} ($("$PYTHON_BIN" -V))"

if [[ -d ".venv" ]]; then
  ok ".venv already exists."
else
  info "Creating .venv..."
  "$PYTHON_BIN" -m venv .venv
  ok ".venv created."
fi

# Activate venv
# shellcheck disable=SC1091
source .venv/bin/activate
ok "Venv activated: $(python -V)"

# ============ Install Python deps ============
if [[ -f "requirements.txt" ]]; then
  info "Installing requirements..."
  python -m pip install --upgrade pip setuptools wheel >/dev/null
  python -m pip install -r requirements.txt
  ok "requirements installed."
else
  warn "requirements.txt not found. Skipping."
fi

# Editable install of acados_template (if present)
if [[ -d "${ACADOS_TEMPLATE_DIR}" ]]; then
  info "Installing acados_template (editable)..."
  python -m pip install -e "${ACADOS_TEMPLATE_DIR}"
  ok "acados_template installed."
else
  warn "acados_template not found at: ${ACADOS_TEMPLATE_DIR}"
fi

deactivate
popd >/dev/null
ok "Python setup completed."

# ============ rosdep (optional if ROS available) ============
if command -v rosdep >/dev/null 2>&1; then
  info "Resolving ROS deps via rosdep..."
  sudo apt-get update -y
  rosdep update --rosdistro "${ROS_DISTRO:-$(command -v rosversion >/dev/null 2>&1 && rosversion -d || echo noetic)}"
  rosdep install -y -r --from-paths "${PROJECT_ROOT}/src" --ignore-src --rosdistro "${ROS_DISTRO:-$(command -v rosversion >/dev/null 2>&1 && rosversion -d || echo noetic)}"
  ok "rosdep done."
else
  warn "rosdep not found. Skipping ROS dependency installation."
fi

say "🎉 All done!"
say "📂 Current dir: $(pwd)"
say "🐍 pyenv local: $(cat "${PROJECT_ROOT}/src/mpc_planner/.python-version" 2>/dev/null || echo 'N/A')"
say "🔒 venv: ${PROJECT_ROOT}/src/mpc_planner/.venv (created)"
