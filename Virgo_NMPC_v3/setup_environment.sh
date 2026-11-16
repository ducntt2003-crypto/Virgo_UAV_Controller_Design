#!/usr/bin/env bash
# ==============================================================================
# Virgo OMAV NMPC Environment Setup Script
# Installs Python dependencies (CasADi, SciPy, Matplotlib) and builds ACADOS
# ==============================================================================

set -e

echo -e "\033[1;36m[1/3] Installing Python dependencies...\033[0m"
pip install casadi scipy matplotlib pyyaml numpy pandas pymavlink

ACADOS_DIR="$(pwd)/third_party/acados"
if [ ! -d "$ACADOS_DIR" ]; then
    echo -e "\n\033[1;36m[2/3] Cloning ACADOS repository...\033[0m"
    mkdir -p "$(pwd)/third_party"
    git clone https://github.com/acados/acados.git "$ACADOS_DIR"
    cd "$ACADOS_DIR"
    git checkout master
    git submodule update --recursive --init

    echo -e "\n\033[1;36m[3/3] Building ACADOS with HPIPM & QPOASES...\033[0m"
    mkdir -p build && cd build
    cmake -DACADOS_WITH_QPOASES=ON -DACADOS_WITH_HPIPM=ON -DACADOS_INSTALL_DIR="$ACADOS_DIR" ..
    make install -j$(nproc)

    echo -e "\n\033[1;32m[Success] Installing acados_template Python interface...\033[0m"
    pip install -e "$ACADOS_DIR/interfaces/acados_template"
    echo "export LD_LIBRARY_PATH=\$LD_LIBRARY_PATH:$ACADOS_DIR/lib" >> ~/.bashrc
    echo "export ACADOS_SOURCE_DIR=$ACADOS_DIR" >> ~/.bashrc
else
    echo -e "\033[1;32m[Info] ACADOS already cloned at $ACADOS_DIR.\033[0m"
fi

echo -e "\n\033[1;32m==============================================================================\033[0m"
echo -e "\033[1;32m Virgo OMAV NMPC Environment is Ready!\033[0m"
echo -e "\033[1;32m - Run Standalone Simulation: python3 simulation/run_nmpc_standalone_sim.py\033[0m"
echo -e "\033[1;32m - Run Benchmark Comparison:  python3 simulation/benchmark_nmpc_square.py\033[0m"
echo -e "\033[1;32m - Run PX4 MAVLink Bridge:    python3 ros2_px4_bridge/px4_mavlink_nmpc_bridge.py\033[0m"
echo -e "\033[1;32m==============================================================================\033[0m"
