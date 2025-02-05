#!/bin/bash

if [ -z "$PATH_TO_WS" ]; then
    mkdir -p openmore_ws/src
    mkdir -p openmore_ws/build
    mkdir -p openmore_ws/install
    cd openmore_ws
    export PATH_TO_WS="$(pwd)"

    echo "Workspace Path: $PATH_TO_WS"
    echo "PATH_TO_WS=$PATH_TO_WS" >> $GITHUB_ENV

    export PATH="$PATH_TO_WS/install/bin:$PATH"
    export LD_LIBRARY_PATH="$PATH_TO_WS/install/lib"
    export CMAKE_PREFIX_PATH="$PATH_TO_WS/install"

    echo "PATH: $PATH"
    echo "LD_LIBRARY_PATH: $LD_LIBRARY_PATH"
    echo "CMAKE_PREFIX_PATH: $CMAKE_PREFIX_PATH"
fi

# Install trajectories_processors_lib
cd $PATH_TO_WS/src
git clone https://github.com/JRL-CARI-CNR-UNIBS/trajectories_processors_lib.git

cd $PATH_TO_WS
mkdir -p build/trajectories_processors_lib
cmake -S src/trajectories_processors_lib -B build/trajectories_processors_lib -DCMAKE_INSTALL_PREFIX=$PATH_TO_WS/install
make -C build/trajectories_processors_lib install