#!/bin/bash
if [ ! -f ./bin/fluid_sim ]; then
     make clean
     make
fi

./bin/fluid_sim --config "scenes/$1" --solver "$2" --output "results/$3" && \
    python3 visualizer.py --input "results/$3" --config "scenes/$1" --output "results/$4" -fs 40