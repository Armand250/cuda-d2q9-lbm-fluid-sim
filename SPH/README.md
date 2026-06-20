# Fluid simulation using Smoothed-Particle Hydrodynamics
## Code structure
### Scenes
The [scenes](./scenes/) folder contains all the predefined scenes described by using `json` files. The folder contains a template file ([_template.json](./scenes/_template.json)) that contains all the possible elements of a scene file. 

Here you can see how a scene file built and how it affects the simulation:

| Configuration Object | Parameter | Description and Simulation Effect |
| :--- | :--- | :--- |
| **`container`** | `width`, `height` | Defines the absolute global bounding box of the 2D simulation area. Particles hitting these boundaries will bounce back inside. |
| **`physics`** | `gravity` | A 2D vector (x, y) applying constant acceleration to all particles. |
| | `time_step` | The delta time used for explicit integration. Smaller values increase accuracy and stability; larger values risk numerical explosions. |
| | `max_frames` | The total number of iterations the simulation will run before successfully terminating. |
| **`fluid_properties`** | `rest_density` | The baseline density the fluid attempts to maintain. Controls the baseline for pressure calculations. |
| | `viscosity` | The internal friction of the fluid. Higher values result in syrup-like behavior; lower values create thin, splashy water. |
| | `particle_mass` | The mass of a single fluid particle. Must be balanced with the smoothing radius to maintain stability. |
| | `smoothing_radius` | The spatial interaction limit. Determines the physical size of a particle's interaction neighborhood. |
| **`fluid_sources`** | `block` | Spawns a static, pre-filled rectangular volume of fluid particles at the start of the simulation, defined by its min/max x and y bounds. |
| | `emitter` | A dynamic source that sprays particles at a defined `speed` along a `direction` vector, spawning `rate` particles per second until the `active_until` timestamp is reached. |
| **`obstacles`** | `rectangle` | A static rectangular collision boundary defined by min/max x and y bounds. |
| | `circle` | A static circular collision boundary defined by a center point (x, y) and a radius. |
| | `line` | A static 1D collision boundary defined by a starting point (x, y) and an ending point (x, y). |


### Source code files
| Filename | Content |
| --- | --- |
|[cli_parser.h](./cli_parser.h)| Command-line argument parser for handling command-line flags and defaults (config path, solver selection, output, profiling, metrics). |
|[config.h](./config.h)| Definitions for `Config`, fluid source and obstacle structures, and enums describing execution modes and source types. |
|[engine.cpp](./engine.cpp)| Implementation of `SimulationEngine`: memory allocation, initial particle spawning, emitter handling, frame export and main simulation loop. |
|[engine.h](./engine.h)| `SimulationEngine` class declaration and lifecycle management (init, run, cleanup). |
|[fluid_solver.h](./fluid_solver.h)| Abstract base class interface for solver implementations (`initialize`, `step`, `cleanup`, `getExecutionMode`). |
|[json.hpp](./json.hpp)| Single-header JSON library (nlohmann::json) used for scene parsing. |
|[main.cpp](./main.cpp)| Program entry point: parses CLI, loads scene, constructs the engine and chosen solver, and runs the simulation. |
|[particle_system.h](./particle_system.h)| `ParticleSystem` struct holding CPU/GPU particle buffers and counts. |
|[scene_reader.h](./scene_reader.h)| Loads and parses JSON scene files into a `Config` instance (container, physics, sources, obstacles). |
|[solver_cpu_naive.cpp](./solver_cpu_naive.cpp)| CPU-based SPH solver: density/pressure computation, force evaluation, integration and collision handling. |
|[solver_cpu_naive.h](./solver_cpu_naive.h)| Header declaring the `CpuNaiveSolver` implementing the CPU solver interface. |
|[solver_factory.h](./solver_factory.h)| Factory that instantiates solver objects by name (`cpu-naive`, `gpu-naive`, `gpu-shared`). |
|[solver_gpu_naive.cu](./solver_gpu_naive.cu)| GPU (CUDA) naive solver with device kernels for density/pressure, force computation and integration. |
|[solver_gpu_naive.h](./solver_gpu_naive.h)| Header for the `GpuNaiveSolver` CUDA implementation and GPU resource pointers. |
|[solver_gpu_shared.cu](./solver_gpu_shared.cu)| Optimized GPU solver using spatial hashing/grid and shared memory to accelerate neighbor searches and force calculations. |
|[solver_gpu_shared.h](./solver_gpu_shared.h)| Header for the optimized GPU solver (grid bookkeeping, sorted buffers, GPU allocations). |
|[visualizer.py](./visualizer.py)| Python script to read the engine's binary output and render/save an animation (matplotlib, GIF). |

### Scripts & run

| Script | Description |
| --- | --- |
|[Makefile](./Makefile)| Build rules to compile the CUDA/C++ sources, produce `bin/fluid_sim` and create `results/` directory. |
|[run_test.sh](./run_test.sh)| Benchmark runner: executes all scene JSONs with each solver, saves outputs and (for GPU) runs `ncu` profiling. |
|[simulate.sh](./simulate.sh)| Convenience script to build if needed, run a single scene with a chosen solver, and visualize the output with `visualizer.py`. |

#### Sample runs

Build the project:

```bash
make
```

Run the full benchmark suite over all scenes (creates results/*.dat and profiler outputs):

```bash
./run_test.sh
```
> Keep in mind that scenes that start with '_' will be skipped in this process!


Run a single scene and produce visualization (args: scene.json solver output_dat output_gif):

```bash
./simulate.sh [name_of_scene] [cpu-naive|gpu-naive|gpu-shared] [out_name].dat [gif_name].gif

./simulate.sh default.json gpu-shared default_gpu-shared.dat default_gpu-shared.gif
```

Run the simulation binary directly:

```bash
./bin/fluid_sim --config scenes/default.json --solver gpu-shared --output results/default_gpu-shared.dat --metrics
```
> Use `-h` or `--help` for more details!


Profile a GPU run with metrics and CUDA profiler (NVIDIA Nsight Compute):

```bash
./bin/fluid_sim --config scenes/default.json --solver gpu-shared --output results/default_gpu-shared.dat --profiling
```

Visualize a binary output with the Python visualizer:

```bash
python3 visualizer.py --input results/default_gpu-shared.dat --config scenes/default.json --output results/default_animation.gif -fs 40
```
> Use `-h` or `--help` for more details!

