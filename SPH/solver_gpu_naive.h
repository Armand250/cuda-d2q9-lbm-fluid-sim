#pragma once
#include "fluid_solver.h"
#include <cuda_runtime.h>

class GpuNaiveSolver : public FluidSolver
{
private:
    // Temporary GPU arrays for forces
    float *d_force_x = nullptr;
    float *d_force_y = nullptr;

    // Device pointers for obstacles
    Bounds *d_rects = nullptr;
    ObstracleCircle *d_circles = nullptr;
    ObstracleLine *d_lines = nullptr;

public:
    void initialize(ParticleSystem &ps, const Config &cfg) override;
    void step(ParticleSystem &ps, const Config &cfg) override;
    void cleanup() override;
    ExecutionMode getExecutionMode() override { return ExecutionMode::GPU; }
    std::string getName() const override { return "GPU_Naive"; }
};