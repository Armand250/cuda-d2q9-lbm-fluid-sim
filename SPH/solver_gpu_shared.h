#pragma once
#include "fluid_solver.h"
#include <cuda_runtime.h>

class GpuSharedMemorySolver : public FluidSolver {
private:
    float* d_force_x = nullptr;
    float* d_force_y = nullptr;

    Bounds* d_rects = nullptr;
    ObstracleCircle* d_circles = nullptr;
    ObstracleLine* d_lines = nullptr;

    int* d_particle_hashes = nullptr;  // Which cell ID the particle belongs to
    int* d_particle_indices = nullptr; // Original index of the particle
    int* d_cell_start = nullptr;       // Where a cell's particles start in the sorted array
    int* d_cell_end = nullptr;         // Where a cell's particles end

    // Sorted particle arrays
    float* d_sorted_pos_x = nullptr;
    float* d_sorted_pos_y = nullptr;
    float* d_sorted_vel_x = nullptr;
    float* d_sorted_vel_y = nullptr;

    int grid_width;
    int grid_height;
    int total_cells;

public:
    void initialize(ParticleSystem& ps, const Config& cfg) override;
    void step(ParticleSystem& ps, const Config& cfg) override;
    void cleanup() override;
    ExecutionMode getExecutionMode() override { return ExecutionMode::GPU; }
    std::string getName() const override { return "GPU_Optimized_Grid"; }
};