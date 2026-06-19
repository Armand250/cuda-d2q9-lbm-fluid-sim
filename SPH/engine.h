#pragma once
#include "config.h"
#include "particle_system.h"
#include <cstdio>
#include "fluid_solver.h"
#include <cuda_runtime.h>
#include <memory>
#include <fstream>
#include <cuda_profiler_api.h>

class SimulationEngine
{
private:
    Config cfg;
    ParticleSystem ps;
    std::unique_ptr<FluidSolver> active_solver;
    int current_frame = 0;
    float current_time = 0.0f;
    FILE *output_file = nullptr;

    void allocateMemory();
    void spawnInitialBlocks();
    void handleEmitters();

    void initializeExport(const std::string &filepath);

    void exportFrame();

public:
    SimulationEngine(const Config &config, std::unique_ptr<FluidSolver> solver)
        : cfg(config), active_solver(std::move(solver))
    {
        initializeExport(cfg.output_path);
        allocateMemory();
        spawnInitialBlocks();
        active_solver->initialize(ps, cfg);
    }

    ~SimulationEngine()
    {
        if (output_file)
        {
            fclose(output_file);
        }

        delete[] ps.h_pos_x;
        delete[] ps.h_pos_y;
        delete[] ps.h_vel_x;
        delete[] ps.h_vel_y;
        delete[] ps.h_density;
        delete[] ps.h_pressure;

        ps.h_pos_x = nullptr;
        ps.h_pos_y = nullptr;
        ps.h_vel_x = nullptr;
        ps.h_vel_y = nullptr;
        ps.h_density = nullptr;
        ps.h_pressure = nullptr;

        // Free GPU memory if allocated
        if (active_solver->getExecutionMode() == ExecutionMode::GPU)
        {
            cudaFree(ps.d_pos_x);
            cudaFree(ps.d_pos_y);
            cudaFree(ps.d_vel_x);
            cudaFree(ps.d_vel_y);
            cudaFree(ps.d_density);
            cudaFree(ps.d_pressure);
        }

        active_solver->cleanup();
    }

    void run();
};