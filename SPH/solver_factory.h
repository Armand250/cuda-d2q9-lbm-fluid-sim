// solver_factory.h
#pragma once
#include <memory>
#include "fluid_solver.h"
#include "solver_cpu_naive.h"
#include "solver_gpu_shared.h"
#include "solver_gpu_naive.h"

class SolverFactory
{
public:
    static std::unique_ptr<FluidSolver> create(const std::string &type)
    {
        if (type == "cpu-naive")
            return std::make_unique<CpuNaiveSolver>();
        if (type == "gpu-shared")
            return std::make_unique<GpuSharedMemorySolver>();
        if (type == "gpu-naive")
            return std::make_unique<GpuNaiveSolver>();

        throw std::runtime_error("Unknown solver type: " + type);
    }
};