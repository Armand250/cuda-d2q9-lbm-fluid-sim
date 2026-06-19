#pragma once
#include "fluid_solver.h"

class CpuNaiveSolver : public FluidSolver
{
public:
    void initialize(ParticleSystem &ps, const Config &cfg) override {}
    void step(ParticleSystem &ps, const Config &cfg) override;
    void cleanup() override {}

    ExecutionMode getExecutionMode() override { return ExecutionMode::CPU; }
    std::string getName() const override { return "CPU_Naive"; }
};