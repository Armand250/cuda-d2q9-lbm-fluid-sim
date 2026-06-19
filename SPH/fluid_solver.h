#pragma once
#include <string>
#include "config.h"
#include "particle_system.h"

class FluidSolver {
public:
    virtual ExecutionMode getExecutionMode() = 0;
    virtual ~FluidSolver() = default;
    virtual void initialize(ParticleSystem& ps, const Config& cfg) = 0;
    virtual void step(ParticleSystem& ps, const Config& cfg) = 0;
    virtual void cleanup() = 0;
    virtual std::string getName() const = 0;
};