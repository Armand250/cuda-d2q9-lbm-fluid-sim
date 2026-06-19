#pragma once
#include <string>
#include <vector>

enum class ExecutionMode
{
    CPU,
    GPU
};
enum class FluidSourceType
{
    BLOCK,
    EMITTER
};

struct Bounds
{
    float x_min, x_max, y_min, y_max;
};

struct ObstracleCircle
{
    float x_center, y_center, radius;
};

struct ObstracleLine
{
    float x1, y1, x2, y2;
};

struct _float2
{
    float x, y;
};

struct FluidSource
{
    FluidSourceType type; // BLOCK or EMITTER
    Bounds bounds;

    // Emitter specific
    _float2 direction;
    float rate;
    float speed;
    float active_until;
};

struct Config
{
    float width, height;
    int max_frames;
    int max_particles;
    float time_step;
    _float2 gravity;
    float rest_density, viscosity, particle_mass, smoothing_radius;
    std::vector<FluidSource> sources;

    // Obstacles
    std::vector<Bounds> obstracle_rectangles;
    std::vector<ObstracleCircle> obstracle_circles;
    std::vector<ObstracleLine> obstracle_lines;


    bool enable_profiling = false;
    bool save_metrics = false;
    std::string output_path = "";
};