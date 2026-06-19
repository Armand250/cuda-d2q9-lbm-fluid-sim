#include "solver_gpu_naive.h"
#include <cmath>
#include <algorithm>
#include <iostream>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

__global__ void computeDensityPressureKernel(
    int N, float h, float m, float rest_density, float k_stiffness,
    const float *pos_x, const float *pos_y,
    float *density, float *pressure)
{
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= N)
        return;

    float poly6_coeff = 4.0f / (M_PI * powf(h, 8.0f));
    float d = 0.0f;

    for (int j = 0; j < N; j++)
    {
        float dx = pos_x[i] - pos_x[j];
        float dy = pos_y[i] - pos_y[j];
        float r2 = dx * dx + dy * dy;

        if (r2 < h * h)
        {
            float term = (h * h - r2);
            d += m * poly6_coeff * term * term * term;
        }
    }

    density[i] = fmaxf(d, 1.0f);
    pressure[i] = fmaxf(0.0f, k_stiffness * (density[i] - rest_density));
}

__global__ void computeForcesKernel(
    int N, float h, float m, float viscosity, float g_x, float g_y,
    const float *pos_x, const float *pos_y, const float *vel_x, const float *vel_y,
    const float *density, const float *pressure,
    float *force_x, float *force_y)
{
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= N)
        return;

    float spiky_grad_coeff = -30.0f / (M_PI * powf(h, 5.0f));
    float visc_lap_coeff = 20.0f / (M_PI * powf(h, 5.0f));

    float fx = 0.0f;
    float fy = 0.0f;

    for (int j = 0; j < N; j++)
    {
        if (i == j)
            continue;

        float dx = pos_x[i] - pos_x[j];
        float dy = pos_y[i] - pos_y[j];
        float r = sqrtf(dx * dx + dy * dy);

        if (r < 0.0001f)
        {
            fx += ((i % 100) / 100.0f - 0.5f) * 0.1f;
            fy += ((j % 100) / 100.0f - 0.5f) * 0.1f;
            continue;
        }

        if (r < h)
        {
            float pressure_term = (pressure[i] + pressure[j]) / (2.0f * density[j]);
            float force_p = -m * pressure_term * spiky_grad_coeff * powf(h - r, 2.0f);

            fx += force_p * (dx / r);
            fy += force_p * (dy / r);

            float vx_diff = vel_x[j] - vel_x[i];
            float vy_diff = vel_y[j] - vel_y[i];
            float visc_term = viscosity * m / density[j];

            if ((vx_diff * dx + vy_diff * dy) < 0.0f)
            {
                visc_term *= 10.0f;
            }

            fx += visc_term * visc_lap_coeff * (h - r) * vx_diff;
            fy += visc_term * visc_lap_coeff * (h - r) * vy_diff;
        }
    }

    force_x[i] = fx + (g_x * density[i]);
    force_y[i] = fy + (g_y * density[i]);
}

__global__ void integrateAndCollideKernel(
    int N, float dt, float width, float height, float dampening, float particle_radius,
    float *pos_x, float *pos_y, float *vel_x, float *vel_y,
    const float *force_x, const float *force_y, const float *density,
    const Bounds *rects, int num_rects,
    const ObstracleCircle *circles, int num_circles,
    const ObstracleLine *lines, int num_lines)
{
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= N)
        return;

    float vx = vel_x[i] + dt * (force_x[i] / density[i]);
    float vy = vel_y[i] + dt * (force_y[i] / density[i]);

    vx *= 0.999f;
    vy *= 0.999f;

    float px = pos_x[i] + dt * vx;
    float py = pos_y[i] + dt * vy;

    // Container boundaries
    if (px < 0.0f)
    {
        px = 0.0f;
        vx *= dampening;
    }
    if (px > width)
    {
        px = width;
        vx *= dampening;
    }
    if (py < 0.0f)
    {
        py = 0.0f;
        vy *= dampening;
    }
    if (py > height)
    {
        py = height;
        vy *= dampening;
    }

    // Rectangle obstacles
    for (int j = 0; j < num_rects; j++)
    {
        const Bounds &rect = rects[j];
        if (px > rect.x_min && px < rect.x_max && py > rect.y_min && py < rect.y_max)
        {
            float d_left = px - rect.x_min;
            float d_right = rect.x_max - px;
            float d_bottom = py - rect.y_min;
            float d_top = rect.y_max - py;

            float min_d = fminf(fminf(d_left, d_right), fminf(d_bottom, d_top));

            if (min_d == d_left)
            {
                px = rect.x_min;
                vx *= dampening;
            }
            else if (min_d == d_right)
            {
                px = rect.x_max;
                vx *= dampening;
            }
            else if (min_d == d_bottom)
            {
                py = rect.y_min;
                vy *= dampening;
            }
            else if (min_d == d_top)
            {
                py = rect.y_max;
                vy *= dampening;
            }
        }
    }

    // Circle obstacles
    for (int j = 0; j < num_circles; j++)
    {
        const ObstracleCircle &circ = circles[j];
        float dx = px - circ.x_center;
        float dy = py - circ.y_center;
        float dist_squared = dx * dx + dy * dy;
        float r_squared = circ.radius * circ.radius;

        if (dist_squared < r_squared)
        {
            float dist = sqrtf(dist_squared);
            if (dist == 0.0f)
            {
                dist = 0.001f;
                dx = 0.001f;
                dy = 0.0f;
            }

            float normal_x = dx / dist;
            float normal_y = dy / dist;
            float overlap = circ.radius - dist;

            px += normal_x * overlap;
            py += normal_y * overlap;

            float dot = vx * normal_x + vy * normal_y;
            vx = (vx - 2.0f * dot * normal_x) * fabsf(dampening);
            vy = (vy - 2.0f * dot * normal_y) * fabsf(dampening);
        }
    }

    // Line obstacles
    for (int j = 0; j < num_lines; j++)
    {
        const ObstracleLine &line = lines[j];
        float line_dx = line.x2 - line.x1;
        float line_dy = line.y2 - line.y1;
        float line_length_squared = line_dx * line_dx + line_dy * line_dy;

        if (line_length_squared == 0.0f)
            continue;

        float px_dx = px - line.x1;
        float py_dy = py - line.y1;

        float t = (px_dx * line_dx + py_dy * line_dy) / line_length_squared;
        t = fmaxf(0.0f, fminf(1.0f, t));

        float closest_x = line.x1 + t * line_dx;
        float closest_y = line.y1 + t * line_dy;

        float dist_dx = px - closest_x;
        float dist_dy = py - closest_y;
        float dist_squared = dist_dx * dist_dx + dist_dy * dist_dy;

        if (dist_squared < particle_radius * particle_radius)
        {
            float dist = sqrtf(dist_squared);
            if (dist == 0.0f)
            {
                dist = 0.001f;
                dist_dx = 0.001f;
                dist_dy = 0.0f;
            }

            float normal_x = dist_dx / dist;
            float normal_y = dist_dy / dist;
            float overlap = particle_radius - dist;

            px += normal_x * overlap;
            py += normal_y * overlap;

            float dot = vx * normal_x + vy * normal_y;
            if (dot < 0.0f)
            {
                vx = (vx - 2.0f * dot * normal_x) * fabsf(dampening);
                vy = (vy - 2.0f * dot * normal_y) * fabsf(dampening);
            }
        }
    }

    pos_x[i] = px;
    pos_y[i] = py;
    vel_x[i] = vx;
    vel_y[i] = vy;
}

void GpuNaiveSolver::initialize(ParticleSystem &ps, const Config &cfg)
{
    cudaMalloc(&d_force_x, ps.max_particles * sizeof(float));
    cudaMalloc(&d_force_y, ps.max_particles * sizeof(float));

    // Copy obstacles to GPU
    if (!cfg.obstracle_rectangles.empty())
    {
        cudaMalloc(&d_rects, cfg.obstracle_rectangles.size() * sizeof(Bounds));
        cudaMemcpy(d_rects, cfg.obstracle_rectangles.data(), cfg.obstracle_rectangles.size() * sizeof(Bounds), cudaMemcpyHostToDevice);
    }
    if (!cfg.obstracle_circles.empty())
    {
        cudaMalloc(&d_circles, cfg.obstracle_circles.size() * sizeof(ObstracleCircle));
        cudaMemcpy(d_circles, cfg.obstracle_circles.data(), cfg.obstracle_circles.size() * sizeof(ObstracleCircle), cudaMemcpyHostToDevice);
    }
    if (!cfg.obstracle_lines.empty())
    {
        cudaMalloc(&d_lines, cfg.obstracle_lines.size() * sizeof(ObstracleLine));
        cudaMemcpy(d_lines, cfg.obstracle_lines.data(), cfg.obstracle_lines.size() * sizeof(ObstracleLine), cudaMemcpyHostToDevice);
    }
}

void GpuNaiveSolver::cleanup()
{
    cudaFree(d_force_x);
    cudaFree(d_force_y);
    if (d_rects)
        cudaFree(d_rects);
    if (d_circles)
        cudaFree(d_circles);
    if (d_lines)
        cudaFree(d_lines);
}

void GpuNaiveSolver::step(ParticleSystem &ps, const Config &cfg)
{
    int N = ps.active_count;
    if (N == 0)
        return;

    int threadsPerBlock = 256;
    int blocksPerGrid = (N + threadsPerBlock - 1) / threadsPerBlock;

    computeDensityPressureKernel<<<blocksPerGrid, threadsPerBlock>>>(
        N, cfg.smoothing_radius, cfg.particle_mass, cfg.rest_density, 100.0f,
        ps.d_pos_x, ps.d_pos_y, ps.d_density, ps.d_pressure);
    cudaDeviceSynchronize();

    computeForcesKernel<<<blocksPerGrid, threadsPerBlock>>>(
        N, cfg.smoothing_radius, cfg.particle_mass, cfg.viscosity, cfg.gravity.x, cfg.gravity.y,
        ps.d_pos_x, ps.d_pos_y, ps.d_vel_x, ps.d_vel_y, ps.d_density, ps.d_pressure,
        d_force_x, d_force_y);
    cudaDeviceSynchronize();

    float particle_radius = cfg.smoothing_radius * 0.5f;
    integrateAndCollideKernel<<<blocksPerGrid, threadsPerBlock>>>(
        N, cfg.time_step, cfg.width, cfg.height, -0.2f, particle_radius,
        ps.d_pos_x, ps.d_pos_y, ps.d_vel_x, ps.d_vel_y,
        d_force_x, d_force_y, ps.d_density,
        d_rects, cfg.obstracle_rectangles.size(),
        d_circles, cfg.obstracle_circles.size(),
        d_lines, cfg.obstracle_lines.size());
    cudaDeviceSynchronize();
}