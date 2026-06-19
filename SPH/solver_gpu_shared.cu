#include "solver_gpu_shared.h"
#include <thrust/sort.h>
#include <thrust/device_ptr.h>
#include <thrust/execution_policy.h>
#include <cmath>
#include <iostream>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

__global__ void calcHashKernel(
    int N, float h, int grid_width, int grid_height,
    const float *pos_x, const float *pos_y,
    int *hashes, int *indices)
{
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= N)
        return;

    // Calculate which grid cell this particle is in
    int cell_x = fmaxf(0.0f, fminf(grid_width - 1.0f, floorf(pos_x[i] / h)));
    int cell_y = fmaxf(0.0f, fminf(grid_height - 1.0f, floorf(pos_y[i] / h)));

    hashes[i] = cell_y * grid_width + cell_x;
    indices[i] = i;
}

__global__ void reorderDataKernel(
    int N, const int *indices,
    const float *pos_x, const float *pos_y, const float *vel_x, const float *vel_y,
    float *sorted_pos_x, float *sorted_pos_y, float *sorted_vel_x, float *sorted_vel_y)
{
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= N)
        return;

    int orig_idx = indices[i];
    sorted_pos_x[i] = pos_x[orig_idx];
    sorted_pos_y[i] = pos_y[orig_idx];
    sorted_vel_x[i] = vel_x[orig_idx];
    sorted_vel_y[i] = vel_y[orig_idx];
}

__global__ void findCellBoundariesKernel(
    int N, const int *hashes, int *cell_start, int *cell_end)
{
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= N)
        return;

    int hash = hashes[i];

    // Check if this thread is the start of a new cell
    if (i == 0)
    {
        cell_start[hash] = 0;
    }
    else
    {
        int prev_hash = hashes[i - 1];
        if (hash != prev_hash)
        {
            cell_end[prev_hash] = i; // End of previous cell
            cell_start[hash] = i;    // Start of new cell
        }
    }
    if (i == N - 1)
    {
        cell_end[hash] = N;
    }
}

__global__ void computeDensityPressureGridKernel(
    int N, float h, float m, float rest_density, int grid_width, int grid_height,
    const float *pos_x, const float *pos_y,
    const int *cell_start, const int *cell_end,
    float *density, float *pressure)
{
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= N)
        return;

    float poly6_coeff = 4.0f / (M_PI * powf(h, 8.0f));
    float d = 0.0f;

    int cell_x = fmaxf(0.0f, fminf(grid_width - 1.0f, floorf(pos_x[i] / h)));
    int cell_y = fmaxf(0.0f, fminf(grid_height - 1.0f, floorf(pos_y[i] / h)));

    for (int dy = -1; dy <= 1; dy++)
    {
        for (int dx = -1; dx <= 1; dx++)
        {
            int nx = cell_x + dx;
            int ny = cell_y + dy;

            if (nx >= 0 && nx < grid_width && ny >= 0 && ny < grid_height)
            {
                int n_hash = ny * grid_width + nx;
                int start = cell_start[n_hash];
                int end = cell_end[n_hash];

                for (int j = start; j < end; j++)
                {
                    float dist_x = pos_x[i] - pos_x[j];
                    float dist_y = pos_y[i] - pos_y[j];
                    float r2 = dist_x * dist_x + dist_y * dist_y;

                    if (r2 < h * h)
                    {
                        float term = (h * h - r2);
                        d += m * poly6_coeff * term * term * term;
                    }
                }
            }
        }
    }

    density[i] = fmaxf(d, 1.0f);
    pressure[i] = fmaxf(0.0f, 100.0f * (density[i] - rest_density)); // k_stiffness = 100.0f
}

__global__ void computeForcesGridKernel(
    int N, float h, float m, float viscosity, float g_x, float g_y,
    int grid_width, int grid_height,
    const float *pos_x, const float *pos_y, const float *vel_x, const float *vel_y,
    const float *density, const float *pressure,
    const int *cell_start, const int *cell_end,
    float *force_x, float *force_y)
{
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= N)
        return;

    float spiky_grad_coeff = -30.0f / (M_PI * powf(h, 5.0f));
    float visc_lap_coeff = 20.0f / (M_PI * powf(h, 5.0f));

    float fx = 0.0f;
    float fy = 0.0f;

    int cell_x = fmaxf(0.0f, fminf(grid_width - 1.0f, floorf(pos_x[i] / h)));
    int cell_y = fmaxf(0.0f, fminf(grid_height - 1.0f, floorf(pos_y[i] / h)));

    for (int dy = -1; dy <= 1; dy++)
    {
        for (int dx = -1; dx <= 1; dx++)
        {
            int nx = cell_x + dx;
            int ny = cell_y + dy;

            if (nx >= 0 && nx < grid_width && ny >= 0 && ny < grid_height)
            {
                int n_hash = ny * grid_width + nx;
                int start = cell_start[n_hash];
                int end = cell_end[n_hash];

                for (int j = start; j < end; j++)
                {
                    if (i == j)
                        continue;

                    float dist_x = pos_x[i] - pos_x[j];
                    float dist_y = pos_y[i] - pos_y[j];
                    float r = sqrtf(dist_x * dist_x + dist_y * dist_y);

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
                        fx += force_p * (dist_x / r);
                        fy += force_p * (dist_y / r);

                        float vx_diff = vel_x[j] - vel_x[i];
                        float vy_diff = vel_y[j] - vel_y[i];
                        float visc_term = viscosity * m / density[j];

                        if ((vx_diff * dist_x + vy_diff * dist_y) < 0.0f)
                        {
                            visc_term *= 10.0f;
                        }

                        fx += visc_term * visc_lap_coeff * (h - r) * vx_diff;
                        fy += visc_term * visc_lap_coeff * (h - r) * vy_diff;
                    }
                }
            }
        }
    }

    force_x[i] = fx + (g_x * density[i]);
    force_y[i] = fy + (g_y * density[i]);
}

__global__ void integrateAndWriteBackKernel(
    int N, float dt, float width, float height, float dampening, float particle_radius,
    const int *indices,
    const float *sorted_pos_x, const float *sorted_pos_y,
    const float *sorted_vel_x, const float *sorted_vel_y,
    const float *force_x, const float *force_y, const float *density,
    float *orig_pos_x, float *orig_pos_y, float *orig_vel_x, float *orig_vel_y,
    const Bounds *rects, int num_rects,
    const ObstracleCircle *circles, int num_circles,
    const ObstracleLine *lines, int num_lines)
{
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= N)
        return;

    float vx = sorted_vel_x[i] + dt * (force_x[i] / density[i]);
    float vy = sorted_vel_y[i] + dt * (force_y[i] / density[i]);

    vx *= 0.999f;
    vy *= 0.999f;

    float px = sorted_pos_x[i] + dt * vx;
    float py = sorted_pos_y[i] + dt * vy;

    // Container bounds
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

    int orig_idx = indices[i];
    orig_pos_x[orig_idx] = px;
    orig_pos_y[orig_idx] = py;
    orig_vel_x[orig_idx] = vx;
    orig_vel_y[orig_idx] = vy;
}



void GpuSharedMemorySolver::initialize(ParticleSystem &ps, const Config &cfg)
{
    grid_width = ceilf(cfg.width / cfg.smoothing_radius);
    grid_height = ceilf(cfg.height / cfg.smoothing_radius);
    total_cells = grid_width * grid_height;

    // Forces
    cudaMalloc(&d_force_x, ps.max_particles * sizeof(float));
    cudaMalloc(&d_force_y, ps.max_particles * sizeof(float));

    // Hash grid
    cudaMalloc(&d_particle_hashes, ps.max_particles * sizeof(int));
    cudaMalloc(&d_particle_indices, ps.max_particles * sizeof(int));
    cudaMalloc(&d_cell_start, total_cells * sizeof(int));
    cudaMalloc(&d_cell_end, total_cells * sizeof(int));

    // Sorted temporary arrays
    cudaMalloc(&d_sorted_pos_x, ps.max_particles * sizeof(float));
    cudaMalloc(&d_sorted_pos_y, ps.max_particles * sizeof(float));
    cudaMalloc(&d_sorted_vel_x, ps.max_particles * sizeof(float));
    cudaMalloc(&d_sorted_vel_y, ps.max_particles * sizeof(float));

    // Density/Pressure can just map directly to sorted data for the intermediate steps
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

void GpuSharedMemorySolver::cleanup()
{
    cudaFree(d_force_x);
    cudaFree(d_force_y);
    cudaFree(d_particle_hashes);
    cudaFree(d_particle_indices);
    cudaFree(d_cell_start);
    cudaFree(d_cell_end);
    cudaFree(d_sorted_pos_x);
    cudaFree(d_sorted_pos_y);
    cudaFree(d_sorted_vel_x);
    cudaFree(d_sorted_vel_y);
    if (d_rects)
        cudaFree(d_rects);
    if (d_circles)
        cudaFree(d_circles);
    if (d_lines)
        cudaFree(d_lines);
}

void GpuSharedMemorySolver::step(ParticleSystem &ps, const Config &cfg)
{
    int N = ps.active_count;
    if (N == 0)
        return;

    int threads = 256;
    int blocks = (N + threads - 1) / threads;

    calcHashKernel<<<blocks, threads>>>(
        N, cfg.smoothing_radius, grid_width, grid_height,
        ps.d_pos_x, ps.d_pos_y, d_particle_hashes, d_particle_indices);
    cudaDeviceSynchronize();

    thrust::device_ptr<int> dev_keys(d_particle_hashes);
    thrust::device_ptr<int> dev_values(d_particle_indices);
    thrust::sort_by_key(thrust::device, dev_keys, dev_keys + N, dev_values);
    cudaDeviceSynchronize();

    reorderDataKernel<<<blocks, threads>>>(
        N, d_particle_indices,
        ps.d_pos_x, ps.d_pos_y, ps.d_vel_x, ps.d_vel_y,
        d_sorted_pos_x, d_sorted_pos_y, d_sorted_vel_x, d_sorted_vel_y);
    cudaDeviceSynchronize();

    cudaMemset(d_cell_start, 0, total_cells * sizeof(int));
    cudaMemset(d_cell_end, 0, total_cells * sizeof(int));
    findCellBoundariesKernel<<<blocks, threads>>>(N, d_particle_hashes, d_cell_start, d_cell_end);
    cudaDeviceSynchronize();

    computeDensityPressureGridKernel<<<blocks, threads>>>(
        N, cfg.smoothing_radius, cfg.particle_mass, cfg.rest_density, grid_width, grid_height,
        d_sorted_pos_x, d_sorted_pos_y, d_cell_start, d_cell_end, ps.d_density, ps.d_pressure);
    cudaDeviceSynchronize();

    computeForcesGridKernel<<<blocks, threads>>>(
        N, cfg.smoothing_radius, cfg.particle_mass, cfg.viscosity, cfg.gravity.x, cfg.gravity.y,
        grid_width, grid_height,
        d_sorted_pos_x, d_sorted_pos_y, d_sorted_vel_x, d_sorted_vel_y,
        ps.d_density, ps.d_pressure,
        d_cell_start, d_cell_end, d_force_x, d_force_y);
    cudaDeviceSynchronize();

    float particle_radius = cfg.smoothing_radius * 0.5f;

    integrateAndWriteBackKernel<<<blocks, threads>>>(
        N, cfg.time_step, cfg.width, cfg.height, -0.2f, particle_radius,
        d_particle_indices,
        d_sorted_pos_x, d_sorted_pos_y, d_sorted_vel_x, d_sorted_vel_y,
        d_force_x, d_force_y, ps.d_density,
        ps.d_pos_x, ps.d_pos_y, ps.d_vel_x, ps.d_vel_y,
        d_rects, cfg.obstracle_rectangles.size(),
        d_circles, cfg.obstracle_circles.size(),
        d_lines, cfg.obstracle_lines.size());
    cudaDeviceSynchronize();
}