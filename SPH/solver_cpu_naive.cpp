#include "solver_cpu_naive.h"
#include <cmath>
#include <algorithm>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

void CpuNaiveSolver::step(ParticleSystem &ps, const Config &cfg)
{
    int N = ps.active_count;
    if (N == 0)
        return;

    float h = cfg.smoothing_radius;
    float m = cfg.particle_mass;

    // SPH kernel coefficients
    float poly6_coeff = 4.0f / (M_PI * std::pow(h, 8.0f));
    float spiky_grad_coeff = -30.0f / (M_PI * std::pow(h, 5.0f));
    float visc_lap_coeff = 20.0f / (M_PI * std::pow(h, 5.0f));

    // Density and pressure calculation
    for (int i = 0; i < N; i++)
    {
        float density = 0.0f;
        for (int j = 0; j < N; j++)
        {
            float dx = ps.h_pos_x[i] - ps.h_pos_x[j];
            float dy = ps.h_pos_y[i] - ps.h_pos_y[j];
            float r2 = dx * dx + dy * dy;

            if (r2 < h * h)
            {
                float term = (h * h - r2);
                density += m * poly6_coeff * term * term * term;
            }
        }

        ps.h_density[i] = std::max(density, 1.0f); // Prevent division by zero

        // Equation of EoS (Clamped to prevent negative pressure explosions)
        float k_stiffness = 100.0f;
        ps.h_pressure[i] = std::max(0.0f, k_stiffness * (ps.h_density[i] - cfg.rest_density));
    }

    // Compute Forces
    std::vector<float> force_x(N, 0.0f);
    std::vector<float> force_y(N, 0.0f);

    for (int i = 0; i < N; i++)
    {
        float fx = 0.0f;
        float fy = 0.0f;

        for (int j = 0; j < N; j++)
        {
            if (i == j)
                continue;

            float dx = ps.h_pos_x[i] - ps.h_pos_x[j];
            float dy = ps.h_pos_y[i] - ps.h_pos_y[j];
            float r = std::sqrt(dx * dx + dy * dy);

            // Handle very small distances to avoid singularities
            if (r < 0.0001f)
            {
                fx += (rand() % 100 / 100.0f - 0.5f) * 0.1f;
                fy += (rand() % 100 / 100.0f - 0.5f) * 0.1f;
                continue;
            }

            if (r < h)
            {
                // Pressure force
                float pressure_term = (ps.h_pressure[i] + ps.h_pressure[j]) / (2.0f * ps.h_density[j]);
                float force_p = -m * pressure_term * spiky_grad_coeff * std::pow(h - r, 2.0f);
                fx += force_p * (dx / r);
                fy += force_p * (dy / r);

                // Viscosity force
                float vx_diff = ps.h_vel_x[j] - ps.h_vel_x[i];
                float vy_diff = ps.h_vel_y[j] - ps.h_vel_y[i];
                float visc_term = cfg.viscosity * m / ps.h_density[j];

                float dot_product = (vx_diff * dx) + (vy_diff * dy);
                if (dot_product < 0.0f)
                {
                    visc_term *= 10.0f;
                }

                fx += visc_term * visc_lap_coeff * (h - r) * vx_diff;
                fy += visc_term * visc_lap_coeff * (h - r) * vy_diff;
            }
        }
        force_x[i] = fx + (cfg.gravity.x * ps.h_density[i]);
        force_y[i] = fy + (cfg.gravity.y * ps.h_density[i]);
    }

    // Integrate positions and velocities, and handle collisions with boundaries and obstacles
    float dt = cfg.time_step;
    float dampening = -0.2f; // Lose 80% energy on wall hit

    for (int i = 0; i < N; i++)
    {
        // Integration
        ps.h_vel_x[i] += dt * (force_x[i] / ps.h_density[i]);
        ps.h_vel_y[i] += dt * (force_y[i] / ps.h_density[i]);

        // Bleed off 0.1% of kinetic energy every frame to prevent numerical explosions
        ps.h_vel_x[i] *= 0.999f;
        ps.h_vel_y[i] *= 0.999f;

        ps.h_pos_x[i] += dt * ps.h_vel_x[i];
        ps.h_pos_y[i] += dt * ps.h_vel_y[i];

        float &x = ps.h_pos_x[i];
        float &y = ps.h_pos_y[i];
        float &vx = ps.h_vel_x[i];
        float &vy = ps.h_vel_y[i];

        // Container
        if (x < 0.0f)
        {
            x = 0.0f;
            vx *= dampening;
        }
        if (x > cfg.width)
        {
            x = cfg.width;
            vx *= dampening;
        }
        if (y < 0.0f)
        {
            y = 0.0f;
            vy *= dampening;
        }
        if (y > cfg.height)
        {
            y = cfg.height;
            vy *= dampening;
        }

        // Rectangle obstacles
        for (const auto &rect : cfg.obstracle_rectangles)
        {
            if (x > rect.x_min && x < rect.x_max && y > rect.y_min && y < rect.y_max)
            {
                float d_left = x - rect.x_min;
                float d_right = rect.x_max - x;
                float d_bottom = y - rect.y_min;
                float d_top = rect.y_max - y;

                float min_d = std::min({d_left, d_right, d_bottom, d_top});

                if (min_d == d_left)
                {
                    x = rect.x_min;
                    vx *= dampening;
                }
                else if (min_d == d_right)
                {
                    x = rect.x_max;
                    vx *= dampening;
                }
                else if (min_d == d_bottom)
                {
                    y = rect.y_min;
                    vy *= dampening;
                }
                else if (min_d == d_top)
                {
                    y = rect.y_max;
                    vy *= dampening;
                }
            }
        }

        // Circle obstacles
        for (const auto &circ : cfg.obstracle_circles)
        {
            float dx = x - circ.x_center;
            float dy = y - circ.y_center;
            float dist_squared = dx * dx + dy * dy;
            float r_squared = circ.radius * circ.radius;

            if (dist_squared < r_squared)
            {
                float dist = std::sqrt(dist_squared);
                if (dist == 0.0f)
                {
                    dist = 0.001f;
                    dx = 0.001f;
                    dy = 0.0f;
                }

                float normal_x = dx / dist;
                float normal_y = dy / dist;
                float overlap = circ.radius - dist;

                // Push out
                x += normal_x * overlap;
                y += normal_y * overlap;

                // Reflect velocity using dot product
                float dot = vx * normal_x + vy * normal_y;
                vx = (vx - 2.0f * dot * normal_x) * std::abs(dampening);
                vy = (vy - 2.0f * dot * normal_y) * std::abs(dampening);
            }
        }

        // Line obstacles
        float particle_radius = cfg.smoothing_radius * 0.5f;

        for (const auto &line : cfg.obstracle_lines)
        {
            float line_dx = line.x2 - line.x1;
            float line_dy = line.y2 - line.y1;
            float line_length_squared = line_dx * line_dx + line_dy * line_dy;

            if (line_length_squared == 0.0f)
                continue;

            float px_dx = x - line.x1;
            float py_dy = y - line.y1;

            float t = (px_dx * line_dx + py_dy * line_dy) / line_length_squared;

            // Clamp t to the segment ends
            t = std::max(0.0f, std::min(1.0f, t));

            float closest_x = line.x1 + t * line_dx;
            float closest_y = line.y1 + t * line_dy;

            float dist_dx = x - closest_x;
            float dist_dy = y - closest_y;
            float dist_squared = dist_dx * dist_dx + dist_dy * dist_dy;

            if (dist_squared < particle_radius * particle_radius)
            {
                float dist = std::sqrt(dist_squared);
                if (dist == 0.0f)
                {
                    dist = 0.001f;
                    dist_dx = 0.001f;
                    dist_dy = 0.0f;
                }

                float normal_x = dist_dx / dist;
                float normal_y = dist_dy / dist;

                float overlap = particle_radius - dist;
                x += normal_x * overlap;
                y += normal_y * overlap;

                float dot = vx * normal_x + vy * normal_y;

                if (dot < 0.0f)
                {
                    vx = (vx - 2.0f * dot * normal_x) * std::abs(dampening);
                    vy = (vy - 2.0f * dot * normal_y) * std::abs(dampening);
                }
            }
        }

        // Limit maximum speed to prevent numerical instability
        float max_speed = 50.0f;
        float speed_squared = vx * vx + vy * vy;
        if (speed_squared > max_speed * max_speed)
        {
            float speed = std::sqrt(speed_squared);
            vx = (vx / speed) * max_speed;
            vy = (vy / speed) * max_speed;
        }
    }
}