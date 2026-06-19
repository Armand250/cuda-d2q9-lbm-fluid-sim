#include "engine.h"
#include <iostream>
#include <cmath>
#include <chrono>

// Memory allocation for particles
void SimulationEngine::allocateMemory()
{
    int total_max = 0;
    float particle_spacing = cfg.smoothing_radius * 0.5f; // Particles sit closer than the smoothing radius

    // Calculate max particles needed
    for (const auto &src : cfg.sources)
    {
        if (src.type == FluidSourceType::BLOCK)
        {
            float width = src.bounds.x_max - src.bounds.x_min;
            float height = src.bounds.y_max - src.bounds.y_min;
            int count_x = std::max(1, (int)(width / particle_spacing));
            int count_y = std::max(1, (int)(height / particle_spacing));
            total_max += (count_x * count_y);
        }
        else if (src.type == FluidSourceType::EMITTER)
        {
            float active_time = std::min(src.active_until, cfg.max_frames * cfg.time_step);
            total_max += (int)(src.rate * active_time) + 100; // +100 buffer just in case
        }
    }

    ps.max_particles = total_max;
    std::cout << "Allocating memory for up to " << ps.max_particles << " particles.\n";

    // Allocate CPU Memory
    ps.h_pos_x = new float[total_max];
    ps.h_pos_y = new float[total_max];
    ps.h_vel_x = new float[total_max];
    ps.h_vel_y = new float[total_max];
    ps.h_density = new float[total_max];
    ps.h_pressure = new float[total_max];

    // Allocate GPU Memory
    if (active_solver->getExecutionMode() == ExecutionMode::GPU)
    {
        cudaMalloc(&ps.d_pos_x, total_max * sizeof(float));
        cudaMalloc(&ps.d_pos_y, total_max * sizeof(float));
        cudaMalloc(&ps.d_vel_x, total_max * sizeof(float));
        cudaMalloc(&ps.d_vel_y, total_max * sizeof(float));
        cudaMalloc(&ps.d_density, total_max * sizeof(float));
        cudaMalloc(&ps.d_pressure, total_max * sizeof(float));
    }
}

// Spawning block particles at the start of the simulation
void SimulationEngine::spawnInitialBlocks()
{
    ps.active_count = 0;
    float spacing = cfg.smoothing_radius * 0.5f;

    for (const auto &src : cfg.sources)
    {
        if (src.type == FluidSourceType::BLOCK)
        {
            for (float x = src.bounds.x_min; x <= src.bounds.x_max; x += spacing)
            {
                for (float y = src.bounds.y_min; y <= src.bounds.y_max; y += spacing)
                {

                    if (ps.active_count >= ps.max_particles)
                        break;

                    int i = ps.active_count;
                    ps.h_pos_x[i] = x;
                    ps.h_pos_y[i] = y;
                    ps.h_vel_x[i] = 0.0f;
                    ps.h_vel_y[i] = 0.0f;
                    ps.h_density[i] = cfg.rest_density;
                    ps.h_pressure[i] = 0.0f;

                    ps.active_count++;
                }
            }
        }
    }
    std::cout << "Spawned " << ps.active_count << " initial block particles.\n";

    // In case of GPU execution, copy to its memory
    if (active_solver->getExecutionMode() == ExecutionMode::GPU)
    {
        cudaMemcpy(ps.d_pos_x, ps.h_pos_x, ps.active_count * sizeof(float), cudaMemcpyHostToDevice);
        cudaMemcpy(ps.d_pos_y, ps.h_pos_y, ps.active_count * sizeof(float), cudaMemcpyHostToDevice);
        cudaMemcpy(ps.d_vel_x, ps.h_vel_x, ps.active_count * sizeof(float), cudaMemcpyHostToDevice);
        cudaMemcpy(ps.d_vel_y, ps.h_vel_y, ps.active_count * sizeof(float), cudaMemcpyHostToDevice);
    }
}

// Handle emitters every frame
void SimulationEngine::handleEmitters()
{
    bool spawned_any = false;
    int start_index = ps.active_count;
    int newly_spawned = 0;

    for (const auto &src : cfg.sources)
    {
        if (src.type == FluidSourceType::EMITTER && current_time < src.active_until)
        {

            int spawn_this_frame = std::round(src.rate * cfg.time_step);

            for (int k = 0; k < spawn_this_frame; k++)
            {
                if (ps.active_count >= ps.max_particles)
                    break;

                int i = ps.active_count;

                // Randomly position the new particle within the emitter bounds
                float random_x = src.bounds.x_min + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / (src.bounds.x_max - src.bounds.x_min)));
                float random_y = src.bounds.y_min + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / (src.bounds.y_max - src.bounds.y_min)));

                ps.h_pos_x[i] = random_x;
                ps.h_pos_y[i] = random_y;

                // Settinng the initial velocity based on the emitter's direction and speed
                float dir_mag = std::sqrt(src.direction.x * src.direction.x + src.direction.y * src.direction.y);
                if (dir_mag > 0.0001f)
                {
                    ps.h_vel_x[i] = (src.direction.x / dir_mag) * src.speed;
                    ps.h_vel_y[i] = (src.direction.y / dir_mag) * src.speed;
                }
                else
                {
                    ps.h_vel_x[i] = 0.0f;
                    ps.h_vel_y[i] = -src.speed;
                }

                ps.h_density[i] = cfg.rest_density;
                ps.h_pressure[i] = 0.0f;

                ps.active_count++;
                newly_spawned++;
                spawned_any = true;
            }
        }
    }

    // In case of GPU execution, copy the newly spawned particles to its memory
    if (spawned_any && active_solver->getExecutionMode() == ExecutionMode::GPU)
    {
        // Copy only the newly spawned particles to GPU memory
        size_t bytes = newly_spawned * sizeof(float);

        cudaMemcpy(ps.d_pos_x + start_index, ps.h_pos_x + start_index, bytes, cudaMemcpyHostToDevice);
        cudaMemcpy(ps.d_pos_y + start_index, ps.h_pos_y + start_index, bytes, cudaMemcpyHostToDevice);
        cudaMemcpy(ps.d_vel_x + start_index, ps.h_vel_x + start_index, bytes, cudaMemcpyHostToDevice);
        cudaMemcpy(ps.d_vel_y + start_index, ps.h_vel_y + start_index, bytes, cudaMemcpyHostToDevice);
        cudaMemcpy(ps.d_density + start_index, ps.h_density + start_index, bytes, cudaMemcpyHostToDevice);
        cudaMemcpy(ps.d_pressure + start_index, ps.h_pressure + start_index, bytes, cudaMemcpyHostToDevice);
    }
}

// Initialize the binary output file for exporting simulation data
void SimulationEngine::initializeExport(const std::string &filepath)
{
    output_file = fopen(filepath.c_str(), "wb");
    if (!output_file)
    {
        throw std::runtime_error("Failed to open binary file for writing.");
    }
}

// Export the current frame's particle data to the binary file
void SimulationEngine::exportFrame()
{
    if (!output_file)
        return;

    fwrite(&ps.active_count, sizeof(int), 1, output_file);
    fwrite(ps.h_pos_x, sizeof(float), ps.active_count, output_file);
    fwrite(ps.h_pos_y, sizeof(float), ps.active_count, output_file);

    fwrite(ps.h_vel_x, sizeof(float), ps.active_count, output_file);
    fwrite(ps.h_vel_y, sizeof(float), ps.active_count, output_file);
}

// Main simulation loop
void SimulationEngine::run()
{
    ExecutionMode mode = active_solver->getExecutionMode();

    // Benchmarking variables
    double total_physics_time = 0.0;
    long long total_particle_updates = 0;

    std::cout << "Starting Simulation: " << active_solver->getName() << "\n";

    while (current_frame < cfg.max_frames)
    {
        handleEmitters();

        bool is_target_frame = cfg.enable_profiling && (current_frame == 0 || 
                                current_frame == cfg.max_frames / 2 || 
                                current_frame == cfg.max_frames - 1);
        if (mode == ExecutionMode::GPU && is_target_frame) {
            cudaProfilerStart();
        }

        auto start_time = std::chrono::high_resolution_clock::now();

        active_solver->step(ps, cfg);

        if (mode == ExecutionMode::GPU)
        {
            cudaDeviceSynchronize();
        }
        if (mode == ExecutionMode::GPU && is_target_frame) {
            cudaProfilerStop();
        }
        auto end_time = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> frame_time = end_time - start_time;
        total_physics_time += frame_time.count();
        total_particle_updates += ps.active_count;

        // Copy data back to CPU for exporting
        if (mode == ExecutionMode::GPU)
        {
            cudaMemcpy(ps.h_pos_x, ps.d_pos_x, ps.active_count * sizeof(float), cudaMemcpyDeviceToHost);
            cudaMemcpy(ps.h_pos_y, ps.d_pos_y, ps.active_count * sizeof(float), cudaMemcpyDeviceToHost);
            cudaMemcpy(ps.h_vel_x, ps.d_vel_x, ps.active_count * sizeof(float), cudaMemcpyDeviceToHost);
            cudaMemcpy(ps.h_vel_y, ps.d_vel_y, ps.active_count * sizeof(float), cudaMemcpyDeviceToHost);
        }
        exportFrame();

        current_frame++;
        current_time += cfg.time_step;

        if (current_frame % (cfg.max_frames / 10) == 0)
        {
            std::cout << "Progress: " << (current_frame * 100 / cfg.max_frames) << "%\n";
        }
    }

    // Finalize and report performance metrics
    double avg_fps = cfg.max_frames / total_physics_time;
    double throughput_mups = (total_particle_updates / total_physics_time) / 1000000.0;

    std::cout << "\n=== SIMULATION COMPLETE ===\n";
    std::cout << "Solver:       " << active_solver->getName() << "\n";
    std::cout << "Total Frames: " << cfg.max_frames << "\n";
    std::cout << "Physics Time: " << total_physics_time << " seconds\n";
    std::cout << "Physics FPS:  " << avg_fps << " frames/sec\n";
    std::cout << "Throughput:   " << throughput_mups << " MUPS\n"; // Million Updates Per Second
    std::cout << "===========================\n";

    if (cfg.save_metrics) {
        std::string csv_path = cfg.output_path;
        size_t dot_pos = csv_path.find_last_of('.');
        if (dot_pos != std::string::npos) {
            csv_path = csv_path.substr(0, dot_pos) + ".csv";
        } else {
            csv_path += ".csv";
        }

        std::ofstream csv_file(csv_path);
        if (csv_file.is_open()) {
            csv_file << "solver,frames,particles,time,fps,mups\n";
            csv_file << active_solver->getName() << ","
                     << cfg.max_frames << ","
                     << ps.active_count << ","
                     << total_physics_time << ","
                     << avg_fps << ","
                     << throughput_mups << "\n";
            csv_file.close();
            std::cout << "Metrics successfully saved to: " << csv_path << "\n";
        } else {
            std::cerr << "Warning: Could not open CSV file for writing: " << csv_path << "\n";
        }
    }
}