#pragma once

struct ParticleSystem {
     int max_particles;
     int active_count;

     // CPU Pointers
     float* h_pos_x;
     float* h_pos_y;
     float* h_vel_x;
     float* h_vel_y;

     float* h_density;
     float* h_pressure;
     

     // GPU Pointers
     float* d_pos_x;
     float* d_pos_y;
     float* d_vel_x;
     float* d_vel_y;

     float* d_density;
     float* d_pressure;
};