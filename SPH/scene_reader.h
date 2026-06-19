#pragma once
#include <fstream>
#include "json.hpp"
#include "config.h"

using json = nlohmann::json;

class SceneReader {
public:
    static Config load(const std::string& filepath) {
        std::ifstream file(filepath);
        if (!file.is_open()) throw std::runtime_error("Cannot open config file.");
        
        json j;
        file >> j;

        // Parse enviroment and physics parameters
        Config cfg;
        cfg.max_frames = j["physics"]["max_frames"];
        cfg.time_step = j["physics"]["time_step"];
        cfg.gravity.x = j["physics"]["gravity"]["x"];
        cfg.gravity.y = j["physics"]["gravity"]["y"];
        cfg.rest_density = j["fluid_properties"]["rest_density"];
        cfg.viscosity = j["fluid_properties"]["viscosity"];
        cfg.particle_mass = j["fluid_properties"]["particle_mass"];
        cfg.smoothing_radius = j["fluid_properties"]["smoothing_radius"];
        cfg.width = j["container"]["width"];
        cfg.height = j["container"]["height"];

        // Parse fluid sources
        for (const auto& src : j["fluid_sources"]) {
            FluidSource source;
            source.type = src["type"] == "block" ? FluidSourceType::BLOCK : FluidSourceType::EMITTER;
            if (source.type == FluidSourceType::BLOCK) {
                source.bounds.x_min = src["bounds"]["x_min"];
                source.bounds.x_max = src["bounds"]["x_max"];
                source.bounds.y_min = src["bounds"]["y_min"];
                source.bounds.y_max = src["bounds"]["y_max"];
            } else if (source.type == FluidSourceType::EMITTER) {
                source.bounds.x_min = src["bounds"]["x_min"];
                source.bounds.x_max = src["bounds"]["x_max"];
                source.bounds.y_min = src["bounds"]["y_min"];
                source.bounds.y_max = src["bounds"]["y_max"];
                source.direction.x = src["direction"]["x"];
                source.direction.y = src["direction"]["y"];
                source.rate = src["rate"];
                source.speed = src["speed"];
                source.active_until = src["active_until"];
            }
            cfg.sources.push_back(source);
        }

        // Parse obstacles
        for (const auto& obst : j["obstacles"]) {
            std::string type = obst["type"];
            if (type == "rectangle") {
                Bounds b;
                b.x_min = obst["bounds"]["x_min"];
                b.x_max = obst["bounds"]["x_max"];
                b.y_min = obst["bounds"]["y_min"];
                b.y_max = obst["bounds"]["y_max"];
                cfg.obstracle_rectangles.push_back(b);
            } else if (type == "circle") {
                ObstracleCircle c;
                c.x_center = obst["center"]["x"];
                c.y_center = obst["center"]["y"];
                c.radius = obst["radius"];
                cfg.obstracle_circles.push_back(c);
            } else if (type == "line") {
                ObstracleLine l;
                l.x1 = obst["start"]["x"];
                l.y1 = obst["start"]["y"];
                l.x2 = obst["end"]["x"];
                l.y2 = obst["end"]["y"];
                cfg.obstracle_lines.push_back(l);
            }
        }

        return cfg;
    }
};