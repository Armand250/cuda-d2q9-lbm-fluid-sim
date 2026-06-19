#include <iostream>
#include "cli_parser.h"
#include "scene_reader.h"
#include "engine.h"
#include "solver_factory.h"

int main(int argc, char** argv) {
    try {
        CLIParser cli;
        cli.parse(argc, argv);

        std::cout << "Starting Simulation...\n";
        std::cout << "Config: " << cli.config_path << "\n";
        std::cout << "Solver: " << cli.solver_type << "\n";
        std::cout << "Output: " << cli.output_path << "\n";
        std::cout << "Profiling: " << (cli.enable_profiling ? "Enabled" : "Disabled") << "\n";
        std::cout << "Metrics: " << (cli.save_metrics ? "Enabled" : "Disabled") << "\n";

        Config config = SceneReader::load(cli.config_path);
        config.enable_profiling = cli.enable_profiling;
        config.save_metrics = cli.save_metrics;
        config.output_path = cli.output_path;
        SimulationEngine engine(config, SolverFactory::create(cli.solver_type));
        engine.run();

        std::cout << "Simulation completed successfully.\n";

    } catch (const std::exception& e) {
        std::cerr << "Fatal Error: " << e.what() << "\n";
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}