#pragma once
#include <string>
#include <iostream>
#include <cstdlib>

class CLIParser
{
public:
    std::string config_path = "default.json";
    std::string solver_type = "gpu-shared";
    std::string output_path = "simulation.dat";
    bool enable_profiling = false;
    bool save_metrics = false;

    // Print usage information
    static void printUsage(const char *program_name)
    {
        std::cout << "\n===================================================\n";
        std::cout << "  CUDA Fluid Simulator - Command Line Interface\n";
        std::cout << "===================================================\n\n";
        std::cout << "Usage: " << program_name << " [options]\n\n";
        std::cout << "Options:\n";
        std::cout << "  -h, --help                 Show this help message and exit.\n";
        std::cout << "  -c, --config <filepath>    Path to the JSON scene file.\n";
        std::cout << "  -o, --output <filepath>    Path to save the binary output data.\n";
        std::cout << "                             (Default: results/simulation.dat)\n";
        std::cout << "  -s, --solver <type>        Select the physics solver execution type.\n";
        std::cout << "                             (Default: gpu-shared)\n\n";
        std::cout << "Solver Types:\n";
        std::cout << "  cpu-naive                  CPU-based naive solver.\n";
        std::cout << "  gpu-naive                  GPU-based naive solver.\n";
        std::cout << "  gpu-shared                 GPU-based optimized solver using shared memory.\n";
        std::cout << "  -p, --profiling            Enable CUDA profiling for performance analysis.\n";
        std::cout << "  -m, --metrics              Save performance metrics to a file.\n";

    }

    // Parse command line arguments
    void parse(int argc, char **argv)
    {
        for (int i = 1; i < argc; ++i)
        {
            std::string arg = argv[i];

            if (arg == "-h" || arg == "--help")
            {
                printUsage(argv[0]);
                exit(EXIT_SUCCESS);
            }
            else if ((arg == "-c" || arg == "--config") && i + 1 < argc)
            {
                config_path = argv[++i];
            }
            else if ((arg == "-o" || arg == "--output") && i + 1 < argc)
            {
                output_path = argv[++i];
            }
            else if ((arg == "-s" || arg == "--solver") && i + 1 < argc)
            {
                solver_type = argv[++i];
                if (solver_type != "cpu-naive" && solver_type != "gpu-naive" && solver_type != "gpu-shared")
                {
                    std::cerr << "\n[ERROR] Unknown solver type: '" << solver_type << "'\n";
                    printUsage(argv[0]);
                    exit(EXIT_FAILURE);
                }
            }
            else if (arg == "-p" || arg == "--profiling")
            {
                enable_profiling = true;
            }
            else if (arg == "-m" || arg == "--metrics")
            {
                save_metrics = true;
            }
            else
            {
                std::cerr << "\n[ERROR] Invalid argument or missing parameter: '" << arg << "'\n";
                printUsage(argv[0]);
                exit(EXIT_FAILURE);
            }
        }
    }
};