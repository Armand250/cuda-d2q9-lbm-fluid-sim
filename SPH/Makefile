# ==========================================
# CUDA Fluid Simulator Makefile
# ==========================================

# Compiler settings
CXX = nvcc
# -O3: Maximum optimization (CRITICAL for benchmarking)
# -std=c++17: Modern C++ features
CXXFLAGS = -O3 -std=c++17 -lineinfo

# Directories
OBJ_DIR = obj
BIN_DIR = bin
RESULTS_DIR = results
SCENES_DIR = scenes

# Target executable
TARGET = $(BIN_DIR)/fluid_sim

# Automatically find all .cpp and .cu files in the current directory
CPP_SRCS = $(wildcard *.cpp)
CU_SRCS  = $(wildcard *.cu)

# Generate corresponding .o file names in the obj/ directory
CPP_OBJS = $(patsubst %.cpp, $(OBJ_DIR)/%.o, $(CPP_SRCS))
CU_OBJS  = $(patsubst %.cu, $(OBJ_DIR)/%.o, $(CU_SRCS))
OBJS = $(CPP_OBJS) $(CU_OBJS)

# Default rule (what runs when you just type 'make')
all: directories $(TARGET)

# Rule to create necessary folders if they don't exist
directories:
	@mkdir -p $(OBJ_DIR)
	@mkdir -p $(BIN_DIR)
	@mkdir -p $(RESULTS_DIR)
	@mkdir -p $(SCENES_DIR)

# Linking the final executable
$(TARGET): $(OBJS)
	@echo "Linking..."
	$(CXX) $(CXXFLAGS) -o $@ $^
	@echo "Build successful! Run with: ./$(TARGET)"

# Compiling standard C++ files
$(OBJ_DIR)/%.o: %.cpp
	@echo "Compiling C++: $<"
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Compiling CUDA files
$(OBJ_DIR)/%.o: %.cu
	@echo "Compiling CUDA: $<"
	$(CXX) $(CXXFLAGS) -c $< -o $@

clean:
	@echo "Cleaning up..."
	rm -rf $(OBJ_DIR) $(BIN_DIR) $(RESULTS_DIR)/*.dat

.PHONY: all directories clean