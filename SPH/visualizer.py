import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib.patches as patches
from matplotlib.colors import LinearSegmentedColormap
import argparse
import sys
import json

def read_binary_data(filepath):
    """
    Reads the custom binary format output by the C++ engine.
    Yields (x_array, y_array, frames_vx, frames_vy) for each frame.
    """
    frames_x = []
    frames_y = []
    frames_vx = []
    frames_vy = []
    
    try:
        with open(filepath, "rb") as f:
            while True:
                # Read the number of active particles
                count_bytes = f.read(4)
                if not count_bytes:
                    break
                
                count = np.frombuffer(count_bytes, dtype=np.int32)[0]
                
                if count == 0:
                    frames_x.append(np.array([]))
                    frames_y.append(np.array([]))
                    frames_vx.append(np.array([]))
                    frames_vy.append(np.array([]))
                    continue

                # Read the x and y positions and velocities
                x = np.frombuffer(f.read(4 * count), dtype=np.float32)
                y = np.frombuffer(f.read(4 * count), dtype=np.float32)
                vx = np.frombuffer(f.read(4 * count), dtype=np.float32)
                vy = np.frombuffer(f.read(4 * count), dtype=np.float32)
                
                frames_x.append(x)
                frames_y.append(y)
                frames_vx.append(vx)
                frames_vy.append(vy)

    except FileNotFoundError:
        print(f"Error: Could not find file {filepath}")
        sys.exit(1)
        
    return frames_x, frames_y, frames_vx, frames_vy

def main():
    parser = argparse.ArgumentParser(description="Fluid Simulation Visualizer")
    parser.add_argument("-i", "--input", type=str, default="results/simulation.dat")
    parser.add_argument("-o", "--output", type=str, default="results/animation.gif")
    parser.add_argument("-c", "--config", type=str, default="scenes/default.json")
    parser.add_argument("-fs", "--frame-stride", type=int, default=20, help="Number of frames to skip for faster visualization")
    args = parser.parse_args()

    # Read the JSON file to get container dimensions
    container_width = 1.0
    container_height = 1.0
    try:
        with open(args.config, 'r') as f:
            scene_data = json.load(f)
            if "container" in scene_data:
                container_width = scene_data["container"].get("width", 1.0)
                container_height = scene_data["container"].get("height", 1.0)
    except Exception as e:
        print(f"Warning: Could not read container size from {args.config}. Using defaults. Error: {e}")

    print(f"Reading data from {args.input}...")
    frames_x, frames_y, frames_vx, frames_vy = read_binary_data(args.input)

    frame_stride = args.frame_stride  # Skip frames to speed up visual time
    frames_x, frames_y, frames_vx, frames_vy = frames_x[::frame_stride], frames_y[::frame_stride], frames_vx[::frame_stride], frames_vy[::frame_stride]

    num_frames = len(frames_x)
    
    if num_frames == 0:
        print("No frames found in the data file.")
        return

    print(f"Successfully loaded {num_frames} frames. Rendering animation...")

    fig, ax = plt.subplots(figsize=(6 * (container_width/container_height), 6))
    
    ax.set_xlim(0.0, container_width) 
    ax.set_ylim(0.0, container_height)
    ax.set_aspect('equal')
    ax.set_title("SPH Fluid Simulation")


    try:
        with open(args.config, 'r') as f:
            scene_data = json.load(f)
            
            if "obstacles" in scene_data:
                for obs in scene_data["obstacles"]:
                    obs_type = obs.get("type")
                    
                    if obs_type == "rectangle":
                        b = obs["bounds"]
                        w = b["x_max"] - b["x_min"]
                        h = b["y_max"] - b["y_min"]
                        rect = patches.Rectangle((b["x_min"], b["y_min"]), w, h, 
                                                 linewidth=1, edgecolor='black', facecolor='darkgray')
                        ax.add_patch(rect)
                        
                    elif obs_type == "circle":
                        c = obs["center"]
                        r = obs["radius"]
                        circle = patches.Circle((c["x"], c["y"]), r, 
                                                linewidth=1, edgecolor='black', facecolor='darkgray')
                        ax.add_patch(circle)
                        
                    elif obs_type == "line":
                        start = obs["start"]
                        end = obs["end"]
                        ax.plot([start["x"], end["x"]], [start["y"], end["y"]], 
                                color='black', linewidth=3)
                                
    except Exception as e:
        print(f"Warning: Could not draw obstacles. Error: {e}")

    water_cmap = LinearSegmentedColormap.from_list('water_colors', ['darkblue', 'cyan'])
    scatter = ax.scatter([], [], c=[], s=40, cmap=water_cmap, vmin=0.0, vmax=6.0, alpha=0.6, edgecolors='none')
    
    def update(frame):
        x = frames_x[frame]
        y = frames_y[frame]
        vx = frames_vx[frame]
        vy = frames_vy[frame]
        
        speed = np.sqrt(vx**2 + vy**2)
        
        scatter.set_offsets(np.column_stack((x, y)))
        
        scatter.set_array(speed) 
        
        return scatter,

    ani = animation.FuncAnimation(fig, update, frames=num_frames, interval=16, blit=True)

    if args.output.endswith('.gif'):
        ani.save(args.output, writer='pillow', fps=60)
    else:
        ani.save(args.output, writer='ffmpeg', fps=60)
        
    print(f"Animation saved to {args.output}")

if __name__ == "__main__":
    main()