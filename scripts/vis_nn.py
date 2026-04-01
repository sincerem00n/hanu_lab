import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import matplotlib.patches as patches

def main():
    # Network architecture: 3 inputs, 3 hidden neurons, 2 output neurons
    layers = [3, 5, 2]
    
    fig = plt.figure(figsize=(12, 9))
    fig.patch.set_facecolor('#f0f0f5')
    
    gs = fig.add_gridspec(2, 1, height_ratios=[2, 1])
    ax_net = fig.add_subplot(gs[0])
    ax_net.set_facecolor('#f0f0f5')
    
    ax_log = fig.add_subplot(gs[1])
    ax_log.set_facecolor('white')
    ax_log.set_title("Logging Data: Activations over Time", fontweight='bold')
    ax_log.set_xlim(0, 120)
    ax_log.set_ylim(-0.1, 1.1)
    ax_log.set_xlabel("Time (frames)")
    ax_log.set_ylabel("Activation / Value")
    ax_log.grid(True, linestyle='--', alpha=0.5)
    
    # Calculate positions for nodes
    def get_positions():
        pos = []
        max_nodes = max(layers)
        for i, n in enumerate(layers):
            layer_pos = []
            y_offset = (max_nodes - n) / 2.0
            for j in range(n):
                layer_pos.append((i * 3, y_offset + j))
            pos.append(layer_pos)
        return pos
    
    positions = get_positions()
    
    # Initialize mock model weights randomly
    np.random.seed(42)
    weights = [
        np.random.randn(layers[0], layers[1]),
        np.random.randn(layers[1], layers[2])
    ]
    # Make the weights from the 3rd input node explicitly strong 
    # so the added observation creates a noticeable change in output
    weights[0][2, :] = np.array([3.0, -2.5, 2.0, -3.0, 1.5])
    
    def sigmoid(x):
        return 1 / (1 + np.exp(-x))
    
    def forward(x, inject_mode=False):
        """Simulate a forward pass."""
        if not inject_mode:
            # Empty slot: zero out the last input neuron
            x[2] = 0.0
            
        a1 = sigmoid(np.dot(x, weights[0]))
        a2 = sigmoid(np.dot(a1, weights[1]))
        return [x, a1, a2]
    
    # Setup visualization elements
    node_circles = []
    edge_lines = []
    
    # Draw edges first so they appear behind nodes
    for i in range(len(layers)-1):
        layer_edges = []
        for j in range(layers[i]):
            node_edges = []
            for k in range(layers[i+1]):
                line, = ax_net.plot([positions[i][j][0], positions[i+1][k][0]], 
                                [positions[i][j][1], positions[i+1][k][1]], 
                                '-', color='#999999', zorder=1, alpha=0.3)
                node_edges.append(line)
            layer_edges.append(node_edges)
        edge_lines.append(layer_edges)
        
    # Draw nodes
    for i in range(len(layers)):
        layer_circles = []
        for j in range(layers[i]):
            circle = patches.Circle(positions[i][j], radius=0.25, ec='#333333', fc='white', lw=1.5, zorder=4)
            ax_net.add_patch(circle)
            layer_circles.append(circle)
            
            # Label layers
            if i == 0:
                if j == 0:
                    ax_net.text(positions[i][j][0]-1, positions[i][j][1], "Base Obs 1", ha='right', va='center', fontweight='bold')
                elif j == 1:
                    ax_net.text(positions[i][j][0]-1, positions[i][j][1], "Base Obs 2", ha='right', va='center', fontweight='bold')
                elif j == 2:
                    ax_net.text(positions[i][j][0]-1, positions[i][j][1], "Added Obs\n(Injected later)", ha='right', va='center', fontweight='bold', color='#2ca02c')
            elif j == 0 and i == 1:
                ax_net.text(positions[i][j][0], positions[i][j][1] - 0.7, "Hidden Layer", ha='center', fontweight='bold')
            elif j == 0 and i == 2:
                ax_net.text(positions[i][j][0] + 0.5, positions[i][j][1], "Output", ha='left', va='center', fontweight='bold')
                
        node_circles.append(layer_circles)
        
    ax_net.set_aspect('equal')
    ax_net.axis('off')
    
    title_text = ax_net.text(3, 3.5, "Neural Network Simulation [3x3x2]", 
                         ha='center', va='center', fontsize=16, fontweight='bold',
                         bbox=dict(facecolor='white', alpha=0.8, edgecolor='none', pad=5))
    
    status_text = ax_net.text(3, -0.8, "", ha='center', va='center', fontsize=12,
                          bbox=dict(facecolor='white', alpha=0.8, edgecolor='none', pad=5))
    
    # Lines for the logging graph
    frames_history = []
    out1_history = []
    out2_history = []
    injected_history = []
    
    line_out1, = ax_log.plot([], [], label='Output 1 Simulation', color='#1f77b4', lw=2)
    line_out2, = ax_log.plot([], [], label='Output 2 Simulation', color='#ff7f0e', lw=2)
    line_inj, = ax_log.plot([], [], label='Added Obs Value', color='#2ca02c', lw=2, linestyle='--')
    ax_log.legend(loc='lower left')

    def update(frame):
        # Frame 0 to 60: Training with empty slot (Input node 3 is empty/0)
        # Frame 60 to 120: Injected value begins to appear
        
        inject_mode = frame >= 60
        
        # Simulated continuous input data
        input_val = np.array([
            np.sin(frame * 0.1) * 0.5 + 0.5, 
            np.cos(frame * 0.15) * 0.5 + 0.5, 
            0.0 # Will be overridden if injected
        ])
        
        if not inject_mode:
            input_val[2] = 0.0 # Force empty slot
            status_text.set_text("Phase 1: Base Observations Only (Slot Empty)")
            status_text.set_color('#d9534f') # Red
            node_circles[0][2].set_edgecolor('#d9534f')
            node_circles[0][2].set_linewidth(3)
            node_circles[0][2].set_linestyle('--')
        else:
            status_text.set_text("Phase 2: Later Added Observation Term Injected!")
            status_text.set_color('#5cb85c') # Green
            node_circles[0][2].set_edgecolor('#5cb85c')
            node_circles[0][2].set_linewidth(3)
            node_circles[0][2].set_linestyle('-')
            
            # Injecting the new observation term dynamically
            input_val[2] = np.sin(frame * 0.4) * 0.5 + 0.5
            
        activations = forward(input_val, inject_mode)
        
        # Logging updates - store data for plot limits if needed, but here we just append
        frames_history.append(frame)
        out1_history.append(activations[2][0])
        out2_history.append(activations[2][1])
        injected_history.append(activations[0][2])
        
        line_out1.set_data(frames_history, out1_history)
        line_out2.set_data(frames_history, out2_history)
        line_inj.set_data(frames_history, injected_history)

        # Update styling based on activations
        for i in range(len(layers)):
            for j in range(layers[i]):
                val = activations[i][j]
                
                # Input empty slot special case color
                if i == 0 and j == 2 and not inject_mode:
                    node_circles[i][j].set_facecolor('#e0e0e0') # Grey out
                else:
                    # Color map from white to vibrant blue
                    node_circles[i][j].set_facecolor(plt.cm.Blues(val * 0.8 + 0.1))
                    
        # Update edges based on weight strength & activation
        for i in range(len(layers)-1):
            for j in range(layers[i]):
                for k in range(layers[i+1]):
                    w = abs(weights[i][j, k])
                    act = activations[i][j]
                    
                    if i == 0 and j == 2 and not inject_mode:
                        # Dead edges for empty slot
                        edge_lines[i][j][k].set_linewidth(1)
                        edge_lines[i][j][k].set_alpha(0.1)
                        edge_lines[i][j][k].set_color('#cccccc')
                    else:
                        # Active edges
                        edge_intensity = w * act
                        edge_lines[i][j][k].set_linewidth(1 + 3 * edge_intensity)
                        edge_lines[i][j][k].set_alpha(0.3 + 0.6 * act)
                        edge_lines[i][j][k].set_color(plt.cm.plasma(edge_intensity * 0.5))
                        
        # Ensure we return a flattened list of artists for blitting
        artists = sum(node_circles, []) + sum(sum(edge_lines, []), []) + [title_text, status_text, line_out1, line_out2, line_inj]
        return artists

    # Create the animation
    import os
    save_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'logs', 'vis')
    os.makedirs(save_dir, exist_ok=True)
    save_path = os.path.join(save_dir, 'nn_simulation.mp4')
    
    print(f"Generating network animation and saving to: {save_path}")
    print("This may take 10-20 seconds...")
    ani = FuncAnimation(fig, update, frames=120, interval=100, blit=True, repeat=False)
    
    plt.tight_layout()
    try:
        ani.save(save_path, writer='ffmpeg', fps=10)
        print("Video saved successfully!")
    except Exception as e:
        print(f"Failed to save video (is ffmpeg installed?): {e}")
        print("Trying to save as GIF instead...")
        try:
            gif_path = save_path.replace('.mp4', '.gif')
            ani.save(gif_path, writer='pillow', fps=10)
            print(f"Saved as GIF successfully: {gif_path}")
        except Exception as e2:
            print(f"Failed to save as GIF as well: {e2}")
            
    # Uncomment next line to show the pop-up graph after saving, if desired:
    # plt.show()

if __name__ == "__main__":
    main()
