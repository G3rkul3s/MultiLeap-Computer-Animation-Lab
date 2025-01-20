import json
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider

# Path to the JSON file
filename = "results/fused_hand.json"

# Read and parse the JSON file
with open(filename, "r") as file:
    data = json.load(file)

# Total number of frames
total_frames = len(data["frame_data"])

# Define a mapping of sensor_id to colors
sensor_colors = {
    1: 'blue',
    2: 'green',
    3: 'red',
    4: 'yellow',
}

# Create a 3D plot
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Function to visualize 3D points dynamically
def plot_frame(frame_idx):
    # Iterate through each frame of data
    frame = data["frame_data"][frame_idx]
    right_hand = frame["right_hand"]
    annotation = frame["annotation"]
    confidence = annotation.get("confidence_right_hand")
    sensor_id = annotation.get("sensor_id")
    timestamp = annotation.get("timestamp")
    # Determine the color based on sensor_id
    color = sensor_colors.get(sensor_id)

    # Clear the plot for the next frame
    ax.clear()

    # Extract X, Y, Z coordinates
    xs, ys, zs = zip(*right_hand)

    # Plot the 3D points
    ax.scatter(xs, ys, zs, c=color, marker='o')

    # Loop through each point and normal
    for sensor in data["sensor_data"]:
        # Plot the point
        point = sensor["position"]
        normal = sensor["normal"]
        ax.scatter(point[0], point[1], point[2], color=sensor_colors.get(sensor["sensor_id"]))
        
        # Plot the normal vector
        ax.quiver(
            point[0], point[1], point[2],  # Starting point
            normal[0], normal[1], normal[2],  # Direction
            length=50, color=sensor_colors.get(sensor["sensor_id"]), normalize=True
        )
    
    # Set labels and plot title
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.set_title(f"Confidence {round(confidence, 2)}\nTimestamp: {timestamp}")

    # Optionally set limits (adjust based on your data range)
    ax.set_xlim([-100, 400])
    ax.set_ylim([0, 700])
    ax.set_zlim([-500, 300])
    
    plt.draw()



def main():
    # Function to handle key press events
    def on_key(event):
        if event.key == "right":
            # Move to the next frame
            if slider.val + 1 > total_frames - 1:
                slider.set_val(0)
            else:
                slider.set_val(slider.val + 1)
        elif event.key == "left":
            # Move to the previous frame
            if slider.val - 1 < 1:
                slider.set_val(total_frames - 1)
            else:
                slider.set_val(slider.val - 1)
        # Plot the new frame
        plot_frame(slider.val)
    
    if total_frames == 0:
        print("Zero recorded frames")
        return
    # Bind the key press event
    fig.canvas.mpl_connect("key_press_event", on_key)

    # Create the slider for frame selection
    ax_slider = plt.axes([0.1, 0.01, 0.8, 0.05])  # Slider position
    slider = Slider(ax_slider, 'Frame', 1, total_frames - 1, valinit=0, valstep=1)
    # Attach the update function to the slider
    slider.on_changed(plot_frame)
    
    # Plot the initial frame
    plot_frame(slider.val)
    plt.show()
    # visualize_right_hand(data)

if __name__ == "__main__":
    main()