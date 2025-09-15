import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
from matplotlib.patches import Arc

# --- 1. Setup the figure and axes ---
fig, ax = plt.subplots(figsize=(8, 8))
plt.subplots_adjust(bottom=0.25) # Adjust subplot to make room for the slider

# Set plot appearance
ax.set_title('Lie Group SO(2) and Lie Algebra so(2)')
# ax.set_xlabel('x')
# ax.set_ylabel('y')
ax.set_xlim(-1.5, 1.5)
ax.set_ylim(-1.5, 1.5)
ax.set_aspect('equal', adjustable='box')
ax.grid(True, linestyle='--', alpha=0.6)

# Remove axis tick labels
ax.set_xticklabels([])
ax.set_yticklabels([])

# --- 2. Draw static elements ---

# Lie Group J (SO(2)) - The Unit Circle
circle = plt.Circle((0, 0), 1, color='black', fill=False, linewidth=1.5)
ax.add_artist(circle)
ax.text(-0.9, 0.9, 'SO(2)', fontsize=14, color='black')

# Identity element 'I'
ax.plot(1, 0, 'ko', markersize=8)
ax.text(1.05, -0.1, 'I', fontsize=14, color='black')

# Lie Algebra g (so(2)) - The Tangent Space at 'p'
ax.axvline(x=1, color='gray', linestyle='--')
ax.text(1.05, 1.1, 'so(2)', fontsize=14, color='black')

# Basis vector 'J' in the Lie Algebra
# This is a vector in the tangent space so(2) at the identity element p=(1,0)
# FIX: Added length_includes_head=True to ensure the arrow tip is exactly at y=1.0
ax.arrow(1, 0, 0, 1, head_width=0.05, head_length=0.1, fc='black', ec='black', length_includes_head=True)
# The label is now placed to the left of the tangent line for clarity
ax.text(0.95, 0.8, 'J', fontsize=14, color='black', ha='right')

# --- 3. Setup interactive elements ---
initial_theta = 1.0

# Define the slider's position and create it
ax_slider = plt.axes([0.2, 0.1, 0.65, 0.03], facecolor='lightgoldenrodyellow')
theta_slider = Slider(
    ax=ax_slider,
    label=r'$\theta$',
    valmin=-2 * np.pi,
    valmax=2 * np.pi,
    valinit=initial_theta,
)
# Add a vertical line at theta = 0 for reference
ax_slider.axvline(x=0, color='gray', linestyle='-', linewidth=1)


# --- 4. Define dynamic elements that will be updated ---
# These are initialized here and will be updated by the slider function
# Vector theta*J in the Lie Algebra
# FIX: Added length_includes_head=True
theta_G_arrow = ax.arrow(1, 0, 0, initial_theta, head_width=0.05, head_length=0.1, fc='green', ec='green', length_includes_head=True)
# The label is now placed to the left of the tangent line for clarity
theta_G_text = ax.text(0.95, initial_theta, r'$\theta J$', fontsize=12, color='green', ha='right')

# R = exp(theta*J) on the Lie Group
R11 = np.cos(initial_theta)
R12 = -np.sin(initial_theta)
R21 = np.sin(initial_theta)
R22 = np.cos(initial_theta)
R = np.array([[R11, R12],
                [R21, R22]])

# Rotate the point
p = np.array([1, 0])
px, py = R @ p

exp_point, = ax.plot(px, py, 'ro', markersize=10)
exp_text = ax.text(px + 0.1, py + 0.1, r'$R = exp(\theta J)$', fontsize=12, color='red')

# Radius from origin to exp(theta*J)
# radius_line, = ax.plot([0, px], [0, py], 'r--', linewidth=1)

# Arc from identity 'p' to exp(theta*J)
# We need to handle the angle for the Arc patch correctly
arc_patch = Arc((0,0), 2, 2, angle=0, theta1=0, theta2=np.rad2deg(initial_theta), color='red', linewidth=2)
ax.add_patch(arc_patch)

# Text display for current values
value_text = ax.text(-1.4, 1.3, f'$\\theta = {initial_theta:.2f}$\n$R = \exp(\\theta J) = ({R11:.2f}, {R12:.2f}, {R21:.2f}, {R22:.2f})$', 
                     bbox=dict(facecolor='white', alpha=0.5, edgecolor='gray'))


# --- 5. The update function ---
def update(val):
    global arc_patch # We need to modify the patch object
    
    # Get the new theta value from the slider
    theta = theta_slider.val
    
    # Update the vector theta*J
    # We need to remove the old arrow and create a new one, as updating quiver/arrow properties is complex
    theta_G_arrow.remove()
    # FIX: Added length_includes_head=True
    # Handle the case where theta is too small to draw a head
    if abs(theta) < 0.01: # Avoid drawing an arrow if it's too small
        globals()['theta_G_arrow'] = ax.arrow(1, 0, 0, 0, head_width=0.05, head_length=0.1, fc='green', ec='green', length_includes_head=True)
    else:
        globals()['theta_G_arrow'] = ax.arrow(1, 0, 0, theta, head_width=0.05, head_length=0.1, fc='green', ec='green', length_includes_head=True)

    # Update the label position to the left of the tangent line
    theta_G_text.set_position((0.95, theta + np.sign(theta)*0.1 if theta != 0 else 0.1)) # Adjust text position
    
    R11 = np.cos(theta)
    R12 = -np.sin(theta)
    R21 = np.sin(theta)
    R22 = np.cos(theta)

    # R = exp(theta*J) on the Lie Group
    R = np.array([[R11, R12],
                  [R21, R22]])
    p = np.array([1, 0])
    px, py = R @ p
    
    # Update the point on the circle
    exp_point.set_data([px], [py])
    exp_text.set_position((px + 0.1, py + 0.1))
    
    # Update the radius line
    # radius_line.set_data([0, px], [0, py])
    
    # Update the arc
    arc_patch.remove() # Remove the old arc
    arc_patch = Arc((0,0), 2, 2, angle=0, theta1=0, theta2=np.rad2deg(theta), color='red', linewidth=2)
    ax.add_patch(arc_patch) # Add the new one
    
    # Update the text display
    value_text.set_text(f'$\\theta = {theta:.2f}$\n$R = \\exp(\\theta J) = ( {R11:.2f}, {R12:.2f}, {R21:.2f}, {R22:.2f})$')
    
    # Redraw the figure
    fig.canvas.draw_idle()

# --- 6. Connect the slider to the update function ---
theta_slider.on_changed(update)

# Show the plot
plt.show()