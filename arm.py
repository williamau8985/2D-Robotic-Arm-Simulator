import pygame
import math
import matplotlib.pyplot as plt
from io import BytesIO
import pygame.surfarray as surfarray

# Initialize Pygame
pygame.init()

# Screen dimensions
WIDTH, HEIGHT = 1000, 700
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("2D Robotic Arm Simulator")

# Colors
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
RED = (255, 0, 0)
BLUE = (0, 0, 255)
GREEN = (0, 255, 0)
PURPLE = (128, 0, 128)
GRAY = (128, 128, 128)

# Clock for controlling frame rate
clock = pygame.time.Clock()

# Robotic arm parameters
L1 = 150  # Length of first segment
L2 = 150  # Length of second segment
BASE_X, BASE_Y = WIDTH // 3, HEIGHT // 2  # Base position

# Font for displaying text
font = pygame.font.SysFont(None, 24)

def inverse_kinematics(x, y):
    """
    Calculate joint angles for the robotic arm to reach (x, y).
    Returns the angles (theta1, theta2).
    """
    # Distance from base to target
    distance = math.sqrt(x**2 + y**2)
    
    if distance > (L1 + L2):
        return None  # Target is unreachable

    # Using cosine rule to calculate angles
    cos_angle2 = (x**2 + y**2 - L1**2 - L2**2) / (2 * L1 * L2)
    theta2 = math.acos(cos_angle2)  # Elbow angle

    k1 = L1 + L2 * math.cos(theta2)
    k2 = L2 * math.sin(theta2)
    theta1 = math.atan2(y, x) - math.atan2(k2, k1)  # Shoulder angle

    return theta1, theta2, k1, k2

def draw_angle_arc(screen, center, start_angle, end_angle, radius, color, angle_text=None):
    """
    Draw an arc to represent an angle, with a label showing the degree value.
    """
    # Draw reference line (horizontal for base angle)
    ref_length = radius + 10
    pygame.draw.line(screen, GRAY, 
                    (center[0], center[1]), 
                    (center[0] + ref_length * math.cos(start_angle), 
                     center[1] - ref_length * math.sin(start_angle)), 1)
    
    # Draw arc
    rect = pygame.Rect(center[0] - radius, center[1] - radius, radius * 2, radius * 2)
    pygame.draw.arc(screen, color, rect, -end_angle, -start_angle, 2)
    
    # Add arrow at the end of the arc
    arrow_length = 10
    end_x = center[0] + radius * math.cos(-end_angle)
    end_y = center[1] + radius * math.sin(-end_angle)
    
    # Draw angle text at a position near the arc
    if angle_text:
        # Position the text at the middle of the arc
        mid_angle = (start_angle + end_angle) / 2
        text = font.render(angle_text, True, color)
        text_x = center[0] + (radius + 15) * math.cos(-mid_angle)
        text_y = center[1] + (radius + 15) * math.sin(-mid_angle)
        screen.blit(text, (text_x - text.get_width()/2, text_y - text.get_height()/2))

def draw_length_indicator(screen, start_pos, end_pos, length, color):
    """
    Draw a length measurement indicator with offset parallel lines and text.
    """
    # Calculate the vector between points
    dx = end_pos[0] - start_pos[0]
    dy = end_pos[1] - start_pos[1]
    
    # Calculate normal vector (perpendicular)
    length = math.sqrt(dx*dx + dy*dy)
    if length != 0:
        normal_x = -dy/length
        normal_y = dx/length
    else:
        return
    
    # Offset for the measurement line
    offset = 20
    
    # Calculate offset points
    start_offset = (start_pos[0] + offset*normal_x, start_pos[1] + offset*normal_y)
    end_offset = (end_pos[0] + offset*normal_x, end_pos[1] + offset*normal_y)
    
    # Draw measurement line
    pygame.draw.line(screen, color, start_offset, end_offset, 1)
    
    # Draw small perpendicular lines at ends
    perp_length = 5
    pygame.draw.line(screen, color, start_pos, start_offset, 1)
    pygame.draw.line(screen, color, end_pos, end_offset, 1)
    
    # Add length text
    text = font.render(f"{int(length)}px", True, color)
    text_pos = ((start_offset[0] + end_offset[0])/2 - text.get_width()/2,
                (start_offset[1] + end_offset[1])/2 - text.get_height()/2)
    screen.blit(text, text_pos)

def draw_arm(theta1, theta2):
    # Calculate joint positions
    joint_x = BASE_X + L1 * math.cos(theta1)
    joint_y = BASE_Y - L1 * math.sin(theta1)

    end_x = joint_x + L2 * math.cos(theta1 + theta2)
    end_y = joint_y - L2 * math.sin(theta1 + theta2)

    # Draw the arm segments
    pygame.draw.line(screen, BLUE, (BASE_X, BASE_Y), (joint_x, joint_y), 5)
    pygame.draw.line(screen, RED, (joint_x, joint_y), (end_x, end_y), 5)

    # Draw joints
    pygame.draw.circle(screen, BLACK, (BASE_X, BASE_Y), 8)  # Base joint
    pygame.draw.circle(screen, BLACK, (int(joint_x), int(joint_y)), 8)  # Middle joint
    pygame.draw.circle(screen, BLACK, (int(end_x), int(end_y)), 8)  # End effector

    # Draw angle measurements with degree values
    draw_angle_arc(screen, (BASE_X, BASE_Y), 0, theta1, 40, PURPLE, 
                  f"θ₁={math.degrees(theta1):.1f}°")  # Base angle
    draw_angle_arc(screen, (int(joint_x), int(joint_y)), theta1, theta1 + theta2, 40, RED,
                  f"θ₂={math.degrees(theta2):.1f}°")  # Elbow angle
    
    # Draw coordinate labels for each joint
    base_text = font.render(f"Base ({BASE_X}, {BASE_Y})", True, BLACK)
    joint_text = font.render(f"Joint ({int(joint_x)}, {int(joint_y)})", True, BLACK)
    end_text = font.render(f"End ({int(end_x)}, {int(end_y)})", True, BLACK)
    
    # Position labels with offset to avoid overlap
    screen.blit(base_text, (BASE_X - 60, BASE_Y + 20))
    screen.blit(joint_text, (int(joint_x) - 60, int(joint_y) + 20))
    screen.blit(end_text, (int(end_x) - 60, int(end_y) + 20))

    # Draw length measurements
    draw_length_indicator(screen, (BASE_X, BASE_Y), (joint_x, joint_y), L1, BLUE)  # First segment
    draw_length_indicator(screen, (joint_x, joint_y), (end_x, end_y), L2, RED)  # Second segment

    return joint_x, joint_y, end_x, end_y

# Function to render LaTeX formulas as static surfaces
def create_latex_surface(formula, color):
    """
    Render a LaTeX formula once and return a Pygame surface.
    """
    # Normalize the RGB color tuple for Matplotlib
    normalized_color = (color[0] / 255, color[1] / 255, color[2] / 255)
    
    # Create a Matplotlib figure
    fig, ax = plt.subplots(figsize=(4, 0.5), dpi=100)
    ax.text(0.5, 0.5, f"${formula}$", fontsize=16, ha='center', va='center', usetex=False, color=normalized_color)
    ax.axis('off')  # Turn off axes

    # Save the figure as an image in memory
    buf = BytesIO()
    plt.savefig(buf, format='png', transparent=True)
    buf.seek(0)
    plt.close(fig)

    # Load the image into Pygame
    formula_image = pygame.image.load(buf, 'png')
    buf.close()

    return formula_image

# Create LaTeX surfaces
latex_surfaces = {
    "theta2": create_latex_surface("\\theta_2 = \\cos^{-1}\\left(\\frac{x^2 + y^2 - L_1^2 - L_2^2}{2 L_1 L_2}\\right)", RED),
    "k1": create_latex_surface("k_1 = L_1 + L_2 \\cdot \\cos(\\theta_2)", GREEN),
    "k2": create_latex_surface("k_2 = L_2 \\cdot \\sin(\\theta_2)", BLUE),
    "theta1": create_latex_surface("\\theta_1 = \\tan^{-1}\\left(\\frac{y}{x}\\right) - \\tan^{-1}\\left(\\frac{k_2}{k_1}\\right)", PURPLE),
}

# Main loop
running = True
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    # Clear the screen
    screen.fill(WHITE)

    # Get mouse position
    mouse_x, mouse_y = pygame.mouse.get_pos()

    # Convert mouse coordinates to robotic arm coordinates
    target_x = mouse_x - BASE_X
    target_y = BASE_Y - mouse_y

    # Calculate inverse kinematics
    angles = inverse_kinematics(target_x, target_y)

    if angles:
        theta1, theta2, k1, k2 = angles
        joint_x, joint_y, end_x, end_y = draw_arm(theta1, theta2)
    else:
        theta1, theta2, k1, k2 = None, None, None, None

    # Draw the target point
    pygame.draw.circle(screen, BLACK, (mouse_x, mouse_y), 5)

    # Display the pre-rendered LaTeX formulas
    screen.blit(latex_surfaces["theta2"], (WIDTH - 400, 100))
    screen.blit(latex_surfaces["k1"], (WIDTH - 400, 200))
    screen.blit(latex_surfaces["k2"], (WIDTH - 400, 300))
    screen.blit(latex_surfaces["theta1"], (WIDTH - 400, 400))

    # Display the results below each formula
    if theta2 is not None:
        theta2_text = font.render(f"\u03B8₂ = {math.degrees(theta2):.2f}°", True, RED)
        screen.blit(theta2_text, (WIDTH - 400, 150))

    if k1 is not None:
        k1_text = font.render(f"k₁ = {k1:.2f}", True, GREEN)
        screen.blit(k1_text, (WIDTH - 400, 250))

    if k2 is not None:
        k2_text = font.render(f"k₂ = {k2:.2f}", True, BLUE)
        screen.blit(k2_text, (WIDTH - 400, 350))

    if theta1 is not None:
        theta1_text = font.render(f"\u03B8₁ = {math.degrees(theta1):.2f}°", True, PURPLE)
        screen.blit(theta1_text, (WIDTH - 400, 450))

    # Update the display
    pygame.display.flip()

    # Cap the frame rate
    clock.tick(60)

# Quit Pygame
pygame.quit()