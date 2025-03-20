import pygame
import math
import matplotlib.pyplot as plt
from io import BytesIO
import numpy as np
from pygame import gfxdraw

# ---------------------
# Pygame Initialization
# ---------------------
pygame.init()

WIDTH, HEIGHT = 1000, 700
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Enhanced 2D Robotic Arm Simulator")

# -------------------
# Color Palette Tweaks
# -------------------
# Slightly softened colors and kept a minimal but clear set
BACKGROUND = (240, 240, 245)
WHITE = (255, 255, 255)
BLACK = (30, 30, 30)
RED = (220, 60, 60)
BLUE = (60, 120, 216)
GREEN = (60, 179, 113)
PURPLE = (138, 43, 226)
GRAY = (128, 128, 128)
ORANGE = (255, 140, 0)
LIGHT_GRAY = (210, 210, 210)    # Lightened slightly to reduce grid dominance
DARK_BLUE = (25, 25, 112)

# Altered to unify the panel outlines and text backgrounds
PANEL_OUTLINE = (120, 120, 150)
SHADOW_ALPHA = (0, 0, 0, 50)

# ---------------
# Font Setup
# ---------------
pygame.font.init()
title_font = pygame.font.SysFont("Arial", 36, bold=True)
header_font = pygame.font.SysFont("Arial", 28, bold=True)
font = pygame.font.SysFont("Arial", 22)       # Slightly smaller for uniformity
small_font = pygame.font.SysFont("Arial", 18)

clock = pygame.time.Clock()

# -------------------------
# Robotic Arm Configuration
# -------------------------
L1 = 150
L2 = 150
BASE_X, BASE_Y = WIDTH // 3, HEIGHT // 2

# ------------------------------------
# Create a Vertical Gradient Background
# ------------------------------------
def create_gradient_surface(width, height, start_color, end_color, vertical=True):
    surface = pygame.Surface((width, height), pygame.SRCALPHA)
    for i in range(height if vertical else width):
        factor = i / (height if vertical else width)
        current_color = [
            int(start_color[j] + (end_color[j] - start_color[j]) * factor)
            for j in range(3)
        ]
        if vertical:
            pygame.draw.line(surface, current_color, (0, i), (width, i))
        else:
            pygame.draw.line(surface, current_color, (i, 0), (i, height))
    return surface

# Subtle gradient: top is a bit lighter; bottom slightly darker
background_surface = create_gradient_surface(
    WIDTH, HEIGHT, (235, 235, 240), (220, 220, 230), vertical=True
)

# -------------
# Draw Grid
# -------------
def draw_grid(surface, grid_size=50):
    for x in range(0, WIDTH, grid_size):
        pygame.draw.line(surface, LIGHT_GRAY, (x, 0), (x, HEIGHT), 1)
    for y in range(0, HEIGHT, grid_size):
        pygame.draw.line(surface, LIGHT_GRAY, (0, y), (WIDTH, y), 1)

# ----------------------
# Inverse Kinematics
# ----------------------
def inverse_kinematics(x, y):
    distance = math.sqrt(x**2 + y**2)
    if distance > (L1 + L2):
        return None
    cos_angle2 = (x**2 + y**2 - L1**2 - L2**2) / (2 * L1 * L2)
    cos_angle2 = max(-1, min(1, cos_angle2))
    theta2 = math.acos(cos_angle2)

    k1 = L1 + L2 * math.cos(theta2)
    k2 = L2 * math.sin(theta2)
    theta1 = math.atan2(y, x) - math.atan2(k2, k1)
    return theta1, theta2, k1, k2

# ---------------------------------------------
# Anti-aliased drawing helpers (lines/circles)
# ---------------------------------------------
def draw_aa_circle(surface, center, radius, color):
    x, y = center
    gfxdraw.aacircle(surface, int(x), int(y), radius, color)
    gfxdraw.filled_circle(surface, int(x), int(y), radius, color)

def draw_aa_line(surface, start, end, color, width=1):
    pygame.draw.line(surface, color, start, end, width)
    if width > 1:
        draw_aa_circle(surface, start, width//2, color)
        draw_aa_circle(surface, end, width//2, color)

# -------------------
# Simple Shadow Draw
# -------------------
def draw_shadow(surface, center, radius, color=(0, 0, 0, 80)):
    """
    Draw a simple semi-transparent circle beneath a point.
    """
    shadow_surf = pygame.Surface((radius*2 + 20, radius*2 + 20), pygame.SRCALPHA)
    pygame.draw.circle(
        shadow_surf, color,
        (radius + 10, radius + 10), radius
    )
    surface.blit(shadow_surf, (center[0] - radius - 10, center[1] - radius - 10))

# ---------------
# Angle Arc
# ---------------
def draw_angle_arc(screen, center, start_angle, end_angle, radius, color, angle_text=None):
    """
    Draw an arc representing an angle, with a small label.
    """
    # Transparent surface for the arc
    arc_surface = pygame.Surface((radius*2 + 4, radius*2 + 4), pygame.SRCALPHA)
    arc_center = (radius + 2, radius + 2)
    arc_color = (*color[:3], 120)  # light alpha
    
    # Polygon for the filled arc
    points = [arc_center]
    for angle in np.linspace(-start_angle, -end_angle, 24):
        points.append((
            arc_center[0] + radius * math.cos(angle),
            arc_center[1] + radius * math.sin(angle)
        ))
    if len(points) > 2:
        pygame.draw.polygon(arc_surface, arc_color, points)
    
    # Arc outline
    rect = pygame.Rect(0, 0, radius * 2, radius * 2)
    pygame.draw.arc(arc_surface, color, rect, -end_angle, -start_angle, 2)
    screen.blit(arc_surface, (center[0] - arc_center[0], center[1] - arc_center[1]))
    
    # Angle text
    if angle_text:
        mid_angle = (start_angle + end_angle) / 2
        text = small_font.render(angle_text, True, color)
        text_x = center[0] + (radius + 20) * math.cos(-mid_angle)
        text_y = center[1] + (radius + 20) * math.sin(-mid_angle)
        
        # Subtle text background
        text_bg = pygame.Surface((text.get_width() + 8, text.get_height() + 4), pygame.SRCALPHA)
        pygame.draw.rect(text_bg, (255, 255, 255, 200), text_bg.get_rect(), border_radius=4)
        text_bg.blit(text, (4, 2))
        screen.blit(text_bg, (text_x - text.get_width()/2 - 4, text_y - text.get_height()/2 - 2))

# -------------------------
# Length Indicator
# -------------------------
def draw_length_indicator(screen, start_pos, end_pos, length, color):
    dx = end_pos[0] - start_pos[0]
    dy = end_pos[1] - start_pos[1]
    seg_len = math.sqrt(dx*dx + dy*dy)
    if seg_len < 1:
        return
    # Normal vector
    nx = -dy/seg_len
    ny = dx/seg_len
    offset = 20
    
    start_o = (start_pos[0] + offset*nx, start_pos[1] + offset*ny)
    end_o = (end_pos[0] + offset*nx, end_pos[1] + offset*ny)
    
    draw_aa_line(screen, start_o, end_o, color, 1)
    draw_aa_line(screen, start_pos, start_o, color, 1)
    draw_aa_line(screen, end_pos, end_o, color, 1)
    
    text = font.render(f"{int(seg_len)}px", True, color)
    text_x = (start_o[0] + end_o[0])/2 - text.get_width()/2
    text_y = (start_o[1] + end_o[1])/2 - text.get_height()/2
    
    # Subtle background for text
    bg_rect = pygame.Surface((text.get_width() + 8, text.get_height() + 4), pygame.SRCALPHA)
    pygame.draw.rect(bg_rect, (255, 255, 255, 220), bg_rect.get_rect(), border_radius=5)
    pygame.draw.rect(bg_rect, color, bg_rect.get_rect(), width=1, border_radius=5)
    bg_rect.blit(text, (4,2))
    screen.blit(bg_rect, (text_x, text_y))

# --------------
# Draw the Arm
# --------------
def draw_arm(theta1, theta2):
    joint_x = BASE_X + L1 * math.cos(theta1)
    joint_y = BASE_Y - L1 * math.sin(theta1)
    end_x = joint_x + L2 * math.cos(theta1 + theta2)
    end_y = joint_y - L2 * math.sin(theta1 + theta2)

    # Subtle shadows behind each joint
    draw_shadow(screen, (BASE_X, BASE_Y), 14, (0, 0, 0, 60))
    draw_shadow(screen, (int(joint_x), int(joint_y)), 14, (0, 0, 0, 60))
    draw_shadow(screen, (int(end_x), int(end_y)), 14, (0, 0, 0, 60))
    
    # Draw segments
    draw_aa_line(screen, (BASE_X, BASE_Y), (joint_x, joint_y), BLUE, 6)
    draw_aa_line(screen, (joint_x, joint_y), (end_x, end_y), RED, 6)
    
    # Joints
    draw_aa_circle(screen, (BASE_X, BASE_Y), 10, DARK_BLUE)
    draw_aa_circle(screen, (int(joint_x), int(joint_y)), 10, BLACK)
    draw_aa_circle(screen, (int(end_x), int(end_y)), 10, ORANGE)
    
    # Metallic shine
    draw_aa_circle(screen, (BASE_X - 3, BASE_Y - 3), 3, (220, 220, 255))
    draw_aa_circle(screen, (int(joint_x - 3), int(joint_y - 3)), 3, (220, 220, 255))
    draw_aa_circle(screen, (int(end_x - 3), int(end_y - 3)), 3, (255, 255, 220))

    # Angle arcs
    draw_angle_arc(screen, (BASE_X, BASE_Y), 0, theta1, 40, PURPLE, f"θ₁ = {math.degrees(theta1):.1f}°")
    draw_angle_arc(screen, (int(joint_x), int(joint_y)), theta1, theta1 + theta2, 40, RED, f"θ₂ = {math.degrees(theta2):.1f}°")

    # Labels (Base, Joint, End)
    labels = [
        (f"Base ({BASE_X}, {BASE_Y})", (BASE_X, BASE_Y + 25), DARK_BLUE),
        (f"Joint ({int(joint_x)}, {int(joint_y)})", (int(joint_x), int(joint_y + 25)), BLACK),
        (f"End ({int(end_x)}, {int(end_y)})", (int(end_x), int(end_y + 25)), ORANGE),
    ]
    for txt, pos, col in labels:
        render = small_font.render(txt, True, col)
        bg = pygame.Surface((render.get_width()+8, render.get_height()+4), pygame.SRCALPHA)
        pygame.draw.rect(bg, (255, 255, 255, 220), bg.get_rect(), border_radius=5)
        pygame.draw.rect(bg, col, bg.get_rect(), width=1, border_radius=5)
        bg.blit(render, (4,2))
        screen.blit(bg, (pos[0] - render.get_width()/2 - 4, pos[1] - render.get_height()/2 - 2))
    
    # Lengths
    draw_length_indicator(screen, (BASE_X, BASE_Y), (joint_x, joint_y), L1, BLUE)
    draw_length_indicator(screen, (joint_x, joint_y), (end_x, end_y), L2, RED)

    # Subtle dashed line from base to end
    total_length = math.sqrt((end_x - BASE_X)**2 + (end_y - BASE_Y)**2)
    dash_len = 5
    gap_len = 4
    curr_dist = 0
    while curr_dist < total_length:
        start_t = curr_dist / total_length
        end_t = min((curr_dist + dash_len) / total_length, 1.0)
        sx = BASE_X + (end_x - BASE_X) * start_t
        sy = BASE_Y + (end_y - BASE_Y) * start_t
        ex = BASE_X + (end_x - BASE_X) * end_t
        ey = BASE_Y + (end_y - BASE_Y) * end_t
        draw_aa_line(screen, (sx, sy), (ex, ey), (100, 100, 100), 1)
        curr_dist += dash_len + gap_len

    return joint_x, joint_y, end_x, end_y

# --------------
# Rounded Rect
# --------------
def create_rounded_rect_surface(width, height, color, radius=10):
    surface = pygame.Surface((width, height), pygame.SRCALPHA)
    rect = pygame.Rect(0, 0, width, height)
    pygame.draw.rect(surface, color, rect, border_radius=radius)
    return surface

# ------------
# LaTeX Render
# ------------
def create_latex_surface(formula, color):
    normalized_color = (color[0]/255, color[1]/255, color[2]/255)
    fig, ax = plt.subplots(figsize=(6, 0.8), dpi=100, facecolor='white')
    ax.text(0.5, 0.5, f"${formula}$", fontsize=18, ha='center', va='center', 
            usetex=False, color=normalized_color, fontweight='bold')
    ax.axis('off')
    plt.tight_layout(pad=0.1)
    buf = BytesIO()
    plt.savefig(buf, format='png', dpi=100, bbox_inches='tight', pad_inches=0.1)
    buf.seek(0)
    plt.close(fig)

    formula_image = pygame.image.load(buf, 'png')
    buf.close()
    return formula_image

# ---------------
# Info Panel
# ---------------
def draw_info_panel():
    panel_rect = pygame.Rect(10, 10, 280, 120)
    shadow_rect = panel_rect.copy()
    shadow_rect.x += 4
    shadow_rect.y += 4
    
    # Slight shadow
    pygame.draw.rect(screen, SHADOW_ALPHA, shadow_rect, border_radius=10)
    
    # Panel background
    pygame.draw.rect(screen, (255, 255, 255, 230), panel_rect, border_radius=10)
    pygame.draw.rect(screen, PANEL_OUTLINE, panel_rect, width=2, border_radius=10)
    
    # Title
    title = header_font.render("Instructions", True, BLACK)
    screen.blit(title, (panel_rect.x + 15, panel_rect.y + 10))
    
    # Instructions
    instructions = [
        "• Move mouse to control the arm",
        "• Watch real-time IK updates",
        "• Arm is limited by segment lengths"
    ]
    for i, text in enumerate(instructions):
        t_surf = small_font.render(text, True, BLACK)
        screen.blit(t_surf, (panel_rect.x + 15, panel_rect.y + 50 + i*22))

# -----------------------------
# Pre-rendered LaTeX Surfaces
# -----------------------------
latex_surfaces = {
    "title": header_font.render("Inverse Kinematics Formulas", True, BLACK),
    "theta2": create_latex_surface("\\theta_2 = \\cos^{-1}\\left(\\frac{x^2 + y^2 - L_1^2 - L_2^2}{2 L_1 L_2}\\right)", RED),
    "k1": create_latex_surface("k_1 = L_1 + L_2 \\cdot \\cos(\\theta_2)", GREEN),
    "k2": create_latex_surface("k_2 = L_2 \\cdot \\sin(\\theta_2)", BLUE),
    "theta1": create_latex_surface("\\theta_1 = \\tan^{-1}\\left(\\frac{y}{x}\\right) - \\tan^{-1}\\left(\\frac{k_2}{k_1}\\right)", PURPLE),
}

# Background for each formula
formula_panels = {}
for key in ["theta2", "k1", "k2", "theta1"]:
    surf = latex_surfaces[key]
    bg_surf = create_rounded_rect_surface(
        surf.get_width() + 24,
        surf.get_height() + 16,
        (255, 255, 255, 230),
        radius=8
    )
    formula_panels[key] = bg_surf

# ---------------------------------
# Main Loop Initialization
# ---------------------------------
running = True
mouse_path = []
MAX_PATH_LENGTH = 60

trail_points = []
MAX_TRAIL_LENGTH = 20

# ----------------
# Main Game Loop
# ----------------
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    # Background & grid
    screen.blit(background_surface, (0, 0))
    draw_grid(screen)
    
    # Info panel
    draw_info_panel()

    mouse_x, mouse_y = pygame.mouse.get_pos()
    
    # Fade trail behind mouse
    trail_points.append((mouse_x, mouse_y))
    if len(trail_points) > MAX_TRAIL_LENGTH:
        trail_points.pop(0)
    for i, point in enumerate(trail_points):
        opacity = int((i / MAX_TRAIL_LENGTH) * 180)
        trail_color = (ORANGE[0], ORANGE[1], ORANGE[2], opacity)
        radius = int(2 + (i / MAX_TRAIL_LENGTH) * 6)
        trail_surface = pygame.Surface((radius*2, radius*2), pygame.SRCALPHA)
        pygame.draw.circle(trail_surface, trail_color, (radius, radius), radius)
        screen.blit(trail_surface, (point[0] - radius, point[1] - radius))

    # Target in arm coords
    target_x = mouse_x - BASE_X
    target_y = BASE_Y - mouse_y
    angles = inverse_kinematics(target_x, target_y)

    if angles:
        theta1, theta2, k1, k2 = angles
        joint_x, joint_y, end_x, end_y = draw_arm(theta1, theta2)
        mouse_path.append((end_x, end_y))
        if len(mouse_path) > MAX_PATH_LENGTH:
            mouse_path.pop(0)
        
        # Faded path from the end effector
        if len(mouse_path) > 1:
            for i in range(1, len(mouse_path)):
                opacity = int((i / len(mouse_path)) * 200)
                path_color = (100, 100, 200, opacity)
                start_p = mouse_path[i-1]
                end_p = mouse_path[i]
                seg_surf = pygame.Surface((WIDTH, HEIGHT), pygame.SRCALPHA)
                pygame.draw.line(seg_surf, path_color, start_p, end_p, 2)
                screen.blit(seg_surf, (0, 0))
    else:
        theta1, theta2, k1, k2 = None, None, None, None
        # Out of reach notice
        out_text = font.render("Target out of reach!", True, RED)
        text_bg = pygame.Surface((out_text.get_width()+20, out_text.get_height()+10), pygame.SRCALPHA)
        pygame.draw.rect(text_bg, (255, 200, 200, 220), text_bg.get_rect(), border_radius=8)
        text_bg.blit(out_text, (10,5))
        screen.blit(text_bg, (mouse_x - out_text.get_width()//2 - 10, mouse_y - 50))

    # Target effect
    t_radius = 6 + math.sin(pygame.time.get_ticks()/250)*2
    draw_shadow(screen, (mouse_x, mouse_y), int(t_radius)+2)
    draw_aa_circle(screen, (mouse_x, mouse_y), int(t_radius), ORANGE)
    
    # Crosshair
    crosshair = 6
    pygame.draw.line(screen, BLACK, (mouse_x - crosshair, mouse_y), (mouse_x + crosshair, mouse_y), 1)
    pygame.draw.line(screen, BLACK, (mouse_x, mouse_y - crosshair), (mouse_x, mouse_y + crosshair), 1)

    # Title text
    title = title_font.render("2D Robotic Arm Simulator", True, DARK_BLUE)
    screen.blit(title, (WIDTH // 2 - title.get_width() // 2, 15))
    
    # Formulas panel
    panel_width = 330
    panel_height = 330
    panel_x = WIDTH - panel_width - 20
    panel_y = 110
    
    # Panel shadow
    shadow_offset = 4
    pygame.draw.rect(screen, SHADOW_ALPHA, 
                     (panel_x+shadow_offset, panel_y+shadow_offset, panel_width, panel_height), 
                     border_radius=12)
    # Panel background
    panel_bg = create_gradient_surface(panel_width, panel_height, (250, 250, 255), (230, 230, 245), True)
    screen.blit(panel_bg, (panel_x, panel_y))
    pygame.draw.rect(screen, PANEL_OUTLINE, (panel_x, panel_y, panel_width, panel_height), 2, border_radius=12)

    # Panel header
    header_h = 48
    pygame.draw.rect(screen, (210, 210, 230, 80), (panel_x, panel_y, panel_width, header_h), border_radius=12)
    pygame.draw.line(screen, PANEL_OUTLINE, 
                     (panel_x+10, panel_y+header_h), 
                     (panel_x+panel_width-10, panel_y+header_h), 1)
    screen.blit(latex_surfaces["title"], 
        (panel_x + panel_width//2 - latex_surfaces["title"].get_width()//2, panel_y + 8)
    )
    
    # Position formulas
    offsets = {
        "theta2": panel_y + 60,
        "k1": panel_y + 125,
        "k2": panel_y + 190,
        "theta1": panel_y + 255
    }
    for key, y_pos in offsets.items():
        f_surf = latex_surfaces[key]
        bg_surf = formula_panels[key]
        
        bg_x = panel_x + panel_width//2 - bg_surf.get_width()//2
        screen.blit(bg_surf, (bg_x, y_pos))
        
        # Formula
        fx = bg_x + (bg_surf.get_width() - f_surf.get_width())//2
        fy = y_pos + (bg_surf.get_height() - f_surf.get_height())//2
        screen.blit(f_surf, (fx, fy))
        
        # Numeric values
        if angles:
            val_text = None
            if key == "theta2" and theta2 is not None:
                val_text = font.render(f"θ₂ = {math.degrees(theta2):.2f}°", True, RED)
            elif key == "k1" and k1 is not None:
                val_text = font.render(f"k₁ = {k1:.2f}", True, GREEN)
            elif key == "k2" and k2 is not None:
                val_text = font.render(f"k₂ = {k2:.2f}", True, BLUE)
            elif key == "theta1" and theta1 is not None:
                val_text = font.render(f"θ₁ = {math.degrees(theta1):.2f}°", True, PURPLE)
            
            if val_text:
                val_bg = pygame.Surface((val_text.get_width()+10, val_text.get_height()+6), pygame.SRCALPHA)
                pygame.draw.rect(val_bg, (255, 255, 255, 220), val_bg.get_rect(), border_radius=5)
                pygame.draw.rect(val_bg, (150, 150, 150), val_bg.get_rect(), 1, border_radius=5)
                val_bg.blit(val_text, (5, 3))
                vx = bg_x + bg_surf.get_width()//2 - val_text.get_width()//2 - 5
                vy = y_pos + bg_surf.get_height() + 4
                screen.blit(val_bg, (vx, vy))

    # Workspace boundary (dashed circle)
    max_reach = L1 + L2
    num_segs = 60
    for i in range(num_segs):
        start_a = i * 2*math.pi / num_segs
        end_a = (i + 0.5) * 2*math.pi / num_segs
        sx = BASE_X + max_reach * math.cos(start_a)
        sy = BASE_Y + max_reach * math.sin(start_a)
        ex = BASE_X + max_reach * math.cos(end_a)
        ey = BASE_Y + max_reach * math.sin(end_a)
        pygame.draw.line(screen, LIGHT_GRAY, (sx, sy), (ex, ey), 1)

    # Status Panel
    status_x = 10
    status_y = HEIGHT - 40
    status_width = 280
    status_height = 30

    st_bg = pygame.Surface((status_width, status_height), pygame.SRCALPHA)
    pygame.draw.rect(st_bg, (255, 255, 255, 210), st_bg.get_rect(), border_radius=6)
    pygame.draw.rect(st_bg, (150, 150, 180), st_bg.get_rect(), 1, border_radius=6)
    st_text_color = GREEN if angles else RED
    if angles:
        st_txt = small_font.render(f"Target: ({target_x:.1f}, {target_y:.1f}) - In Reach", True, st_text_color)
    else:
        st_txt = small_font.render(f"Target: ({target_x:.1f}, {target_y:.1f}) - Out of Reach", True, st_text_color)
    st_bg.blit(st_txt, (10, 5))
    screen.blit(st_bg, (status_x, status_y))

    pygame.display.flip()
    clock.tick(60)

pygame.quit()