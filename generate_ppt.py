import os
import pptx
from pptx.util import Inches, Pt
from pptx.dml.color import RGBColor
from pptx.enum.text import PP_ALIGN
from pptx.enum.shapes import MSO_SHAPE

BASE_DIR = r"C:\Users\prith\Documents\MYSPACE\RPP-Project-2026"

# Color Palette: Clean White, Amber Yellow, Charcoal Black
COLOR_BG_LIGHT = RGBColor(255, 255, 255)       # Crisp White
COLOR_BG_DARK = RGBColor(15, 23, 42)           # Deep Matte Slate-Black
COLOR_YELLOW = RGBColor(245, 158, 11)          # Accent Amber Yellow
COLOR_TEXT_DARK = RGBColor(15, 23, 42)         # Deep Black / Slate
COLOR_TEXT_BODY = RGBColor(51, 65, 85)         # Charcoal Slate
COLOR_TEXT_MUTED = RGBColor(100, 116, 139)     # Subdued Gray
COLOR_LINE_GRAY = RGBColor(226, 232, 240)      # Clean Border Gray
COLOR_WHITE = RGBColor(255, 255, 255)          # White

FONT_HEADING = "Trebuchet MS"
FONT_BODY = "Calibri"

prs = pptx.Presentation()
prs.slide_width = Inches(13.333)
prs.slide_height = Inches(7.5)
blank_layout = prs.slide_layouts[6]

def set_slide_background(slide, color):
    bg = slide.background
    fill = bg.fill
    fill.solid()
    fill.fore_color.rgb = color

def add_header(slide, category_text, title_text, dark_mode=False):
    """Draws a minimal, modern header with category pill and clean typography."""
    bar = slide.shapes.add_shape(MSO_SHAPE.RECTANGLE, Inches(0.8), Inches(0.48), Inches(0.35), Inches(0.08))
    bar.fill.solid()
    bar.fill.fore_color.rgb = COLOR_YELLOW
    bar.line.fill.background()

    cat_box = slide.shapes.add_textbox(Inches(1.25), Inches(0.40), Inches(8.0), Inches(0.3))
    tf_cat = cat_box.text_frame
    tf_cat.word_wrap = True
    tf_cat.margin_left = tf_cat.margin_top = tf_cat.margin_right = tf_cat.margin_bottom = 0
    p_cat = tf_cat.paragraphs[0]
    p_cat.text = category_text.upper()
    p_cat.font.name = FONT_BODY
    p_cat.font.size = Pt(11)
    p_cat.font.bold = True
    p_cat.font.color.rgb = COLOR_YELLOW

    title_box = slide.shapes.add_textbox(Inches(0.8), Inches(0.68), Inches(11.5), Inches(0.6))
    tf_title = title_box.text_frame
    tf_title.word_wrap = True
    tf_title.margin_left = tf_title.margin_top = tf_title.margin_right = tf_title.margin_bottom = 0
    p_title = tf_title.paragraphs[0]
    p_title.text = title_text
    p_title.font.name = FONT_HEADING
    p_title.font.size = Pt(23)
    p_title.font.bold = True
    p_title.font.color.rgb = COLOR_WHITE if dark_mode else COLOR_TEXT_DARK

    rule = slide.shapes.add_shape(MSO_SHAPE.RECTANGLE, Inches(0.8), Inches(1.30), Inches(11.733), Inches(0.015))
    rule.fill.solid()
    rule.fill.fore_color.rgb = RGBColor(40, 50, 70) if dark_mode else COLOR_LINE_GRAY
    rule.line.fill.background()

def add_image_fit(slide, img_path, left_in, top_in, max_w_in, max_h_in, caption=None):
    """Places image maintaining exact aspect ratio within bounds with optional caption."""
    if not os.path.exists(img_path):
        print(f"Warning: Image not found {img_path}")
        return
    import cv2
    img = cv2.imread(img_path)
    if img is None:
        return
    h, w = img.shape[:2]
    aspect = w / h
    
    w_fit = max_w_in
    h_fit = w_fit / aspect
    if h_fit > max_h_in:
        h_fit = max_h_in
        w_fit = h_fit * aspect
        
    final_left = left_in + (max_w_in - w_fit) / 2.0
    final_top = top_in + (max_h_in - h_fit) / 2.0
    
    border = slide.shapes.add_shape(MSO_SHAPE.RECTANGLE, 
                                   Inches(final_left - 0.03), Inches(final_top - 0.03), 
                                   Inches(w_fit + 0.06), Inches(h_fit + 0.06))
    border.fill.solid()
    border.fill.fore_color.rgb = RGBColor(241, 245, 249)
    border.line.color.rgb = RGBColor(203, 213, 225)
    border.line.width = Pt(1)
    
    slide.shapes.add_picture(img_path, Inches(final_left), Inches(final_top), width=Inches(w_fit), height=Inches(h_fit))

    if caption:
        cap_box = slide.shapes.add_textbox(Inches(final_left), Inches(final_top + h_fit + 0.08), Inches(w_fit), Inches(0.35))
        tf_c = cap_box.text_frame
        tf_c.word_wrap = True
        tf_c.margin_left = tf_c.margin_top = tf_c.margin_right = tf_c.margin_bottom = 0
        p_c = tf_c.paragraphs[0]
        p_c.alignment = PP_ALIGN.CENTER
        p_c.text = caption
        p_c.font.name = FONT_BODY
        p_c.font.size = Pt(10)
        p_c.font.italic = True
        p_c.font.color.rgb = COLOR_TEXT_MUTED

def add_bullet_item(tf, title, body, is_first=False, font_size_body=12.5):
    """Adds a structured, formal technical bullet point."""
    p = tf.add_paragraph() if not is_first else tf.paragraphs[0]
    p.space_before = Pt(11) if not is_first else Pt(0)
    p.space_after = Pt(2)
    p.level = 0
    
    run_bullet = p.add_run()
    run_bullet.text = "▪  "
    run_bullet.font.name = FONT_BODY
    run_bullet.font.size = Pt(font_size_body + 1)
    run_bullet.font.bold = True
    run_bullet.font.color.rgb = COLOR_YELLOW
    
    run_title = p.add_run()
    run_title.text = title + " — "
    run_title.font.name = FONT_HEADING
    run_title.font.size = Pt(font_size_body + 0.5)
    run_title.font.bold = True
    run_title.font.color.rgb = COLOR_TEXT_DARK
    
    run_body = p.add_run()
    run_body.text = body
    run_body.font.name = FONT_BODY
    run_body.font.size = Pt(font_size_body)
    run_body.font.color.rgb = COLOR_TEXT_BODY


# ======================================================================
# SLIDE 1: INTRO SLIDE (Modern Minimalist Title)
# ======================================================================
slide1 = prs.slides.add_slide(blank_layout)
set_slide_background(slide1, COLOR_BG_DARK)

# Accent Banner Bar
bar1 = slide1.shapes.add_shape(MSO_SHAPE.RECTANGLE, Inches(1.2), Inches(1.8), Inches(0.8), Inches(0.1))
bar1.fill.solid()
bar1.fill.fore_color.rgb = COLOR_YELLOW
bar1.line.fill.background()

# Tagline Pill
tag_box = slide1.shapes.add_textbox(Inches(1.2), Inches(2.1), Inches(10.0), Inches(0.4))
tf_tag = tag_box.text_frame
p_tag = tf_tag.paragraphs[0]
p_tag.text = "AUTONOMOUS MOBILE ROBOTICS & DIGITAL TWIN".upper()
p_tag.font.name = FONT_BODY
p_tag.font.size = Pt(13)
p_tag.font.bold = True
p_tag.font.color.rgb = COLOR_YELLOW

# Title
t_box = slide1.shapes.add_textbox(Inches(1.2), Inches(2.5), Inches(11.0), Inches(1.4))
tf_t = t_box.text_frame
tf_t.word_wrap = True
p_t = tf_t.paragraphs[0]
p_t.text = "SesameNav"
p_t.font.name = FONT_HEADING
p_t.font.size = Pt(54)
p_t.font.bold = True
p_t.font.color.rgb = COLOR_WHITE

# Subtitle
sub_box = slide1.shapes.add_textbox(Inches(1.2), Inches(3.9), Inches(10.5), Inches(0.9))
tf_sub = sub_box.text_frame
tf_sub.word_wrap = True
p_sub = tf_sub.paragraphs[0]
p_sub.text = "Vision-Guided Autonomous Quadruped Navigation & 3D Digital Twin"
p_sub.font.name = FONT_BODY
p_sub.font.size = Pt(22)
p_sub.font.color.rgb = RGBColor(226, 232, 240)

# Technical Pillars Summary Bar
pillars = [
    ("PERCEPTION", "Overhead Planar Homography & ArUco Tracking"),
    ("PLANNING", "Dynamic A* Search with 5 cm Safety Margin"),
    ("CONTROL", "Stanley Path-Following & Stride Cadence Sync"),
    ("SIMULATION", "ROS 2 Humble, RViz & Gazebo Digital Twin"),
]

for idx, (col_title, col_desc) in enumerate(pillars):
    col_left = Inches(1.2 + idx * 2.75)
    c_box = slide1.shapes.add_textbox(col_left, Inches(5.3), Inches(2.5), Inches(1.5))
    tf_c = c_box.text_frame
    tf_c.word_wrap = True
    
    p_h = tf_c.paragraphs[0]
    p_h.text = col_title
    p_h.font.name = FONT_HEADING
    p_h.font.size = Pt(11)
    p_h.font.bold = True
    p_h.font.color.rgb = COLOR_YELLOW
    
    p_d = tf_c.add_paragraph()
    p_d.space_before = Pt(4)
    p_d.text = col_desc
    p_d.font.name = FONT_BODY
    p_d.font.size = Pt(11)
    p_d.font.color.rgb = RGBColor(148, 163, 184)


# ======================================================================
# SLIDE 2: WHAT THE PROJECT IS & WHAT IT DOES
# ======================================================================
slide2 = prs.slides.add_slide(blank_layout)
set_slide_background(slide2, COLOR_BG_LIGHT)
add_header(slide2, "System Overview", "What the Project Is & Core Operational Pipeline")

# Left Column (Text)
tb2 = slide2.shapes.add_textbox(Inches(0.8), Inches(1.55), Inches(6.8), Inches(5.4))
tf2 = tb2.text_frame
tf2.word_wrap = True
tf2.margin_left = tf2.margin_top = tf2.margin_right = tf2.margin_bottom = 0

add_bullet_item(tf2, "Core Objective", 
                "Achieve fully autonomous, closed-loop navigation on a lightweight 8-DOF micro-quadruped without relying on heavy or expensive on-board LiDAR or depth sensors.", is_first=True)

add_bullet_item(tf2, "Overhead Global Perception", 
                "An overhead camera captures an unobstructed arena view. Perspective distortion is corrected in real time into an orthogonal metric plane (10 px/cm) using 4-corner ArUco planar homography.")

add_bullet_item(tf2, "Dynamic A* Path Planning", 
                "Computes the optimal shortest collision-free path on a 2D occupancy grid, inflating obstacles by >= 5.0 cm and applying Euclidean line-of-sight shortcutting.")

add_bullet_item(tf2, "Stanley Path-Following Control", 
                "Formulates discrete steering decisions by minimizing heading error and cross-track error (CTE) relative to the planned path trajectory.")

add_bullet_item(tf2, "Hardware-in-the-Loop Actuation", 
                "Translates path decisions into discrete gait commands dispatched over Wi-Fi HTTP (/cmd?go=...) to an on-board ESP32 microcontroller.")

add_bullet_item(tf2, "Synchronized 3D Digital Twin", 
                "Broadcasts real-time telemetry over UDP to ROS 2 Humble, mirroring the physical robot's joint states, path, and obstacle bounds inside RViz.")

# Right Column (Image)
img_setup = os.path.join(BASE_DIR, r"docs\evidence\real_world_arena_setup.jpeg")
add_image_fit(slide2, img_setup, left_in=7.9, top_in=1.55, max_w_in=4.6, max_h_in=5.2, 
              caption="Physical 100 x 80 cm Arena with Overhead Camera & Quadruped")


# ======================================================================
# SLIDE 3: KINEMATICS & LOCOMOTION
# ======================================================================
slide3 = prs.slides.add_slide(blank_layout)
set_slide_background(slide3, COLOR_BG_LIGHT)
add_header(slide3, "Locomotion Mechanics", "Quadruped Kinematics & Stride Synchronization")

tb3 = slide3.shapes.add_textbox(Inches(0.8), Inches(1.55), Inches(6.8), Inches(5.4))
tf3 = tb3.text_frame
tf3.word_wrap = True
tf3.margin_left = tf3.margin_top = tf3.margin_right = tf3.margin_bottom = 0

add_bullet_item(tf3, "8-DOF Articulated Anatomy", 
                "Actuated by 8 independent metal-gear micro-servos: 4 hip servos (R1, R2, L1, L2) controlling horizontal leg sweep, and 4 knee servos (R3, R4, L3, L4) controlling vertical elevation.", is_first=True)

add_bullet_item(tf3, "6-Phase Diagonal Trot Gait", 
                "Implements a stable crawl-trot sequence alternating diagonal limb pairs (FR+RL, then FL+RR) to ensure the center of mass remains firmly within the static polygon of support.")

add_bullet_item(tf3, "1.2-Second Cadence Synchronization", 
                "Physical strides require ~1.25s to complete. Pacing controller commands with 1.2s intervals allows full mechanical foot placement, eliminating lateral drift and servo stutter.")

add_bullet_item(tf3, "Sagittal Flank Mirroring Compensation", 
                "Because left and right servos are physically mounted in reverse, right knees extend with increasing angles (87° -> 151°), while left knees extend with decreasing angles (79° -> 19°).")

add_bullet_item(tf3, "Empirical Angle Calibration Table", 
                "Sitting-to-standing joint angle offsets were tuned and mapped to compensate for mechanical spline teeth offsets, guaranteeing a level torso and symmetrical ground contact.")

# Right Column (Images)
img_topdown = os.path.join(BASE_DIR, r"docs\sesame-topdown.png")
img_hud = os.path.join(BASE_DIR, r"docs\evidence\real_setup_nav_hud.png")

add_image_fit(slide3, img_topdown, left_in=7.9, top_in=1.55, max_w_in=4.6, max_h_in=2.5, 
              caption="CAD Top-Down Anatomical Joint Configuration")
add_image_fit(slide3, img_hud, left_in=7.9, top_in=4.30, max_w_in=4.6, max_h_in=2.5, 
              caption="Rectified Planar Homography HUD (10 px/cm Metric Frame)")


# ======================================================================
# SLIDE 4: GAZEBO PHYSICS SIMULATION
# ======================================================================
slide4 = prs.slides.add_slide(blank_layout)
set_slide_background(slide4, COLOR_BG_LIGHT)
add_header(slide4, "Physics Simulation", "Gazebo Dynamic Validation & Arena Modeling")

tb4 = slide4.shapes.add_textbox(Inches(0.8), Inches(1.55), Inches(6.8), Inches(5.4))
tf4 = tb4.text_frame
tf4.word_wrap = True
tf4.margin_left = tf4.margin_top = tf4.margin_right = tf4.margin_bottom = 0

add_bullet_item(tf4, "Pre-Deployment Kinematic Verification", 
                "Provides a safe virtual sandbox to evaluate quadruped stability, center-of-mass trajectory, and obstacle avoidance physics prior to physical hardware execution.", is_first=True)

add_bullet_item(tf4, "Arena Environment Modeling", 
                "Constructed a calibrated simulation world (chartpaper_arena.world) with exact real-world dimensions (100 x 80 cm), realistic ground contact friction, and gravity models.")

add_bullet_item(tf4, "Rigid-Body Dynamics & URDF Definitions", 
                "Incorporates link mass distributions, principal inertia tensors, joint limits, and friction coefficients derived from Fusion 360 CAD specifications.")

add_bullet_item(tf4, "Interactive Dynamic Obstacle Relocation", 
                "Supports live drag-and-drop repositioning of 3D obstacle boxes during runtime to rigorously test dynamic re-planning responsiveness and collision margins.")

add_bullet_item(tf4, "Sim-to-Real Kinematic Parity", 
                "Ensures standard ROS 2 coordinate conventions (+x forward, +y left, +z up) match across simulation and physical hardware actuation.")

img_gazebo = os.path.join(BASE_DIR, r"docs\evidence\gazebo_arena_simulation.png")
add_image_fit(slide4, img_gazebo, left_in=7.9, top_in=1.55, max_w_in=4.6, max_h_in=5.2, 
              caption="Gazebo Classic Arena with Sesame Robot & Dynamic Obstacles")


# ======================================================================
# SLIDE 5: RVIZ DIGITAL TWIN WITH CAMERA TELEMETRY
# ======================================================================
slide5 = prs.slides.add_slide(blank_layout)
set_slide_background(slide5, COLOR_BG_LIGHT)
add_header(slide5, "Perception & Telemetry", "Planar Homography & Live RViz Digital Twin")

tb5 = slide5.shapes.add_textbox(Inches(0.8), Inches(1.55), Inches(6.8), Inches(5.4))
tf5 = tb5.text_frame
tf5.word_wrap = True
tf5.margin_left = tf5.margin_top = tf5.margin_right = tf5.margin_bottom = 0

add_bullet_item(tf5, "Planar Homography Coordinate Rectification", 
                "Computes a 3x3 homography matrix (H) from 4 arena corner ArUco markers, continuously rectifying tilted camera perspectives into an orthogonal top-down space (10 px/cm).", is_first=True)

add_bullet_item(tf5, "Sub-Pixel Temporal Filtering", 
                "Applies an Exponential Moving Average (EMA, alpha = 0.65) filter on robot coordinates and heading angles, attenuating camera noise and sub-pixel detection jitter.")

add_bullet_item(tf5, "Low-Latency UDP State Streaming", 
                "Streams robot coordinates, heading, goal, and obstacle boundaries as JSON datagrams over UDP port 9090 from the Windows vision engine to WSL ROS 2.")

add_bullet_item(tf5, "RViz Digital Twin Synchronization", 
                "Subscribes to incoming telemetry and publishes /joint_states, TF frame transforms (/map -> /base_footprint), planned /path, and 3D obstacle markers.")

add_bullet_item(tf5, "Human Presence Safety Gating", 
                "Integrates real-time HSV skin-color detection to pause locomotion automatically whenever a human hand enters the arena to rearrange obstacle blocks.")

img_rviz = os.path.join(BASE_DIR, r"docs\evidence\rviz_camera_path_navigation.jpeg")
add_image_fit(slide5, img_rviz, left_in=7.9, top_in=1.55, max_w_in=4.6, max_h_in=5.2, 
              caption="RViz Digital Twin Displaying Camera Feed, TF Tree & Planned Path")


# ======================================================================
# SLIDE 6: PROBLEMS FACED & HOW WE SOLVED THEM
# ======================================================================
slide6 = prs.slides.add_slide(blank_layout)
set_slide_background(slide6, COLOR_BG_LIGHT)
add_header(slide6, "Experimental Analysis", "Technical Challenges Faced & Implemented Solutions")

tb6 = slide6.shapes.add_textbox(Inches(0.8), Inches(1.50), Inches(11.733), Inches(5.6))
tf6 = tb6.text_frame
tf6.word_wrap = True
tf6.margin_left = tf6.margin_top = tf6.margin_right = tf6.margin_bottom = 0

add_bullet_item(tf6, "1. Surface Slip & Trajectory Deviation", 
                "Challenge: Smooth 3D-printed PLA foot tips suffered from a low coefficient of static friction on polished floor tiles, causing feet to slip and the robot to deviate from straight walking.\n"
                "Solution: Applied a layer of Fevibond rubberized adhesive to the foot tips and allowed it to dry, forming high-traction rubberized contact pads that eliminated ground slip.", is_first=True, font_size_body=11.5)

add_bullet_item(tf6, "2. Actuation Flooding & Crab-Walking (Lateral Drift)", 
                "Challenge: Streaming high-frequency steering commands (10 Hz) overwhelmed the ESP32 servo queue, interrupting 6-phase step cycles mid-stride and causing erratic crab-like lateral walking.\n"
                "Solution: Enforced a 1.2-second discrete step cadence matching the hardware's 1.25s physical stride, allowing legs to complete their swing-plant cycle before re-evaluating CTE.", font_size_body=11.5)

add_bullet_item(tf6, "3. Flank Asymmetry from Mirror-Mounted Servos", 
                "Challenge: Symmetrical servo PWM values curled the left leg up while extending the right leg down because opposite flanks have physically inverted mounting orientations.\n"
                "Solution: Built an empirical joint calibration matrix in firmware, establishing independent left vs. right angle transformation equations for level stance and gait execution.", font_size_body=11.5)

add_bullet_item(tf6, "4. Sub-Pixel Vision Jitter & Steering Oscillation", 
                "Challenge: Natural ambient lighting variations and sub-millimeter visual marker jitter caused rapid, alternating left-right steering chatter.\n"
                "Solution: Implemented a +/-10° heading deadband in the Stanley controller; within this threshold, the robot commands pure forward walking without unnecessary turns.", font_size_body=11.5)

add_bullet_item(tf6, "5. Marker Transposition Handling", 
                "Challenge: Accidentally swapping physical markers (ID 5 on robot, ID 4 on floor) caused the planner to command the floor target toward the robot.\n"
                "Solution: Integrated a runtime --swap-ids CLI parameter to dynamically flip robot and goal tracking identities in software without re-taping physical stickers.", font_size_body=11.5)


# ======================================================================
# SLIDE 7: CONCLUSION & FUTURE SCOPE
# ======================================================================
slide7 = prs.slides.add_slide(blank_layout)
set_slide_background(slide7, COLOR_BG_LIGHT)
add_header(slide7, "Conclusion & Future Horizons", "Project Summary & Future Research Directions")

# Left Column (Conclusion)
tb7_left = slide7.shapes.add_textbox(Inches(0.8), Inches(1.55), Inches(5.6), Inches(5.4))
tf7_left = tb7_left.text_frame
tf7_left.word_wrap = True
tf7_left.margin_left = tf7_left.margin_top = tf7_left.margin_right = tf7_left.margin_bottom = 0

add_bullet_item(tf7_left, "Project Summary", 
                "Demonstrated that robust, autonomous closed-loop navigation can be achieved on micro-scale quadrupeds using an overhead global perception architecture, bypassing on-board weight constraints.", is_first=True)

add_bullet_item(tf7_left, "Full Pipeline Verification", 
                "Successfully validated complete hardware-in-the-loop operation: overhead ArUco homography, dynamic A* path planning, Stanley steering, and wireless ESP32 execution.")

add_bullet_item(tf7_left, "Digital Twin Synchronization", 
                "Connected real-world locomotion with low-latency ROS 2 Humble RViz telemetry and validated dynamic kinematic constraints within Gazebo Classic simulation.")

add_bullet_item(tf7_left, "Pragmatic Engineering", 
                "Proved that combining simple physical enhancements (Fevibond friction soles) with disciplined control pacing (1.2s cadence) overcomes low-cost hardware limitations.")

# Vertical Divider Rule
vrule = slide7.shapes.add_shape(MSO_SHAPE.RECTANGLE, Inches(6.7), Inches(1.55), Inches(0.015), Inches(5.2))
vrule.fill.solid()
vrule.fill.fore_color.rgb = COLOR_LINE_GRAY
vrule.line.fill.background()

# Right Column (Future Scope)
tb7_right = slide7.shapes.add_textbox(Inches(7.1), Inches(1.55), Inches(5.4), Inches(5.4))
tf7_right = tb7_right.text_frame
tf7_right.word_wrap = True
tf7_right.margin_left = tf7_right.margin_top = tf7_right.margin_right = tf7_right.margin_bottom = 0

add_bullet_item(tf7_right, "Reinforcement Learning (RL) Locomotion", 
                "Train deep RL policies (PPO in Isaac Gym or MuJoCo) to replace fixed kinematic trot gaits with adaptive, robust locomotion capable of traversing carpet, gravel, and uneven terrains.", is_first=True)

add_bullet_item(tf7_right, "On-Board Inertial Sensor Fusion", 
                "Integrate an on-board 6-DOF IMU (MPU6050) with an Extended Kalman Filter (EKF) to fuse overhead vision with on-board acceleration for real-time slip detection and balance recovery.")

add_bullet_item(tf7_right, "Dynamic Obstacle Velocity Tracking", 
                "Incorporate predictive Kalman filtering and Velocity Obstacle (VO) algorithms to autonomously detect, track, and bypass moving obstacles in dense environments.")

add_bullet_item(tf7_right, "Vision-Based Auto-Calibration", 
                "Leverage the overhead camera to automatically detect foot contact slip and iteratively tune individual servo PWM offsets without manual trim calibration.")


# Save Presentation
out_pptx = os.path.join(BASE_DIR, "SesameNav_Presentation.pptx")
prs.save(out_pptx)
print(f"Presentation saved successfully to: {out_pptx}")
