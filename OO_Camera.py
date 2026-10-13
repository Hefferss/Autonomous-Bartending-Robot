# ======================================================================
# OO_CameraAngles.py  --  the list of camera views for the GUI dropdown
#
# TO ADD A VIEW:   add one line to CAMERA_ANGLES below.
# TO EDIT A VIEW:  change the numbers on its line.
# TO REMOVE ONE:   delete its line.
# The dropdown in OO_GUI.py rebuilds itself from this list, so you never
# need to touch the GUI code. The first entry is the view used at start-up.
#
# Each view is:   "Name": ((x, y, z), (x, y, z)),
#                          camera position   point it looks at
# Units are metres in the world frame (z is up).
#
# Tips:
#   - Change one number at a time and re-select the view to see the effect.
#   - Swift's lights stay fixed in the scene, so cameras very far away can
#     look dark. Keep views within a few metres of the robots.
#   - Straight-down views can flip or tilt oddly. If so, nudge the camera
#     x or y by a small amount (like 0.01).
# ======================================================================

# ---------------------------- IMPORTS ---------------------------------
from math import cos, sin, radians              # used by the orbit helper below


# ---- orbit: makes a camera position by circling a point ------------------
# Overview: instead of working out x and y yourself, say "stand 2.5 m away
# at 45 degrees round the scene, 1.2 m up, looking at the target".
# angle 0 = along the +x axis, 90 = along the +y axis.
def orbit(angle_deg, distance, height, target=(0.0, 0.0, 0.6)):
    x = target[0] + distance * cos(radians(angle_deg))    # sideways position on the circle
    y = target[1] + distance * sin(radians(angle_deg))    # forward position on the circle
    return ((x, y, height), target)                       # (camera position, look-at point)


# ---- the list of views ------------------------------------------------------
# These numbers are starting guesses - adjust them to suit your scene.
CAMERA_ANGLES = {
    "Front":       orbit(0,   2.5, 1.2),                  # looking at the scene from the +x side
    "Side":        orbit(90,  2.5, 1.2),                  # from the +y side
    "Back":        orbit(180, 2.5, 1.2),                  # from the -x side
    "Corner":      orbit(45,  3.0, 1.8),                  # diagonal, a bit higher
    "Top down":    ((0.01, 0.01, 3.5), (0.0, 0.0, 0.0)),  # straight down at the floor
    "Close up":    orbit(30,  1.3, 0.9),                  # near the robot
    # "My view":   ((2.0, 1.0, 1.0), (0.0, 0.0, 0.5)),    # <- example of typing a view by hand
}