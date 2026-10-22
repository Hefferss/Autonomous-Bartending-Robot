# OO_Camera.py: camera views for the GUI dropdown. each view is "Name": (camera position, point it looks at), metres. first one is the start view

from math import cos, sin, radians


# orbit: camera distance away at angle_deg round the target (0 = +x, 90 = +y), height up
def orbit(angle_deg, distance, height, target=(0.0, 0.0, 0.6)):
    x = target[0] + distance * cos(radians(angle_deg))    # sideways position on the circle
    y = target[1] + distance * sin(radians(angle_deg))    # forward position on the circle
    return ((x, y, height), target)                       # (camera position, look-at point)


CAMERA_ANGLES = {
    "Front":       orbit(0,   2.5, 1.2),                  # looking at the scene from the +x side
    "Side":        orbit(90,  2.5, 1.2),                  # from the +y side
    "Back":        orbit(180, 2.5, 1.2),                  # from the -x side
    "Corner":      orbit(45,  3.0, 1.8),                  # diagonal, a bit higher
    "Top down":    ((0.01, 0.01, 3.5), (0.0, 0.0, 0.0)),  # straight down at the floor
    "Close up":    orbit(30,  1.3, 0.9),                  # near the robot
    # "My view":   ((2.0, 1.0, 1.0), (0.0, 0.0, 0.5)),    # <- example of typing a view by hand
}# OO_Camera.py: camera views for the GUI dropdown. each view is "Name": (camera position, point it looks at), metres. first one is the start view

from math import cos, sin, radians


# orbit: camera distance away at angle_deg round the target (0 = +x, 90 = +y), height up
def orbit(angle_deg, distance, height, target=(0.0, 0.0, 0.6)):
    x = target[0] + distance * cos(radians(angle_deg))    # sideways position on the circle
    y = target[1] + distance * sin(radians(angle_deg))    # forward position on the circle
    return ((x, y, height), target)                       # (camera position, look-at point)


CAMERA_ANGLES = {
    "Front":       orbit(0,   2.5, 1.2),                  # looking at the scene from the +x side
    "Side":        orbit(90,  2.5, 1.2),                  # from the +y side
    "Back":        orbit(180, 2.5, 1.2),                  # from the -x side
    "Corner":      orbit(45,  3.0, 1.8),                  # diagonal, a bit higher
    "Top down":    ((0.01, 0.01, 3.5), (0.0, 0.0, 0.0)),  # straight down at the floor
    "Close up":    orbit(30,  1.3, 0.9),                  # near the robot
    # "My view":   ((2.0, 1.0, 1.0), (0.0, 0.0, 0.5)),    # <- example of typing a view by hand
}