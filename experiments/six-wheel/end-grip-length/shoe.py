"""Equal pitch radii permit an exact four-pulley outer tangent belt path."""
from dataclasses import dataclass
import math

RADIUS = 60/(2*math.pi)
DRIVE = -84.5
IDLER = 95.5
BACK = -60.0
FRONT = 71.0
OUTER_FACE = 8.0
HALF = 150.8125
LIMITS = (-6.0,6.0)

@dataclass(frozen=True)
class Design:
    spring_preload: float = 24.0
    spring_rate: float = .60
    tension: float = 15.
    drive_effort: float = 4.
    setup: float = 0.

def routing(displacement):
    offset = OUTER_FACE - displacement
    back_span = math.hypot(BACK - DRIVE,offset)
    # L pitch = 2pi R + lower straight + upper straight + two oblique sides.
    # Set the surviving outboard idler's front x so the loop stays 420 mm.
    remain = 420-60-(FRONT-BACK)-back_span+DRIVE-FRONT
    front_dx = (remain**2-offset**2)/(2*remain)
    end = FRONT+front_dx
    af, ab=math.atan2(offset,front_dx), math.atan2(offset,BACK-DRIVE)
    length=60+end-DRIVE+FRONT-BACK+back_span+math.hypot(front_dx,offset)
    return {"idlerX":end,"pitchLength":length,"frontAngle":af,"backAngle":ab,
            "driveWrapTeeth":20*(math.pi-ab)/(2*math.pi),"noseSpacing":min(back_span,math.hypot(front_dx,offset))}

def load(displacement,design,drive_sign=1):
    line=routing(displacement)
    front=design.tension+drive_sign*design.drive_effort/2
    back=design.tension-drive_sign*design.drive_effort/2
    return design.spring_preload+design.spring_rate*displacement-front*math.sin(line["frontAngle"])-back*math.sin(line["backAngle"])

def projection(length,side_error,yaw,side,setup=0):
    angle=math.radians(yaw)
    # Radial ends of the annular center-height patch bound both directions.
    center=length/2*math.cos(angle)-HALF+side*side_error-setup
    radial=57.15*abs(math.sin(angle))
    return center-radial,center+radial
