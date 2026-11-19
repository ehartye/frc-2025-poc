"""Demand at the gearbox, the bolted inner rail and its welded ear; N, mm, MPa."""
from math import hypot, pi, sqrt
from joint import HERE, ROOT, read, write


def calculate(factor, side_force_n, throat, weld_spacing):
    original=read(ROOT/"design/drivetrain/route-receipt.json")["candidate"]["supportScreen"]
    torque=original["shaftTorqueNm"]*1000
    radial=original["innerBearingRadialN"]
    span=624
    # Each bearing force component is bounded by its radial magnitude. Give one
    # end all longitudinal reactions. Vertical reactions of the simple supports
    # sum by influence lines, incl the centered gearbox reaction couple.
    fx=factor*3*radial
    fz=factor*(sum(radial*(x+312)/span for x in (-247.65,0,247.65))+torque/span)
    fy=factor*side_force_n
    # Give all local demand to one hole for bearing/tearout; bolt proof tension
    # includes out-of-plane seating eccentricity; no friction credit.
    eccentric_y=17 if weld_spacing==20 else 10
    mx=fy*abs(110-76.2)+fz*eccentric_y
    my=abs(110-76.2)*fx+18*fz
    mz=18*fy+eccentric_y*fx
    sh=hypot(fx,fz)/2+abs(my)/40
    # Pair is vertical. A moment about Z cannot be reacted by bolt spacing
    # alone; require inboard contact on an independently measured lever arm.
    lever=10
    tension=fy/2+abs(mx)/40+abs(mz)/lever
    area=2*64*throat
    iy=2*throat*64**3/12
    half=weld_spacing/2
    iz=2*throat*64*half**2
    # Crossrail inner-wall plane normal X, weld group shear in Y/Z.
    sigma=abs(fx)/area+abs(my)*32/iy+abs(mz)*half/iz
    tau=hypot(fy,fz)/area+abs(mx)*hypot(32,half)/(iy+iz)
    return {"factor":factor,"weldSpacingMm":weld_spacing,"unfactoredSideN":side_force_n,"effectiveThroatMm":throat,
            "jointForceXYZ_N":[fx,fy,fz],"jointMomentXYZ_Nmm":[mx,my,mz],
            "gearboxScrewTangentialN":factor*torque/(2*25.4),
            "adapterM5TangentialN":factor*torque/(4*hypot(40,12)),
            "maxNominalShearN":sh,"M8ThreadShearMPa":sh/36.6,
            "maxTensionNAt10MmContactLever":tension,"M8ThreadTensileMPa":tension/36.6,
            "minimumVerifiedContactLeverMm":lever,
            "M6ControlEquivalentMPa":hypot(tension/20.1,sqrt(3)*sh/20.1),
            "M8EquivalentMPa":hypot(tension/36.6,sqrt(3)*sh/36.6),
            "railHoleBearingMPa":sh/(8*6),"earHoleBearingMPa":sh/(8*8),
            "earInnerShearoutMPa":sh/(2*8*(12-4.5)),"railEndShearoutMPa":sh/(2*6*(14-4.5)),
            "pilotOnlyShearMPa":hypot(fx,fz)/(pi*3**2),
            "pilotOnlyRailBearingMPa":hypot(fx,fz)/(4*6),
            "weldEquivalentMPa":hypot(sigma,sqrt(3)*tau)}


if __name__=="__main__":
    write(HERE/"demand.json",[calculate(*case) for case in
          [(1,0,1.75,20),(2,500,1.75,20),(2,500,1.0,20),(2,500,1.75,8)]])
