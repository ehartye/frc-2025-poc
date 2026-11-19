"""Elastic members and fastener-group demand at the project 2 g comparison load."""
import json
import math
from pathlib import Path
HERE=Path(__file__).resolve().parent
ROOT=HERE.parent


def mechanics(parts, wall, scale):
    gravity=2*9.80665
    E=69e9
    poisson=.33
    G=E/(2*(1+poisson))
    t=wall/1000
    L=.3595
    B=.4
    I=(.015**4-(.015-2*t)**4)/12
    J=(.015-t)**3*t  # thin-walled closed square, midline Bredt relation
    installed=parts["camera-envelope"]["kg"]
    post=scale*parts["left-post"]["kg"]
    beam=scale*parts["top-tube"]["kg"]
    end=scale*sum(v["kg"] for k,v in parts.items() if k.startswith(("camera-M4","camera-pin")) or k=="top-plate-and-keepers")
    # All the 100 g cable/clip and 15 g weld upper allowances are pessimistically put at the distal mount.
    tip=end+installed+.115
    p=gravity*(tip+beam)/2
    post_mm=1000*(p*L**3/3+post*gravity*L**3/8)/(E*I)
    post_rad=(p*L**2/2+post*gravity*L**2/6)/(E*I)
    cross_mm=1000*(gravity*tip*B**3/48+5*gravity*beam*B**3/384)/(E*I)
    cross_rad=gravity*(tip*B**2/16+beam*B**2/24)/(E*I)
    torsion=tip*gravity*.0455*B/(4*G*J) # centered torque, two equally stiff ends
    # Two 4 mm wide lands extend 3 mm above the 3 mm plate for the first 36 mm.
    # Ignore helpful parallel-axis effects and the plate outside the two outer 16 mm lands.
    tip_force=gravity*(installed+.115+end)
    length=.038
    last=.002
    stiff=(.024*.003**3+ .008*.006**3)/12
    plain=.032*.003**3/12
    plate_mm=1000*tip_force/3/E*((length**3-last**3)/stiff+last**3/plain)
    plate_rad=tip_force/2/E*((length**2-last**2)/stiff+last**2/plain)
    stress=(p*L+post*gravity*L/2)*.0075/I/1e6
    return {"wall_mm":wall,"steel_and_aluminum_scale":scale,"young_pa":E,"assumed_poisson":poisson,"I_m4":I,"J_m4_estimate":J,
        "tip_mass_kg":tip,"post_mm":post_mm,"crossbar_mm":cross_mm,"torsion_mm_at_lens":1000*torsion*.0455,"plate_mm":plate_mm,
        "post_deg":math.degrees(post_rad),"crossbar_deg":math.degrees(cross_rad),"torsion_deg":math.degrees(torsion),
        "plate_deg":math.degrees(plate_rad),"summed_tip_mm":post_mm+cross_mm+1000*torsion*.0455+plate_mm,
        "summed_rotation_deg":math.degrees(post_rad+cross_rad+torsion+plate_rad),"post_base_outer_fiber_mpa":stress,
        "force_per_post_n":p+post*gravity,"moment_per_post_nm":p*L+post*gravity*L/2}


def run():
    read=lambda p:json.loads(p.read_bytes())
    p=read(HERE/"native.json")["parts"]
    actual=sum(v["kg"] or 0 for v in p.values())
    prior=read(ROOT/"moving-observer/mass.json")
    # Remove the lowest old metal allocation and add the highest new metal allocation once.
    change_upper=1.05*(actual-.061)-.95*(prior["nominal_kit_kg"]-.061)
    mass=read(ROOT/"design/fixed-no-air/inspection.json")["cases"]["low-camera-red"]
    new=mechanics(p,2,1.05)
    same_load_old_section=mechanics(p,1,1.05)
    dz=new["moment_per_post_nm"]/.047
    shear=new["force_per_post_n"]/2
    pad=math.pi*(8**2-4**2)/4
    # 100 N and 300 N are fixture trial clamp forces per screw. Torque/friction are deliberately absent.
    trials=[]
    for clamp in (25,100,300):
        trials.append({"installed_clamp_n_assumed":clamp,"residual_n":clamp-dz,"compression_mpa":(clamp+dz)/pad,
                       "joint_seated_under_model":clamp>dz,"M4_tensile_stress_upper_mpa":(clamp+dz)/8.78,
                       "friction_mu_needed_if_keys_unloaded":shear/max(1e-6,clamp-dz) if clamp>dz else None})
    # Tangential sleeve clearance can load the two keys unequally. Assign all horizontal load to one 6 mm locator.
    loc_load=2*new["force_per_post_n"]
    load={"whole_lateral_n":loc_load,"locator_shear_demand_mpa":loc_load/(math.pi*3**2),
          "2mm_locator_bearing_projection_mpa":loc_load/(6*1.8),"foot_heel_uplift_n_per_fastener":dz,
          "foot_bolt_shear_n_each":shear,"metal_annulus_mm2":pad,"trial_preloads":trials,
          "unsupported_cable_bracket_tip_load_N_per_10N":10,"extra_10N_heel_uplift_N":(10*.3595/2)/.047}
    # Drawing fit stack at the two permanent stations. A second installation on opposite
    # sides of the same clearance can move twice the single-installation radial gap.
    locating_radial=(6.10-5.99)/2
    bar_radial=(8.25-7.98)/2
    point=locating_radial+bar_radial+.04+.03
    datums={"locating_radial_mm":locating_radial,"boss_to_shelf_radial_mm":bar_radial,"single_station_xy_mm":point,
            "fixed_saddle_reseat_translation_mm":2*locating_radial,"fixed_saddle_reseat_clock_deg":math.degrees(math.atan(4*locating_radial/400)),
            "two_station_absolute_clock_deg":math.degrees(math.atan(2*point/400)),"pad_pitch_at_0_04mm_over_47_deg":math.degrees(math.atan(.04/47)),
            "two_plain_camera_bolts_clock_deg":math.degrees(math.atan(.5/75)),"two_plain_foot_bolts_clock_deg":math.degrees(math.atan(.5/47)),
            "nominal_right_slot_travel_per_side_mm":(8.08-6)/2,"within_20K_if_differential_growth_below_per_K":.75/400/20}
    result={"source_load":"moving-observer/checks.json","derived_2g":new,"datum_stack":datums,"prior_1mm_section_under_new_upper_load":same_load_old_section,
            "joint_demand":load,"native_non_shelf_mass_kg":actual,"extra_upper_moving_mass_kg":change_upper,
            "no_credit_for_removed_polymer":True,"prior_independent_R103_headroom_kg":mass["checker_upper_headroom_kg"],
            "successor_upper_headroom_kg":mass["checker_upper_headroom_kg"]-change_upper,"weighed_robot":False}
    (HERE/"mechanics.json").write_text(json.dumps(result,indent=2)+"\n",encoding="utf8",newline="\n")
if __name__=="__main__":run()
