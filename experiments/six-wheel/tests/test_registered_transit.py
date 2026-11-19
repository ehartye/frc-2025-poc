import json
import math
import unittest
from scripts import registered_transit as field, station_l1_mission as sim
from scripts.station_l1_settle import continue_contact


class RegisteredFieldTest(unittest.TestCase):
    def test_pose_registration_and_bumper_surface(self):
        station_in=[x/field.IN for x in field.STATION]
        self.assertLess(math.dist(station_in,(657.37,291.2)),.3)
        body=json.loads((field.ROOT/"design/station-l1/review.json").read_text())["poses"]["transport"]["boundsMm"]
        a,c,b=field.routes(-.18)
        for route in (a,c,b):
            self.assertGreaterEqual(min(r["gap_m"] for r in field.audit(route,body)),.00999)
        direct=field.Route((*field.STAGE,0))
        direct.go((field.LIP[0]-.49,field.LIP[1]))
        self.assertIn((0,"Red reef"),[(r["gap_m"],r["closest"]) for r in field.audit(direct,body)])

    def test_wheel_integration_matches_pose_and_limits(self):
        for route in field.routes(-.18):
            # Independent midpoint integration of differential wheel controls without pose resets.
            x,y,h=route.start
            now=0
            old=(0,0)
            while now<route.duration:
                step=min(.002,route.duration-now)
                _,(l,r)=route.at(now+step/2)
                dh=(r-l)*step/field.DRIVE["trackM"]
                travel=(r+l)*step/2
                x+=travel*math.cos(h+dh/2)
                y+=travel*math.sin(h+dh/2)
                h+=dh
                self.assertLessEqual(max(abs(l),abs(r)),field.DRIVE["maximumWheelMps"]+1e-8)
                self.assertLessEqual(max(abs(l-old[0]),abs(r-old[1])),.0025)
                old=(l,r)
                now+=step
            self.assertLess(math.dist((x,y),route.end[:2]),.00005)
            self.assertLess(abs(h-route.end[2]),.00005)

    def test_dense_source_and_registered_contact(self):
        path=field.RegisteredMission(-.18)
        run=sim.run(dt=.001,sample_every_step=True,target_y=-.18,trajectory=path,piece_id="red-fixture")
        rows=run["trace"]
        self.assertLessEqual(max(b["time_s"]-a["time_s"] for a,b in zip(rows,rows[1:])),.001001)
        self.assertAlmostEqual(rows[-1]["shelf_front_robot_u_m"],.535)
        self.assertEqual(run["outcome"],"paired-trough-first-contact")
        self.assertEqual(run["events"][-1]["piece_id"],"red-fixture")
        second=continue_contact(sector=-.18,upstream=run,piece_id="red-fixture",withdrawal="after-separation",
                                wait_after_true_separation_s=.127)
        self.assertEqual(second["stop"],"sustained-paired-floor-and-lip")
