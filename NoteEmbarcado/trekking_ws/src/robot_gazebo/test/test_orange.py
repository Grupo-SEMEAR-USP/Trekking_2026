import math
import sys
import unittest
from pathlib import Path
from dataclasses import replace
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src'))
from orange_core import (Detection, MissionConfig, OrangeMission, LaserGuard, OrangeTracker,
                         orange_class_ids, box_detection, cone_depth_m)


def cone(i=1,x=319.5,depth=2.08):
    return Detection(i,x,240.,60.,100.,.9,2,depth)


def scan(guard,now,ranges=None,stamp=None):
    return guard.update([5.]*360 if ranges is None else ranges,-math.pi,math.pi/180,.05,10.,
                        now if stamp is None else stamp,now,now)


class VisionTests(unittest.TestCase):
    def test_model_only_accepts_orange_classes(self):
        names={0:'blue_cone',1:'large_orange_cone',2:'orange_cone',3:'unknown_cone',4:'yellow_cone'}
        self.assertEqual(orange_class_ids(names),[1,2])
        self.assertEqual(orange_class_ids(['cat','orange_cone']),[1])
        with self.assertRaises(ValueError):
            orange_class_ids({0:'person'})

    def test_center_is_midpoint_of_original_box(self):
        d=box_detection([100,50,160,150],.9,2,640,480)
        self.assertEqual((d.x,d.y,d.width,d.height),(130.,100.,60.,100.))

    def test_clipped_and_invalid_boxes(self):
        self.assertIsNone(box_detection([math.nan,1,50,60],.9,2,640,480))
        self.assertIsNone(box_detection([30,20,10,80],.9,2,640,480))
        d=box_detection([-20,-10,40,80],.9,2,640,480)
        self.assertEqual((d.x,d.y),(20.,40.))

    def test_depth_median_ignores_holes_and_spikes(self):
        depth=[[280]*50 for _ in range(50)]
        depth[25][25]=0
        depth[24][24]=9000
        d=Detection(1,25.,25.,30.,30.,.9,2)
        self.assertAlmostEqual(cone_depth_m(depth,d,.001),.28)
        self.assertTrue(math.isnan(cone_depth_m([[0]*50 for _ in range(50)],d,.001)))

    def test_depth_rejects_mixed_background(self):
        depth=[[300 if x%2 else 3000 for x in range(50)] for _ in range(50)]
        self.assertTrue(math.isnan(cone_depth_m(depth,Detection(1,25,25,30,30,.9,2),.001)))

    def test_tracking_stable_across_class_size_change(self):
        tracker=OrangeTracker()
        a=tracker.update([cone(x=200)],10.)[0]
        b=tracker.update([replace(cone(x=210),class_id=1)],10.1)[0]
        self.assertEqual(a.track_id,b.track_id)
        new=tracker.update([cone(x=210)],12.)[0]
        self.assertNotEqual(new.track_id,a.track_id)

    def test_two_cones_have_distinct_nonrecycled_ids(self):
        tracker=OrangeTracker()
        before=tracker.update([cone(x=100),cone(x=500)],10.)
        after=tracker.update([cone(x=490),cone(x=110)],10.1)
        self.assertEqual([d.track_id for d in after],[before[1].track_id,before[0].track_id])


class GuardTests(unittest.TestCase):
    def setUp(self):
        self.guard=LaserGuard(MissionConfig())

    def test_fresh_front_and_rear_clear(self):
        scan(self.guard,100.)
        self.assertTrue(self.guard.permits(.1,0,100.,100.)[0])
        self.assertTrue(self.guard.permits(-.08,0,100.,100.)[0])
        self.assertFalse(self.guard.permits(.1,0,100.5,100.5)[0])

    def test_front_obstacle_blocks_forward_not_reverse(self):
        ranges=[5.]*360; ranges[180]=.28
        scan(self.guard,100.,ranges)
        self.assertFalse(self.guard.permits(.1,0,100.,100.)[0])
        self.assertTrue(self.guard.permits(-.08,0,100.,100.)[0])

    def test_rear_obstacle_blocks_reverse(self):
        ranges=[5.]*360; ranges[0]=.16
        scan(self.guard,100.,ranges)
        self.assertFalse(self.guard.permits(-.08,0,100.,100.)[0])

    def test_bad_or_blind_scans_never_authorize_motion(self):
        for ranges in ([math.nan]*360,[0.]*360,[math.inf]*360):
            scan(self.guard,100.,ranges)
            self.assertFalse(self.guard.permits(.1,0,100.,100.)[0])

    def test_duplicate_scan_does_not_renew_timeout(self):
        scan(self.guard,100.)
        self.assertFalse(scan(self.guard,100.2,stamp=100.))
        self.assertFalse(self.guard.fresh(100.4,100.4))

    def test_missing_coverage_blocks_reverse(self):
        self.guard.update([5.]*181,-math.pi/2,math.pi/180,.05,10.,100.,100.,100.)
        self.assertTrue(self.guard.permits(.1,0,100.,100.)[0])
        self.assertFalse(self.guard.permits(-.08,0,100.,100.)[0])

    def test_sensor_yaw_and_translation_transform_obstacle(self):
        guard=LaserGuard(MissionConfig(lidar_yaw_rad=math.pi,lidar_x_m=.1))
        ranges=[5.]*360; ranges[0]=.18  # scan -pi + montagem pi -> frente; x=.28
        scan(guard,100.,ranges)
        self.assertFalse(guard.permits(.1,0,100.,100.)[0])


class MissionTests(unittest.TestCase):
    def setUp(self):
        self.now=100.
        self.m=OrangeMission(MissionConfig(mode='mission'))
        self.m.start(self.now)

    def feed(self,detections,dt=.1):
        self.now+=dt
        scan(self.m.guard,self.now)
        self.m.observe(detections,self.now,self.now,self.now)
        return self.m.step(self.now,self.now)

    def lock(self,d=None):
        d=d or cone()
        for _ in range(3):
            out=self.feed([d])
        self.assertEqual(out.state,'ALIGN')
        return out

    def arrive(self,i=1):
        self.lock(cone(i))
        for _ in range(3):
            self.feed([cone(i)])
        for _ in range(3):
            out=self.feed([cone(i,depth=.28)])
        self.assertEqual(out.state,'BLINK')
        return out

    def test_twenty_cm_front_is_twenty_eight_cm_camera(self):
        self.assertAlmostEqual(self.m.gap(cone(depth=.28)),.20)
        self.assertAlmostEqual(self.m.c.arrival_gap_m+self.m.c.camera_to_front_m,.28)

    def test_preview_has_pixel_even_without_depth_and_never_moves(self):
        self.m=OrangeMission(MissionConfig(mode='preview'))
        self.m.start(self.now)
        self.lock(cone(depth=math.nan))
        out=self.feed([cone(x=500,depth=math.nan)])
        self.assertEqual((out.v,out.omega),(0.,0.))
        self.assertIsNotNone(out.target)

    def test_left_and_right_correct_servo_direction(self):
        self.lock(cone(x=100))
        for x in (100.,500.):
            out=self.feed([cone(x=x)])
            self.assertGreater(out.v,0.)
            self.assertEqual(out.omega>0,x<319.5)
            wheel_angle=math.atan(.3*out.omega/out.v)
            self.assertEqual(34-math.degrees(wheel_angle)>34,x>319.5)

    def test_centered_robot_continues_approach(self):
        self.lock()
        for _ in range(3):
            out=self.feed([cone()])
        self.assertEqual(out.state,'APPROACH')
        self.assertGreater(out.v,0.)
        self.assertEqual(out.omega,0.)

    def test_arrival_requires_distinct_recent_frames(self):
        self.lock()
        self.feed([cone(depth=.28)])
        for _ in range(10):
            self.m.observe([cone(depth=.28)],self.now,self.now,self.now)
            out=self.m.step(self.now,self.now)
        self.assertEqual(out.visited,0)
        for _ in range(2):
            out=self.feed([cone(depth=.28)])
        self.assertEqual((out.state,out.visited,out.v),('BLINK',1,0.))

    def test_three_blinks_and_reverse_before_search(self):
        self.arrive()
        t=self.now
        self.assertTrue(self.m.step(t+.05,t+.05).led)
        self.assertFalse(self.m.step(t+.25,t+.25).led)
        out=self.m.step(t+1.21,t+1.21)
        self.now=t+1.21
        self.assertEqual(out.state,'RETREAT')
        out=self.feed([cone(depth=.58)])
        self.assertLess(out.v,0.)
        out=self.feed([cone(depth=1.08)])
        self.assertEqual(out.state,'SEARCH_RIGHT')
        out=self.feed([])
        self.assertGreater(out.v,0.)
        self.assertLess(out.omega,0.)

    def test_visited_target_cannot_be_selected_again(self):
        self.m.visited_ids.add(1)
        self.m.transition('SEARCH_RIGHT',self.now)
        for _ in range(4):
            out=self.feed([cone(1,x=500)])
        self.assertEqual(out.state,'SEARCH_RIGHT')
        self.assertIsNone(self.m.target_id)

    def test_search_does_not_select_left_cone(self):
        self.m.transition('SEARCH_RIGHT',self.now)
        for _ in range(4):
            out=self.feed([cone(2,x=100)])
        self.assertEqual(out.state,'SEARCH_RIGHT')

    def test_search_prefers_nearer_right_cone_over_opposite_vertex(self):
        self.m.transition('SEARCH_RIGHT',self.now)
        for _ in range(3):
            out=self.feed([cone(2,x=550,depth=2.08),cone(3,x=400,depth=4.08)])
        self.assertEqual((out.state,self.m.target_id),('ALIGN',2))

    def test_missing_lidar_prevents_arrival_signal(self):
        for i in range(6):
            self.now+=.1
            self.m.observe([cone(depth=.28)],self.now,self.now,self.now)
            out=self.m.step(self.now,self.now)
        self.assertEqual((out.visited,out.v,out.led),(0,0.,False))

    def test_cone_too_close_faults_without_blink(self):
        self.lock()
        out=self.feed([cone(depth=.15)])
        self.assertEqual((out.state,out.v,out.led),('FAULT',0.,False))

    def test_camera_loss_stops_then_latches_fault(self):
        self.lock(cone(x=500))
        out=self.m.step(self.now+.5,self.now+.5)
        self.assertEqual(out.v,0.)
        out=self.m.step(self.now+3.,self.now+3.)
        self.assertEqual(out.state,'FAULT')
        out=self.feed([cone(x=500)])
        self.assertEqual(out.v,0.)

    def test_invalid_depth_stops_current_target(self):
        self.lock(cone(x=500))
        out=self.feed([cone(x=500,depth=math.nan)])
        self.assertEqual(out.v,0.)

    def test_obstacle_and_lidar_loss_stop(self):
        self.lock(cone(x=500))
        self.now+=.1
        ranges=[5.]*360; ranges[180]=.25
        scan(self.m.guard,self.now,ranges)
        self.m.observe([cone(x=500)],self.now,self.now,self.now)
        self.assertEqual(self.m.step(self.now,self.now).v,0.)
        self.now+=.5
        self.m.observe([cone(x=500)],self.now,self.now,self.now)
        self.assertEqual(self.m.step(self.now,self.now).v,0.)

    def test_stop_service_state_does_not_restart_on_detections(self):
        self.lock()
        self.m.stop(self.now)
        out=self.feed([cone(x=500)])
        self.assertEqual((out.state,out.v),('IDLE',0.))

    def test_clockwise_search_times_out(self):
        self.m.transition('SEARCH_RIGHT',self.now)
        out=self.feed([],dt=18.)
        self.assertEqual((out.state,out.v),('FAULT',0.))

    def test_four_cone_mission_finishes_and_stays_stopped(self):
        for i in range(1,5):
            if i == 1:
                self.lock(cone(i))
            else:
                for _ in range(3):
                    out=self.feed([cone(i,x=500)])
                self.assertEqual(out.state,'ALIGN')
            for _ in range(3):
                self.feed([cone(i)])
            for _ in range(3):
                out=self.feed([cone(i,depth=.28)])
            self.assertEqual((out.state,out.visited),('BLINK',i))
            self.now+=1.21
            out=self.m.step(self.now,self.now)
            if i < 4:
                self.assertEqual(out.state,'RETREAT')
                out=self.feed([cone(i,depth=1.08)])
                self.assertEqual(out.state,'SEARCH_RIGHT')
        self.assertEqual((out.state,out.v,out.led),('COMPLETE',0.,False))
        self.assertEqual(self.feed([cone(9,x=500)]).state,'COMPLETE')

    def test_lidar_can_supply_missing_camera_depth(self):
        d=replace(cone(depth=math.nan),bearing_rad=0.,half_width_rad=.1)
        ranges=[5.]*360
        # Câmera x=.22-.08=.14; cone x=.42 -> profundidade axial .28.
        for i in (179,180,181):
            ranges[i]=.42/math.cos((i-180)*math.pi/180)
        scan(self.m.guard,self.now,ranges)
        self.m.resolve_range(replace(d,depth_m=.28),self.now,self.now,self.now)
        resolved=self.m.resolve_range(d,self.now,self.now,self.now)
        self.assertAlmostEqual(resolved.depth_m,.28,places=3)
        self.assertEqual(resolved.range_source,'lidar')

    def test_lidar_background_cannot_replace_unknown_depth(self):
        d=replace(cone(depth=math.nan),bearing_rad=0.,half_width_rad=.1)
        scan(self.m.guard,self.now)
        self.assertTrue(math.isnan(self.m.resolve_range(d,self.now,self.now,self.now).depth_m))

    def test_lidar_range_jump_does_not_reacquire_background(self):
        d=replace(cone(depth=.4),bearing_rad=0.,half_width_rad=.1)
        # Sem scan: memoriza a medida inicial da câmera.
        self.m.resolve_range(d,self.now,self.now,self.now)
        self.now+=.1
        scan(self.m.guard,self.now)
        self.assertTrue(math.isnan(self.m.resolve_range(replace(d,depth_m=math.nan),self.now,self.now,self.now).depth_m))

    def test_camera_lidar_disagreement_is_rejected(self):
        d=replace(cone(depth=2.),bearing_rad=0.,half_width_rad=.1)
        ranges=[5.]*360
        for i in (179,180,181):
            ranges[i]=.42
        scan(self.m.guard,self.now,ranges)
        self.assertTrue(math.isnan(self.m.resolve_range(d,self.now,self.now,self.now).depth_m))

    def test_invalid_configuration_fails_before_motion(self):
        for kwargs in (dict(arrival_gap_m=.05),dict(wheelbase_m=0.),dict(max_cones=0),dict(mode='bad')):
            with self.assertRaises(ValueError):
                MissionConfig(**kwargs)


if __name__ == '__main__':
    unittest.main(verbosity=2)
