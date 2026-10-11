import sys
import math
import struct
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
from control_core import ackermann, command_frame, TelemetryParser, CommandLease, OdometryState, depth_roi
from centraliza_cone import Config, ConeController


class ControlTests(unittest.TestCase):
    def test_straight_motion(self):
        l,r,a=ackermann(.2,0.,.3,.195,.0475,.35,10.)
        self.assertAlmostEqual(l,r)
        self.assertEqual(a,0.)
        self.assertAlmostEqual(l*.0475,.2)

    def test_saturated_curve_matches_wheels(self):
        for v in (.1,-.1):
            for omega in (-1.,1.):
                l,r,a=ackermann(v,omega,.3,.195,.0475,.35,10.)
                actual_v=(l+r)*.0475/2
                actual_omega=(r-l)*.0475/.195
                self.assertAlmostEqual(actual_omega,actual_v*math.tan(a)/.3)
                self.assertLessEqual(abs(a),.35)

    def test_wheel_limit_preserves_curvature(self):
        l,r,a=ackermann(10.,5.,.3,.195,.0475,.35,10.)
        self.assertAlmostEqual(max(abs(l),abs(r)),10.)
        self.assertAlmostEqual((r-l)/.195,(l+r)/2*math.tan(a)/.3)

    def test_no_rotation_in_place(self):
        self.assertEqual(ackermann(0.,1.,.3,.195,.0475,.35,10.),(0.,0.,0.))

    def test_invalid_commands(self):
        with self.assertRaises(ValueError):
            ackermann(float('nan'),1.,.3,.195,.0475,.35,10.)
        with self.assertRaises(ValueError):
            command_frame(float('inf'),0.,0.,False)

    def test_wire_command_layout(self):
        wire=command_frame(1.25,-2.5,.125,True)
        self.assertEqual(len(wire),16)
        self.assertEqual((wire[0],wire[-1]),(0xAA,0xBB))
        self.assertEqual(struct.unpack('<fffB',wire[1:14]),(1.25,-2.5,.125,1))
        self.assertEqual(wire[14],sum(wire[1:14])&255)

    def telemetry(self):
        payload=struct.pack('<iiiI',1000000,-500000,1571,0xFFFFFFF0)
        return b'\xaa'+payload+bytes([sum(payload)&255,0xBB])

    def test_every_serial_fragment_boundary(self):
        wire=self.telemetry()
        for split in range(1,len(wire)):
            parser=TelemetryParser()
            self.assertEqual(parser.feed(wire[:split]),[])
            self.assertEqual(parser.feed(wire[split:]),[(1000000,-500000,1571,0xFFFFFFF0)])

    def test_serial_noise_and_checksum_recovery(self):
        good=self.telemetry()
        bad=bytearray(good); bad[10]^=1
        parser=TelemetryParser()
        result=parser.feed(b'boot log\n'+bad+good+good)
        self.assertEqual(len(result),2)
        self.assertLess(len(parser.buffer),19)

    def test_command_expiry(self):
        lease=CommandLease(.3)
        self.assertEqual(lease.get(1.),(0.,0.,0.))
        lease.update((2.,2.,.2),1.)
        self.assertEqual(lease.get(1.1),(2.,2.,.2))
        self.assertEqual(lease.get(1.4),(0.,0.,0.))
        lease.clear()
        self.assertEqual(lease.get(1.1),(0.,0.,0.))

    def test_micrometers_to_meters(self):
        odom=OdometryState()
        odom.update(0,0,0,1000)
        odom.update(1000000,0,0,2000)
        self.assertEqual((odom.x,odom.vx),(1.,1.))

    def test_local_velocity_and_yaw_wrap(self):
        odom=OdometryState()
        odom.update(0,0,1571,1000)
        odom.update(0,100000,1571,1100)
        self.assertAlmostEqual(odom.vx,1.,places=5)
        odom.update(0,100000,3140,1200)
        odom.update(0,100000,-3140,1300)
        self.assertLess(abs(odom.vth),.1)

    def test_timestamp_wrap_and_reboot(self):
        odom=OdometryState()
        odom.update(0,0,0,0xFFFFFFF0)
        odom.update(10000,0,0,24)
        self.assertAlmostEqual(odom.vx,.25)
        odom.update(0,0,0,1)
        self.assertEqual(odom.vx,0.)

    def test_roi_integer_bounds(self):
        class ArrayProbe:
            shape=(240,424)
            def __getitem__(self,key):
                return key[0].indices(240),key[1].indices(424)
        self.assertEqual(depth_roi(ArrayProbe(),0.1,0.5),((0,3,1),(0,3,1)))
        self.assertEqual(depth_roi(ArrayProbe(),423.9,239.9),((237,240,1),(421,424,1)))

    def test_pixel_to_wheel_command_end_to_end(self):
        controller=ConeController(Config())
        for i,x in enumerate((80.,340.)):
            now=100.+i
            controller.receive(x,120.,now,now,now)
            cmd=controller.command(now,now)
            l,r,a=ackermann(cmd.linear_x,cmd.angular_z,.3,.195,.0475,.35,10.)
            self.assertGreater(cmd.linear_x,0.)
            self.assertEqual(a>0,x<211.5)
            servo_deg=34.-math.degrees(a)
            self.assertTrue(0<=servo_deg<=68)
            self.assertEqual(servo_deg>34,x>211.5)
            self.assertEqual(len(command_frame(l,r,a,False)),16)

    def test_camera_loss_stops_full_chain(self):
        controller=ConeController(Config())
        controller.receive(80.,120.,100.,100.,100.)
        cmd=controller.command(100.5,100.5)
        self.assertEqual(ackermann(cmd.linear_x,cmd.angular_z,.3,.195,.0475,.35,10.),(0.,0.,0.))


if __name__=='__main__':
    unittest.main(verbosity=2)
