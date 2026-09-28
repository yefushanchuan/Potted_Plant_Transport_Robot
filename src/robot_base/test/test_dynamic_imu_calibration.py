"""Dynamic-only analytic checks with unequal clocks/rates and independent records."""
import copy
import importlib.util
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
import numpy as np
from scipy.spatial.transform import Rotation

SCRIPT=Path(__file__).resolve().parents[1]/'scripts/dynamic_imu_calibration.py'
SPEC=importlib.util.spec_from_file_location('dynamic_imu_calibration',SCRIPT)
cal=importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(cal)

def fixture(phase=0.,noise=False,planar=False,coincident=False,static=False):
    R=Rotation.from_euler('xyz',[17,-11,48],degrees=True).as_matrix()
    arm=np.zeros(3) if coincident else np.array([.12,-.08,.03])
    td=.037
    bg=np.array([[.011,-.007,.009],[-.013,.008,.004]])
    ba=np.array([[.03,-.04,.08],[-.05,.09,-.02]])
    pair=[];rng=np.random.default_rng(471+int(phase*100))
    for i,hz in enumerate([400,200]):
        t=np.arange(0,24,1/hz);u=t+phase
        w=np.column_stack([np.sin(.8*u)+.15*np.cos(1.7*u),.7*np.cos(.6*u),.9*np.sin(1.1*u)])
        alpha=np.column_stack([.8*np.cos(.8*u)-.255*np.sin(1.7*u),-.42*np.sin(.6*u),.99*np.cos(1.1*u)])
        if static:
            s=np.clip((t-4)/2,0,1);envelope=s*s*(3-2*s);derivative=3*s*(1-s)
            alpha=alpha*envelope[:,None]+w*derivative[:,None];w=w*envelope[:,None]
        if planar:w[:,:2]=0;alpha[:,:2]=0
        f=np.column_stack([2*np.sin(.4*u),1.4*np.cos(.7*u),9.81+.5*np.sin(.9*u)])
        if static:f=np.array([0.,0.,9.81])+(f-[0.,0.,9.81])*envelope[:,None]
        if i:
            f=(f+np.cross(alpha,arm)+np.cross(w,np.cross(w,arm)))@R
            w=w@R
        f+=ba[i];w+=bg[i]
        if noise:
            f+=rng.normal(0,.008,f.shape);w+=rng.normal(0,.0002,w.shape)
        pair.append(np.column_stack([1700000000.+t-i*td,f,w]))
    return pair,dict(R=R.tolist(),t_m=arm.tolist(),td_s=td,
        gyro_bias0_rad_s=bg[0].tolist(),gyro_bias1_rad_s=bg[1].tolist(),
        accel_difference_bias_m_s2=(R@ba[1]-ba[0]).tolist())

class DynamicCalibrationTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.pair,cls.truth=fixture()
        cls.fit=cal.calibrate(cls.pair,{'method':'joint_free'})

    def test_recovers_all_parameters_without_static(self):
        self.assertFalse(self.fit['quality_failures'])
        self.assertTrue(self.fit['diagnostics']['no_static_information'])
        d=cal.compare(self.truth,self.fit['estimate'])
        self.assertLess(d['rotation_deg'],.01)
        self.assertLess(d['translation_mm'],.1)
        self.assertLess(abs(d['td_ms']),.02)
        self.assertLess(max(d['gyro_bias_difference_norm_rad_s']),.0001)
        np.testing.assert_allclose(self.fit['estimate']['accel_difference_bias_m_s2'],
                                   self.truth['accel_difference_bias_m_s2'],atol=.001)

    def test_independent_noisy_record_frozen_validation(self):
        pair,_=fixture(phase=1.7,noise=True)
        v=cal.validate_pair(self.pair,pair,self.fit)
        self.assertFalse(v['failures'])
        self.assertLess(v['frozen_forward']['accel_rmse_m_s2'],.01)
        self.assertLess(v['frozen_reverse']['accel_rmse_m_s2'],.01)

    def test_wrong_time_sign_detected_in_prediction(self):
        wrong=copy.deepcopy(self.fit['estimate']);wrong['td_s']*=-1
        good=cal.predict(self.pair,self.fit['estimate'])
        bad=cal.predict(self.pair,wrong)
        self.assertGreater(bad['accel_rmse_m_s2'],100*good['accel_rmse_m_s2'])
        self.assertGreater(bad['gyro_rmse_rad_s'],.01)

    def test_planar_data_rejected(self):
        with self.assertRaisesRegex(ValueError,'multi-axis'):
            cal.calibrate(fixture(planar=True)[0],{'method':'joint_free'})

    def test_coincident_imus_cannot_observe_absolute_gyro_bias(self):
        result=cal.calibrate(fixture(coincident=True)[0],{'method':'joint_free'})
        self.assertIn('weak_parameter_observability',result['quality_failures'])

    def test_duplicate_time_and_bad_units_rejected(self):
        pair=[a.copy() for a in self.pair];pair[1][100,0]=pair[1][99,0]
        with self.assertRaisesRegex(ValueError,'timestamps'):cal.calibrate(pair)
        pair=[a.copy() for a in self.pair];pair[1][:,1:4]/=9.80665
        with self.assertRaisesRegex(ValueError,'specific force'):cal.calibrate(pair)

    def test_holdout_cannot_duplicate_training(self):
        with self.assertRaisesRegex(ValueError,'duplicates'):cal.validate_pair(self.pair,self.pair,self.fit)

    def test_measured_chain_composition(self):
        Rb0=Rotation.from_euler('xyz',[3,-7,25],degrees=True).as_matrix()
        R1l=Rotation.from_euler('x',8,degrees=True).as_matrix()
        tb0=np.array([-.4,.02,.1]);t1l=np.array([-.01,-.02,.04])
        mounts=dict(T_body_imu0=dict(R=Rb0.tolist(),t_m=tb0.tolist()),
                    T_imu1_lidar=dict(R=R1l.tolist(),t_m=t1l.tolist()))
        out=cal.compose_mounts(self.truth,mounts)['transforms']['T_body_lidar']
        point=np.array([1.3,-.2,.7]);R01=np.array(self.truth['R']);t01=np.array(self.truth['t_m'])
        expected=Rb0@(R01@(R1l@point+t1l)+t01)+tb0
        np.testing.assert_allclose(np.array(out['R'])@point+out['t_m'],expected,atol=1e-12)

    def test_cli_g_units_and_existing_output(self):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder);raw=[a.copy() for a in self.pair]
            raw[1][:,1:4]/=9.80665
            np.savez(root/'pair.npz',external=raw[0],mid360=raw[1])
            command=[sys.executable,str(SCRIPT),'--input',str(root/'pair.npz'),
                     '--imu0','external','--imu1','mid360','--accel-scale1','9.80665',
                     '--method','joint_free',
                     '--output',str(root/'result.json')]
            result=subprocess.run(command,capture_output=True,text=True)
            self.assertEqual(result.returncode,0,result.stderr)
            saved=json.loads((root/'result.json').read_text())
            self.assertEqual(saved['status'],'fit_passed_needs_independent_validation')
            self.assertLess(cal.compare(self.truth,saved['estimate'])['translation_mm'],.1)
            before=(root/'result.json').read_bytes()
            repeated=subprocess.run(command,capture_output=True,text=True)
            self.assertEqual(repeated.returncode,2)
            self.assertEqual((root/'result.json').read_bytes(),before)

class StaticConstrainedCalibrationTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.pair,cls.truth=fixture(static=True)
        cls.fit=cal.calibrate(cls.pair)

    def test_default_recovers_parameters_and_retains_staged_baseline(self):
        self.assertEqual(self.fit['method'],'joint_static_soft')
        self.assertFalse(self.fit['quality_failures'])
        self.assertTrue(self.fit['diagnostics']['static_priors'])
        self.assertFalse(self.fit['diagnostics']['gravity_direction_input'])
        d=cal.compare(self.truth,self.fit['estimate'])
        self.assertLess(d['rotation_deg'],.01)
        self.assertLess(d['translation_mm'],.2)
        self.assertLess(abs(d['td_ms']),.03)
        self.assertLess(max(d['gyro_bias_difference_norm_rad_s']),.0001)
        self.assertIn('estimate',self.fit['staged_baseline'])
        np.testing.assert_allclose(self.fit['estimate']['accel_difference_bias_m_s2'],
                                   self.truth['accel_difference_bias_m_s2'],atol=.002)

    def test_missing_static_fails_without_silent_fallback(self):
        with self.assertRaisesRegex(ValueError,'No qualifying static window'):
            cal.calibrate(fixture()[0])

    def test_static_does_not_fix_unobservable_planar_geometry(self):
        with self.assertRaisesRegex(ValueError,'multi-axis'):
            cal.calibrate(fixture(static=True,planar=True)[0])

    def test_static_prior_resolves_coincident_gyro_bias_degeneracy(self):
        p,truth=fixture(static=True,coincident=True)
        result=cal.calibrate(p)
        self.assertFalse(result['quality_failures'])
        self.assertLess(cal.compare(truth,result['estimate'])['translation_mm'],.2)

    def test_staged_and_fixed_bias_options(self):
        for method in ['staged','joint_fixed_bg']:
            result=cal.calibrate(self.pair,{'method':method})
            self.assertFalse(result['quality_failures'])
            self.assertLess(cal.compare(self.truth,result['estimate'])['translation_mm'],.2)
            np.testing.assert_allclose(result['estimate']['gyro_bias0_rad_s'],self.truth['gyro_bias0_rad_s'],atol=1e-10)
        self.assertEqual(cal.calibrate(self.pair,{'method':'staged'})['diagnostics']['nfev'],0)

    def test_explicit_reference_zero_mode_documents_assumption(self):
        result=cal.calibrate(fixture()[0],{'method':'joint_reference_fixed'})
        self.assertTrue(result['assumptions']['reference_gyro_bias_fixed_zero'])
        self.assertTrue(result['diagnostics']['no_static_information'])
        np.testing.assert_array_equal(result['estimate']['gyro_bias0_rad_s'],[0,0,0])

    def test_independent_record_keeps_baseline_and_all_parameters_frozen(self):
        other,_=fixture(phase=1.7,noise=True,static=True)
        v=cal.validate_pair(self.pair,other,self.fit)
        self.assertFalse(v['failures'])
        self.assertFalse(v['staged_baseline']['automatic_method_selection'])
        self.assertTrue(v['frozen_forward']['all_parameters_frozen'])
        self.assertLess(v['frozen_forward']['accel_rmse_m_s2'],.01)
        np.testing.assert_allclose(v['staged_baseline']['frozen_forward']['accel_rmse_m_s2'],
            cal.predict(other,self.fit['staged_baseline']['estimate'])['accel_rmse_m_s2'])

    def test_cli_default_static_pipeline(self):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder);np.savez(root/'pair.npz',imu0=self.pair[0],imu1=self.pair[1])
            result=subprocess.run([sys.executable,str(SCRIPT),'--input',str(root/'pair.npz'),
                '--imu0','imu0','--imu1','imu1','--output',str(root/'fit.json')],capture_output=True,text=True)
            self.assertEqual(result.returncode,0,result.stderr)
            saved=json.loads((root/'fit.json').read_text())
            self.assertEqual(saved['method'],'joint_static_soft')
            self.assertTrue(saved['diagnostics']['static_priors'])

if __name__=='__main__':unittest.main()
