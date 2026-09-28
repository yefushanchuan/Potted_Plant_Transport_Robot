#!/usr/bin/env python3
"""Dynamic-only rigid dual-IMU calibration; NumPy/SciPy, optional rosbags.

Convention: p0=R01*p1+t01, same physical event stamp0=stamp1+dt.
Inputs: timestamp seconds, specific force m/s² (gravity retained), gyro rad/s.
No static detector, static bias estimate, gravity direction or known extrinsic.
"""
import argparse
import hashlib
import json
from pathlib import Path
import sys
import numpy as np
from scipy.integrate import cumulative_trapezoid
from scipy.ndimage import uniform_filter1d, minimum_filter1d
from scipy.optimize import least_squares, minimize_scalar
from scipy.signal import savgol_filter
from scipy.spatial.transform import Rotation

CONFIG = dict(grid_s=.01, integration_s=.16, gyro_smoothing_s=.15,
              dt_bound_s=.1, max_gap_s=.04, activity_window_s=.4,
              activity_std_min_rad_s=.03, min_active_s=3.,
              gyro_excitation_min_rad_s=.05, gyro_excitation_ratio_min=.05,
              gyro_sigma=.005, accel_sigma=.05, max_nfev=200,
              max_gyro_rmse_rad_s=.03, max_accel_rmse_m_s2=.25,
              max_gyro_bias_norm_rad_s=.1, max_scaled_condition=1e5,
              repeat_rotation_max_deg=.5, repeat_translation_max_m=.015,
              repeat_dt_max_s=.003, repeat_gyro_bias_max_rad_s=.02)
SCALES=np.array([.01]*3+[.1]*3+[.01]+[.01]*6+[.1]*3)

def clean_pair(pair):
    arrays=[]
    for i,raw in enumerate(pair):
        a=np.asarray(raw,dtype=float).copy()
        if a.ndim!=2 or a.shape[1]!=7 or len(a)<200 or not np.isfinite(a).all():
            raise ValueError(f'IMU{i}: require >=200 finite rows [t,ax,ay,az,gx,gy,gz]')
        delta=np.diff(a[:,0])
        if np.any(delta<=0):raise ValueError(f'IMU{i}: duplicate/reversed header timestamps; repair acquisition, do not silently sort')
        if np.median(delta)>.025:raise ValueError(f'IMU{i}: sampling rate below 40 Hz')
        norm=float(np.median(np.linalg.norm(a[:,1:4],axis=1)))
        if not 3.<norm<30.:
            raise ValueError(f'IMU{i}: median acceleration norm {norm:.3g}; require raw specific force in m/s², gravity retained')
        arrays.append(a)
    origin=max(a[0,0] for a in arrays)
    for a in arrays:a[:,0]-=origin
    return arrays

def interp(ts,a):
    return np.column_stack([np.interp(ts,a[:,0],a[:,i]) for i in range(1,7)])

def smooth(a,seconds,step):
    return savgol_filter(a,max(5,int(round(seconds/step))|1),3,axis=0)

def skew(v):
    a=np.zeros((*v.shape[:-1],3,3))
    a[...,0,1]=-v[...,2];a[...,0,2]=v[...,1]
    a[...,1,0]=v[...,2];a[...,1,2]=-v[...,0]
    a[...,2,0]=-v[...,1];a[...,2,1]=v[...,0]
    return a

def valid_at(ts,a,gap):
    j=np.clip(np.searchsorted(a[:,0],ts),1,len(a)-1)
    return (a[j,0]-a[j-1,0]<gap)&(ts>=a[0,0])&(ts<=a[-1,0])

def make_grid(pair,cfg):
    step=cfg['grid_s'];margin=max(.5,cfg['dt_bound_s']+.25)
    ts=np.arange(margin,min(a[-1,0] for a in pair)-margin,step)
    if len(ts)<100:raise ValueError('Common recording too short after time-offset margins')
    good=valid_at(ts,pair[0],cfg['max_gap_s'])
    # Enforce a single support for all candidate time shifts.
    count=int(np.ceil(2*cfg['dt_bound_s']/(step/2)))+1
    for shift in np.linspace(-cfg['dt_bound_s'],cfg['dt_bound_s'],count):
        good &= valid_at(ts+shift,pair[1],cfg['max_gap_s'])
    w=interp(ts,pair[0])[:,3:]
    n=max(5,int(round(cfg['activity_window_s']/step))|1)
    mean=uniform_filter1d(w,size=n,axis=0,mode='nearest')
    variance=np.maximum(0,uniform_filter1d(w*w,size=n,axis=0,mode='nearest')-mean*mean)
    active=np.sqrt(variance.sum(1))>cfg['activity_std_min_rad_s']
    support=int(np.ceil((cfg['integration_s']/2+.03)/step))*2+1
    # Only varying-motion windows; not even bias/noise priors are taken from stops.
    use=minimum_filter1d((good&active).astype(int),size=support,mode='constant',cval=0).astype(bool)
    return ts,use

def align_gyro(pair,ts,use,cfg):
    g0=smooth(interp(ts,pair[0])[:,3:],cfg['gyro_smoothing_s'],cfg['grid_s'])
    def objective(td,details=False):
        g1=smooth(interp(ts-td,pair[1])[:,3:],cfg['gyro_smoothing_s'],cfg['grid_s'])
        a=g0[use];b=g1[use]
        u,_,vt=np.linalg.svd((b-b.mean(0)).T@(a-a.mean(0)))
        R=vt.T@np.diag([1,1,np.linalg.det(vt.T@u.T)])@u.T
        c=(a-b@R.T).mean(0)
        res=a-b@R.T-c
        return (R,c,res) if details else float(np.mean(res*res))
    grid=np.linspace(-cfg['dt_bound_s'],cfg['dt_bound_s'],81)
    costs=np.array([objective(t) for t in grid]);j=int(costs.argmin())
    if j in (0,len(grid)-1):raise ValueError('Gyro dt initialization hits search boundary')
    opt=minimize_scalar(objective,bounds=(grid[j-1],grid[j+1]),method='bounded',options={'xatol':1e-8})
    R,c,res=objective(opt.x,True)
    return R,float(opt.x),c,res

def features(pair,ts,R,td,bg0,cfg):
    step=cfg['grid_s'];h=int(round(cfg['integration_s']/(2*step)))
    s0=interp(ts,pair[0]);s1=interp(ts-td,pair[1])
    w=s0[:,3:]-bg0;q=skew(w)@skew(w)
    w=smooth(w,.05,step);q=smooth(q,.05,step)
    def avg(a):
        v=cumulative_trapezoid(smooth(a,.05,step),dx=step,axis=0,initial=0)
        return (v[2*h:]-v[:-2*h])/(2*h*step)
    # q already filtered; integrate directly, with the same one filter as acceleration.
    qi=cumulative_trapezoid(q,dx=step,axis=0,initial=0)
    A=skew((w[2*h:]-w[:-2*h])/(2*h*step))+(qi[2*h:]-qi[:-2*h])/(2*h*step)
    y=avg(s1[:,:3]@R.T)-avg(s0[:,:3])
    g0=smooth(s0[:,3:],cfg['gyro_smoothing_s'],step)[h:-h]
    g1=smooth(s1[:,3:],cfg['gyro_smoothing_s'],step)[h:-h]@R.T
    return A,y,g0-g1

def initialize(pair,ts,use,cfg):
    R,td,c,res=align_gyro(pair,ts,use,cfg)
    h=int(round(cfg['integration_s']/(2*cfg['grid_s'])))
    A,y,_=features(pair,ts,R,td,np.zeros(3),cfg);mask=use[h:-h]
    X=np.concatenate([A,np.broadcast_to(np.eye(3),A.shape)],axis=2)[mask].reshape(-1,6)
    target=y[mask].ravel();p=np.linalg.lstsq(X,target,rcond=None)[0]
    for _ in range(10):
        r=X@p-target;scale=max(1e-6,1.4826*np.median(np.abs(r-np.median(r))))
        weights=np.minimum(1,1.5*scale/np.maximum(np.abs(r),1e-12))
        p=np.linalg.lstsq(X*np.sqrt(weights[:,None]),target*np.sqrt(weights),rcond=None)[0]
    return R,np.r_[np.zeros(3),p[:3],td,c,np.zeros(3),p[3:]]

def pack(R,t,td,c,bg0,bdiff):
    # c=bg0-R*bg1. Both absolute gyro biases remain free in optimization.
    bg1=R.T@(bg0-c)
    return dict(R=R.tolist(),t_m=t.tolist(),td_s=float(td),gyro_bias0_rad_s=bg0.tolist(),
                gyro_bias1_rad_s=bg1.tolist(),accel_difference_bias_m_s2=bdiff.tolist(),
                quaternion_xyzw=Rotation.from_matrix(R).as_quat().tolist())

def calibrate(raw_pair,config=None):
    cfg={**CONFIG,**(config or {})};pair=clean_pair(raw_pair)
    ts,use=make_grid(pair,cfg)
    active_s=float(use.sum()*cfg['grid_s'])
    if active_s<cfg['min_active_s']:raise ValueError(f'Insufficient varying rotation: {active_s:.2f} s; need {cfg["min_active_s"]} s')
    w=smooth(interp(ts,pair[0])[:,3:],cfg['gyro_smoothing_s'],cfg['grid_s'])[use]
    sv=np.linalg.svd(w-w.mean(0),compute_uv=False)/np.sqrt(len(w))
    if sv[-1]<cfg['gyro_excitation_min_rad_s'] or sv[-1]/sv[0]<cfg['gyro_excitation_ratio_min']:
        raise ValueError(f'Insufficient multi-axis excitation: singular values {sv}; record nonparallel rotations')
    R0,center=initialize(pair,ts,use,cfg)
    h=int(round(cfg['integration_s']/(2*cfg['grid_s'])));mask=use[h:-h]
    def decode(z):
        x=center+z*SCALES
        return Rotation.from_rotvec(x[:3]).as_matrix()@R0,x[3:6],x[6],x[7:10],x[10:13],x[13:16]
    def blocks(z):
        R,t,td,c,bg0,bdiff=decode(z)
        A,y,g=features(pair,ts,R,td,bg0,cfg)
        return (y-A@t-bdiff)[mask],(g-c)[mask]
    def residual(z):
        a,g=blocks(z)
        return np.r_[a.ravel()/cfg['accel_sigma'],g.ravel()/cfg['gyro_sigma']]
    lower=np.full(16,-np.inf);upper=-lower
    lower[6]=(-cfg['dt_bound_s']-center[6])/SCALES[6]
    upper[6]=(cfg['dt_bound_s']-center[6])/SCALES[6]
    opt=least_squares(residual,np.zeros(16),bounds=(lower,upper),jac='3-point',
                      loss='soft_l1',max_nfev=cfg['max_nfev'],ftol=1e-9,xtol=1e-9,gtol=1e-8)
    est=pack(*decode(opt.x));a,g=blocks(opt.x)
    js=np.linalg.svd(opt.jac,compute_uv=False)
    condition=float(js[0]/max(js[-1],1e-30))
    metrics=dict(accel_rmse_m_s2=float(np.sqrt(np.mean(a*a))),gyro_rmse_rad_s=float(np.sqrt(np.mean(g*g))))
    failures=[]
    if not opt.success:failures.append('optimizer_not_converged')
    if abs(est['td_s'])>cfg['dt_bound_s']-.001:failures.append('time_offset_at_search_boundary')
    if condition>cfg['max_scaled_condition']:failures.append('weak_parameter_observability')
    for k in ['accel_rmse_m_s2','gyro_rmse_rad_s']:
        if metrics[k]>cfg['max_'+k]:failures.append('large_'+k)
    if max(np.linalg.norm(est[f'gyro_bias{i}_rad_s']) for i in (0,1))>cfg['max_gyro_bias_norm_rad_s']:
        failures.append('large_fitted_gyro_bias_check_model_and_recording')
    return dict(schema_version=1,status='fit_passed_needs_independent_validation' if not failures else 'rejected',
        quality_failures=failures,config=cfg,estimate=est,
        initialization=pack(*decode(np.zeros(16))),
        convention='p0=R01*p1+t01; same event timestamp0=timestamp1+dt; t is IMU1 origin expressed in IMU0',
        scope='Rigid relative IMU calibration only; no wheel-axle origin, individual accelerometer biases, scales, or misalignment intrinsics',
        diagnostics=dict(no_static_information=True,static_priors=False,gravity_direction_input=False,
            active_s=active_s,samples=int(mask.sum()),gyro_excitation_singular_values_rad_s=sv.tolist(),
            scaled_jacobian_singular_values=js.tolist(),scaled_condition=condition,
            optimizer_success=bool(opt.success),nfev=opt.nfev,termination=opt.message,
            optimality=float(opt.optimality),training=metrics))

def predict(raw_pair,est,config=None):
    cfg={**CONFIG,**(config or {})};pair=clean_pair(raw_pair);ts,use=make_grid(pair,cfg)
    h=int(round(cfg['integration_s']/(2*cfg['grid_s'])));mask=use[h:-h]
    if mask.sum()<100:raise ValueError('Not enough dynamic samples for validation')
    R=np.array(est['R']);t=np.array(est['t_m']);bg0=np.array(est['gyro_bias0_rad_s'])
    c=bg0-R@np.array(est['gyro_bias1_rad_s']);bd=np.array(est['accel_difference_bias_m_s2'])
    A,y,g=features(pair,ts,R,est['td_s'],bg0,cfg)
    r=(y-A@t-bd)[mask];g=(g-c)[mask];zero=(y-bd)[mask]
    return dict(samples=int(mask.sum()),accel_rmse_m_s2=float(np.sqrt(np.mean(r*r))),
                gyro_rmse_rad_s=float(np.sqrt(np.mean(g*g))),zero_arm_accel_rmse_m_s2=float(np.sqrt(np.mean(zero*zero))),
                per_axis_accel_rmse_m_s2=np.sqrt(np.mean(r*r,axis=0)).tolist(),all_parameters_frozen=True)

def compare(a,b):
    delta=np.array(b['t_m'])-a['t_m']
    return dict(rotation_deg=float(np.rad2deg(Rotation.from_matrix(np.array(b['R'])@np.array(a['R']).T).magnitude())),
                translation_mm=float(1000*np.linalg.norm(delta)),translation_xy_mm=float(1000*np.linalg.norm(delta[:2])),
                translation_components_mm=(delta*1000).tolist(),td_ms=float(1000*(b['td_s']-a['td_s'])),
                gyro_bias_difference_norm_rad_s=[float(np.linalg.norm(np.array(b[f'gyro_bias{i}_rad_s'])-a[f'gyro_bias{i}_rad_s'])) for i in (0,1)])

def validate_pair(train_pair,holdout_pair,train_fit,other=None):
    cfg=train_fit['config']
    if input_hash(train_pair)==input_hash(holdout_pair):raise ValueError('Holdout duplicates training measurements')
    other=calibrate(holdout_pair,cfg) if other is None else other
    diff=compare(train_fit['estimate'],other['estimate'])
    forward=predict(holdout_pair,train_fit['estimate'],cfg)
    reverse=predict(train_pair,other['estimate'],cfg)
    failures=[]
    if train_fit['quality_failures']:failures.append('training_fit_rejected')
    if other['quality_failures']:failures.append('holdout_independent_fit_rejected')
    for direction,m in [('forward',forward),('reverse',reverse)]:
        if m['accel_rmse_m_s2']>cfg['max_accel_rmse_m_s2'] or m['gyro_rmse_rad_s']>cfg['max_gyro_rmse_rad_s']:
            failures.append(direction+'_prediction_failed')
    if diff['rotation_deg']>cfg['repeat_rotation_max_deg']:failures.append('rotation_not_repeatable')
    if diff['translation_mm']/1000>cfg['repeat_translation_max_m']:failures.append('translation_not_repeatable')
    if abs(diff['td_ms'])/1000>cfg['repeat_dt_max_s']:failures.append('time_offset_not_repeatable_check_clock_session')
    if max(diff['gyro_bias_difference_norm_rad_s'])>cfg['repeat_gyro_bias_max_rad_s']:
        failures.append('individual_gyro_biases_not_repeatable')
    return dict(independent_fit=other,repeatability=diff,frozen_forward=forward,frozen_reverse=reverse,failures=failures)

def compose_mounts(est,mounts):
    """Optional measured transforms, with explicit destination/source frame names."""
    def check(key):
        m=mounts[key];R=np.asarray(m['R'],float);t=np.asarray(m['t_m'],float)
        if R.shape!=(3,3) or t.shape!=(3,) or not np.isfinite(R).all() or not np.isfinite(t).all():
            raise ValueError(f'{key}: expected finite R 3x3 and t_m length 3')
        if not np.allclose(R.T@R,np.eye(3),atol=1e-6) or abs(np.linalg.det(R)-1)>1e-6:
            raise ValueError(f'{key}: rotation must be proper orthonormal')
        return R,t
    def pack_transform(R,t):return dict(R=R.tolist(),t_m=t.tolist())
    R01=np.asarray(est['R']);t01=np.asarray(est['t_m'])
    out={}
    if 'T_body_imu0' in mounts:
        Rb0,tb0=check('T_body_imu0');out['T_body_imu1']=pack_transform(Rb0@R01,Rb0@t01+tb0)
    if 'T_imu1_lidar' in mounts:
        R1l,t1l=check('T_imu1_lidar');R0l=R01@R1l;t0l=R01@t1l+t01
        out['T_imu0_lidar']=pack_transform(R0l,t0l)
        if 'T_body_imu0' in mounts:out['T_body_lidar']=pack_transform(Rb0@R0l,Rb0@t0l+tb0)
    if not out:raise ValueError('Mount file must provide T_body_imu0 and/or T_imu1_lidar')
    return dict(transforms=out,convention='T_destination_source maps source coordinates into destination',
                note='Measured anchors only; wheel center and lidar-to-internal-IMU geometry are not estimated here. No lidar timestamp correction inferred.')

def load_pair(path,imu0,imu1):
    p=Path(path)
    if p.suffix=='.npz':
        with np.load(p,allow_pickle=False) as z:return [z[imu0].copy(),z[imu1].copy()]
    from rosbags.highlevel import AnyReader
    from rosbags.typesys import Stores,get_typestore
    rows={imu0:[],imu1:[]}
    with AnyReader([p],default_typestore=get_typestore(Stores.ROS2_HUMBLE)) as reader:
        connections=[c for c in reader.connections if c.topic in rows]
        if set(c.topic for c in connections)!=set(rows):raise ValueError('Requested IMU topics not both present')
        if any(c.msgtype!='sensor_msgs/msg/Imu' for c in connections):raise ValueError('Expected sensor_msgs/Imu topics')
        for c,_,raw in reader.messages(connections=connections):
            m=reader.deserialize(raw,c.msgtype)
            if m.angular_velocity_covariance[0]<0 or m.linear_acceleration_covariance[0]<0:
                raise ValueError('IMU marks angular velocity or acceleration unavailable')
            stamp=m.header.stamp.sec+m.header.stamp.nanosec*1e-9
            a,w=m.linear_acceleration,m.angular_velocity
            rows[c.topic].append([stamp,a.x,a.y,a.z,w.x,w.y,w.z])
    return [np.array(rows[t]) for t in (imu0,imu1)]

def input_hash(pair):
    return [hashlib.sha256(np.asarray(p,dtype='<f8').tobytes()).hexdigest() for p in pair]

def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--input',required=True,help='ROS1 bag, ROS2 bag directory, or NPZ')
    ap.add_argument('--imu0',required=True,help='Reference IMU topic or NPZ key')
    ap.add_argument('--imu1',required=True,help='Secondary IMU topic or NPZ key')
    ap.add_argument('--holdout',help='Independent recording, same hardware clock session; parameters remain frozen')
    ap.add_argument('--output',required=True,help='New JSON path; existing files are not overwritten')
    ap.add_argument('--dt-bound',type=float,default=.1,help='Symmetric time-offset search bound, seconds')
    for i in (0,1):
        ap.add_argument(f'--accel-scale{i}',type=float,default=1.,help='Multiply incoming acceleration to obtain m/s²; use 9.80665 for g')
        ap.add_argument(f'--gyro-scale{i}',type=float,default=1.,help='Multiply incoming gyro to obtain rad/s')
    ap.add_argument('--mounts',help='Optional JSON with measured T_body_imu0 and/or T_imu1_lidar')
    args=ap.parse_args()
    dest=Path(args.output)
    if dest.exists():ap.error('Output already exists; select another path')
    if args.imu0==args.imu1:ap.error('Two distinct IMU topics/keys required')
    if not .005<=args.dt_bound<=1:ap.error('--dt-bound must be between 0.005 and 1 seconds')
    cfg={**CONFIG,'dt_bound_s':args.dt_bound}
    def read_scaled(path):
        pair=load_pair(path,args.imu0,args.imu1)
        for i,p in enumerate(pair):
            ac=getattr(args,f'accel_scale{i}');gy=getattr(args,f'gyro_scale{i}')
            if not np.isfinite([ac,gy]).all() or min(ac,gy)<=0:raise ValueError('Unit scales must be finite and positive')
            p[:,1:4]*=ac;p[:,4:7]*=gy
        return pair
    pair=read_scaled(args.input)
    out=calibrate(pair,cfg)
    out['input']=dict(path=str(Path(args.input).resolve()),imu0=args.imu0,imu1=args.imu1,numerical_sha256=input_hash(pair))
    out['input']['unit_scales']={f'{kind}{i}':getattr(args,f'{kind}{i}') for kind in ['accel_scale','gyro_scale'] for i in (0,1)}
    out['script_sha256']=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    if args.holdout:
        hp=read_scaled(args.holdout)
        out['validation']=validate_pair(pair,hp,out)
        out['validation'].update(path=str(Path(args.holdout).resolve()),numerical_sha256=input_hash(hp))
        if not out['validation']['failures']:out['status']='passed_recording_checks_not_absolute_accuracy_certified'
        else:out['status']='rejected'
    if args.mounts:
        out['measured_mounts']=json.loads(Path(args.mounts).read_text())
        out['composed_chain']=compose_mounts(out['estimate'],out['measured_mounts'])
        out['composed_chain']['validated']=out['status']=='passed_recording_checks_not_absolute_accuracy_certified'
    dest.parent.mkdir(parents=True,exist_ok=True)
    with dest.open('x') as f:json.dump(out,f,indent=2,allow_nan=False);f.write('\n')
    print(json.dumps(dict(status=out['status'],quality_failures=out['quality_failures'],output=str(dest),estimate=out['estimate']),indent=2))
    return 2 if out['status']=='rejected' else 0

if __name__=='__main__':
    try:sys.exit(main())
    except (ValueError,KeyError,FileNotFoundError) as e:
        print(f'Calibration failed: {e}',file=sys.stderr);sys.exit(2)
