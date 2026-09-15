#!/usr/bin/env python3
"""Synthetic jump ROS/parser contracts; no Gazebo or actuator plugin launched."""
import fcntl
import hashlib
import math
import os
from pathlib import Path
import random
import signal
import subprocess
import tempfile
import time
import uuid
from test_ros_contract import discover_executable, NAMES


def main():
    try:
        executable = discover_executable()
        import rclpy
        from rclpy.node import Node
        from sensor_msgs.msg import Imu, JointState
        from geometry_msgs.msg import TransformStamped
        from std_msgs.msg import Bool, String, Float64MultiArray
        from tf2_msgs.msg import TFMessage
        from ros_gz_interfaces.msg import Contacts, Contact
        from rosgraph_msgs.msg import Clock
    except (ImportError, RuntimeError) as exc:
        print(f"UNAVAILABLE: {exc}"); return 2
    domains = list(set(range(120, 200))-{int(os.environ.get("ROS_DOMAIN_ID", "0"))})
    random.SystemRandom().shuffle(domains)
    lock = None
    for domain in domains:
        candidate = open(Path(tempfile.gettempdir())/f"tap-qa-domain-{domain}.lock", "a")
        try: fcntl.flock(candidate, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            candidate.close(); continue
        os.environ.update(ROS_DOMAIN_ID=str(domain), ROS_LOCALHOST_ONLY="1")
        rclpy.init(); probe = Node("jump_probe_"+uuid.uuid4().hex[:8])
        until = time.monotonic()+1
        while time.monotonic()<until: rclpy.spin_once(probe, timeout_sec=.05)
        occupied = any(n!=probe.get_name() for n, _ in probe.get_node_names_and_namespaces())
        probe.destroy_node(); rclpy.shutdown()
        if not occupied: lock=candidate; break
        candidate.close()
    if lock is None: print("UNAVAILABLE: no isolated DDS domain"); return 2
    process = node = None
    failures = []; count = 0
    namespace = "/tap_jump_contract_"+uuid.uuid4().hex[:10]
    with tempfile.TemporaryFile(mode="w+") as log:
        try:
            rclpy.init(); node=Node("driver", namespace=namespace)
            pubs={t:node.create_publisher(k,t,10) for t,k in (
                ("imu",Imu),("joint_states",JointState),("model_pose",TFMessage),
                ("left_wheel_contacts",Contacts),("right_wheel_contacts",Contacts),
                ("clock",Clock),
                ("target_pose",Float64MultiArray),
                ("enable",Bool),("emergency_stop",Bool),("reset_controller",Bool),("jump_request",Bool))}
            jump={}; mode={}; outputs={}
            def status(target,msg):
                fields=msg.data.split('|'); target.clear()
                target.update(mode=fields[0],**dict(f.split('=',1) for f in fields[1:] if '=' in f))
            def output(key,msg):
                values=msg.velocity if key=='legref' else ([] if key=='gate' else msg.data)
                if not all(math.isfinite(v) for v in values):
                    raise RuntimeError(f"nonfinite {key} output")
                outputs[key]=(msg,time.monotonic())
            subscriptions=[node.create_subscription(String,'jump_status',lambda m:status(jump,m),10),
                           node.create_subscription(String,'control_mode',lambda m:status(mode,m),10)]
            for topic,kind,key in (("actuator_enable",Bool,"gate"),
                    ("leg_velocity_reference",JointState,"legref"),
                    ("wheel_effort_controller/commands",Float64MultiArray,"wheel"),
                    ("leg_effort_controller/commands",Float64MultiArray,"leg")):
                subscriptions.append(node.create_subscription(kind,topic,lambda m,key=key:output(key,m),10))
            args=[str(executable),'--ros-args','-r',f'__ns:={namespace}']
            for p in ('use_sim_time:=false','auto_enable:=false','jump.enabled:=true',
                      'jump.crouch_duration_s:=2.0','whole_body.enabled:=false','pitch_source:=sim_pose',
                      f'sim_pose_topic:={namespace}/model_pose','sensor_timeout_s:=0.15',
                      'jump.contact_timeout_s:=0.08','mpc.nb_timesteps:=20',
                      'jump.contact_debounce_s:=0.05','station_keeper.enabled:=true',
                      'pose_hip_max_rad:=1.05','pose_knee_min_rad:=-1.6','pose_knee_max_rad:=0.2'):
                args+=['-p',p]
            process=subprocess.Popen(args,stdout=log,stderr=subprocess.STDOUT,env=os.environ.copy(),start_new_session=True)
            print(f'Executable SHA256={hashlib.sha256(executable.read_bytes()).hexdigest()} domain={domain}',flush=True)
            packet='valid'; request=False; neutral=True; frozen_stamp=None; cad_zero=True
            sim_ns=None; advance_clock=True; orientation_stamp_ns=None; world_x=0.0
            def pump(duration,action=None):
                nonlocal frozen_stamp,sim_ns
                until=time.monotonic()+duration
                last_wall=time.monotonic()
                while time.monotonic()<until:
                    if process.poll() is not None: raise RuntimeError(f'controller exit {process.returncode}')
                    stamp=node.get_clock().now().to_msg()
                    if sim_ns is not None:
                        wall_now=time.monotonic()
                        if advance_clock: sim_ns+=int((wall_now-last_wall)*1000000000)
                        last_wall=wall_now
                        stamp.sec,stamp.nanosec=divmod(sim_ns,1000000000)
                        pubs['clock'].publish(Clock(clock=stamp))
                    imu=Imu();imu.header.stamp=stamp;imu.orientation.w=1.;imu.linear_acceleration.z=9.81
                    pubs['imu'].publish(imu)
                    joints=JointState();joints.header.stamp=stamp;joints.name=NAMES
                    joints.position=[.115483096,0.,0.,.115483096,0.,0.] if neutral else [.5,-.7,0.,.5,-.7,0.]
                    if cad_zero:joints.position=[0.]*6
                    joints.velocity=[0.]*6;pubs['joint_states'].publish(joints)
                    pose=TransformStamped();pose.header.stamp=stamp;pose.child_frame_id='tap_robot'
                    if orientation_stamp_ns is not None:
                        seconds,nanoseconds=divmod(orientation_stamp_ns,1000000000)
                        pose.header.stamp=type(stamp)(sec=seconds,nanosec=nanoseconds)
                    pose.transform.translation.z=.237;pose.transform.rotation.w=1.
                    pose.transform.translation.x=world_x
                    pubs['model_pose'].publish(TFMessage(transforms=[pose]))
                    for side in ('left','right'):
                        msg=Contacts();msg.header.stamp.sec=stamp.sec;msg.header.stamp.nanosec=stamp.nanosec
                        if side=='left':
                            if packet in ('repeat','backward'):
                                if frozen_stamp is None: frozen_stamp=(stamp.sec,stamp.nanosec)
                                msg.header.stamp.sec,msg.header.stamp.nanosec=frozen_stamp
                                if packet=='backward': msg.header.stamp.sec-=1
                            elif packet in ('future','near_future'):
                                offset=200000000 if packet=='future' else 5000000
                                ns=stamp.sec*1000000000+stamp.nanosec+offset
                                msg.header.stamp.sec,msg.header.stamp.nanosec=divmod(ns,1000000000)
                            elif packet=='zero': msg.header.stamp.sec=0;msg.header.stamp.nanosec=0
                        if packet!='empty':
                            c=Contact();which=side
                            if side=='left' and packet=='misroute': which='right'
                            c.collision1.name=f'tap_robot::cad_{which}_wheel_link::cad_{which}_wheel_link_collision'
                            c.collision2.name='ground_plane::link::collision'
                            if side=='left' and packet=='wrong_body': c.collision1.name='tap_robot::cad_body_collision'
                            if side=='left' and packet=='wrong_ground': c.collision2.name='tap_robot::cad_body_collision'
                            msg.contacts=[c]
                        pubs[f'{side}_wheel_contacts'].publish(msg)
                    pubs['jump_request'].publish(Bool(data=request))
                    if action: action()
                    rclpy.spin_once(node,timeout_sec=.005)
            def check(label,ok):
                nonlocal count
                count+=1;print(('PASS ' if ok else 'FAIL ')+label,flush=True)
                if not ok: failures.append((label,dict(jump),dict(mode)))
            def stopped():
                return (all(k in outputs and time.monotonic()-outputs[k][1]<.15 for k in ('gate','wheel','leg','legref'))
                        and not outputs['gate'][0].data and len(outputs['wheel'][0].data)==2
                        and len(outputs['leg'][0].data)==4
                        and all(v==0 for k in ('wheel','leg') for v in outputs[k][0].data)
                        and len(outputs['legref'][0].velocity)==4 and all(v==0 for v in outputs['legref'][0].velocity))
            def enable():
                pump(.15,lambda:pubs['enable'].publish(Bool(data=True)));pump(.15)
            def reset_fault():
                pump(.05,lambda:pubs['reset_controller'].publish(Bool(data=True)))
                pump(.05,lambda:pubs['reset_controller'].publish(Bool(data=False)))
                pump(.1)
            def terminate_owned():
                if process is not None and process.poll() is None:
                    os.killpg(process.pid,signal.SIGINT)
                    try:process.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        os.killpg(process.pid,signal.SIGKILL);process.wait(timeout=5)
            pump(1.0)
            check('disabled startup gate false',stopped() and jump['ready']=='0')
            pump(.15,lambda:pubs['enable'].publish(Bool(data=False)))
            def configured_neutral_target():
                target=[float(v) for v in mode.get('pose_tgt','nan,nan').split(',')]
                return len(target)==2 and abs(target[0]-.115483096)<.006 and abs(target[1])<.001
            check('initial disable at CAD zero retains configured neutral target',
                  stopped() and configured_neutral_target())
            enable()
            check('first enable commands configured neutral without a pose request',
                  configured_neutral_target() and outputs['gate'][0].data and
                  outputs['leg'][0].data[0]>0 and outputs['leg'][0].data[2]>0)
            cad_zero=False;pump(.2)
            enable();check('fresh grounded neutral advertises ready',jump['ready']=='1')
            def pose_request(values):pubs['target_pose'].publish(Float64MultiArray(data=values))
            pump(.05,lambda:pose_request([.5,-.7,.5,-.7]))
            check('pending pose prevents jump readiness despite neutral encoders',
                  jump.get('pose_pending')=='1' and jump['ready']=='0')
            request=True;pump(.05)
            check('pending pose rejects jump request',jump['active']=='0')
            request=False;reset_fault();enable()
            check('reset discards queued pose and permits fresh neutral readiness',
                  jump.get('pose_pending')=='0' and jump['ready']=='1')
            check('outside jump leg references zero', 'legref' in outputs and
                  len(outputs['legref'][0].velocity)==4 and all(v==0 for v in outputs['legref'][0].velocity))
            neutral=False;pump(.1)
            check('nonneutral core readiness false',jump['ready']=='0')
            request=True;pump(.05)
            check('nonneutral request cannot arm',jump['active']=='0')
            request=False;neutral=True;pump(.15)
            packet='empty';pump(.12)
            check('empty heartbeat is fresh airborne not ready',jump['contact_fresh']=='1' and
                  jump['left_contact']=='0' and jump['right_contact']=='0' and jump['ready']=='0')
            packet='valid';pump(.2)
            packet='near_future';pump(.15)
            check('small contact-clock skew stays fresh and ready',jump['ready']=='1' and jump['contact_fresh']=='1')
            packet='valid';pump(.15)
            for case in ('wrong_body','wrong_ground','misroute','future','zero','repeat','backward'):
                packet=case;frozen_stamp=None;pump(.12)
                check(case+' invalidates stream and ready',jump['contact_fresh']=='0' and jump['ready']=='0' and jump['active']=='0')
                packet='valid';pump(.2)
                check(case+' permits fresh monotonic recovery',jump['ready']=='1')
            request=True;pump(.05)
            check('rising request starts crouch',jump['phase']=='Crouch' and jump['ready']=='0')
            pump(.025,lambda:pose_request([.5,-.7,.5,-.7]))
            check('active jump rejects external pose target',jump['phase']=='Crouch' and jump.get('pose_pending')=='0')
            world_x=.04;packet='empty';pump(.015)
            check('short airborne crouch gap has zero station correction',
                  jump['phase']=='Crouch' and jump['contact_fresh']=='1' and
                  mode.get('station_v')=='0.0000')
            packet='valid';pump(.03)
            check('crouch contact recovery preserves original world anchor',
                  jump['phase']=='Crouch' and abs(float(mode.get('anchor_ex','nan'))+.04)<.002 and
                  float(mode.get('station_v','0'))<0)
            world_x=0.;pump(.025)
            ref=outputs['legref'][0]
            check('active jump publishes stamped bounded named leg rates',
                  ref.name==[NAMES[i] for i in (0,1,3,4)] and ref.header.stamp.sec>0 and
                  len(ref.velocity)==4 and any(abs(v)>1e-4 for v in ref.velocity) and
                  all(abs(v)<=12.5665 for v in ref.velocity))
            packet='repeat';frozen_stamp=None;pump(.025)
            check('short duplicate burst retains grounded sample during jump',jump['phase']=='Crouch' and jump['contact_fresh']=='1')
            packet='valid';pump(.05)
            packet='future';pump(.025)
            check('short far-future burst ignored without false flight or abort',jump['phase']=='Crouch' and
                  jump['contact_fresh']=='1' and jump['airborne_seen']=='0')
            packet='valid';pump(.05)
            packet='wrong_body';pump(.08)
            check('wrong pair during jump faults not fabricated flight',jump['phase']=='Fault' and jump['airborne_seen']=='0' and stopped())
            packet='valid';pump(.2)
            check('held request plus recovered contacts cannot repeat',jump['phase']=='Fault' and stopped())
            pump(.05,lambda:pose_request([.5,-.7,.5,-.7]))
            check('fault rejects external pose target and clears queued pose',
                  jump['phase']=='Fault' and jump.get('pose_pending')=='0' and stopped())
            request=False;enable()
            check('enable cannot clear jump fault',jump['phase']=='Fault' and jump['ready']=='0' and stopped())
            reset_fault()
            check('explicit reset clears jump fault but leaves motors off',jump['phase']!='Fault' and stopped())
            enable();check('enable after explicit reset restores readiness',jump['ready']=='1')
            request=True;pump(.05)
            pump(.1,lambda:pubs['enable'].publish(Bool(data=False)))
            check('disable aborts jump and actuator gate',jump['phase']=='Fault' and stopped())
            request=False;reset_fault();enable();request=True;pump(.05)
            pump(.1,lambda:pubs['emergency_stop'].publish(Bool(data=True)))
            check('physical estop aborts jump and actuator gate',jump['phase']=='Fault' and stopped())
            pump(.12,lambda:pubs['emergency_stop'].publish(Bool(data=False)))
            check('clearing estop cannot auto-repeat',jump['phase']=='Fault' and stopped())
            terminate_owned()
            drain_until=time.monotonic()+.3
            while time.monotonic()<drain_until:rclpy.spin_once(node,timeout_sec=.01)
            # Restart only our node on simulated time; repeated clock packets
            # must not supply synthetic elapsed time to the jump sequencer.
            jump.clear();mode.clear();outputs.clear()
            request=False;packet='valid';sim_ns=10000000000
            sim_args=[('use_sim_time:=true' if a=='use_sim_time:=false' else a) for a in args]
            sim_args+=['-r','__node:=jump_sim_clock','-r',f'/clock:={namespace}/clock']
            process=subprocess.Popen(sim_args,stdout=log,stderr=subprocess.STDOUT,
                                     env=os.environ.copy(),start_new_session=True)
            discovery_deadline=time.monotonic()+10
            while not jump and time.monotonic()<discovery_deadline:pump(.1)
            pump(.3);enable();request=True;pump(.05);request=False
            check('sim-clock setup arms crouch',jump.get('phase')=='Crouch')
            advance_clock=False;pump(.1)
            frozen_phase=jump.get('phase');frozen_elapsed=jump.get('phase_elapsed')
            frozen_pose=mode.get('pose_tgt');frozen_stale=mode.get('stale_n')
            frozen_reference=outputs['legref'][0]
            pump(.25)
            check('paused repeated clock cannot advance jump',frozen_phase=='Crouch' and
                  jump.get('phase')==frozen_phase and frozen_elapsed is not None and
                  jump.get('phase_elapsed')==frozen_elapsed)
            check('paused clock leaves pose target and stale grace unchanged',
                  frozen_pose is not None and frozen_stale is not None and
                  mode.get('pose_tgt')==frozen_pose and mode.get('stale_n')==frozen_stale)
            check('paused clock publishes no advanced leg trajectory sample',
                  outputs['legref'][0].header.stamp==frozen_reference.header.stamp and
                  outputs['legref'][0].velocity==frozen_reference.velocity)
            pump(.05,lambda:pubs['emergency_stop'].publish(Bool(data=True)))
            advance_clock=True;pump(.05)
            check('estop received while paused faults on resumed control tick',
                  jump.get('phase')=='Fault' and stopped())
            terminate_owned()
            for orientation_ns,expected_fresh in ((4495000000,'1'),(4495010000,'0')):
                # Preload the exact future sample at 4.484, then step to 4.494.
                # A new node avoids clock-rewind/reset semantics obscuring the boundary.
                sim_ns=4484000000;advance_clock=False;orientation_stamp_ns=orientation_ns
                request=False;packet='valid';jump.clear();mode.clear();outputs.clear()
                boundary_name=f'jump_stamp_{orientation_ns}'
                boundary_args=[f'__node:={boundary_name}' if a=='__node:=jump_sim_clock' else a for a in sim_args]
                process=subprocess.Popen(boundary_args,stdout=log,stderr=subprocess.STDOUT,
                                         env=os.environ.copy(),start_new_session=True)
                discovery_deadline=time.monotonic()+10
                while time.monotonic()<discovery_deadline:
                    if any(info.node_name==boundary_name for info in
                           node.get_subscriptions_info_by_topic(namespace+'/imu')):break
                    pump(.1)
                pump(.5)
                mode.clear();sim_ns=4494000000;pump(.2)
                check(f'now 4.494 orientation {orientation_ns/1e9:.5f} freshness {expected_fresh}',
                      mode.get('fresh')==expected_fresh and mode.get('ori_age_ms')=='-1.0')
                terminate_owned()
                drain_until=time.monotonic()+.3
                while time.monotonic()<drain_until:rclpy.spin_once(node,timeout_sec=.01)
            orientation_stamp_ns=None
            for parameter in ('jump.leg_torque_limit_nm:=4.0','jump.wheel_torque_limit_nm:=0.12',
                              'jump.optimized_plan:=true', 'jump.plan_samples:=[0.0]',
                              'jump.plan_durations_s:=[0.2, 0.3, 0.4]'):
                with tempfile.TemporaryFile(mode='w+') as rejection_log:
                    process=subprocess.Popen(args+['-p',parameter],stdout=rejection_log,
                                             stderr=subprocess.STDOUT,env=os.environ.copy(),start_new_session=True)
                    try:process.wait(timeout=5)
                    except subprocess.TimeoutExpired:pass
                    rejected=process.poll() is not None and process.returncode!=0
                    check('jump constructor rejects unsupported '+parameter,rejected)
                    terminate_owned()
            process=None
            print(f'{count-len(failures)}/{count} jump ROS contracts passed')
            for row in failures:print(row)
            if failures:log.seek(0);print(log.read()[-6000:])
            return 1 if failures else 0
        except (RuntimeError,KeyError) as exc:
            print(f'FAIL harness: {exc}');return 1
        finally:
            if process is not None and process.poll() is None:
                os.killpg(process.pid,signal.SIGINT)
                try:process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid,signal.SIGKILL);process.wait(timeout=5)
            if process is not None and process.returncode not in (0,-signal.SIGINT):
                log.seek(0);print(log.read()[-3000:])
            if node is not None:node.destroy_node()
            if rclpy.ok():rclpy.shutdown()
            lock.close()


if __name__=='__main__':raise SystemExit(main())
