"""Plan/check both colors while stopped; --execute runs the checked Lua controller.

Uses scene object positions for this initial manipulation integration. Objects are
never attached or teleported during execution; grasps rely on physics contacts.
"""
import argparse
import json
import math
from pathlib import Path
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

HERE=Path(__file__).resolve().parent

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute',action='store_true')
    args=parser.parse_args()
    c=RemoteAPIClient(); s=c.require('sim'); ik=c.require('simIK')
    assert s.getSimulationState()==s.simulation_stopped,'Stop and reset the simulation first.'
    assert s.getIntArrayProperty(s.handle_scene,'dynamicsEngine')==[0,283],'This run requires the tested Bullet 2.83 engine.'
    base,tip,target=[s.getObject(p) for p in ['/Franka','/PandaIKTip','/PandaIKTarget']]
    joints=[s.getObject('/Franka/joint',{'index':i}) for i in range(7)]
    initial=[s.getJointPosition(j) for j in joints]
    balls={k:s.getObject('/'+k+'Ball') for k in ['Red','Blue']}
    bowls={k:s.getObject('/'+k+'Bowl') for k in balls}
    positions={k:s.getObjectPosition(h,-1) for k,h in balls.items()}
    destinations={k:s.getObjectPosition(h,-1) for k,h in bowls.items()}
    opening=s.getObject('/Franka/FrankaGripper/openCloseJoint')
    pad_positions=[s.getObjectPosition(s.getObject('/Franka/FrankaGripper/'+name),tip) for name in ['leftPad','rightPad']]
    assert math.sqrt(sum(((a+b)/2)**2 for a,b in zip(*pad_positions)))<.001,'TCP is not at the pad midpoint.'
    table=s.getObject('/customizableTable/top')
    surface=s.getObjectPosition(table,-1)[2]+s.getObjectFloatParam(table,s.objfloatparam_objbbox_max_z)
    assert abs(surface-.005)<.001,'Table height changed; replan grasp heights before moving.'
    audit=s.createScript(s.scripttype_customization,(HERE/'collision_audit.lua').read_text(),0,'lua')
    s.initScript(audit)
    env=ik.createEnvironment(); plan=[]; settled={}; previous=initial
    try:
        print('Collision audit shapes/pairs',s.callScriptFunction('initAudit',audit),flush=True)
        g=ik.createGroup(env); ik.setGroupCalculation(env,g,ik.method_damped_least_squares,.1,300)
        _,mapping,_=ik.addElementFromScene(env,g,base,tip,target,ik.constraint_pose)
        pose=s.getObjectPose(tip,-1)
        def waypoint(color,phase,xyz,carried=False):
            nonlocal previous
            pose[:3]=xyz
            ik.setObjectPose(env,mapping[target],-1,pose)
            result,flags,precision=ik.handleGroup(env,g)
            assert result==ik.result_success,(color,phase,xyz,'IK failed',flags,precision)
            q=[ik.getJointPosition(env,mapping[j]) for j in joints]
            safe,count=s.callScriptFunction('checkSegment',audit,previous,q,.051 if carried else .082,balls[color] if carried else -1,settled)
            assert safe,(color,phase,xyz,count)
            plan.append(dict(color=color,phase=phase,xyz=list(xyz),joints=q,carried=carried))
            previous=q
            print('CHECKED',color,phase,xyz,'samples',count,flush=True)
        for color in balls:
            x,y,_=positions[color]; dx,dy,dz=destinations[color]
            # All horizontal transport is above the bowl rims.
            if plan:
                last=plan[-1]['xyz']
                steps=max(1,math.ceil(math.dist(last[:2],[x,y])/.03))
                for n in range(1,steps+1):
                    waypoint(color,'approach',[last[0]+(x-last[0])*n/steps,last[1]+(y-last[1])*n/steps,.22])
            else: waypoint(color,'approach',[x,y,.125])
            for z in [.125,.100,.085,.075,.060,.045,.034]:
                if not (len(plan)==1 and z==.125):waypoint(color,'descent',[x,y,z])
            plan.append(dict(color=color,phase='grasp'))
            for z in [.045,.060,.075,.085,.100,.125,.18,.22]:waypoint(color,'lift',[x,y,z],True)
            count=max(1,math.ceil(math.dist([x,y],[dx,dy])/.03))
            for n in range(1,count+1):waypoint(color,'transfer',[x+(dx-x)*n/count,y+(dy-y)*n/count,.22],True)
            waypoint(color,'place',[dx,dy,.165],True)
            plan.append(dict(color=color,phase='release'))
            settled[color]=[dx,dy,dz+.0302]
            waypoint(color,'retreat',[dx,dy,.22])
        (HERE/'pick_place_plan.json').write_text(json.dumps(plan,indent=2))
    finally:
        ik.eraseEnvironment(env); s.removeObjects([audit])
        assert max(abs(s.getJointPosition(j)-q) for j,q in zip(joints,initial))<1e-8
        assert s.getSimulationState()==s.simulation_stopped
    print('Full route checked; stopped-scene joints restored.',flush=True)
    if not args.execute:return
    monitor=s.createScript(s.scripttype_customization,(HERE/'collision_audit.lua').read_text(),0,'lua')
    s.initScript(monitor)
    try:
        s.callScriptFunction('initAudit',monitor)
        execute(c,s,plan,balls,bowls,opening,tip,monitor)
    finally:s.removeObjects([monitor])

def execute(c,s,plan,balls,bowls,opening,tip,monitor):
    held=None; grip_offset=None; released=set(); log=[]; active_ball=-1
    c.setStepping(True)
    s.setFloatSignal('pandaGripperCommand',0)
    s.clearFloatSignal('pandaMoveRequest');s.clearFloatSignal('pandaMoveDone')
    s.startSimulation()
    def step():
        c.step()
        safe,collision=s.callScriptFunction('runtimeCheck',monitor,active_ball)
        if not safe:raise RuntimeError(('Unexpected runtime collision',collision))
        if held:
            p=s.getObjectPosition(balls[held],tip)
            if math.dist(p,grip_offset)>.012:raise RuntimeError(('Grip slipped',held,p,grip_offset))
    def record(row):
        data=dict(color=row['color'],phase=row['phase'],time=s.getSimulationTime(),
                  tcp=s.getObjectPosition(tip,-1),ball=s.getObjectPosition(balls[row['color']],-1),opening=s.getJointPosition(opening))
        log.append(data);print(json.dumps(data),flush=True)
    try:
        for _ in range(20):step()
        initial_positions={k:s.getObjectPosition(h,-1) for k,h in balls.items()}
        for row in plan:
            color=row['color'];phase=row['phase']
            active_ball=balls[color]
            if phase=='grasp':
                s.setJointTargetForce(opening,5,False)
                s.setFloatSignal('pandaGripperCommand',1)
                for _ in range(41):
                    step()
                    p=s.getObjectPosition(balls[color],-1)
                    if math.dist(p[:2],initial_positions[color][:2])>.01:raise RuntimeError('Ball pushed during closure')
                assert .045<s.getJointPosition(opening)<.060,'No sphere-sized grip'
                held=color;grip_offset=s.getObjectPosition(balls[color],tip)
            elif phase=='release':
                held=None
                s.setFloatSignal('pandaGripperCommand',0)
                for _ in range(60):step()
                p=s.getObjectPosition(balls[color],-1);b=s.getObjectPosition(bowls[color],-1)
                assert math.dist(p[:2],b[:2])<.07 and b[2]+.02<p[2]<b[2]+.10,('Placement failed',p,b)
                released.add(color)
            else:
                s.clearFloatSignal('pandaMoveDone')
                for i,q in enumerate(row['joints'],1):s.setFloatSignal('pandaTargetJoint'+str(i),q)
                s.setFloatSignal('pandaMoveRequest',1)
                for _ in range(800):
                    step()
                    if phase=='descent':
                        p=s.getObjectPosition(balls[color],-1)
                        if math.dist(p[:2],initial_positions[color][:2])>.004:raise RuntimeError('Ball pushed during descent')
                    done=s.getFloatSignal('pandaMoveDone')
                    if done==1:break
                    if done==-1:raise RuntimeError('Lua controller rejected command')
                else:raise RuntimeError('Motion timed out')
                assert math.dist(s.getObjectPosition(tip,-1),row['xyz'])<.015,'Excessive tracking error'
            record(row)
        assert released=={'Red','Blue'}
        for _ in range(60):step()
        final={}
        for color in released:
            p=s.getObjectPosition(balls[color],-1);b=s.getObjectPosition(bowls[color],-1)
            assert math.dist(p[:2],b[:2])<.07 and b[2]+.02<p[2]<b[2]+.10
            velocity,_=s.getObjectVelocity(balls[color])
            final[color]=dict(ball=p,bowl=b,speed=math.sqrt(sum(v*v for v in velocity)))
        (HERE/'pick_place_result.json').write_text(json.dumps(dict(success=True,simulation_time=s.getSimulationTime(),objects=final),indent=2))
        s.pauseSimulation()
        print('SUCCESS: both balls settled in their matching bowls. Simulation paused.',flush=True)
    except BaseException:
        s.stopSimulation()
        print('Stopped/reset after failed verification.',flush=True)
        raise
    finally:
        c.setStepping(False)
        (HERE/'pick_place_run.json').write_text(json.dumps(log,indent=2))

if __name__=='__main__':main()
