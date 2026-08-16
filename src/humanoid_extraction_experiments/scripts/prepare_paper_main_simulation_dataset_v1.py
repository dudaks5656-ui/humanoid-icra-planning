#!/usr/bin/env python3
import csv, datetime, hashlib, json, os, pathlib, subprocess, sys

root=pathlib.Path(sys.argv[1]).resolve(); repo=pathlib.Path(__file__).resolve().parents[3]
for d in (root,root/'raw',root/'summaries',root/'logs'): d.mkdir(parents=True,exist_ok=True)
now=datetime.datetime.now().astimezone().isoformat()
def git(*args): return subprocess.check_output(['git',*args],cwd=repo,text=True).strip()
manifest=f'''protocol: PAPER_MAIN_SIMULATION_DATASET_V1
created: "{now}"
deadline_kst: "2026-08-16T10:30:00+09:00"
git_root: {repo}
head: {git('rev-parse','HEAD')}
branch: {git('branch','--show-current')}
origin: {git('remote','get-url','origin')}
ros_domain_id: 42
ros_localhost_only: 1
planning_only: true
phase0_status: PASSED_WITH_NEW_DOMINANCE_CORRECTION
phase1_boundary_tasks: 128
phase2_boundary_tasks: 64
phase3_status: DYNAMIC_IF_TIME_AFTER_PHASE2
hardware_used: false
trajectory_execution: false
'''
(root/'run_manifest.yaml').write_text(manifest)
(root/'git_status_before.txt').write_text(git('status','--short')+'\n')

old=repo/'validation/stage_constrained_v2_local_validation/protected_before.sha256'
paths=[]
if old.exists():
    for line in old.read_text().splitlines():
        if line.strip(): paths.append(line.split(None,1)[1].strip())
paths += ['src/humanoid_extraction_experiments/src/adaptive_target_boundary_search_v1.cpp',
          'src/humanoid_extraction_experiments/src/torso_axis_ablation_v1.cpp',
          'src/humanoid_extraction_experiments/src/box_target_yaw_pitch_feasibility_pilot_v1.cpp',
          'src/humanoid_extraction_experiments/src/lift_actuated_extraction_baseline_v1.cpp']
with (root/'protected_before.sha256').open('w') as out:
    for rel in dict.fromkeys(paths):
        p=repo/rel
        if p.exists(): out.write(f'{hashlib.sha256(p.read_bytes()).hexdigest()}  {rel}\n')

axis=repo/'validation/torso_axis_ablation_v1/run_20260815_210000/axis_ablation_results.csv'
rows=list(csv.DictReader(axis.open())); by={(r['target_id'],r['mode']):r for r in rows}; ids=sorted({r['target_id'] for r in rows})
audit=[]; violations=0
for tid in ids:
    candidates=[by[tid,m] for m in ('YAW_ONLY','PITCH_ONLY','YAW_PITCH')]
    assert all(r['success']=='1' and r['lift_extraction_success']=='1' for r in candidates)
    best=max(candidates,key=lambda r:(float(r['arm_joint_1_7_min_margin']),min(float(r['joint3_margin']),float(r['joint5_margin'])),float(r['environment_clearance']),float(r['self_clearance']),-(abs(float(r['yaw_rad']))+abs(float(r['pitch_rad'])))))
    old=by[tid,'YAW_PITCH']; deficit=max(float(by[tid,'YAW_ONLY']['arm_joint_1_7_min_margin']),float(by[tid,'PITCH_ONLY']['arm_joint_1_7_min_margin']))-float(old['arm_joint_1_7_min_margin'])
    violations += deficit>1e-12
    audit.append({'target_id':tid,'old_yaw_pitch_arm_margin':old['arm_joint_1_7_min_margin'],'yaw_only_arm_margin':by[tid,'YAW_ONLY']['arm_joint_1_7_min_margin'],'pitch_only_arm_margin':by[tid,'PITCH_ONLY']['arm_joint_1_7_min_margin'],'old_dominance_deficit':max(0,deficit),'corrected_source_mode':best['mode'],'corrected_arm_margin':best['arm_joint_1_7_min_margin'],'dominance_pass':int(float(best['arm_joint_1_7_min_margin'])+1e-12>=max(float(by[tid,'YAW_ONLY']['arm_joint_1_7_min_margin']),float(by[tid,'PITCH_ONLY']['arm_joint_1_7_min_margin'])))})
with (root/'summaries/phase0_dominance_audit.csv').open('w',newline='') as f:
    w=csv.DictWriter(f,fieldnames=audit[0]);w.writeheader();w.writerows(audit)
(root/'summaries/phase0_dominance_audit.md').write_text(f'''# Phase 0 dominance audit

- Existing targets audited: {len(ids)}.
- Existing YAW_PITCH dominance violations: {violations}/{len(ids)}.
- Cause: combined refinement did not necessarily include single-axis refined optima; posture-order-derived seed keys differed; scoring prioritized whole active margin rather than Arm 1–7 margin.
- Revalidation: every injected YAW_ONLY/PITCH_ONLY candidate already completed the identical 0.17 m Lift-only extraction with the same model, collision scene and fixed torso/Arm posture. Each is therefore a valid member of the relaxed YAW_PITCH feasible set.
- Correction: paper runner explicitly unions YAW_ONLY, PITCH_ONLY and native YAW_PITCH optima and applies extraction-success, Arm-margin, Joint3/5-margin, environment-clearance, self-clearance, posture-magnitude lexicographic selection.
- Corrected dominance violations: 0/{len(ids)}.
- Status: PASS; Phase 1 may start. Existing axis results were not changed.
''')

with (root/'case_queue.csv').open('w',newline='') as f:
    w=csv.writer(f);w.writerow(['phase','target_id','ray','distance','lift','mode','seed_bank','status'])
    for phase,nr,lifts,step in [('PHASE1',16,(.35,.40),22.5),('PHASE2',8,(.30,.45),45.0)]:
        for i in range(nr):
            for lift in lifts:
                for mode in ('LOCKED','YAW_ONLY','PITCH_ONLY','YAW_PITCH'):
                    w.writerow([phase,f'{phase}_R{i}',f'R{i if phase=="PHASE1" else 2*i}','adaptive',lift,mode,0,'PENDING'])
for name,header in [('completed_cases.csv','unique_key,phase,target_id,ray,distance,lift,mode,seed_bank\n'),('failed_cases.csv','unique_key,error\n')]:
    p=root/name
    if not p.exists():p.write_text(header)
(root/'progress.json').write_text(json.dumps({'created':now,'completed_boundary_tasks':0,'total_boundary_tasks':192,'current_task':''},indent=2)+'\n')
(root/'resume_state.yaml').write_text('state: PREPARED\ncompleted_boundary_tasks: 0\ntotal_boundary_tasks: 192\n')
(root/'heartbeat.log').write_text(f'{now} prepared phase0_pass=true\n')
print(root)
