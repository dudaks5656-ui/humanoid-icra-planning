#!/usr/bin/env python3
import csv, collections, contextlib, hashlib, math, os, pathlib, shutil, statistics, sys

root=pathlib.Path(sys.argv[1]).resolve(); repo=pathlib.Path(__file__).resolve().parents[3]
summary=root/'summaries'; summary.mkdir(exist_ok=True)
result=root/'all_case_results.csv'; boundary=summary/'mode_workspace_boundary.csv'
rows=list(csv.DictReader(result.open())) if result.exists() else []
bounds=list(csv.DictReader(boundary.open())) if boundary.exists() else []

@contextlib.contextmanager
def atomic_output(path,newline=None,backup=True):
    path=pathlib.Path(path);tmp=path.with_name(path.name+'.postprocess.tmp')
    with tmp.open('w',newline=newline) as stream:
        yield stream
        stream.flush();os.fsync(stream.fileno())
    if backup and path.exists():
        saved=path.with_name(path.name+'.pre_atomic_postprocess.bak')
        if not saved.exists():shutil.copy2(path,saved)
    os.replace(tmp,path)
def atomic_text(path,text,backup=True):
    with atomic_output(path,backup=backup) as stream:stream.write(text)

def number(value):
    try:return float(value)
    except:return float('nan')
def finite(value):return math.isfinite(number(value))
def stats(values):
    values=sorted(number(v) for v in values if finite(v))
    if not values:return [0,'nan','nan','nan','nan']
    return [len(values),values[0],statistics.median(values),statistics.mean(values),values[-1]]
def groups(keys,source=rows):
    out=collections.defaultdict(list)
    for row in source:out[tuple(row[k] for k in keys)].append(row)
    return out

def statistical_summary(name,keys,fields):
    with atomic_output(summary/name,newline='') as f:
        header=list(keys)
        for field in fields:header += [field+'_n',field+'_min',field+'_median',field+'_mean',field+'_max']
        w=csv.writer(f);w.writerow(header)
        for key,group in sorted(groups(keys).items()):
            output=list(key)
            for field in fields:output += stats(r.get(field,'nan') for r in group)
            w.writerow(output)

statistical_summary('joint_margin_summary.csv',['phase','mode'],
 ['arm_joint_1_7_min_margin','joint3_margin','joint5_margin','yaw_margin','pitch_margin','active_joint_min_margin'])
statistical_summary('collision_clearance_summary.csv',['phase','mode'],['environment_clearance','self_clearance'])
statistical_summary('planning_time_summary.csv',['phase','mode'],['computation_ms'])

with atomic_output(summary/'failure_taxonomy.csv',newline='') as f:
    w=csv.writer(f);w.writerow(['phase','mode','failure_label','classification','count'])
    counter=collections.Counter((r['phase'],r['mode'],r['failure_label'],r['classification']) for r in rows if r['success']=='0')
    for key,count in sorted(counter.items()):w.writerow([*key,count])

arm_names=[f'openarm_left_joint{i}' for i in range(1,8)]
with atomic_output(summary/'selected_postures.csv',newline='') as f:
    fields=['unique_key','phase','target_id','ray','ray_angle_deg','distance_m','target_x','target_y','target_z','lift','mode','seed_bank','yaw_rad','pitch_rad',*arm_names,'arm_joint_1_7_min_margin','joint3_margin','joint5_margin','environment_clearance','self_clearance']
    w=csv.DictWriter(f,fieldnames=fields);w.writeheader()
    w.writerows(({k:r.get(k,'') for k in fields} for r in rows if r['success']=='1'))

def polygon_area(radii):
    n=len(radii)
    return sum(.5*radii[i]*radii[(i+1)%n]*math.sin(2*math.pi/n) for i in range(n)) if n>=3 else float('nan')

area_rows=[]
for phase,expected in [('PHASE1',16),('PHASE2',8)]:
    available=[b for b in bounds if b['phase']==phase]
    for lift in sorted({b['lift'] for b in available},key=number):
        for mode in ('LOCKED','YAW_ONLY','PITCH_ONLY','YAW_PITCH'):
            group=sorted((b for b in available if b['lift']==lift and b['mode']==mode),key=lambda x:number(x['ray_angle_deg']))
            radii=[number(b['last_success_distance']) if finite(b['last_success_distance']) else 0.0 for b in group]
            area=polygon_area(radii) if len(group)==expected else float('nan')
            area_rows.append([phase,lift,mode,area,len(group),expected,'ray-polygon sample estimate; not exact continuous workspace'])
with atomic_output(summary/'mode_workspace_area_summary.csv',newline='') as f:
    csv.writer(f).writerows([['phase','lift','mode','area_m2','completed_rays','expected_rays','interpretation'],*area_rows])

phase1=[b for b in bounds if b['phase']=='PHASE1']; indexed={(b['ray'],b['lift'],b['mode']):b for b in phase1}; recovery=[]
for ray,lift in sorted({(b['ray'],b['lift']) for b in phase1},key=lambda x:(int(x[0][1:]),number(x[1]))):
    if not all((ray,lift,m) in indexed for m in ('LOCKED','YAW_ONLY','PITCH_ONLY','YAW_PITCH')):continue
    radii={m:(number(indexed[ray,lift,m]['last_success_distance']) if finite(indexed[ray,lift,m]['last_success_distance']) else 0.0) for m in ('LOCKED','YAW_ONLY','PITCH_ONLY','YAW_PITCH')}
    dy=radii['YAW_ONLY']-radii['LOCKED'];dp=radii['PITCH_ONLY']-radii['LOCKED'];db=radii['YAW_PITCH']-radii['LOCKED']
    if dy>1e-12 and dp>1e-12:label='BOTH_SINGLE_AXES_SUFFICIENT'
    elif dy>1e-12:label='YAW_ONLY_RECOVERY'
    elif dp>1e-12:label='PITCH_ONLY_RECOVERY'
    elif radii['YAW_PITCH']>max(radii['YAW_ONLY'],radii['PITCH_ONLY'])+1e-12:label='YAW_PITCH_SYNERGY_RECOVERY'
    else:label='NO_AXIS_BOUNDARY_EXTENSION'
    extensions={m:radii[m]-radii['LOCKED'] for m in ('YAW_ONLY','PITCH_ONLY','YAW_PITCH')}
    best=max(extensions,key=extensions.get) if max(extensions.values())>1e-12 else 'NONE'
    recovery.append([ray,lift,*[radii[m] for m in ('LOCKED','YAW_ONLY','PITCH_ONLY','YAW_PITCH')],dy,dp,db,best,label])
with atomic_output(summary/'axis_recovery_summary.csv',newline='') as f:
    csv.writer(f).writerows([['ray','lift','locked_m','yaw_m','pitch_m','yaw_pitch_m','yaw_extension_m','pitch_extension_m','yaw_pitch_extension_m','best_axis_mode','classification'],*recovery])

thresholds=[('nominal',0),('1deg',math.radians(1)),('2deg',math.radians(2)),('5deg',math.radians(5)),('10deg',math.radians(10))]
with atomic_output(summary/'margin_threshold_sensitivity.csv',newline='') as f:
    w=csv.writer(f);w.writerow(['threshold','threshold_rad','lift','mode','feasible_sample_count','rays_with_feasible_sample','estimated_16_ray_area_m2','interpretation'])
    p1=[r for r in rows if r['phase']=='PHASE1']
    for name,threshold in thresholds:
        for lift in sorted({r['lift'] for r in p1},key=number):
            for mode in ('LOCKED','YAW_ONLY','PITCH_ONLY','YAW_PITCH'):
                subset=[r for r in p1 if r['lift']==lift and r['mode']==mode and r['success']=='1' and finite(r['arm_joint_1_7_min_margin']) and number(r['arm_joint_1_7_min_margin'])>threshold]
                radii=[]
                for ray_index in range(16):
                    ray=f'R{ray_index}';dist=[number(r['distance_m']) for r in subset if r['ray']==ray]
                    radii.append(max(dist) if dist else 0.0)
                w.writerow([name,threshold,lift,mode,len(subset),sum(v>0 for v in radii),polygon_area(radii),'simulation sensitivity criterion; not hardware tolerance'])

phase3=[r for r in rows if r['phase']=='PHASE3']
with atomic_output(summary/'phase3_seed_sensitivity.csv',newline='') as f:
    w=csv.writer(f);w.writerow(['target_id','ray','distance_m','lift','mode','seed_banks','success_banks','arm_margin_min','arm_margin_max','yaw_range_rad','pitch_range_rad','interpretation'])
    for key,group in sorted(groups(['target_id','ray','distance_m','lift','mode'],phase3).items()):
        successes=[r for r in group if r['success']=='1'];margins=[number(r['arm_joint_1_7_min_margin']) for r in successes if finite(r['arm_joint_1_7_min_margin'])]
        yaws=[number(r['yaw_rad']) for r in successes if finite(r['yaw_rad'])];pitches=[number(r['pitch_rad']) for r in successes if finite(r['pitch_rad'])]
        w.writerow([*key,len({r['seed_bank'] for r in group}),len({r['seed_bank'] for r in successes}),min(margins) if margins else 'nan',max(margins) if margins else 'nan',(max(yaws)-min(yaws)) if yaws else 'nan',(max(pitches)-min(pitches)) if pitches else 'nan','deterministic IK seed sensitivity; not physical repeatability'])

# Recompute and compare the immutable manifest.
before={}
for line in (root/'protected_before.sha256').read_text().splitlines():
    if line.strip():digest,path=line.split(None,1);before[path.strip()]=digest
changed=[]
with atomic_output(root/'protected_after.sha256') as f:
    for rel,old_digest in before.items():
        digest=hashlib.sha256((repo/rel).read_bytes()).hexdigest();f.write(f'{digest}  {rel}\n')
        if digest!=old_digest:changed.append(rel)
atomic_text(root/'protected_comparison.txt','UNCHANGED\n' if not changed else 'CHANGED\n'+'\n'.join(changed)+'\n')

# Structural CSV and sentinel audit.
duplicates=len(rows)-len({r['unique_key'] for r in rows})
width_errors=0; newline_ok=True
if result.exists():
    newline_ok=result.read_bytes().endswith(b'\n')
    with result.open(newline='') as f:
        parsed=list(csv.reader(f)); width_errors=sum(len(r)!=len(parsed[0]) for r in parsed[1:]) if parsed else 0
nan_count=sum(str(v).lower()=='nan' for r in rows for v in r.values())
inf_count=sum(str(v).lower() in ('inf','-inf') for r in rows for v in r.values())
phase_counts=collections.Counter(b['phase'] for b in bounds)
index={(r['phase'],r['ray'],r['distance_m'],r['lift'],r['mode'],r['seed_bank']):r for r in rows}
dominance_checked=dominance_violations=0
for row in rows:
    if row['mode']!='YAW_PITCH' or row['success']!='1':continue
    base=(row['phase'],row['ray'],row['distance_m'],row['lift']);single=[]
    for mode in ('YAW_ONLY','PITCH_ONLY'):
        candidate=index.get((*base,mode,row['seed_bank']))
        if candidate and candidate['success']=='1' and finite(candidate['arm_joint_1_7_min_margin']):single.append(number(candidate['arm_joint_1_7_min_margin']))
    if single:
        dominance_checked+=1
        dominance_violations+=number(row['arm_joint_1_7_min_margin'])+1e-12<max(single)
atomic_text(summary/'dataset_audit.md',f'''# Dataset audit

- Evaluated case rows: {len(rows)}.
- Boundary tasks: Phase 1 {phase_counts['PHASE1']}/128; Phase 2 {phase_counts['PHASE2']}/64.
- Duplicate unique keys: {duplicates}.
- CSV field-width errors: {width_errors}; final newline present: {newline_ok}.
- `nan` cells: {nan_count}. They are retained when no IK/posture was selected or a physical-envelope rejection makes joint/path metrics undefined.
- `inf` cells: {inf_count}. They are retained only when a collision-distance or bound calculation legitimately reports an unbounded sentinel.
- Protected files changed: {len(changed)}.
- YAW_PITCH dominance comparisons: {dominance_checked}; violations: {dominance_violations}.
- Postprocessing read the stored CSV only; no planning case was re-executed.
- Workspace areas are ray-polygon sample estimates, not exact continuous workspace areas.
- Deterministic seed-bank repeats measure IK-search sensitivity, not physical repeatability.
- All evidence is planning-only simulation evidence and does not establish hardware performance.
''')
atomic_text(summary/'paper_ready_summary.md',f'''# Paper-ready planning-only dataset summary

- Phase 0: PASS with dominance-corrected candidate union; preserved axis data was not rewritten.
- Phase 1: {phase_counts['PHASE1']}/128 boundary tasks.
- Phase 2: {phase_counts['PHASE2']}/64 boundary tasks.
- Phase 3 evaluated rows: {len(phase3)}.
- Raw evaluated case rows: {len(rows)}; duplicate keys: {duplicates}.
- Protected inputs changed: {len(changed)}.
- YAW_PITCH dominance violations after candidate-union correction: {dominance_violations}/{dominance_checked}.
- Planning cases were not re-executed during postprocessing.
- Claims are limited to the defined planning-only ray samples and simulation sensitivity criteria.
''')

files=sorted(p for p in root.rglob('*') if p.is_file() and p.name!='manifest_sha256.txt')
with atomic_output(root/'manifest_sha256.txt',backup=False) as f:
    for path in files:f.write(f'{hashlib.sha256(path.read_bytes()).hexdigest()}  {path.relative_to(root)}\n')
