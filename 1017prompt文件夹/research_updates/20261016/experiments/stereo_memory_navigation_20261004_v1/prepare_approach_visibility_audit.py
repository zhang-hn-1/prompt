from pathlib import Path
import hashlib,json,datetime
H=Path(__file__).resolve().parent
src=H/'audit_short_obstacle_visibility.py';s=src.read_text();s=s.replace("cases=['short_obstacle_current_run','short_obstacle_history_run','fresh_rolling_prior_shadow_current_run']","cases=['fresh_rolling_prior_shadow_current_run','occluded_obstacle_memory_navigation_history_run']").replace('frames=[0,60,110]','frames=[60,100,140,180,220,260,295]').replace('short_obstacle_visibility','approach_visibility').replace('exact nine historical exposures','exact fourteen historical exposures')
(H/'audit_approach_visibility.py').write_text(s)
(H/'approach_visibility_derivation.json').write_text(json.dumps(dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),parent=str(src),parent_sha256=hashlib.sha256(src.read_bytes()).hexdigest(),changes='only completed-case/frame/output namespace, geometry unchanged',new_sha256=hashlib.sha256(s.encode()).hexdigest(),no_new_simulation=True,old_gates_unchanged=True),indent=2)+'\n')
