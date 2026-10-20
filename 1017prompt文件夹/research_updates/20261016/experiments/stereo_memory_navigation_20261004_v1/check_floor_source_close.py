from pathlib import Path
import json,hashlib,datetime
w=Path('/home/an/Documents/ChatGPT/球型机器人导航'); h=w/'experiments/stereo_memory_navigation_20261004_v1'; sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest(); checks=0
for n in ['floor_hypothesis_protocol.json','semantic_floor_protocol.json']:
 p=json.loads((h/n).read_text())
 for k,m in p.items():
  if k.endswith('_sha256') and isinstance(m,dict):
   for f,s in m.items(): assert sha(f)==s,(n,f);checks+=1
frames=[0,60,153,307]
for stem,expected in [('floor_hypothesis',set(frames)),('semantic_floor',{(i,e) for i in frames for e in ['left','right']})]:
 m=json.loads((h/(stem+'_predictions/manifest.json')).read_text());a=json.loads((h/(stem+'_audit.json')).read_text()); keys=lambda rows:[(r['frame'],r['eye']) if stem=='semantic_floor' else r['frame'] for r in rows]
 for rows in [m['rows'],a['rows']]:
  kk=keys(rows);assert len(kk)==len(expected) and len(set(kk))==len(expected) and set(kk)==expected;checks+=1
 assert not a['candidate_accepted'] and not a['whole_research_complete'];checks+=1
 for r in m['rows']:assert sha(r['prediction'])==r['sha256'] and r['ended_utc']<a['started_utc'];checks+=1
r=json.loads((h/'floor_source_review.json').read_text()); report=w/'reports/benchmark_floor_source_feasibility_20261006.md';assert hashlib.sha256(report.read_bytes()[:r['report_body_bytes']]).hexdigest()==r['report_body_sha256'];checks+=1
for name,p in [('research_mission.md',w/'research_mission.md'),('2026-10-04-stereo-memory-navigation.md',w/'docs/superpowers/plans/2026-10-04-stereo-memory-navigation.md'),('progress.md',h/'progress.md')]:
 archived=h/('floor_source_pre_close_'+name);assert p.read_bytes().startswith(archived.read_bytes());checks+=1
assert '\nstatus = "ACTIVE"' in Path('/home/an/.codex/automations/automation/automation.toml').read_text();checks+=1
owned={str(p):sha(p) for p in h.iterdir() if p.is_file() and (p.name.startswith(('floor_source','floor_hypothesis','semantic_floor','floor_overlap','diagnose_floor_overlap','run_floor_hypothesis','run_semantic_floor','audit_floor_hypothesis','audit_semantic_floor','test_floor_hypothesis','test_semantic_floor')))};owned[str(report)]=sha(report)
identity=dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),owned_sha256=owned,checks=checks,report_reviewed_prefix_sha256=r['report_body_sha256'],master_prefix_preserved=True,stage_complete=True,whole_research_complete=False,automation='ACTIVE',new_science_runs=0)
(h/'floor_source_final_identity.json').write_text(json.dumps(identity,indent=2)+'\n');(h/'floor_source_final_check.json').write_text(json.dumps(dict(utc=identity['utc'],passed=True,checks=checks,owned_files=len(owned),stage_complete=True,whole_research_complete=False,automation='ACTIVE',identity_sha256=sha(h/'floor_source_final_identity.json')),indent=2)+'\n'); print('FLOOR_SOURCE_FINAL_PASS',checks,len(owned))
