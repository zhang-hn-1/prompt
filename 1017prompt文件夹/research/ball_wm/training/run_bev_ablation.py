"""Run the approved 2026-09-07 paired experiments, preserving raw child logs."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time

from .ablation_protocol import budget_episode
from .navigation_metrics import summarize_navigation
from .evaluate_navigation_suite import context_tuples, scene_seed

ROOT = Path(__file__).resolve().parents[3]
WM = ROOT / 'research/ball_wm'
DATA = WM / 'data/trajectories_dubins_2p5d_20260828'
MANIFEST = WM / 'runs/navigation_eval_visibility_epoch01_strict_clearance02_full_20260902/context_manifest.json'
OLD = WM / 'runs/joint_wm_visibility_candidate_20260902/joint_wm_epoch01.ckpt'
SPLIT = WM / 'configs/ablation_scene_split.json'


def write(path, data):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, indent=2, allow_nan=False))


def child(command, log):
    """Never overwrite a previous child run or accept its output after failure."""
    log.parent.mkdir(parents=True, exist_ok=True)
    started = time.time()
    print('START', str(log), flush=True)
    with log.open('x') as stream:
        result = subprocess.run([str(v) for v in command], cwd=ROOT,
                                stdout=stream, stderr=subprocess.STDOUT, check=False)
    write(log.with_suffix('.process.json'), {'command': [str(v) for v in command],
                                            'return_code': result.returncode,
                                            'elapsed_seconds': time.time()-started})
    if result.returncode:
        raise RuntimeError(f'child failed ({result.returncode}): {log}')
    print('DONE', str(log), flush=True)


def navigation(checkpoint, directory, scenes, safety):
    results = []
    expected = json.loads(MANIFEST.read_text())
    for scene in scenes:
        index = int(scene.rsplit('_', 1)[1])
        output = directory / (scene + '.json')
        child([sys.executable, '-m', 'training.evaluate_navigation',
               '--checkpoint', checkpoint, '--data_root', DATA, '--out', output,
               '--scene', scene, '--num_eval', 0, '--goal_offset', 20,
               '--context_manifest', MANIFEST, '--min_initial_goal_distance', .5,
               '--min_initial_clearance', .2, '--require_initial_clearance', '--goal_tolerance', .2,
               '--max_mpc_steps', 200, '--horizon', 5, '--samples', 64,
               '--elites', 8, '--iterations', 3, '--safety_weight', safety,
               '--safety_threshold', .5, '--seed', scene_seed(3072,index),
               '--device', 'cuda', '--save_planner_diagnostics'], output.with_suffix('.log'))
        value = json.loads(output.read_text())
        if value['runtime_error_count']:
            raise RuntimeError('runtime errors in ' + str(output))
        results.append(value)
    episodes = [e for result in results for e in result['episodes']]
    expected_episodes = [e for e in expected['contexts']
                         if 'scene_' + Path(e['scene_path']).parent.name in scenes]
    if context_tuples({'episodes': episodes}) != context_tuples({'episodes': expected_episodes}):
        raise ValueError('navigation contexts do not match the frozen manifest')
    if len(context_tuples({'episodes':episodes})) != len(set(context_tuples({'episodes':episodes}))):
        raise ValueError('duplicate navigation contexts')
    summaries = {}
    for budget in (50,100,200):
        clipped = [budget_episode(e,budget,.2) for e in episodes]
        summary = summarize_navigation(clipped,.2,budget)
        summary['reference_spl'] = summary.pop('spl')
        summary['budget_seconds'] = budget / 10
        write(directory / f'budget_{budget}.json', {'summary':summary,'episodes':clipped})
        summaries[str(budget)] = summary
    write(directory / 'budget_comparison.json', {
        'paired_prefixes':True, 'checkpoint':str(checkpoint), 'safety_weight':safety,
        'context_manifest_sha256':hashlib.sha256(MANIFEST.read_bytes()).hexdigest(),
        'scenes':scenes, 'summaries':summaries})
    return results, summaries


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out_dir', type=Path, required=True)
    parser.add_argument('--phase', choices=('budget','train','eval','all'), default='all')
    args = parser.parse_args()
    out = args.out_dir.resolve()
    os.environ['PYTHONPATH'] = str(WM)
    split = json.loads(SPLIT.read_text())
    if args.phase in ('budget','all'):
        navigation(OLD,out/'old_budget',[f'scene_easy_{i}' for i in range(10)],.1)
    if args.phase in ('train','all'):
        for name in ('a_pose','b_spatial'):
            child([sys.executable,'-m','training.train_joint',
                   '--data_root',DATA,'--config',WM/f'configs/ablation_{name}.yaml',
                   '--scene_split_manifest',SPLIT,'--out_dir',out/name,
                   '--epochs',10,'--batch_size',8,'--lr',5e-5,'--seed',3072,
                   '--device','cuda'],out/(name+'.train.log'))
        a,b = [json.loads((out/name/'run_manifest.json').read_text()) for name in ('a_pose','b_spatial')]
        if a['initial_weights_sha256'] != b['initial_weights_sha256'] or a['scene_split'] != b['scene_split']:
            raise ValueError('unpaired initialization or split')
        write(out/'training_pair.json', {'initial_weights_sha256':a['initial_weights_sha256'],
                                         'scene_split':a['scene_split'],'epochs':10,'seed':3072,
                                         'checkpoint_selection':'fixed final epoch09'})
    if args.phase in ('eval','all'):
        policies = {}
        for name in ('a_pose','b_spatial'):
            checkpoint = out/name/'joint_wm_epoch09.ckpt'
            output = out/name/'future_prediction.json'
            child([sys.executable,'-m','training.evaluate_world_model',
                   '--checkpoint',checkpoint,'--data_root',DATA,'--out',output,
                   '--test_scene_fraction',.3,'--seed',3072,'--device','cuda'],
                  out/name/'future_prediction.log')
            prediction = json.loads(output.read_text())
            if sorted(prediction['held_out_scenes']) != sorted(split['test']):
                raise ValueError('offline evaluator test split mismatch')
            test_scene_names = ['scene_' + Path(path).parent.name for path in split['test']]
            policies[name], _ = navigation(checkpoint,out/name/'navigation',test_scene_names,0.)
        a,b = [context_tuples({'episodes':[e for r in policies[n] for e in r['episodes']]})
               for n in ('a_pose','b_spatial')]
        if a != b:
            raise ValueError('A/B navigation contexts differ')
        write(out/'completed.json', {'status':'complete','test_contexts':len(a),
                                     'completed_at_unix':time.time()})


if __name__ == '__main__':
    main()
