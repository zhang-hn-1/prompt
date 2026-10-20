import ast
from pathlib import Path

def test_scalar_query_identity_never_overwrites_truth_quaternion():
 text=Path(__file__).with_name('run_native_scalar_depth_shadow_v1.py').read_text();tree=ast.parse(text)
 scalar=[n for n in ast.walk(tree) if isinstance(n,ast.Assign) and isinstance(n.value,ast.Subscript) and isinstance(n.value.value,ast.Name) and n.value.value.id=='surface' and isinstance(n.value.slice,ast.Constant) and n.value.slice.value=='query_pixel_id']
 assert len(scalar)==1 and isinstance(scalar[0].targets[0],ast.Name) and scalar[0].targets[0].id=='scalar_query_ids'
 truth=[k for n in ast.walk(tree) if isinstance(n,ast.Call) for k in n.keywords if k.arg=='root_quaternion_world_wxyz']
 assert truth and any(isinstance(k.value,ast.Name) and k.value.id=='q' for k in truth)
 assert 'query_pixel_id=scalar_query_ids' in text
