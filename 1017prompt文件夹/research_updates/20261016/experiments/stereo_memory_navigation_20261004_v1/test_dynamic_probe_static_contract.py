from pathlib import Path
from audit_static_globals import unbound_global_names

def test_symbol_audit_catches_undefined_capture_read():
 assert unbound_global_names('def capture():\n return depths\n')==['depths']
 assert unbound_global_names('depths={}\ndef capture():\n return depths\n')==[]

def test_probe_capture_all_module_globals_bound():
 p=Path(__file__).resolve().parent/'run_dynamic_mask_probe_v2.py'
 assert unbound_global_names(p.read_text())==[]
