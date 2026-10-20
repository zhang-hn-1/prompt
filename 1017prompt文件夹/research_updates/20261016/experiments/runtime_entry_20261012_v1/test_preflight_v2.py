import json
from preflight_v2 import build_parser

def test_default_uses_complete_primary_asset_identity():
 path=build_parser().parse_args([]).requirements
 assert path.name=='requirements_v3.json'
 req=json.loads(path.read_text())
 assert all(asset in req['frozen_identity_sha256'] for asset in req['primary_assets'])
