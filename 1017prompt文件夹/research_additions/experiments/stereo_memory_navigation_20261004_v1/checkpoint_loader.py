"""Hashed official checkpoint, static globals audit and weights_only safe loader."""
import hashlib,importlib,json,pickletools,sys,zipfile
from pathlib import Path
HASH='af0658f289ec840b292645f8d5538978f06e8cabaa1fd31e84acc91af268e990'
CLASS_MODULES=('core.','omegaconf.','timm.','torch.nn.')
STANDARD={'__builtin__ dict','__builtin__ list','__builtin__ long','__builtin__ set','_codecs encode',
          'collections OrderedDict','collections defaultdict','numpy dtype','numpy.core.multiarray scalar',
          'torch CharStorage','torch FloatStorage','torch HalfStorage','torch LongStorage','torch float32',
          'torch._utils _rebuild_parameter','torch._utils _rebuild_tensor_v2','typing Any'}

def verify_checkpoint(path):
    if hashlib.sha256(Path(path).read_bytes()).hexdigest()!=HASH:raise ValueError('checkpoint hash mismatch')

def inspect_globals(path):
    with zipfile.ZipFile(path) as z:
        data=z.read(next(n for n in z.namelist() if n.endswith('/data.pkl')))
    names=sorted(set(arg for op,arg,_ in pickletools.genops(data) if op.name=='GLOBAL'))
    for name in names:
        if name not in STANDARD and not name.split(' ')[0].startswith(CLASS_MODULES):
            raise ValueError('unapproved executable global: '+name)
    return names

def load_model(repository):
    import torch,numpy as np
    repository=Path(repository).resolve();path=repository/'weights/23-36-37/model_best_bp2_serialize.pth'
    verify_checkpoint(path);names=inspect_globals(path)
    sys.path.insert(0,str(repository));importlib.import_module('core.foundation_stereo')
    allowed=[]
    for name in names:
        module,attr=name.split(' ')
        if module.startswith(CLASS_MODULES):
            obj=getattr(importlib.import_module(module),attr)
            if not isinstance(obj,type):raise ValueError('only reviewed classes allowed: '+name)
            allowed.append(obj)
    from collections import defaultdict
    from typing import Any
    allowed += [set,list,dict,int,defaultdict,Any,np.dtype,np.core.multiarray.scalar]
    allowed += [type(np.dtype(t)) for t in (np.float64,np.float32,np.int64,np.int32,np.bool_)]
    with torch.serialization.safe_globals(allowed):
        model=torch.load(path,map_location='cpu',weights_only=True)
    return model
