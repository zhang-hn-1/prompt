import os,sys,subprocess,time,json
from pathlib import Path
p=subprocess.Popen([sys.executable,'-c','import time;time.sleep(60)'])
Path(sys.argv[1]).write_text(json.dumps(dict(pid=os.getpid(),grandchild=p.pid,process_group_id=os.getpgrp()))+'\n')
time.sleep(60)
