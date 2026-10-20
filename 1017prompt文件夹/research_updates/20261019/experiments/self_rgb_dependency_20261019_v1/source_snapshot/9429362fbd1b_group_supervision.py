"""Local bounded process-group supervisor. Does not change shared/old supervision."""
from pathlib import Path
import os,signal,subprocess,json,datetime,time
class Interrupted(Exception):
 def __init__(self,s):self.s=s

def members(pgid):
    live=[];zombies=[]
    for p in Path('/proc').iterdir():
        if not p.name.isdigit():continue
        try:
            fields=(p/'stat').read_text().rsplit(') ',1)[1].split()
            if int(fields[2])==pgid:(zombies if fields[0]=='Z' else live).append(int(p.name))
        except (OSError,ValueError,IndexError):pass
    return sorted(live),sorted(zombies)

def run_group(args,status_path,log_path,*,timeout,grace=15,env=None):
    status_path=Path(status_path);assert not status_path.exists();now=lambda:datetime.datetime.now(datetime.timezone.utc).isoformat()
    state=dict(status='starting',start_utc=now(),parent_pid=os.getpid(),child_pid=None,process_group_id=None,child_returncode=None,timeout=False,parent_signal=None,forced_kill=False,no_retry=True)
    def save():
        tmp=status_path.with_suffix('.tmp');tmp.write_text(json.dumps(state,indent=2)+'\n');os.replace(tmp,status_path)
    def interrupt(s,f):raise Interrupted(s)
    previous={s:signal.getsignal(s) for s in [signal.SIGTERM,signal.SIGINT]};p=None;save()
    try:
        for s in previous:signal.signal(s,interrupt)
        with Path(log_path).open('w') as log:
            p=subprocess.Popen(args,stdout=log,stderr=subprocess.STDOUT,start_new_session=True,env=env)
            state.update(status='running',child_pid=p.pid,process_group_id=p.pid);save()
            try:p.wait(timeout=timeout)
            except subprocess.TimeoutExpired:state.update(status='timeout',timeout=True)
            except Interrupted as e:state.update(status='interrupted',parent_signal=e.s)
            if state['status']=='running':state['status']='finished' if p.returncode==0 else 'runtime_failed'
    except Interrupted as e:state.update(status='interrupted',parent_signal=e.s)
    except Exception as e:state.update(status='runtime_failed',failure_type=type(e).__name__,failure_message=str(e))
    finally:
        for s in previous:signal.signal(s,signal.SIG_IGN)
        if p is not None:
            if p.poll() is not None:
                until=time.monotonic()+2
                while members(p.pid)[0] and time.monotonic()<until:time.sleep(.02)
            live,z=members(p.pid)
            if live:
                state['cleanup_live_pids_before_TERM']=live
                if state['status']=='finished':state['status']='runtime_failed';state['failure_message']='Unexpected live descendants after main returned'
                try:os.killpg(p.pid,signal.SIGTERM)
                except ProcessLookupError:pass
                until=time.monotonic()+grace
                while members(p.pid)[0] and time.monotonic()<until:
                    p.poll();time.sleep(.02)
                if members(p.pid)[0]:
                    state['forced_kill']=True
                    try:os.killpg(p.pid,signal.SIGKILL)
                    except ProcessLookupError:pass
                    until=time.monotonic()+5
                    while members(p.pid)[0] and time.monotonic()<until:p.poll();time.sleep(.02)
            try:p.wait(timeout=5)
            except subprocess.TimeoutExpired:state['status']='cleanup_failed'
            state.update(child_returncode=p.returncode,remaining_live_group_pids=members(p.pid)[0],remaining_zombie_group_pids=members(p.pid)[1])
            if state['remaining_live_group_pids']:state['status']='cleanup_failed'
        state['end_utc']=now();save()
        for s,h in previous.items():signal.signal(s,h)
    return state
