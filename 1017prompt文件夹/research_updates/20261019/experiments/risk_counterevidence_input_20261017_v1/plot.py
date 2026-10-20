from common import *
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
r=load(D/'result.json');a=load(D/'author_audit.json');c=load(D/'cause_partition.json')
fig,axs=plt.subplots(2,2,figsize=(12,8));ts=np.array([x['time_s'] for x in r['rows']])
ax=axs[0,0];ax.plot(ts,[x['historical_false'] for x in r['rows']],color='red',label='Teacher false historical risks');ax.plot(ts,[x['all_anchor_dispute_false'] for x in r['rows']],color='blue',label='Strict advisory disputes (correct)');ax.set(xlabel='Source capture time (s)',ylabel='Cell queries',title='All180 active source exposures');ax.legend(fontsize=8)
ax=axs[0,1];keys=list(c['false_historical_partition']);values=list(c['false_historical_partition'].values());labs=['Strict dispute','Some far, incomplete','Compatible pair','Closer occluder','Partial / mixed','No pair available'];ax.barh(labs,values,color=['royalblue','skyblue','gray','orange','gold','lightgray']);ax.invert_yaxis();ax.set(xlabel='Teacher-false historical cell queries',title='Complete false historical domain:25,363');
for j,v in enumerate(values):ax.text(v+80,j,str(v),va='center',fontsize=8)
ax.set_xlim(0,max(values)*1.22)
ax=axs[1,0];rr=a['camera_anchor_input_reasons'];ax.barh(['Outside axial range','Outside native FOV','CAD','Matcher / depth invalid','Usable'],list(rr.values()),color=['lightgray','silver','gray','orange','royalblue']);ax.invert_yaxis();ax.set(xlabel='Camera-anchor queries',title='739,048 queries;18,873 usable (2.55%)')
ax=axs[1,1]
with np.load(D/'advisories/frame_012.npz') as z:
    cell=z['cells'];xy=cell*.1+z['origin'];badline=c['first_event_all15_false_cells'];badset={tuple(x['cell']) for x in badline};bad=np.array([tuple(x) in badset for x in cell]);ax.scatter(xy[~bad,0],xy[~bad,1],s=14,c='black',label='GT-compatible risk');ax.scatter(xy[bad,0],xy[bad,1],s=25,c='red',label='GT-false risk (15)');f=z['all_anchor_dispute'];ax.scatter(xy[f,0],xy[f,1],s=90,facecolors='none',edgecolors='blue',label='Strict dispute (4)')
ax.axis('equal');ax.set(xlabel='World X(m)',ylabel='World Y(m)',title='First no-route event6.1s / source6.0s');ax.legend(fontsize=8)
fig.suptitle('Counterevidence INPUT AUDIT ONLY - no map deletion or navigation\nTeacher labels scored after sealed RGB/native-pose advisories; unknown is not free',fontsize=12)
fig.tight_layout(rect=[0,0,1,.93]);fig.savefig(D/'input_audit.png',dpi=140);plt.close(fig)
print('PLOT',D/'input_audit.png')
