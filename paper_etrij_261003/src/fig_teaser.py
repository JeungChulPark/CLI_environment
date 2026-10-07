"""Figure 1: why capture-time keyframe anchoring is needed (real run, large figure-eight, ORB-SLAM3).

(a) timing schematic; (b)/(c) top view of the final map of the same run with the same recognition
results registered (b) at the capture-time pose in the map frame and (c) relative to the capture-time
reference keyframe. Data: objpose/pc/anchor_ablation.py positions dumped to corr04.json.
"""
import json, sys, numpy as np
import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
plt.rcParams.update({'font.family':'Liberation Serif','font.size':7,'mathtext.fontset':'stix','savefig.dpi':300,
                     'axes.edgecolor':'#8a8984','xtick.color':'#52514e','ytick.color':'#52514e'})
BLUE,ORANGE,AQUA,GRAY='#2a78d6','#eb6834','#1baf7a','#8a8984'
D=json.load(open(sys.argv[1]))
NAMES={'milk':'milk carton','Bear':'teddy bear','Febreze_high':'spray bottle','saffron':'jug','Dinosaur':'dinosaur doll',
       'Mugcup_high':'mug','Sikhye_high':'can','choco_hazelnut_high':'snack box'}
fig=plt.figure(figsize=(6.9,2.45))
gs=fig.add_gridspec(1,3,width_ratios=[1.25,1,1],wspace=0.18)

# (a) schematic
ax=fig.add_subplot(gs[0]); ax.set_xlim(0,10); ax.set_ylim(-0.4,4.4); ax.axis('off')
for y,t in {3.5:'SLAM poses',2.3:'keyframes',1.0:'6D results'}.items():
    ax.plot([1.7,9.9],[y,y],color='#d6d5cf',lw=1); ax.text(1.6,y,t,ha='right',va='center',color='#333')
for x in np.arange(1.8,9.9,0.3): ax.plot([x,x],[3.4,3.6],color=BLUE,lw=0.7)
for x,k in [(2.4,'K6'),(3.9,'K7'),(7.0,'K8'),(9.0,'K9')]:
    ax.add_patch(plt.Rectangle((x-0.32,2.12),0.64,0.36,color=BLUE)); ax.text(x,2.3,k,color='w',ha='center',va='center',weight='bold',fontsize=6.5)
tc,tl,ta=4.5,5.8,8.0
ax.plot([tc,tc],[1.1,3.4],ls='--',color=GRAY,lw=0.8)
ax.add_patch(plt.Rectangle((tc,0.92),ta-tc,0.16,color=AQUA,alpha=0.3))
ax.plot(tc,1.0,'o',color=AQUA,ms=5); ax.plot(ta,1.0,'s',color=AQUA,ms=5)
ax.text(tc,0.62,'capture $t_c$',ha='center',va='top'); ax.text(ta,0.62,'arrival',ha='center',va='top')
ax.text((tc+ta)/2,1.22,'0.7–6.6 s',ha='center',color='#0d5c40')
ax.plot([tl,tl],[1.95,3.75],color=ORANGE,lw=1.8); ax.text(tl,3.95,'loop closure moves K6, K7',ha='center',color='#a63c12',weight='bold')
ax.text(0.2,-0.2,'(b) store at $T_{map\\leftarrow cam}(t_c)$: stays behind\n(c) store relative to K7 sent at $t_c$: moves with K7',va='bottom',fontsize=6.8)
ax.set_title('(a) A late result and a map correction in flight',fontsize=7.5,loc='left')

REF={}
for o,p,i in D['positions']['D']: REF.setdefault(o,[]).append(p)
REF={o:np.median(np.array(v),axis=0) for o,v in REF.items()}
K=np.array(list(D['kf'].values()))[np.argsort(np.array(list(map(int,D['kf'].keys()))))]
R=np.array(list(REF.values()))
def panel(i,v,title):
    ax=fig.add_subplot(gs[i])
    ax.plot(K[:,0],K[:,2],color='#d0cfc9',lw=0.7,zorder=1)
    P=D['positions'][v]
    xy=np.array([[p[0],p[2]] for _,p,_ in P])
    d=np.array([np.hypot(p[0]-REF[o][0],p[2]-REF[o][2]) for o,p,_ in P]); far=d>0.10
    ax.scatter(xy[~far,0],xy[~far,1],s=10,color=BLUE,lw=0,zorder=3,label='within 10 cm of the final object position')
    ax.scatter(xy[far,0],xy[far,1],s=12,color=ORANGE,lw=0,zorder=4,label='more than 10 cm away')
    ax.scatter(R[:,0],R[:,1],s=14,marker='+',color='#111',lw=0.7,zorder=5,label='final object position')
    ax.set_aspect('equal'); ax.set_xticks([]); ax.set_yticks([])
    for sp in ax.spines.values(): sp.set_visible(False)
    ax.set_title(title,fontsize=7.5,loc='left')
    ax.text(0.0,-0.02,f'{far.sum()} of {len(P)} results more than 10 cm away',transform=ax.transAxes,fontsize=6.8,va='top')
    return ax,xy,far
ax1,xy1,far1=panel(1,'B','(b) Capture-time pose, map frame')
ax2,_,_=panel(2,'D','(c) Capture-time keyframe (ours)')
lo=(min(ax1.get_xlim()[0],ax2.get_xlim()[0]),max(ax1.get_xlim()[1],ax2.get_xlim()[1]))
la=(min(ax1.get_ylim()[0],ax2.get_ylim()[0]),max(ax1.get_ylim()[1],ax2.get_ylim()[1]))
for a in (ax1,ax2): a.set_xlim(*lo); a.set_ylim(*la)
# annotate the left-behind copies in (b)
top=xy1[far1]; c=top[np.argmax(top[:,1])] if len(top) else None
if c is not None:
    ax1.annotate('copies left behind\nby a 3 m loop correction',xy=(c[0]+0.1,c[1]-0.1),xytext=(c[0]+0.45,c[1]-0.15),fontsize=6.8,color='#a63c12',va='top',ha='left',
                 arrowprops=dict(arrowstyle='->',color='#a63c12',lw=0.8))
ax2.plot([lo[1]-1.2,lo[1]-0.2],[la[0]+0.1]*2,color='#333',lw=1); ax2.text(lo[1]-0.7,la[0]+0.2,'1 m',ha='center',fontsize=6.5)
h,l=ax1.get_legend_handles_labels(); fig.legend(h,l,loc='lower center',ncol=3,frameon=False,bbox_to_anchor=(0.66,-0.08),fontsize=6.6,markerscale=1.3)
fig.savefig(sys.argv[2],bbox_inches='tight')
