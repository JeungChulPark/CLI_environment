"""Small panels embedded in the main figure (fig_main.html): timeline and the two final maps of a real run."""
import json, sys, numpy as np
import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
plt.rcParams.update({'font.family':'Liberation Serif','font.size':9,'mathtext.fontset':'stix','savefig.dpi':300})
BLUE,ORANGE,AQUA,GRAY='#2a78d6','#eb6834','#1baf7a','#8a8984'
D=json.load(open(sys.argv[1])); out=sys.argv[2]

# timeline (no text except keyframe ids; labels are in HTML)
fig,ax=plt.subplots(figsize=(3.0,1.25)); ax.set_xlim(0,10); ax.set_ylim(0.5,3.9); ax.axis('off')
for y in (3.4,2.3,1.1): ax.plot([0.2,9.8],[y,y],color='#d6d5cf',lw=1)
for x in np.arange(0.3,9.8,0.3): ax.plot([x,x],[3.28,3.52],color=BLUE,lw=0.8)
for x,k in [(1.2,'K6'),(3.0,'K7'),(6.3,'K8'),(8.6,'K9')]:
    ax.add_patch(plt.Rectangle((x-0.45,2.08),0.9,0.44,color=BLUE)); ax.text(x,2.3,k,color='w',ha='center',va='center',weight='bold',fontsize=8)
tc,tl,ta=3.4,4.9,7.6
ax.plot([tc,tc],[1.2,3.3],ls='--',color=GRAY,lw=1)
ax.add_patch(plt.Rectangle((tc,1.0),ta-tc,0.2,color=AQUA,alpha=0.3))
ax.plot(tc,1.1,'o',color=AQUA,ms=7); ax.plot(ta,1.1,'s',color=AQUA,ms=7)
ax.plot([tl,tl],[1.9,3.75],color=ORANGE,lw=2.4)
fig.savefig(out+'/timeline.png',bbox_inches='tight',pad_inches=0.02,transparent=False)

REF={}
for o,p,i in D['positions']['D']: REF.setdefault(o,[]).append(p)
REF={o:np.median(np.array(v),axis=0) for o,v in REF.items()}
R=np.array(list(REF.values()))
K=np.array(list(D['kf'].values()))[np.argsort(np.array(list(map(int,D['kf'].keys()))))]
lim=None
for v,name in (('B','map_b'),('D','map_c')):
    fig,ax=plt.subplots(figsize=(1.9,2.2))
    ax.plot(K[:,0],K[:,2],color='#d0cfc9',lw=0.8)
    P=D['positions'][v]; xy=np.array([[p[0],p[2]] for _,p,_ in P])
    far=np.array([np.hypot(p[0]-REF[o][0],p[2]-REF[o][2])>0.10 for o,p,_ in P])
    ax.scatter(xy[~far,0],xy[~far,1],s=14,color=BLUE,lw=0,zorder=3)
    ax.scatter(xy[far,0],xy[far,1],s=16,color=ORANGE,lw=0,zorder=4)
    ax.scatter(R[:,0],R[:,1],s=22,marker='+',color='#111',lw=0.8,zorder=5)
    ax.set_aspect('equal'); ax.axis('off')
    if lim is None: lim=(ax.get_xlim(),(min(ax.get_ylim()[0],K[:,2].min()-0.2),max(ax.get_ylim()[1],xy[:,1].max()+0.2)))
    ax.set_xlim(*lim[0]); ax.set_ylim(*lim[1])
    if v=='D':
        x0=lim[0][1]-1.15; y0=lim[1][0]+0.05; ax.plot([x0,x0+1],[y0,y0],color='#333',lw=1.2)
    fig.savefig(out+f'/{name}.png',bbox_inches='tight',pad_inches=0.02)
    print(name, far.sum(), len(P))
