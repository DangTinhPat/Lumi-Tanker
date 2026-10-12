import re, struct, sys, numpy as np
from pathlib import Path
M = Path(__file__).resolve().parents[1] / 'src/tap_sim/model'
xml = (M/'gim_dynamic.xml').read_text()
i = xml.index('name="gim_body"'); j = xml.index('<body', i+20); blk = xml[i:j]
meshfile = dict(re.findall(r'<mesh name="(cad_\d+_0)" file="([^"]+)"', xml))
geoms = re.findall(r'<geom name="(cad_(\d+)_0)"[^>]*? pos="([^"]+)"', blk)

def load(path):
    b = Path(path).read_bytes(); n = struct.unpack('<I', b[80:84])[0]
    a = np.frombuffer(b[84:84+50*n], dtype=np.dtype([('n','<3f4'),('v','<9f4'),('a','<u2')]))
    return a['v'].reshape(-1,3,3).astype(float)*1e-3

def props(tri):
    # unit-density solid: volume, centroid, inertia tensor about centroid (tetra decomposition to origin)
    a,b,c = tri[:,0],tri[:,1],tri[:,2]
    d = np.einsum('ij,ij->i', a, np.cross(b,c))        # 6*signed vol
    V = d.sum()/6
    cen = (d[:,None]*(a+b+c)).sum(0)/(24*V)
    # second moments int x_i x_j dV
    S = np.zeros((3,3))
    for t in range(3):
        pass
    P = np.stack([a,b,c],1)  # n,3,3
    sums = np.einsum('nki,nkj->nij', P, P) + np.einsum('ni,nj->nij', a+b+c, a+b+c)
    S = (d[:,None,None]*sums).sum(0)/120
    I0 = np.trace(S)*np.eye(3) - S
    cm = cen
    Ic = I0 - V*((cm@cm)*np.eye(3) - np.outer(cm,cm))
    return V, cen, Ic

parts = {}
for name, num, pos in geoms:
    num = int(num); pos = np.array([float(x) for x in pos.split()])
    V, cen, Ic = props(load(M/meshfile[name]))
    parts[num] = (V, pos+cen, Ic)
cat = {}
for n in (13,14,35,36): cat[n] = 'motor'
cat[49] = 'battery'
residual = [48]+[n for n in parts if 18<=n<=21 or n==24 or 51<=n<=54 or 56<=n<=59]
for n in residual: cat[n] = 'resid'

def combine(items):   # items: (mass, com, Icom)
    m = sum(i[0] for i in items); c = sum(i[0]*i[1] for i in items)/m
    I = sum(i[2] + i[0]*((i[1]-c)@(i[1]-c)*np.eye(3)-np.outer(i[1]-c,i[1]-c)) for i in items)
    return m, c, I

def build(frame):
    for n in parts:
        if n not in cat: cat[n] = 'frame' if n in frame else None
    groups = {'frame':0.278,'motor':1.6,'battery':0.21,'resid':0.312}
    items = []
    for g, mg in groups.items():
        ns = [n for n in parts if cat[n]==g]
        vt = sum(parts[n][0] for n in ns)
        for n in ns:
            m = mg*parts[n][0]/vt
            items.append((m, parts[n][1], parts[n][2]*(m/parts[n][0])))
    return combine(items)

if __name__ == '__main__':
    print('parts in body:', sorted(parts))
    frame_sets = {
      'all other (incl 7,8,31,32)': [n for n in parts if n not in cat],
      'no hip mounts 7,8,31,32': [n for n in parts if n not in cat and n not in (7,8,31,32)],
    }
    for k, fs in frame_sets.items():
        cat2 = dict(cat)
        m, c, I = build(set(fs))
        print(k, 'frame parts', sorted(fs)); print(' mass', round(m,4), 'com', np.round(c,8)); print(' I', np.round(I,9).tolist())
        for n in fs: cat.pop(n, None)
    print('lab: com -0.00509198 0.00012916 0.01377824  I diag 0.008962725 0.004829976 0.009367159')
