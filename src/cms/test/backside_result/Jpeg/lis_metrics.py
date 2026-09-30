import re, statistics, os, glob
def _unit(s):
    m=re.match(r'([-0-9.eE+]+)\s*([pnfumkxg]?)',s)
    if not m: return None
    return float(m.group(1))*{'p':1e-12,'n':1e-9,'f':1e-15,'u':1e-6,'m':1e-3,'k':1e3,'x':1,'g':1e9,'':1}[m.group(2)]
def metrics(d):
    # any *.lis in the dir (backside or frontside deck)
    cands=glob.glob(os.path.join(d,"*.lis"))
    if not cands: return None
    lis=cands[0]
    ts={}; ss={}; power=None
    for l in open(lis,errors='ignore'):
        m=re.match(r'\s*(t_sink_\d+)=\s*([-0-9.eE+]+[a-z]?)',l)
        if m: ts[m.group(1)]=_unit(m.group(2)); continue
        m=re.match(r'\s*(slew_sink_\d+)=\s*([-0-9.eE+]+[a-z]?)',l)
        if m: ss[m.group(1)]=_unit(m.group(2)); continue
        m=re.match(r'\s*avg_power=\s*([-0-9.eE+]+[a-z]?)',l)
        if m: power=_unit(m.group(1))
    tv=sorted(v*1e12 for v in ts.values() if v and 0<v<1e-6)
    sv=[v*1e12 for v in ss.values() if v and 0<v<1e-6]
    if not tv:
        return _mt0(d)   # .lis lacked per-measure dump -> fall back to .mt0
    n=len(tv)
    return dict(n=n, toggled=n,
                skew=tv[-1]-tv[0],
                skew_p=tv[int(0.99*n)]-tv[int(0.01*n)],
                ins=statistics.mean(tv),
                pw=power*1e3 if power else 0.0,
                slew_mean=statistics.mean(sv) if sv else 0, slew_max=max(sv) if sv else 0)

def _mt0(d):
    # positional .mt0 parse (all names then all values). Correct for CLEAN
    # configs (every measure produces a numeric value; no 'failed'/omitted).
    cands=glob.glob(os.path.join(d,"*.mt0"))
    if not cands: return None
    toks=[]
    for l in open(cands[0],errors='ignore'):
        if l.startswith('$') or l.strip().startswith('.TITLE'): continue
        toks+=l.split()
    names=[t for t in toks if re.match(r'[A-Za-z]',t)]
    vals =[t for t in toks if not re.match(r'[A-Za-z]',t)]
    if len(names)!=len(vals): return None   # misaligned (failed measures) -> unreliable, skip
    dd=dict(zip(names,vals))
    tv=sorted(float(v)*1e12 for k,v in dd.items()
              if re.match(r't_sink_\d+$',k) and re.match(r'[-0-9.eE+]+$',v) and 0<float(v)<1e-6)
    sv=[float(v)*1e12 for k,v in dd.items()
        if re.match(r'slew_sink_\d+$',k) and re.match(r'[-0-9.eE+]+$',v) and 0<float(v)<1e-6]
    if not tv: return None
    n=len(tv); pw=dd.get('avg_power')
    return dict(n=n, toggled=n, skew=tv[-1]-tv[0], skew_p=tv[int(0.99*n)]-tv[int(0.01*n)],
                ins=statistics.mean(tv), pw=float(pw)*1e3 if pw and re.match(r'[-0-9.eE+]+$',pw) else 0.0,
                slew_mean=statistics.mean(sv) if sv else 0, slew_max=max(sv) if sv else 0)
