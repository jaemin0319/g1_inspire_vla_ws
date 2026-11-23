"""Classify policy-eval episodes: success / stance (|dy| > 4 cm or |dx| > 4 cm at reach) / no-close / stall (hand closed,
arm never lifted: right elbow target stays > 0.4 rad) / slip (arm lifted, object < 5 cm) / other."""
import json, sys, glob
import numpy as np
OFF = np.array([-0.292, 0.136])
for d in sys.argv[1:]:
    rows = [json.loads(l) for l in open(d + '/summary.jsonl')]
    cats = []
    for r in rows:
        z = np.load(f"{d}/ep{r['i']:02d}_trace.npz"); a = z['action']; bp = z['base_pos']
        mv = np.abs(a[:, :14] - a[0, :14]).max(1) > 0.15
        on = int(np.argmax(mv)) if mv.any() else len(a) - 1
        e = bp[on, :2] - (np.asarray(r['object_xy']) + OFF)
        hand = a[:, 14:26]
        closed = (np.abs(hand - hand[0]).max(1) > 0.3)
        ic = int(np.argmax(closed)) if closed.any() else None
        if r['outcome'] == 'success': c = 'success'
        elif abs(e[1]) > 0.04 or abs(e[0]) > 0.04: c = 'stance'
        elif ic is None: c = 'no-close'
        elif a[ic:, 10].min() > 0.4: c = 'stall'
        else: c = 'slip'
        cats.append(f"{c}({e[1]*100:+.0f})")
    print(f"{d.split('/')[-1]:45s} " + ' '.join(cats))
