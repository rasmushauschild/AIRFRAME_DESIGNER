"""Sync by what changes: the fan tone's pitch and the loudness edges, against the log's motor commands.

  sync2.py <wav> <ulg> ...   -> per log: motor-on lengths, best lag (video t = log t + lag), r_tone, r_edges
"""
import sys, json
import numpy as np
from scipy.io import wavfile
from scipy import signal
from pyulog import ULog

DT = 0.1

def audio_feats(p):
    fs, x = wavfile.read(p); x = x.astype(float) / 32768
    f, t, S = signal.spectrogram(x, fs, nperseg=4096, noverlap=4096 - int(fs * DT))
    band = (f > 300) & (f < 900)
    loud = 10 * np.log10(S[(f > 150) & (f < 4000)].sum(0) + 1e-14)
    tone = f[band][np.argmax(S[band], 0)]
    on = loud > np.percentile(loud, 10) + 15
    # longest on-run
    best, s = (0, 0), None
    for i, v in enumerate(np.r_[on, False]):
        if v and s is None: s = i
        if not v and s is not None:
            if i - s > best[1] - best[0]: best = (s, i)
            s = None
    return t, loud, tone, (t[best[0]], t[best[1] - 1])

def log_feats(p):
    u = ULog(p, ["actuator_motors", "actuator_armed"])
    t0 = u.start_timestamp
    D = {d.name: d.data for d in u.data_list}
    am = D["actuator_motors"]; t = (am["timestamp"].astype(float) - t0) / 1e6
    m = np.nan_to_num(np.stack([am["control[%d]" % i] for i in range(9)], 1)).clip(0, 1)
    aa = D["actuator_armed"]; ta = (aa["timestamp"].astype(float) - t0) / 1e6
    live = aa["armed"].astype(bool) & ~aa["lockdown"].astype(bool)
    if "manual_lockdown" in aa: live &= ~aa["manual_lockdown"].astype(bool)
    lv = np.interp(t, ta, live.astype(float)) > 0.5
    mean = m.mean(1) * lv
    tt = np.arange(0, t[-1], DT)
    mm = np.interp(tt, t, mean)
    on = tt[mm > 0.05]
    return tt, mm, ((on[0], on[-1]) if len(on) else (0, 0))

def z(v):
    s = v.std(); return (v - v.mean()) / (s if s > 0 else 1)

def corr_at(lag, ta, fa, tl, fl, span):
    """correlate audio feature fa(t) with log feature fl(t - lag) over the log's on-span"""
    a0, a1 = span
    tq = np.arange(a0, a1, DT)
    if len(tq) < 50: return -9
    va = np.interp(tq + lag, ta, fa); vl = np.interp(tq, tl, fl)
    if va.std() == 0 or vl.std() == 0: return -9
    return float(np.corrcoef(va, vl)[0, 1])

if __name__ == "__main__":
    ta, loud, tone, (as0, as1) = audio_feats(sys.argv[1])
    sm = lambda v, n: np.convolve(v, np.ones(n) / n, mode="same")
    dloud = np.r_[0, np.diff(sm(loud, 5))]
    for p in sys.argv[2:]:
        tl, mm, (ls0, ls1) = log_feats(p)
        if ls1 - ls0 < 5: continue
        dlm = np.r_[0, np.diff(sm(10 * np.log10(mm + 0.01), 5))]
        # coarse: align kill edges, then search +-3 s
        lag0 = as1 - ls1
        lags = np.arange(lag0 - 3, lag0 + 3, DT)
        span = (ls0 + 1, ls1 - 0.5)
        rt = [corr_at(L, ta, sm(tone, 5), tl, mm, span) for L in lags]
        re = [corr_at(L, ta, dloud, tl, dlm, (ls0 - 2, ls1 + 2)) for L in lags]
        i = int(np.argmax(np.array(rt) + np.array(re)))
        print(json.dumps({"log": p.split("/")[-1][:7], "audio_on_s": round(as1 - as0, 1), "log_on_s": round(ls1 - ls0, 1),
                          "lag_s": round(float(lags[i]), 2), "r_tone": round(rt[i], 2), "r_edges": round(re[i], 2),
                          "audio_on": [round(as0, 1), round(as1, 1)], "log_on": [round(ls0, 1), round(ls1, 1)]}))
