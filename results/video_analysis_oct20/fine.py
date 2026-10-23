import sys, json, numpy as np
from scipy.io import wavfile
from scipy import signal
from scipy.ndimage import median_filter
exec(open("sync2.py").read().split("if __name__")[0])
def hf(p):
    fs, x = wavfile.read(p); x = x.astype(float) / 32768
    f, t, S = signal.spectrogram(x, fs, nperseg=4096, noverlap=4096 - int(fs * DT))
    e = 10 * np.log10(S[(f > 1500) & (f < 4000)].mean(0) + 1e-16)
    band = (f > 300) & (f < 900)
    tone = f[band][np.argmax(S[band], 0)]
    return t, e, median_filter(tone, 7)
video, end_a = sys.argv[1], float(sys.argv[2])
ta, e, tone = hf(video)
for p in sys.argv[3:]:
    tl, mm, (l0, l1) = log_feats(p)
    lm = 10 * np.log10(mm + 0.01)
    best = None
    for lag in np.arange(end_a - l1 - 3, end_a - l1 + 3, 0.02):
        tq = np.arange(max(l0 - 1, ta[0] - lag), min(l1 + 1.5, ta[-1] - lag), 0.02)
        a = np.interp(tq + lag, ta, e); b = np.interp(tq, tl, lm)
        r = np.corrcoef(np.diff(a), np.diff(b))[0, 1]   # edges
        if best is None or r > best[1]: best = (lag, r)
    lag = best[0]
    tq = np.arange(l0 + 2, l1 - 0.5, DT)
    rt = np.corrcoef(np.interp(tq + lag, ta, tone), np.interp(tq, tl, mm))[0, 1]
    # residual: motor-stop edge in video vs log
    print(json.dumps({"log": p.split("/")[-1][:7], "lag_video_minus_log": round(float(lag), 2), "r_edges": round(float(best[1]), 3), "r_tone_vs_cmd": round(float(rt), 3)}))
