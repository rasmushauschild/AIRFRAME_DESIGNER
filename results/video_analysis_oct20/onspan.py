import sys, numpy as np
from scipy.io import wavfile
from scipy import signal
from scipy.ndimage import median_filter
for p in sys.argv[1:]:
    fs, x = wavfile.read(p); x = x.astype(float) / 32768
    f, t, S = signal.spectrogram(x, fs, nperseg=4096, noverlap=4096 - 4800)
    e = 10 * np.log10(S[(f > 1500) & (f < 4000)].mean(0) + 1e-16)
    e = median_filter(e, size=11)  # ~1 s
    lo, hi = np.percentile(e, 5), np.percentile(e, 95)
    on = e > (lo + hi) / 2
    spans, s = [], None
    for i, v in enumerate(np.r_[on, False]):
        if v and s is None: s = i
        if not v and s is not None:
            if t[i - 1] - t[s] > 2: spans.append((round(t[s], 1), round(t[i - 1], 1), round(t[i - 1] - t[s], 1)))
            s = None
    print(p.split('/')[-1], "on/off level %.0f/%.0f dB" % (hi, lo), "motor spans (start, end, length):", spans)
