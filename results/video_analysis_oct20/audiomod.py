"""Does the fan noise wobble at the gyro's frequency? Envelope-modulation spectrum of the sound vs the gyro spectrum, powered phase."""
import sys, json, numpy as np
from scipy.io import wavfile
from scipy import signal
from pyulog import ULog
wav, ulg, off = sys.argv[1], sys.argv[2], float(sys.argv[3])
res = json.load(open(sys.argv[4]))
A, B = res["takeoff"] + 1.0, res["kill"] - 0.5
fs, x = wavfile.read(wav); x = x.astype(float) / 32768
b, a = signal.butter(4, [200, 4000], btype="band", fs=fs); y = signal.filtfilt(b, a, x)
env = np.abs(signal.hilbert(y))
# downsample envelope to 1 kHz
env = signal.decimate(signal.decimate(env, 8), 6); fe = fs / 48
te = np.arange(len(env)) / fe
m = (te >= A + off) & (te <= B + off)
f, Pe = signal.welch(env[m] - env[m].mean(), fe, nperseg=2048)
band = (f > 5) & (f < 60)
Pn = Pe / np.median(Pe[band])
pk, _ = signal.find_peaks(Pn * band, prominence=2)
top = sorted([(round(float(f[i]), 1), round(float(Pn[i]), 1)) for i in pk], key=lambda z: -z[1])[:5]
u = ULog(ulg, ["sensor_combined"]); t0 = u.start_timestamp; sc = u.data_list[0].data
ts = (sc["timestamp"].astype(float) - t0) / 1e6; g = np.stack([sc["gyro_rad[%d]" % i] for i in range(2)], 1)
mg = (ts >= A) & (ts <= B); fsg = 1 / np.median(np.diff(ts))
fg, Pg = signal.welch(g[mg], fsg, nperseg=512, axis=0); Pg = Pg.sum(1)
bg = (fg > 5) & (fg < 60); Pgn = Pg / np.median(Pg[bg])
pk2, _ = signal.find_peaks(Pgn * bg, prominence=2)
gtop = sorted([(round(float(fg[i]), 1), round(float(Pgn[i]), 1)) for i in pk2], key=lambda z: -z[1])[:5]
# tone (blade-pass) region: strongest lines 100-2000 Hz
f2, P2 = signal.welch(y[(np.arange(len(y)) / fs >= A + off) & (np.arange(len(y)) / fs <= B + off)], fs, nperseg=16384)
mb = (f2 > 100) & (f2 < 2000); pk3, _ = signal.find_peaks(10 * np.log10(P2[mb]), prominence=8)
tones = sorted([round(float(f2[mb][i])) for i in pk3[np.argsort(P2[mb][pk3])[::-1][:6]]])
# coherence: does the sound wobble move with the gyro wobble?
gi = np.interp(te[m] - off, ts, np.linalg.norm(g, axis=1))
fc, C = signal.coherence(env[m] - env[m].mean(), gi - gi.mean(), fe, nperseg=1024)
cb = (fc > 10) & (fc < 40)
print(json.dumps({"log": res["log"], "sound_wobble_peaks_hz": top, "gyro_peaks_hz": gtop, "tones_hz": tones,
                  "coherence_10_40hz_max": round(float(C[cb].max()), 2), "at_hz": round(float(fc[cb][np.argmax(C[cb])]), 1)}))
