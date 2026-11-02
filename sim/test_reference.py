#!/usr/bin/env python3
"""Executed software reference checks, not Verilog simulation/FPGA fit."""
import math, json, pathlib
import numpy as np
from scipy.signal import freqz, lfilter
ROOT=pathlib.Path(__file__).resolve().parents[1]
results={}

def checked(name,cond,details):
    results[name]={"pass":bool(cond),"details":details}
    print(('PASS' if cond else 'FAIL'),name,details)
    if not cond: raise AssertionError(name)

# Verify signed 24x16 decomposed hardware product independent of RTL.
rng=np.random.default_rng(123)
a=rng.integers(-(1<<23),1<<23,size=20000,dtype=np.int64)
b=rng.integers(-32768,32768,size=20000,dtype=np.int64)
lo=a & 0xFFFF
hi=a >> 16
split=(lo*b)+(hi*b<<16)
checked('mul24x16_signed_split',np.array_equal(split,a*b),{"random_vectors":len(a)})

# Verify signed Q2.30 * Q1.15 decomposition into two 18x18-compatible products.
xf=rng.integers(-(1<<31),1<<31,size=20000,dtype=np.int64)
e=rng.integers(-32768,32768,size=20000,dtype=np.int64)
p=(xf & 65535)*e + ((xf >>16)*e<<16)
checked('gradient32x16_signed_split',np.array_equal(p,xf*e),{"random_vectors":len(xf)})

# Q2.38 retains fractional gradients that old Q2.30 integer shift removes.
q30_e,q30_x,shift=100,20,11
legacy=(q30_e*q30_x)>>shift
# representation of 20 Q1.15 LSBs as Q2.30
xfq30=q30_x*(1<<15)
newgrad=(q30_e*xfq30) >> (7+shift)
q38_stored=0
for _ in range(300): q38_stored += newgrad
checked('gradient_sub_lsb_accumulation',legacy==0 and newgrad>0 and q38_stored>=65536,
        {"legacy_Q30_step":legacy,"new_Q38_step":newgrad,"Q38_after_300":q38_stored,"quantized_Q22_after_300":q38_stored>>16})

# Negative fixed-point step is symmetric rather than >>> rounding toward -infinity.
trunc_shift=lambda x,k:(x>>k if x>=0 else -((-x)>>k))
test_steps=[1000,-1000,2000,-2000,50000,-50000]
checked('gradient_signed_symmetric_rounding',all(trunc_shift(v,11)==-trunc_shift(-v,11) for v in test_steps),
        {"examples":{str(v):trunc_shift(v,11) for v in test_steps}})

# Coefficient ROM frequency response and integer DC gain.
q=np.loadtxt(ROOT/'sim/fir127_q15.txt',dtype=int)
assert len(q)==127
w,h=freqz(q/32768,1,worN=131072,fs=48000)
def gain(f):
    return float(20*np.log10(max(abs(h[np.argmin(abs(w-f))]),1e-14)))
checked('anti_alias_filter',sum(q)==32768 and gain(800)>-1 and gain(3000)<-50,
        {"taps":127,"passband_800Hz_dB":round(gain(800),2),"stopband_3000Hz_dB":round(gain(3000),2),"dc_gain":int(sum(q))})

# Synchronous circular-buffer address arithmetic, wrap and no aliasing.
N=256
ring=np.zeros(N,dtype=np.int32)
for n in range(2000):
    ring[n%N]=n
    for k in range(min(127,n+1)):
        assert ring[(n-k)%N]==n-k
checked('circular_buffer_address',True,{"depth":N,"writes_checked":2000,"max_lookback":127})

# Simple reference closed-loop duct simulation: plant @6k, sine 200Hz.
# This is algorithm feasibility ONLY; not physical measurement/RTL simulation.
fs=6000
D=12
M=128
L=64
S=np.zeros(M)
S[0]=0.55;S[9]=0.12;S[23]=-0.05
T=21000
u=rng.choice([-1.,1.],T)*.18
sy=np.zeros(T)
se=np.zeros(T)
Slearn=np.zeros(M)
mu_s=.13
# Simple normalized LMS during calibration. Example ideal modeled path only.
for n in range(D+M,T):
    hist=u[n-D -np.arange(M)]
    desired=np.dot(S,hist)
    err=desired-np.dot(Slearn,hist)
    Slearn+=mu_s*err*hist
check=np.array([u[n-D -np.arange(M)] for n in range(T-500,T)])
err=check@(S-Slearn)
ref=check@S
nmse=10*np.log10(np.mean(err**2)/np.mean(ref**2))
checked('offline_secondary_identification',nmse < -18,{"validation_nmse_dB":round(float(nmse),2),"secondary_delay_samples":D})

# Feedback FxLMS simulation, secondary path estimate frozen, synthetic periodic noise.
T=24000
x=np.zeros(T);y=np.zeros(T);eobs=np.zeros(T);xf=np.zeros(T)
wgt=np.zeros(L)
mu=.045
lam=1-2**-15
for n in range(D+M+L,T):
    d=.12*math.sin(2*math.pi*200*n/fs)
    # Physical duct plant, received signal from OLD speaker outputs.
    sec=0.
    for k,val in enumerate(S):
        if val: sec += val*y[n-D-k]
    eobs[n]=d+sec
    spred=0.
    for k,val in enumerate(Slearn):
        if abs(val)>1e-11: spred+=val*y[n-D-k]
    x[n]=eobs[n]-spred
    y[n]=float(np.dot(wgt,x[n-np.arange(L)]))
    # Safety clamp in simulator, otherwise diverged trajectories are not valid.
    y[n]=np.clip(y[n],-.25,.25)
    xf[n]=np.dot(Slearn,x[n-D-np.arange(M)])
    wgt=lam*wgt-mu*eobs[n]*xf[n-np.arange(L)]
    wgt=np.clip(wgt,-.75,.75)
pre=np.sqrt(np.mean((.12*np.sin(2*np.pi*200*np.arange(T-3000,T)/fs))**2))
post=np.sqrt(np.mean(eobs[-3000:]**2))
nr=20*np.log10(pre/post)
checked('floating_feedback_anc_200hz',np.isfinite(nr) and nr>6,
        {"noise_reduction_dB":round(float(nr),2),"pre_rms":round(float(pre),5),"post_rms":round(float(post),5)})

# Numeric group delay note: 127-tap at 48 kHz: 63/48000 = 1.3125 ms
results['timing_nominal']={"pass":True,"details":{"CLOCK_50_cycles_per_DSP_sample":round(50e6/6000,3),"FIR_group_delay_ms":63/48,"codec_pcm_rate":48000,"DSP_rate":6000}}
# Export ideal reference plot; this does NOT come from HDL/hardware.
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
window=600
pwr=np.convolve(eobs**2,np.ones(window)/window,mode='valid')
fig,ax=plt.subplots(figsize=(8,3.6))
ax.plot(np.arange(len(pwr))/fs,20*np.log10(np.sqrt(np.maximum(pwr,1e-16))/pre),label='Error RMS vs baseline')
ax.axhline(-6,ls='--',label='6 dB simulation target')
ax.set(xlabel='Simulation time (s)',ylabel='RMS relative to ANC OFF (dB)',
       title='Floating-point idealized 200 Hz feedback ANC (not FPGA measurement)')
ax.set_ylim(-60,10);ax.grid(alpha=.25);ax.legend()
fig.tight_layout();fig.savefig(ROOT/'results/ideal_float_anc_200hz.png',dpi=140);plt.close(fig)
(ROOT/'results/reference_test_results.json').write_text(json.dumps(results,indent=2,ensure_ascii=False))
print('RESULT',len(results),'checks all PASS')
