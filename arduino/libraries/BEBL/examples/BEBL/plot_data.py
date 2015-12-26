from pylab import *
from numpy import *
import scipy.signal

sample_period = 12e-3
dt = sample_period
sample_rate = 1 / sample_period
fmax = 5.
down_fmax = 1.

N = 2
Wn = fmax / sample_rate
down_Wn = down_fmax / sample_rate
ff, fb = scipy.signal.butter(N, Wn, btype='low', analog=False, output='ba')
down_ff, down_fb = scipy.signal.butter(N, down_Wn, btype='low', analog=False, output='ba')
print 'sample_rate:', sample_rate, 'Hz'
print 'Bandwidth:', sample_rate/2, 'Hz'
print 'ff = {', ', '.join(map(str, ff)), '};'
print 'fb = {', ', '.join(map(str, fb)), '};'
print 'down_ff = {', ', '.join(map(str, down_ff)), '};'
print 'down_fb = {', ', '.join(map(str, down_fb)), '};'
data = open('data.txt').readlines()
data = [l.split() for l in data]
data = array([map(float, l) for l in data if len(l) == 9])
# data = data - mean(data, axis=0)[newaxis]
time = arange(len(data)) * dt

fdata = scipy.signal.lfilter(ff, fb, data[:,::3], 0)
down_fdata = scipy.signal.lfilter(down_ff, down_fb, data[:,::3], 0)
corr = abs(scipy.signal.correlate(fdata[:,2], data[:,2*3]))
delay = (argmax(corr) - (len(data) - 1)) * dt
print 'filter_delay', delay

D = fft.rfft(data, axis=0)
fD = fft.rfft(fdata, axis=0)
down_fD = fft.rfft(down_fdata, axis=0)
colors = 'bgr'
for i in range(3):
    color = colors[i]
    plot(time, data[:,3*i], '%s.' % color)
    plot(time, fdata[:,i], '%s--' % color, linewidth=1)
    plot(time, down_fdata[:,i], '%s-' % color, linewidth=3)

plot(time, sqrt(sum(data[:,::3] ** 2, axis=1)), '-')
plot(time, sqrt(sum(data[:,1::3] ** 2, axis=1)), '-')
# plot(time, sqrt(sum(data[:,2::3] ** 2, axis=1)), '-')
# plot(fft.rfftfreq(len(data), sample_period), abs(D[:,::3]) ** 2)
# plot(fft.rfftfreq(len(data), sample_period), abs(fD) ** 2, linewidth=3)
figure();
plot(time, data[:,0])
show()
