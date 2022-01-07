clear;
clc;

sample = 10000;
Nyquist = sample/2;
 
Wn = [1950/Nyquist 2050/Nyquist]
N = 100;
B = fir1(N, Wn, 'bandpass')

freqz(B)
arraytoCformat(B')

