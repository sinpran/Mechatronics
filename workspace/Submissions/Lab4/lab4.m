clc;
clear;

Nyquist = 4000/2;
c = 500/Nyquist;

b = fir1(31, c)
freqz(b)
arraytoCformat(b')