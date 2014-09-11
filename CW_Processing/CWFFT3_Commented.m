function [peak,freq]=CWFFT3_Commented(w,res)
% Inputs ( variable length sampled waveform , bitwise
% resolution of the FFT i.e. 8 is an FFT of 2^8 points)
% 
% Outputs ( peak power in Db, frequency at which that peak is located)
% The true frequency output depends on the sampling frequency and sample time
% Frequency will be slightly off if those values are not correct
% Added comments by Kyle Cutler

% w waveform, res FFT 2^rest, xlim flag for 20KHz
% close all 

% Sampling frequency in Hz
Fs = 3.5e5;           

% Time between samples
T = 1/Fs;             

% Length of the input waveform
L = size(w,1);                     

% t = (0:L-1)*T;                % Time vector
% shift=(min(w)+max(w))/2;

% Shifting the waveform removes the average DC component
% this results in less low power noise on the FFT
shift=mean(w); %takes average of waveform
w=w-shift; %subtracts average from whole waveform

% figure, plot(w,':x')

% Different windowing functions
% hamming() returns periodic hamming window with length L
h1=hamming(L,'periodic');
% blackman() returns periodic blackman window with length L
b1=blackman(L,'periodic');
% flattopwin() returns periodic blackman window with length L
f1=flattopwin(L,'periodic');

% multiply the window function with waveform
wh1=h1(:).*w(:);
wb1=b1(:).*w(:);
wf1=f1(:).*w(:);

% apples a Savitzky-Golay FIR smoothing filter to data in w
% sgolayfilt ( sample, polynomial order, frame size(must be less than k and
% odd)
ws=sgolayfilt(w,3,7);

% Sum of a 50 Hz sinusoid and a 120 Hz sinusoid
% x = 0.1636*sin(2*pi*2e4*t); 
% y = x;     % Sinusoids plus noise
% figure, plot(Fs*t,y,':bo')
% title('Signal Corrupted with Zero-Mean Random Noise')
% xlabel('time (milliseconds)')
% 
% hold on, plot(w,'r*')

% 2^res = length of fft array
NFFT = 2^res; % Next power of 2 from length of y
% Y = fft(y,NFFT)/L;

% Y1 through Y5 compute fft on original, windowed, and filtered data
Y1=fft(w,NFFT)/L;
Y2=fft(wh1,NFFT)/L;
Y3=fft(wb1,NFFT)/L;
Y4=fft(wf1,NFFT)/L;
Y5=fft(ws,NFFT)/L;

% f is the frequency array that corresponds to the sampling rate and length
% linspace gives an evenly distributed array from a to b with x data points
% linspace (a,b,x)
f = Fs/2*linspace(0,1,NFFT/2);

% Compute the power at each frequency using different FFT results
% windowed, original, filtered

powY3=sqrt(real(Y3(1:NFFT/2)).^2+imag(Y3(1:NFFT/2)).^2)/NFFT;
powY1=sqrt(real(Y1(1:NFFT/2)).^2+imag(Y1(1:NFFT/2)).^2)/NFFT;
powY2=sqrt(real(Y2(1:NFFT/2)).^2+imag(Y2(1:NFFT/2)).^2)/NFFT;
powY4=sqrt(real(Y4(1:NFFT/2)).^2+imag(Y4(1:NFFT/2)).^2)/NFFT;
powY5=sqrt(real(Y5(1:NFFT/2)).^2+imag(Y5(1:NFFT/2)).^2)/NFFT;

 
% Peak is the db (20*log(x) of the max computed power

% Outputs the peak power to [peak]
peak=20*log10(max(powY3));

% Outputs the frequency of the peak power to [freq]
freq=f(find(powY3==max(powY3)));


end