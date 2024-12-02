close all
clear all
clc

%% README
% 
% INPUT: everything written in the DATA section, all those
% variables can be modified depending on the system
% OUTPUT: links margins 
% made by giorgino
%
% PS: the code works although it needs some kind of validation, expecially
% on the atmospheric losses part where some ITU-R had to be followed with
% some confusion on my part. It still misses the losses caused by
% scintillations.

%% INITIAL DATA

% orbita
orbit=450;          % KM    - orbit altitude
min_elev=12;        % °     - minimum elevation
flyover = 05;       %       - fly over in minutes
m = 0.7;            % [-]   - satellite fly over margin
alt_gs = 240;       % m     - ground station altitude (amsl)

% data
dru=9600;           % bit/s - datarate uplink
drd=512000;         % bit/s - datarate downlink
req=14;             % dB/BER10-6 GMSK / TELEMETRYDATA - e0n0 required
frequency=2255;  % MHz   - carrier frequency

% data packets
frame = 128;        % bits  - packet frame size
payload = 2048;     % bits  - payload packet size
compr_rate = 0.3;   % [-]   - compression rate  
samplerate = 20;    % Hz    - sample rate
N_measu = 48;       % [-]   - # of single-precision floats
T_sample = 90;      %       - time elaspes sampling in minutes

% GS
beanwitdh_gs=7.5;    % °     - ground station antenna beamwitdh
gain_gs=28;         % dB    - ground station antenna gain
power_gs=7;      % dbW   - total ground station HPA gain
tegs=300;           % K     - ground station temperature
lnags=56;           % dB    - total ground station LNA gain
lgs=4;              % dB    - ground station RX line losses
lnanfgs=0.9;       % dB    - ground station LNA noise figure

% SAT
beanwitdh_sat=80;  % °     - satellite antenna beamwitdh
gain_sat=6.5;         % dB    - satellite antenna gain
power_sat=0;        % dBW   - total satellite HPA gain
tesat=370;          % K     - satellite temperature
lsat=2;             % dB    - satellite RX line losses
lnasat=4;           % dB    - total satellite LNA gain 
lnanfsat=1;         % dB    - satellite LNA noise figure

%% CALCULATIONS

% POSITIONING LOSSES
ptlsat = -12*((5/(beanwitdh_sat))^2);
ptlgs = -12*((2/(beanwitdh_gs))^2);

% SLAT DISTANCE AND FSPL
slat=6378000*(sqrt(((orbit*1000+6378000)^2)/(6378000^2)-(cos(deg2rad(min_elev)))^2)-sin(deg2rad(min_elev)));
fspl=-22-20*log10(slat./(300./frequency));

% ATMOSPHERIC LOSSES
polar=-2;
lion=-2;
rain = 0;
Agas = 0;
if frequency*1e6 > 1e9

    % ITU-R P.676-13: ceiling atmosphere = 100Km

    ilow = floor(100*log(1e4*(alt_gs/1e3)*exp(1/100)+1)-1);
    itot = ilow:922;
    delta = 1e-4*exp((itot-1)/100);
    
    h = alt_gs/1e3 + cumsum(delta);
    r = 6371 + cumsum(delta);

    n = 1 + 315*1e-6*exp(-h/7350);
    beta1 = 90-min_elev;
    beta = [beta1, rad2deg(asin(((n(1)*r(1))./(n(2:end).*r(2:end))).*sin(deg2rad(beta1))))];
    
    path = -r.*cos(deg2rad(beta))+sqrt(r.^2.*cos(deg2rad(beta)).^2+2*r.*delta+delta.^2);
    
    [T, kk, P, rho] = atmosisa(h*1e3);

    for i = 1:length(path)
        agas = gaspl(path(i)*1e3, frequency*1e6, T(i)-273.15, P(i), rho(i));
    end

    Agas = -sum (agas);
    disp(["Atmospheric attenuation due to Oxigen and Vapor", Agas])

end

gasloss = Agas;

% SYSTEMS TEMPERATURE
t0=290;
Tsat = 290 + t0*((10^(lnanfsat/10)-1)) + t0*(10^(lsat/10)-1)/10^(lnasat/10);
Tgs = 15 + t0*((10^(lnanfgs/10)-1)) + t0*(10^(lgs/10)-1)/10^(lnags/10);

% EIRP
EIRPgs = power_gs + gain_gs - lgs;
EIRPsat = power_sat + gain_sat - lsat;

% TOTAL SPACE LOSS
spaceloss=fspl+polar+lion+gasloss+rain;

% ISOTROPIC POWER REICEVED
ISOgs = EIRPsat + spaceloss + ptlsat + ptlgs;
ISOsat = EIRPgs + spaceloss + ptlgs + ptlsat;  

% FIGURE OF MERIT5
GTgs = gain_gs - 10*log10(Tgs);
GTsat = gain_sat - 10*log10(Tsat);

% C/N0
CN0gs = ISOgs + GTgs + 228.6;
CN0sat = ISOsat + GTsat + 228.6;

%% UPLINK & DOWNLINK

ebn0_up = CN0sat - 10*log10(dru);
marginup = ebn0_up-req;
disp(["Uplink margin - dB:", marginup])

ebn0_down = CN0gs - 10*log10(drd);
margindown = ebn0_down-req;
disp(["Downlink margin - dB:", margindown])

%% DATA VOLUME
measure_data = N_measu*32;
payload_data = (measure_data*T_sample*60*samplerate)*(1-compr_rate);
n_packets = payload_data/measure_data;
dataframe = frame*floor(n_packets);

payload_data;
disp(["Saved Payload Data - Mbits", payload_data/(1e6*(1-compr_rate))])

downlinkdata = dataframe + payload_data;
disp(["Total Data in Downlink - Mbits", downlinkdata/1e6])

timetodownlink = downlinkdata / drd;
disp(["Time to Downlink - Seconds", timetodownlink])
