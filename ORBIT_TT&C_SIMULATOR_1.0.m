clear
close all
clc
warning("off", "all")

% download all maps and std models

maps = exist('maps.mat','file');
p836 = exist('p836.mat','file');
p837 = exist('p837.mat','file');
p840 = exist('p840.mat','file');
matFiles = [maps p836 p837 p840];
if ~all(matFiles)
    if ~exist('ITURDigitalMaps.tar.gz','file')
        url = 'https://www.mathworks.com/supportfiles/spc/P618/ITURDigitalMaps.tar.gz';
        websave('ITURDigitalMaps.tar.gz',url);
        untar('ITURDigitalMaps.tar.gz');
    else
        untar('ITURDigitalMaps.tar.gz');
    end
    addpath(cd);
end

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
%
% now with data volume and orbits calculations!!

%% DATA

% sat_orbit = [altitude [KM], eccentricity [°], inclination [°], RAAN [°], perigee argument [°],  true anomaly [°]]
sat_orbit = [450, 0, 97.2188, 0, 0, 0];

% ground_station = [latitude [°], longitudine [°], minimum elevation [°], altitude [m]]
gs = [45.062932177699984, 7.659119150857138, 20, 240];

% COM_SYS CHANNELS

% data_layer = [framing size [bit], payload size [bit], compression rate, sample rate, # measurements per sample]
data_science = [128, 2048, 0.3, 0.25, 2*11*7];
data_TM = [128, 2048, 0.3, 0.25, 5];

datas = [data_science; data_TM];

% physic_layer = [gain antenna [dB], beanwirdh [°], TX power [dBW], RX gain [dB], TX losses [dB], RX losses [dB], LNA NF [dB], Temperature Noise [K], is sat? 1/0]
gs_S = [28, 7.5, 7, 56, 0, 4, 0.9, 100, 0];
gs_U = [12, 30, 18, 40, 0, 4, 0.9, 100, 0];
sat_S = [6.5, 80, 0, 4, 0, 4, 1, 100, 1];
sat_U = [0, 179, 7, 4, 0, 4, 0.9, 100, 1];

% channel_layer = [frequency [MHz], datarate [bit/s] required e0/n0 [dB], from/to]
ch1 = [2255, 9600, 14, gs_S, sat_S];
ch2 = [438, 19600, 14, gs_U, sat_U];
ch3 = [2400, 512000, 14, sat_S, gs_S];
ch4 = [438, 9600, 14, sat_U, gs_U];

links = [ch1; ch2; ch3; ch4];

% simulation time = (YEAR, MONTH, DAY, HOUR, MINUTE, SECOND)
start = datetime(2027, 5, 1, 0, 0, 0, TimeZone="UTC");
stop = datetime(2027, 5, 2, 0, 0, 0, TimeZone="UTC");
sample = 60;

%% SCENERY CONFIGURATION

% scenario definition
ss = satelliteScenario(start, stop, sample);

% satellite and ground station definition
gs_om = groundStation(ss, gs(1), gs(2), "MinElevationAngle",gs(3), "Name","C3", "Altitude",gs(4));
sat = satellite(ss, (sat_orbit(1)+6378)*1000, sat_orbit(2), sat_orbit(3), sat_orbit(4), sat_orbit(5), sat_orbit(6), "Name","ELECTRA");

% antennas placing and pointing
% ground station
gs_gimbal = gimbal(gs_om, MountingLocation = [0,0,-1]);
sat_gimbal = gimbal(sat, MountingLocation = [0,0,1]);

for j = 1:size(links,1)
    if links(j,12) == 1
        sat_antenna(j) = conicalSensor(sat_gimbal, "MaxViewAngle", links(j,5));
    elseif links(j,12) == 0
        gs_antenna(j) = conicalSensor(gs_gimbal, "MaxViewAngle", links(j,5));
    end
end

pointAt(gs_gimbal, sat);
pointAt(sat, gs_om);

[~,elev,range] = aer(gs_om,sat);

for j = 1:size(elev,2)
    if elev(j) < 0
        elev(j) = 0;
        range(j) = 0;
    end
end

ac = access(gs_antenna(1), sat_antenna(1));
ac2 = access(gs_antenna(2), sat_antenna(2));
accesstab = accessIntervals(ac);
accesstab2 = accessIntervals(ac2);

% POST PROCESSING TRACKING
% fieldOfView(gs_antenna1);
% fieldOfView(gs_antenna2);
% fieldOfView(sat_antenna1);
% fieldOfView(sat_antenna2);

%% LINK-BUDGET

% atmospheric losses

tic
atmoloss = [];
temperature = [];
for j = 1:size(links,1)
    freq = links(j,1)*1e6;
    if freq > 1e9
        pathloss(j,:) = fspl(range(1,:), physconst('LightSpeed')/freq);
        for i = 1:size(elev,2)
            if elev(i) >= 5
                cfg = p618Config('Frequency',freq, "ElevationAngle", elev(i), "Latitude", gs(1), "Longitude", gs(2));
                [loss, ~, tsky] = p618PropagationLosses(cfg, "StationHeight",gs(4)/1000);
                atmoloss(j,i) = sum(struct2array(loss));
                temperature(j,i) = tsky;
            else
                atmoloss(j,i) = 0;
                temperature(j,i) = 0;
            end
        end
    else
        atmoloss(j,1:size(elev,2)) = 0;
        temperature(j,1:size(elev,2)) = 0;
        pathloss(j,:) = fspl(range(1,:), physconst('LightSpeed')/freq);
    end
end
toc

implementation_loss = 4;

% adverse case
pathloss_adv = max(pathloss, [], "all");
atmoloss_adv = max(atmoloss, [], "all");
temperature_adv = max(temperature, [], "all");

for j = 1:size(links,1)
    EIRP(j) = links(j,4)+links(j,6)+links(j,8);
    ISOTROPIC(j) = EIRP(j) - pathloss_adv - atmoloss_adv - implementation_loss;
    TEMPERATURE_SYS(j) = temperature_adv + 290*((10^(links(j,19)/10)-1)) + 290*(10^(links(j,18)/10)-1)/10^(links(j,16)/10);
    GT(j) = links(j,13) - 10*log10(TEMPERATURE_SYS(j));
    EBN0(j) = ISOTROPIC(j) + GT(j) + 228.6 - 10*log10(links(j,2));
end

MARGINS = EBN0' - links(:,3);

%% POST-SIM ORBITAL MECHANICS

% time between two accesses and access duration vectors
duration = accesstab.Duration;
t1 = accesstab.StartTime;
t2 = accesstab.EndTime;

for j = 1:(size(accesstab,1))
    if j == 1
       flight_between_access_i (j) = seconds(t1(j) - start);
    elseif j == (size(accesstab,1))
       flight_between_access_i (j) = seconds(stop - t2(j));
    else
       flight_between_access_i (j) = seconds(t1(j+1)-t2(j));
    end
end
flight_between_access_i = flight_between_access_i';

% elevation and range only during passages

%% DATA VOLUME - TO BE FIX -

% compressed payload calculation

measure_data = data_science(5)*64;
payload_data = (measure_data.*flight_between_access_i.*data_science(4)).*(1-data_science(3));

% framing data
n_packets = payload_data/measure_data;
dataframe = data_science(1)*floor(n_packets);

% net data downlinked and data produced between access
data_to_downlink = (dataframe + payload_data);
down_linkable = duration*drd*m;

respas_vect = [];
respas = 1;

for j = 1:size(payload_data,1)
    data_to_downlink(j,1) = data_to_downlink(j) + respas;
    respas = data_to_downlink(j) - down_linkable(j);
    if respas < 0
        respas = 0;
    end
    respas_vect(j,1) = respas;
end

respas_vect(size(payload_data,1),1) = respas_vect(size(payload_data,1)-1);          % because of size

data_history = table(data_to_downlink, accesstab.StartTime, respas_vect, accesstab.EndTime, VariableNames = ["MemoryStart", "TimeStart", "MemoryStop", "TimeStop"]);

%% FIGURES

data_history_plot_x = [];
data_history_plot_y = [];

for j = 1:size(data_history,1)
    
    data_history_plot_x = [data_history_plot_x; data_history.TimeStart(j) ; data_history.TimeStop(j)];
    data_history_plot_y = [data_history_plot_y; data_history.MemoryStart(j); data_history.MemoryStop(j)];
    
end

figure
hold on
title("Data Volume History")
ylabel("Mbit netto")
xlabel("Time Frame")
plot(data_history_plot_x, data_history_plot_y./1e6, "LineWidth",2)
hold off
