clear
close all
clc
format shortG
warning("off", "all")
f = waitbar(0, "Starting Simulation", "Name", "SATCOM SIMULATOR");

%% README
%
% INPUT: everything written in the DATA section, all those
% variables can be modified depending on the system and channels you want
% to test.
%
% OUTPUT: links margins in the MARGIN variable, following the data
% structure in the LINKS vector. Data volume graphs and data in downlink 
% dipending on LINKS chosen in downlink (IS SAT 1/0).
%
% !! Takes around 30 minutes for a month of simulation. !!
%
% Made for the ESA: FYS! DB2, ELECTRA, GS: C3, modular enough to be 
% used for every cubesat mission design and every ground station.
%
% TBA: doppler effect calculation, skygraphs, signal generators.
%
% for help: giorgio.abbate@studenti.polito.it

%% PREPARATION FOR THE SIMULATION

% download all ITU maps and std models. As of 03/2025 they are all updated 
% to latest.
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

%% DATA INPUT

% sat_orbit = [altitude [KM], eccentricity [°], inclination [°], RAAN [°], perigee argument [°],  true anomaly [°]]
sat_orbit = [450, 0, 97.2188, 0, 0, 0];     

% ground_station = [latitude [°], longitudine [°], minimum elevation [°], altitude [m]]
gs = [45.062932177699984, 7.659119150857138, 5, 280];

% COM_SYS CHANNELS
% data_layer = [framing size [bit], payload size [bit], compression rate, sample rate, # measurements per sample]
data_science = [128, 2048, 0.3, 1, 960*12];        % 960*12 FULL science
                                                   % 640*12 TEC-MODE
                                                   % 832*12 POD-MODE
                                                   % 2080 telemetry

% physic_layer = [gain antenna [dB], beanwirdh [°], TX power [dBW], RX gain [dB], TX losses [dB], RX losses [dB], LNA NF [dB], Accuracy [°], is sat? 1/0]
gs_S = [26, 7.5, 7, 56, 6, 6, 0.9, 0.5, 0];
gs_U = [12, 30, 18, 40, 5, 5, 0.4, 0.5, 0];
sat_S = [6.5, 80, 0, 0, 4.24, 4.24, 2, 15, 1];
sat_U = [0, 179, 0, 0, 2.16, 2.16, 1, 15, 1];
sat_U_beacon = [0, 179, -6, 0, 2.16, 2.16, 1, 15, 1];

% channel_layer = [frequency [MHz], datarate [bit/s] required e0/n0 [dB], from/to]
ch1 = [2056, 9600, 14, gs_S, sat_S];                % uplink TC
ch2 = [437.555, 9600, 14, gs_U, sat_U];             % uplink TC
ch3 = [2255, 512000, 14, sat_S, gs_S];              % downlink full/science S-BAND
ch4 = [2255, 387000, 14, sat_S, gs_S];              % downlink Test data volume S-BAND
ch5 = [2255, 256000, 14, sat_S, gs_S];              % downlink contigency/TM
ch6 = [438.555, 19200, 14, sat_U, gs_U];            % downlink full UHF
ch7 = [438.555, 9600, 14, sat_U_beacon, gs_U];      % downlink beacon

% add the channel added to this matrix as a new row
links = [ch1; ch2; ch3; ch4; ch5; ch6; ch7];

% link budget implementation
implementation_loss = 1;

% simulation time = (YEAR, MONTH, DAY, HOUR, MINUTE, SECOND)
start = datetime(2027, 5, 1, 0, 0, 0, TimeZone="UTC");
stop = datetime(2027, 5, 11, 0, 0, 0, TimeZone="UTC");
    
% sample rate [s]
sample = 30;

% satellite points to antenna? true/false
sat_point = true;

%% SCENERY CONFIGURATION AND ORBITAL PROPAGATION

% scenario definition
ss = satelliteScenario(start, stop, sample);

% satellite and ground station definition
gs_ss = groundStation(ss, gs(1), gs(2), "MinElevationAngle",gs(3), "Name","C3", "Altitude",gs(4));
sat = satellite(ss, (sat_orbit(1)+6378)*1000, sat_orbit(2), sat_orbit(3), sat_orbit(4), sat_orbit(5), sat_orbit(6), "Name","ELECTRA");

% antennas placing and pointing
% ground station
sat_gimbal = gimbal(sat, MountingLocation = [0,0,1]);

if sat_point
    pointAt(sat_gimbal, gs_ss);
end

% satellite antennas for access calculation
sat_antenna1 = conicalSensor(sat_gimbal, "MaxViewAngle", sat_S(1,2));
sat_antenna2 = conicalSensor(sat_gimbal, "MaxViewAngle", sat_U(1,2));

sat_antenna = [sat_antenna1, sat_antenna2];

% range and elevation calculations
[~,elev_temp,range_temp] = aer(gs_ss,sat);

% elevation and range data cleaning, removing zeros and negative numbers
elev = [];
range = [];
for j = 1:size(elev_temp,2)
    if elev_temp(j) > gs(3)
        elev = [elev, elev_temp(j)];
        range = [range, range_temp(j)];
    end
end

%% LOSSES COMPUTATION

tic
% pre allocation
atmoloss = zeros(size(elev,2),1)';
temperature = zeros(size(elev,2),1)';
pathloss = zeros(size(range,2),1)';
freq_database = [];

% waiting bar
k=0;
y = waitbar(1-(size(elev,2)-k)/size(elev,2), "Attenuation for frequency %s GHz");
waitbar(0,f, "Attenuation calculations, this might take a while...")

% calculation atmospheric losses, temperature, fpsl
for j = 1:size(links,1)
    freq = links(j,1)*1e6;
    pathloss(j,:) = fspl(range(1,:), physconst('LightSpeed')/freq);
    if freq > 1e9
        % to avoid redoing already done calculations
        if ismember(freq, freq_database)
            l = find(freq_database == freq);
            atmoloss(j,:) = atmoloss(l(1),:);
            temperature(j,:) = temperature(l(1),:);
        else
            for i = 1:size(elev,2)
                waitbar(1-(size(elev,2)-i)/size(elev,2), y, sprintf("Attenuation for frequency %s GHz", freq/1e9));
                if elev(i) >= 5
                    cfg = p618Config('Frequency',freq, "ElevationAngle", elev(i), "Latitude", gs(1), "Longitude", gs(2));
                    [loss, ~, tsky] = p618PropagationLosses(cfg, "StationHeight",gs(4)/1000);
                    atmoloss(j,i) = loss.At;
                    temperature(j,i) = tsky;
                else
                    atmoloss(j,i) = 0;
                    temperature(j,i) = 0;
                end
            end
        end
        freq_database(j) = freq;
    else
        atmoloss(j,1:size(elev,2)) = 0;
        temperature(j,1:size(elev,2)) = 0;
    end
    waitbar(1-(size(links,1)-j)/size(links,1),f, "Attenuation calculations, this might take a while...")
end
toc
close(y)

%% LINK BUDGET CALCULATIONS

% pre allocation
EIRP = zeros(size(links,1),3);
ISOTROPIC = zeros(size(links,1),3);
TEMPERATURE_SYS = zeros(size(links,1),3);
GT = zeros(size(links,1),3);
EBN0 = zeros(size(links,1),3);
pointing_loss = zeros(size(links,1),3);
POWER_FLUX_DENS = zeros(size(links,1),3);

waitbar(0,f, "Link Budget Calculation")

% matrix invariant for elevations
for j = 1:size(links,1)
    % waitbar((1-(size(links,1)-j)/size(links,1)),f, "Link Budget Calculation")
    % dividing in adverse, mean, favorable conditions for link budget
    lossvect(j,:) = [max(pathloss(j,:)) + max(atmoloss(j,:)), mean(pathloss(j,:)) + mean(atmoloss(j,:)), min(pathloss(j,:)) + min(atmoloss(j,:))];
    tempvect(j,:) = [max(temperature(j,:)), mean(temperature(j,:)), min(temperature(j,:))];
    psdvect = [10*log10(4*pi*max(range)^2), 10*log10(4*pi*mean(range)^2), 10*log10(4*pi*min(range)^2)];
    % pointing losses
    pointing_loss(j,:) = [(12*(deg2rad(links(j,11))/deg2rad(links(j,5)))^2 + 12*(deg2rad(links(j,20))/deg2rad(links(j,14)))^2)*1.30, 12*(deg2rad(links(j,11))/deg2rad(links(j,5)))^2 + 12*(deg2rad(links(j,20))/deg2rad(links(j,14)))^2, (12*(deg2rad(links(j,11))/deg2rad(links(j,5)))^2 + 12*(deg2rad(links(j,20))/deg2rad(links(j,14)))^2)*0.70];
    % EIRP
    EIRP(j,:) = [(links(j,4)+links(j,6)-links(j,8))-abs((links(j,4)+links(j,6)-links(j,8))*0.30), (links(j,4)+links(j,6)-links(j,8))-abs((links(j,4)+links(j,6)-links(j,8))*0.15), links(j,4)+links(j,6)-links(j,8)];
end

% matrix variant for elevations
for i = 1:size(lossvect,2)
    for j = 1:size(links,1)
    ISOTROPIC(j,:) = EIRP(j) - lossvect(j,i) - implementation_loss - pointing_loss(j,:);
    POWER_FLUX_DENS(j,:) = ISOTROPIC(j,:) - psdvect;
    if tempvect(j,i) == 0
        tempvect(j,i) = 35;
    end
    if links(j, 21) == 1
        tempvect(j,i) = 290;
    end
    TEMPERATURE_SYS(j,:) = tempvect(j,i) + 290*((10^(links(j,19)/10)-1)) + 290*(10^(links(j,18)/10)-1)/10^(links(j,16)/10);
    GT(j,:) = links(j,13) - 10*log10(TEMPERATURE_SYS(j,:));
    EBN0(j,:) = ISOTROPIC(j,:) + GT(j,:) + 228.6 - 10*log10(links(j,2));
    MARGINS = EBN0 - links(:,3);
    LNK_STATS(i,:) = {table(MARGINS, EIRP, ISOTROPIC, POWER_FLUX_DENS, TEMPERATURE_SYS, GT, EBN0, VariableNames=["Margins (dB)", "EIRP (dBW)", "Received (dBW)", "PFD (dBW/m^2)", "Temperature (K)", "G/T (dB/K)",  "Eb/N0 (dB)"])};
    end
end

lossvectpointing = table(lossvect, pointing_loss, VariableNames=["Lossvect (dB)","Pointing (dB)"]);

for j = 1:size(LNK_STATS, 1)
    LNK_STATS{j,1} = [lossvectpointing, LNK_STATS{j}];
end

waitbar(1,f, "Link Budget Calculation")

% MARGINS for inclination
for j = 1:size(links,1)
    for i = 1:size(EIRP,2)
        EBN0_INC = EIRP(j,i) - atmoloss(j,:) - pathloss(j,:) - pointing_loss(j,i) - implementation_loss + links(j,13) - 10*log10(temperature(j,:)+290*((10^(links(j,19)/10)-1)) + 290*(10^(links(j,18)/10)-1)/10^(links(j,16)/10)) + 228.6 - 10*log10(links(j,2));
        MARGINS_INC = EBN0_INC - links(j,3);
        LNK_INC_MRG(j,i) = {MARGINS_INC};
    end
end

%% DATA VOLUME PT.1

waitbar(0,f, "Data Volume and Accesses calculation")

% data volume for each antenna
h = 0;
for i = 1:size(sat_antenna, 2)
    % accesses calculation
    accesstab = accessIntervals(access(sat_antenna(i), gs_ss));
    duration = accesstab.Duration;
    t1 = accesstab.StartTime;
    t2 = accesstab.EndTime;
    
    flight_between_access_i = 0;
    for j = 1:(size(accesstab,1))
        if j == 1
           flight_between_access_i (j,:) = seconds(t1(j) - start);
        elseif j == size(accesstab,1)
            flight_between_access_i (j,:) = flight_between_access_i(j-1);
        else
           flight_between_access_i (j,:) = seconds(t1(j+1)-t2(j));
        end
    end
    
    % compressed payload calculation
    payload_data = (data_science(5).*flight_between_access_i.*data_science(4)).*(1-data_science(3));
    
    % size of frame
    dataframe = data_science(1)*floor(payload_data./data_science(2));
    
    % net data downlinked and data produced between access
    data_to_downlink = (dataframe + payload_data);
    
    % vector time
    timevect = sort([accesstab.StartTime; accesstab.EndTime]);
    
    % necessary for indexing of each downlink channel
    k = [];
    for j = 1:size(links,1)
        if links(j,12) == 1
            k = [k, j];
        end
    end
    
    % residual and downlinked data for each downlink channel
    for m = k
        h = h+1;
        respas = 0;
        data = [];

        down_linkable = [0; duration*links(m, 2)];
        data_to_downlink_pass = [0; data_to_downlink];

        for j = 1:(size(data_to_downlink_pass,1)-1)
            respas = data_to_downlink_pass(j) - down_linkable(j);
            if respas < 0
                respas = 0;
            end
            data_to_downlink_pass(j+1) = data_to_downlink_pass(j+1) + respas;
            data = [data; data_to_downlink_pass(j); respas];
        end
        data_history(:,h) = {data, timevect};
    end
end

waitbar(1,f, "Data Volume and Accesses calculation")
pause(2)
waitbar(1,f, "End Calculations")

%% POST-PROCESSING ANALYSIS

% data volume plot
figure("Name", "SATCOM SIMULATOR");
sgtitle("OBC MEMORY VOLUME");
h=0;
for p = 1:size(data_history, 2)
    % counter for plots
    h=h+1;
    % to iterate subplots (channels) for each antenna
    if h > length(k)
        h=1;
    end
    % indexing and parsing the data history database
    plot_y = data_history{1, p};
    plot_x = data_history{2, p};
    % PLOTTING
    subplot(length(sat_antenna),length(k),p)
    plot(plot_x, plot_y./1e6, "LineWidth", 1.5, "Color", [0 0 0])
    ylabel("Net Mbits")
    xlabel("Time Frame")
    title(sprintf("%s Kbps", string(links(k(h),2)/1e3)))
end

% plot MARGINS for inclination and every link
for j = 1:size(LNK_INC_MRG,1)
    figure;
    yline(3, "LineWidth", 1, "LineStyle","--", "Color", "r")
    ylabel("MARGINS [dB]")
    xlabel(sprintf("Interpolation points with sample rate of %d Hz", sample))
    hold on
    for i = 1:size(LNK_INC_MRG,2)
        plot(LNK_INC_MRG{j,i}, "LineWidth", 1.5)
    end
    legend(["3dB Margin", "Adverse", "Nominal", "Favorable"])
    hold off
end


% csv with stats
for j = 1:size(LNK_STATS,1)
    writetable(LNK_STATS{j}, sprintf("LNK_STATS_%d.xlsx", j))
end

for j = 1:size(LNK_STATS,1)
    disp(LNK_STATS{j})
end

% cool ground track and antenna FOVs
if true
    fieldOfView(sat_antenna(1));
    fieldOfView(sat_antenna(2));
    groundTrack(sat, "LeadTime", 3*struct2table(orbitalElements(sat)).Period);
    satelliteScenarioViewer(ss, "Dimension","2D", "Basemap", "streets_dark", "ShowDetails", true);
end

close waitbar
close(f);
