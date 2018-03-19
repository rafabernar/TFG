%
%  SCRIPT TO OUTPUT SIMULATION RESULTS OF CONTROL ALGORITHM FOR AUTONOMOUS
%  DRIVING WHEN INTRODUCING GAUSS-MARKOV CORRELATED ERRORS IN POSITION,
%  VELOCITY AND ATTITUDE MEASUREMENTS. ESCAPE PROJECT. GMV AEROSPACE &
%  DEFENCE. AUTHOR: RAFAEL BERNAR ETCHENIQUE. 2017.
%

%% VARIABLE SETTINGS AND CONTROL PARAMETERS

deltat_array = [0.01 0.02 0.04 0.05 0.1 0.2 0.5];      %Sample time values. [s]

tau_array = [1 5 10 20 25 30];                 %Correlation times for Gauss-Markov position error generation [s]. 
tau_vel_att = 2;                               %Correlation time for velocity and attitude error generation. [s]

s_e  = [0.2 0.7 3  ];                          %Expected across error values used to compute Variance [m].
s_e1 = [0.3 1   3.5];                          %Expected along error values [m].
s_vel_e = 0.5 / 3.6;                           %Expected absolute velocity error. [0.5 km/h]
s_att_e = 0.5 * pi / 180;                      %Expected attitude error. [0.5º]

latency = [0 50 100 300 500 1000];             %Latency in [ms].                         

l_i = length(deltat_array); 
l_j = length(tau_array);
l_k = length(s_e);
l_h = length(latency);

f=2;                                           %Figure #, for graph generation.

performance = {'\sigma Across = 0.2','\sigma Across = 0.7','\sigma Across = 3',}; %Permormance for graph title.
t=150;                                         %Simulation time. More time, less speed.
lat_acc = 3.25;                                %Maximum Comfortable Lateral Acceleration. [m/s^2].

filename = 'SIM_DATA';                         %Set delimited file were data is appended.
%% SIMULATION LOOPS 

% These loops output different simulation results depending on the error
% values set above.

for h= 1:l_h
    for k = 1:l_k
        for j = 1:l_j
           for i = 1:l_i

                    %Set parameters for error generation, latency,
                    %sampling time and correlation. For each iteration.
                    
                    deltat = deltat_array(i);           %Parameter for control signal blocks.             
                    lat = latency(h)*0.001;             %Parameter for latency, Latency Block.
                    tau=tau_array(j);                   %Used to compute alpha (a_cross,a_vel_att)

                    a_cross = exp(-deltat/tau);         %These parameters set the gain of the alpha, alpha1, alpha2, alpha3 Gain blocks in the Error Emulator subsystem.
                    a_long = a_cross;
                    a_vel_att = exp(-deltat/tau_vel_att);

                    s_cross = sqrt(1-a_cross^2)*s_e(k);     %These parameters set the Variance on the Random Number block in the Error Emulator Subsystem
                    s_long = sqrt(1-a_long^2)*s_e1(k);
                    s_vel = sqrt(1-a_vel_att^2)*s_vel_e;
                    s_att = sqrt(1-a_vel_att^2)*s_att_e;

                    %Execute the simulation model
                    
                    sim('simulator_model_PVA_FINAL')
                    
                    % Plot across error against simulation time. 
                    
                    figure(f)
                    subplot(l_j,1,j)
                    plot(across)
                    title(['Latency= ',num2str(lat),'s; Correlation Time=',num2str(tau),' ; Performance: ',performance{k}])
                    ylabel('Across Error [m]')
                    xlabel('Time [s]')
                    legend('Freq = 100 Hz','Freq = 50 Hz','Freq = 25 Hz','Freq = 20 Hz','Freq = 10 Hz','Freq = 5 Hz')
                    hold on
                    grid on
                   
                    % Calculation of RMS error.
%                     
                    acrossrms = acrossrms.^2;
                    alongrms = alongrms.^2;
                    mean_acrossrms = mean(acrossrms);
                    mean_alongrms = mean(alongrms);
                    y_rms = sqrt(mean_acrossrms);
                    x_rms = sqrt(mean_alongrms);
                    
                    %Appending rms data, along with its navigation parameters to a comma-delimited file.
                    
                    A = [y_rms x_rms deltat tau lat k];
                    dlmwrite(filename,A,'delimiter',',','-append');

           end

        end
       f=f+1;       %Opens new figure for each new expected error value.
    end
end    