% MTRN4010/2021.T1

% Example program, showing how to read the measurements, from the data
% file.
% see "ShowData2021_Example2.m"


%Jose Guivant. MTRN4010


function Main()

Data=load('Measurements_AAS01.mat');       %We load the data
Data=Data.A; 

    t=Data.t;                                                 % sample times, {t}.     
    t=double(t)*0.0001;                             %scale time to seconds; original data is integer type, 1 count = 0.1 ms.

    %t0=t(1); 
    
    %Data.Z(1,:): speed from UGV 's encoder. Experessed in mm/sec 
    %Data.Z(2,:): rate provided by GyroZ. Integer, 1 count=0.01 degree/second.  
    
   % Scale speed to m/s ; originally in mm/s, integer type.
   % scale ehading rate to degrees/s ; originally in integer type, [1 count = 1/100 degrees/sec].
    wz=double(Data.Z(2,:))/100;                    % GyroZ readings, "{Wz(t)}"
    v=double(Data.Z(1,:))/1000;                     % speed readings   "{v(t)}" 
   
    
    
% plot those measurements.
figure(1) ; clf();
subplot(211) ; plot(t,v);  xlabel('time (seconds'); ylabel('speed (m/s)');
subplot(212) ; plot(t,wz); xlabel('time (seconds'); ylabel('angular rate (degrees/s)');



 ii = find(Data.Z(3,:)>0);
 timeL = t(ii);
 nL = numel(ii);
 
 
 angleL = [0:360]/2;
 
 scan = single(Data.scans(:,1))*0.01;
 figure(2) ; clf(); hL=plot(angleL,scan,'.') ;  zoom on; grid on;
 title('showing LiDAR scans');
 ylabel('range (m)'); xlabel('angle (degrees)');
 axis([0,180,0,20]);
 
 % show all the scans
 
 
 % show LiDAR scans (animation)
 
 u0=200;
 for u=u0:nL
 i = ii(u);
     scan = single(Data.scans(:,i))*0.01;
    set(hL,'ydata',scan);
     pause(0.1);
 end;
  
 
end 

% -------------------------------------------------------


