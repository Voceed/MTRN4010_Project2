
% ---------------------------------------------------------------------
% MTRN4010/2021.T1

% Example program, showing how to read the measurements, from the data file.
% by Jose Guivant. MTRN4010.

% You can modify this example, for solving some parts of the projects.

function ShowData2021_Example2()
    
    %Here, we load the data
    Data = load('Measurements_AAS01.mat');
    Data = Data.A; 

    %Here, we create a figure in which we will show the LiDAR scans, soon.
    angleScan = (0:360)/2;
    
    [r,~] = GetRangeAndIntensityFromRawScan(Data.scans(:,1));
    figure(2); clf(); hL = plot(angleScan,r,'.'); 
    zoom on; grid on;
    title('showing LiDAR scans (in polar, local frame)'); 
    ylabel('range (m)'); xlabel('angle (degrees)'); axis([0,180,0,20]); 

    hold on;
    hL2 = plot(0,0,'.r');   %For showing brillant points, later.
    legend({'points','brilliant points'});
    
    i0 = 1;   %You should start at i0 = 1 or close.
    
    X = [0;0;pi/2] ;	%Initial pose. You will need it.
    t0 = 0.0001*double(Data.t(i0));	%Initial time.  
    
    L = Data.L;	%Number of samples in this dataset.
    i0 = i0 + 1;
    for i = i0:L
        t = 0.0001*double(Data.t(i));
        m = Data.Z(:,i);    %Data, related to sample-i
        
        NewSpeed = double(m(1))*0.001;          %Speed in m/s
        NewAngRate = double(m(2))*0.01;         %Rate in deg/sec
        
        %However, you decide the engineering units you want to use.
        fprintf('(%d) measured speed [%.2f]m/s, heading rate=[%.2f]deg/sec\n',i,NewSpeed,NewAngRate);
        
        indexScan = m(3);  
        
        dt = t-t0;
        t0 = t;
        
        %Here you could/should/may use a kinematic model, for the small horizon dt.
        %So you keep estimating the state vector X.
        %X(t+dt) = F( X(t), inputs(t),..)        
            
        
        %Now, if there is a LiDAR scan at this time?
        if (indexScan > 1)
            
            %Extract ranges and intensities, of the 361 "pixels"
            [r,I] = GetRangeAndIntensityFromRawScan(Data.scans(:, indexScan));
            
            ProcessLiDAR(r, I, X);
            %Here you may process the LiDAR
            %data. variable "X" is the current pose of the platform
            %(assuming you keep estimating it). If your procesing of the
            %LiDAR data does require it.

            %If we want to show it...
            set(hL,'ydata',r);
            %Which points do have intensity > 0 ?
            ii = find(I>0);
            set(hL2,'xdata',angleScan(ii),'ydata',r(ii));
            
            pause(0.015);  %Short pause, just to see some animation, more slowly.You may change this.
            
        end
    end
end 

% -------------------------------------------------------

%Function for parsing the binary scan; extracting ranges and intensities of reflections.
function [r,I] = GetRangeAndIntensityFromRawScan(scan)
    r = 0.01*single(bitand(scan,8191));
    I = bitshift(scan,-13);      
    %Bits [0:12] are range, bits [13:15] intensity
end


function ProcessLiDAR(r, I, X)
	%Whatever you want/need to do, can be done here.
end
% ---------------------------------------------------------------------

% --- END of example.
% Questions: ask the lecturer : j.guivant@unsw.edu.au
% or ask the demonstrators.

% ---------------------------------------------------------------------


