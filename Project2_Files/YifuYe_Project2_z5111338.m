
% ---------------------------------------------------------------------
% MTRN4010/2021.T1

% Example program, showing how to read the measurements, from the data file.
% by Jose Guivant. MTRN4010.

% You can modify this example, for solving some parts of the projects.

function YifuYe_Project2_z5111338()
    clc;clear;
    main();
end

function main()
    
    %Here, we load the data
    Data = load('Measurements_AAS01.mat');
    Data = Data.A; 

    %Here, we create X-span for figures.
    angleScan = (0:360)/2;      %Polar
    
    %Get ranges from LIDAR
    [r,~] = GetRangeAndIntensityFromRawScan(Data.scans(:,1));
    %Open figure(1), and set its window properties.
    figure(1); clf();
    set(figure(1), 'Position', [700,150,600,750]);
    
    %First plot, the polar view.
    subplot(2,1,1);
    hold on;
    axis([0,180,0,20]);
    zoom on; grid on;
    title('showing LiDAR scans (in polar, local frame)'); 
    ylabel('range (m)'); xlabel('angle (degrees)');
    legend({'points','brilliant points'});
    hL1 = plot(angleScan, r, '.');  %For showing all laser points.
    hL2 = plot(0, 0, '.r');         %For showing brillant points, later.
    hold off;
    
    %Second plot, the cartesian view.
    subplot(2,1,2);
    hold on;
    axis([-10,10,-2,12]);
    zoom on; grid on;
    title('showing LiDAR scans (in Cartesian, local frame)'); 
    ylabel('Y (m)'); xlabel('X (m)');
    legend({'points','brilliant points'});
    hL3 = plot(0, 0, '.');  %For showing all laser points.
    hL4 = plot(0, 0, '.r'); %For showing brillant points, later.
    hold off;
    
    i0 = 1;             %You should start at i0 = 1 or close.
    X = [0;0;pi/2] ;	%Initial pose. [x; y; theta]
    t0 = 0.001*double(Data.t(i0));	%Initial time.  
    L = Data.L;         %Number of samples in this dataset.
    i0 = i0 + 1;
    for i = i0:L
        t = 0.001*double(Data.t(i));    %Current sample-i timestanp
        m = Data.Z(:,i);                %Data, related to sample-i
        
        NewSpeed = double(m(1))*0.001;          %Speed in m/s
        NewAngRate = double(m(2))*0.01;         %Rate in deg/sec
        %Filter out flickering noise between -2 to 2 degrees.
        if abs(NewAngRate) < 2
            NewAngRate = 0;
        end
        indexScan = m(3);           %If indexScan > 1, there is a scan.
        
        %However, you decide the engineering units you want to use.
        fprintf('(%d) measured speed=[%.2f]m/s, heading rate=[%.2f]deg/sec, indexScan=[%d]\n',...
            i, NewSpeed, NewAngRate, indexScan);
        
        dt = t - t0;
        t0 = t;
        
        %Here you could/should/may use a kinematic model, for the small horizon dt.
        %So you keep estimating the state vector X.
        %X(t+dt) = F( X(t), inputs(t),..)
        X = update_positionX(X, NewSpeed, deg2rad(NewAngRate), dt);
        %disp(X);
        
        %Now, if there is a LiDAR scan at this time?
        if (indexScan > 1)
            %Extract ranges and intensities, of the 361 "pixels"
            %flip function applied to the raw data, not sure??
            [r,I] = GetRangeAndIntensityFromRawScan(Data.scans(:, indexScan));
            
            ProcessLiDAR(r, I, X);
            %Here you may process the LiDAR
            %data. variable "X" is the current pose of the platform
            %(assuming you keep estimating it). If your procesing of the
            %LiDAR data does require it.
            
            %Polar to Cart
            cartPos = polar2cart(r, angleScan);
            cartX = cartPos(1, :);
            cartY = cartPos(2, :);
            %Update new lasers' plot
            set(hL1, 'ydata', r);
            set(hL3, 'xdata', cartX, 'ydata', cartY);
            %Update the brilliant plot
            ii = find(I > 0);
            set(hL2, 'xdata', angleScan(ii), 'ydata', r(ii));
            set(hL4, 'xdata', cartX(ii), 'ydata', cartY(ii));
            
            pause(0.001);  %Short pause, just to see some animation, more slowly.You may change this.
            
        end
    end
end

% -------------------------------------------------------
%Function that can transfer polar to cartesian frame.
function [cart_pos] = polar2cart(ranges, degrees)
    cart_pos = zeros(2, 361);
    %Be careful, ranges is 361 by 1 matrix, do it transpose.
    %Also, function inputs are in degrees, change it to radian.
    cart_pos(1, :) = ranges'.*cos(deg2rad(degrees));
    cart_pos(2, :) = ranges'.*sin(deg2rad(degrees));
end

%Function that update its global position (STILL WORKING ON IT)
function [X_new] = update_positionX(X_old, vel, omega, dt)
    theta = X_old(3);
    dx = vel*cos(theta)*dt;
    dy = vel*sin(theta)*dt;
    dtheta = omega*dt;
    X_new = X_old + [dx; dy; dtheta];
end

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


