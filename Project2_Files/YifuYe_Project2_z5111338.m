
% ---------------------------------------------------------------------
% MTRN4010/2021.T1

% Example program, showing how to read the measurements, from the data file.
% by Jose Guivant. MTRN4010.

% You can modify this example, for solving some parts of the projects.

function YifuYe_Project2_z5111338()
    clear;clc;
    main();
end

function main()
    
    %Here, we load the data
    Data = load('Measurements_AAS01.mat');
    Data = Data.A;
    
    %Calculate gyro bias.
    gyro_bias = get_gyro_bias(Data);

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
    hL1 = plot(angleScan, r, '.');  %For showing all laser points.
    hL2 = plot(0, 0, '.r');         %For showing brillant points, later.
    legend({'points','brilliant points'});
    hold off;
    
    %Second plot, the cartesian view.
    subplot(2,1,2);
    hold on;
    axis([-10,10,-2,12]);
    zoom on; grid on;
    title('showing LiDAR scans (in Cartesian, local frame)'); 
    ylabel('Y (m)'); xlabel('X (m)');
    hL3 = plot(0, 0, '.');  %For showing all laser points.
    hL4 = plot(0, 0, '.r'); %For showing brillant points, later.
    hL4_ooi = plot(0, 0, '.');
    legend({'points','brilliant points'});
    hold off;
    
    %Global view/path
    figure(2);
    hold on;
    axis([-10,10,-2,12]);
    zoom on; grid on;
    title('Dead reckoning path'); 
    ylabel('Y (m)'); xlabel('X (m)');
    hL5 = plot(0, 0,'.r');
    hold off;
    
    i0 = 1;             %You should start at i0 = 1 or close.
    X = [0;0;pi/2] ;	%Initial pose. [x; y; theta]
    t0 = 0.0001*double(Data.t(i0));	%Initial time.  
    L = Data.L;         %Number of samples in this dataset.
    i0 = i0 + 1;
    X_array = zeros(3, Data.L+10);
    for i = i0:L
        t = 0.0001*double(Data.t(i));    %Current sample-i timestanp
        m = Data.Z(:,i);                 %Data, related to sample-i
        
        NewSpeed = double(m(1))*0.001;           %Speed in m/s
        NewAngRate = (double(m(2)) - gyro_bias)*0.01; %Rate in deg/sec, bias~=97.5
        
        indexScan = m(3);           %If indexScan > 1, there is a scan.
        
        %However, you decide the engineering units you want to use.
        fprintf('(%d) measured speed=[%.2f]m/s, heading rate=[%.2f]deg/sec, indexScan=[%d]\n',...
            i, NewSpeed, NewAngRate, indexScan);
        
        dt = t - t0;
        t0 = t;
        
        %Here you could/should/may use a kinematic model, for the small horizon dt.
        %So you keep estimating the state vector X.
        %X(t+dt) = F( X(t), inputs(t),..)
        X_new = update_positionX(X, NewSpeed, deg2rad(NewAngRate), dt);
        X = X_new;
        disp(X);
        X_array(1, i) = X(1);
        X_array(2, i) = X(2);
        X_array(3, i) = X(3);
        
        
        %Now, if there is a LiDAR scan at this time?
        if (indexScan > 1)
            %Extract ranges and intensities, of the 361 "pixels"
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
            %Find high intense-index
            ii = find(I > 0);
            [clust_array, valid_clust]  = get_clusters_pos(cartX(ii), cartY(ii));
            %Update new lasers' plot and brilliant points
            %\/\/\/\/\/\/\/\/\/\
            set(hL1, 'ydata', r);
            set(hL2, 'xdata', angleScan(ii), 'ydata', r(ii));
            %\/\/\/\/\/\/\/\/\/\
            set(hL3, 'xdata', cartX, 'ydata', cartY);
            set(hL4, 'xdata', cartX(ii), 'ydata', cartY(ii));
            %\/\/\/\/\/\/\/\/\/\
            set(hL5, 'xdata', X_array(1, :), 'ydata', X_array(2, :));
            %\/\/\/\/\/\/\/\/\/\
            if (valid_clust == 0)
                set(hL4_ooi, 'xdata', 0, 'ydata', 0, 'Marker', '.', 'MarkerSize', 1);
            else
                set(hL4_ooi, 'xdata', clust_array(1, :), 'ydata', clust_array(2, :),...
                'Marker', 'o', 'MarkerSize', 10);
            end
            
            pause(0);  %Short pause, just to see some animation, more slowly.You may change this.
        end
    end
end

% -------------------------------------------------------

function [pos_array, valid_clust] = get_clusters_pos(Xii, Yii)
    
    valid_clust = 1;
    if (isempty(Xii))
        pos_array = [0; 0];
        valid_clust = 0;
        return;
    end
    if (length(Xii) == 1)
        pos_array = [Xii; Yii];
        return;
    end
    
    %Create the clusters position matrix
    thresh_hold = 0.1; %In meters
    j = 1;
    r = 1;
    for i = 1:(length(Xii) - 1)
        distance = sqrt((Yii(i+1)-Yii(i))^2 + (Xii(i+1)-Xii(i))^2);
        if distance <= thresh_hold
            pos_mat(r, j) = Xii(i);
            pos_mat(r, j+1) = Yii(i);
            pos_mat(r+1, j) = Xii(i+1);
            pos_mat(r+1, j+1) = Yii(i+1);
            r = r + 1;
        else
            j = j + 2;
            r = 1;
        end
    end
    
    %Get Xii and Yii average and insert it into the pos_array
    xn = 1; yn = 1;
    if (rem(length(pos_mat(1, :)), 2) == 0)
        for i = 1:length(pos_mat(1, :))
            if (rem(i, 2) == 1)
                pos_array(1, xn) = mean(nonzeros(pos_mat(:, i)));
                xn = xn + 1;
            else
                pos_array(2, yn) = mean(nonzeros(pos_mat(:, i)));
                yn = yn + 1;
            end
        end
    else
        pos_array = [0; 0];
        valid_clust = 0;
    end
    
end

%Function that can calculate the gyro bias by inputting Data Struct.
function [bias] = get_gyro_bias(Data)
    gyro_data = Data.Z(2,:);
    first_3500 = gyro_data(1:3500);
    bias = mean(first_3500);
end

%Function that can transfer polar to cartesian frame.
function [cart_pos] = polar2cart(ranges, degrees)
    cart_pos = zeros(2, 361);
    %Be careful, ranges is 361 by 1 matrix, do it transpose.
    %Also, function inputs are in degrees, change it to radian.
    cart_pos(1, :) = ranges'.*cos(deg2rad(degrees));
    cart_pos(2, :) = ranges'.*sin(deg2rad(degrees));
end

%Function that update its global position
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


