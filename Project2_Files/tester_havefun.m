

figure(1);
hold on;
axis([-50,50,-50,50]);
zoom on; grid on;
title('Dead reckoning path'); 
ylabel('Y (m)'); xlabel('X (m)');
X = [-20 -10 10 20]; Y = [0 0 0 0];
hL1 = circles(X, Y, 2);
hold off;
hold on;
X = [-20 -10 10 20]; Y = [10 20 30 40];
remov_circles(hL1);
hL1 = circles(X, Y, 2);
hold off;
hold on;
X = [-20 -10 10 20 0]; Y = [40 30 20 10 0];
remov_circles(hL1);
hL1 = circles(X, Y, 2);
hold off;

function hL = new_circles(hL, X, Y, r)
    hold on;
    hL_dup = zeros(1:length(X));
    for i = 1:length(X)
        hL_dup(i) = circles(X(i), Y(i), r);
        delete(hL(i));
        hL(i) = hL_dup(i);
    end
    hold off;
end

function hL = remov_circles(hL)
    hold on;
    for i = 1:length(hL)
        delete(hL(i));
    end
    hold off;
end


function h = circles(X, Y, r)
    hold on;
    th = 0:pi/50:2*pi;
    h = zeros(1:length(X));
    for i = 1:length(X)
        xunit = r*cos(th) + X(i);
        yunit = r*sin(th) + Y(i);
        h(i) = plot(xunit, yunit, 'r');
    end
    hold off;
end