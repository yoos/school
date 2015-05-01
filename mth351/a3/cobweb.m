function x = cobweb(fcnName, domain, initVal, nStart, nStop)
% COBWEB     A function iterator that generates a cobweb.
%
% Syntax: x = cobweb(fcnName, domain, initVal, nStart, nStop)
%
%            fcnName - The function to be iterated,  
%            domain  - The domain of the x variable,
%            initVal - The inital x value(s); can be a vector,
%            nStart  - The first iterate to be plotted,
%            nStop   - The last iterate tp be plotted.
%            x       - The forward orbit of 'initVal'.
%
% Example calls: 
% >> cobweb('3.8*x*(1-x)', [0 1], 0.3, 1, 40);
% >> cobweb('x+0.4*sin(2*x)', [0 2*pi], [0.1 3 3.2 6.1], 1, 15);
% >> cobweb('mod(2*x,1)', [0, .99], 0.401, 1, 50);
% >> cobweb('(x < 0.5)*2*x + (x >= 0.5)*2*(1 - x)',[0, 1],0.401,1,50);
 
% Author: Warwick Tucker
% Email : warwick@math.uu.se
% Edited: Thu Jan 22 14:14:32 CET 2004

% Clear the plot widow.
clf;

% Get input data (if any).
if (nargin == 0)
    %fcnName = 'sqrt(x+1)';
    %fcnName = 'x^2-1';
    fcnName = '(x^2+1)/(2*x-1)';
    domain  = [1 2];
    initVal = 1.5;
    nStart  = 1;
    nStop   = 10;
end

% Non-zero only for pedagogical reasons.
snoozeTime = 0.00;

% Start the clock...
tic

% Compute the iterates.
f    = inline(vectorize(fcnName));
x    = zeros(length(initVal),nStop);
x    = initVal;
for n = 1:nStop-1
    x(n+1,:) = f(x(n,:));
end

% Set up the graphics.
h=figure(1);                    % Without these two lines, a MATLAB 
set(h,'doublebuffer','on');     % bug gives an annoying flicker.

xMin = domain(1); xMax = domain(2);
xGrid = linspace(xMin, xMax, 777);  
plot(xGrid,f(xGrid),'r-',...
     'LineWidth',2);            % Plot the user-specified function.
hold on
plot(xGrid, xGrid,'k-');        % Plot the diagonal.
axis([xMin xMax xMin xMax]);    % Scale the picture, and 
axis('square');                 % ensure the plot is square.
title(strcat(['f(x) = ' fcnName '.'], ...
             [' Showing iterates ' num2str(nStart)], ...
             [' to ' num2str(nStop) '.']), 'FontWeight','bold');
wBar = waitbar(0, 'Plotting the iterates...');

for n = nStart:nStop
    waitbar((n - nStart)/(nStop - nStart))
    for k = 1:length(initVal)
        if (n > 1) 
            xplot = [x(n-1,k) x(n,k)];
            yplot = [x(n,k) x(n,k)];
            plot(xplot,yplot,'b:')  % Draw horizontal part of cobweb.
            pause(snoozeTime)
        end
        if (n < nStop)
            xplot = [x(n,k) x(n,k)];
            yplot = [x(n,k) x(n+1,k)];
            plot(xplot,yplot,'b')   % Draw vertical part of cobweb.
            pause(snoozeTime)
        end
    end
end 
close(wBar);
disp(['Elapsed time: ' num2str(toc) ' seconds.'])
hold off
