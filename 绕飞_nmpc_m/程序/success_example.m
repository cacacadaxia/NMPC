function success_example
% example_7_2
% For this example, the origin (0, 0) is stabilized although the running
% cost does not tell the optimization problem to steer the second component
% of the state to 0.
    addpath('./nmpcroutine');
    clear all;
    close all;

    mpciterations = 60;
    N             = 5;
    T             = 1;
    tmeasure      = 0.0;
    xmeasure      = [-1, 1.2];%状态量1*2
    u0            = 0.2*ones(2,N);
    tol_opt       = 1e-8;
    opt_option    = 0;
    iprint        = 5;
    type          = 'difference equation';
    atol_ode_real = 1e-12;
    rtol_ode_real = 1e-12;
    atol_ode_sim  = 1e-4;
    rtol_ode_sim  = 1e-4;

    nmpc(@runningcosts, @terminalcosts, @constraints, ...
         @terminalconstraints, @linearconstraints, @system_dt, ...
         mpciterations, N, T, tmeasure, xmeasure, u0, ...
         tol_opt, opt_option, ...
         type, atol_ode_real, rtol_ode_real, atol_ode_sim, rtol_ode_sim, ...
         iprint, @printHeader, @printClosedloopData, @plotTrajectories);

    rmpath('./nmpcroutine');
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Definition of the NMPC functions
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function cost = runningcosts(t, x, u)%cheak
    cost = x(1)^2+u(1)^2;
end

function cost = terminalcosts(t, x)%check
    cost = 0.0;
end

function [c,ceq] = constraints(t, x, u)%这个约束有什么用,和下面的一个是一样的啊
%     c(1) =  x(1)-1;%-1<x(1)<1
%     c(2) = -x(1)-1;
%     c(3) =  x(2)-1;
%     c(4) = -x(2)-1;
    c(1)=0.25-(x(1)-1)^2-x(2)^2;
    c(2)=x(1)-4.7;
    ceq  = [];
end

function [c,ceq] = terminalconstraints(t, x)
%     c(1) =  x(1)-1;
%     c(2) = -x(1)-1;
%     c(3) =  x(2)-1;
%     c(4) = -x(2)-1;
    ceq  = [];
    c=[];
end

function [A, b, Aeq, beq, lb, ub] = linearconstraints(t, x, u)
% check
    A   = [];
    b   = [];
    Aeq = [];
    beq = [];
    
    lb  = -0.10;
    ub  =  0.10;
end

function y = system_dt(t, x, u, T)%checking
%     y(1) = x*[1;T]+T^2*[u(1)]/2;
%     y(2) = x(2)+T*u(1);
y=x*[1 0;T 1]+u(1)*[T^2/2 T]+u(2)*[T 0];
% Ad=[1 T;0 1];
% Bd=[T^2/2;T];
% y(1)=
% y(2)=
end

% function dx = system_ct(t, x, u, T)
%     dx    = zeros(2,1);
%     dx(1) = x(2);
%     dx(2) = u(1);
% end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Definition of output format
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function printHeader()
    fprintf('   k  |      u(k)        x(1)        x(2)     Time\n');
    fprintf('--------------------------------------------------\n');
end

function printClosedloopData(mpciter, u, x, t_Elapsed)
    fprintf(' %3d  | %+11.6f %+11.6f %+11.6f  %+6.3f', mpciter, ...
            u(1,1), x(1), x(2), t_Elapsed);
end

function plotTrajectories(dynamic, system, T, t0, x0, u, ...
                          atol_ode, rtol_ode, type)
    [x , t_intermediate, x_intermediate] = dynamic(system, T, t0, ...
                                          x0, u, atol_ode, rtol_ode, type);
    figure(1);
        title('x_1/x_2 closed loop trajectory');
        xlabel('x_1');
        ylabel('x_2');
        grid on;
        hold on;
        plot(0, -0.5,'ok','MarkerSize',8);
        plot(x_intermediate(:,1),x_intermediate(:,2),'-ok');
        axis([-5 5 -5 5]);
        axis square;
        hold on;
        x0=1;
        y0=0;
        r=0.5;
        theta=0:pi/50:2*pi;
        x=x0+r*cos(theta);
        y=y0+r*sin(theta);
        plot(x,y,'-',x0,y0,'.');
        axis square;
    figure(2);
        title(['x_1 and x_2 closed loop trajectory']);
        xlabel('n');
        ylabel('x_1(n), x_2(n)');
        grid on;
        hold on;
        plot(t_intermediate,x_intermediate(:,1),'-ok');
        plot(t_intermediate,x_intermediate(:,2),'-ok');
        axis([0 60 -2 5]);
        axis square;


end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%