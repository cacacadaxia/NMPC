%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Minimal main program for the use with nmpc.m
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function minimal_example
    addpath('./nmpcroutine');
    mpciterations = 50;
    N             = 4;
    T             = 0.1;%仿真步长
    tmeasure      = 0.0;
    xmeasure      = [2.0];
    u0            = zeros(1,N);

    [t, x, u] = nmpc(@runningcosts, @terminalcosts, @constraints, ...
         @terminalconstraints, @linearconstraints, @system, ...
         mpciterations, N, T, tmeasure, xmeasure, u0)
    rmpath('./nmpcroutine');%这里的path用来干嘛？

%     size(u)
%     size(x)
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Definition of the NMPC functions
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function cost = runningcosts(t, x, u)%评价函数
    cost = 0;

function cost = terminalcosts(t, x)
    cost = 0;

function [c,ceq] = constraints(t, x, u)%无约束
    c   = [];%针对状态量和控制量
    ceq = [];

function [c,ceq] = terminalconstraints(t, x)%无终端约束
    c   = 0;%终端约束值针对状态量的
    ceq = [];

function [A, b, Aeq, beq, lb, ub] = linearconstraints(t, x, u)
    A   = [];%只是线性约束么？
    b   = [];
    Aeq = [];
    beq = [];
    lb  = [];
    ub  = [];

function y = system(t, x, u, T)
    y = u;%系统中x和y之间的关系