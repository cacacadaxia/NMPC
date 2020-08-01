%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Minimal main program for the use with nmpc.m
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function minimal_example
    addpath('./nmpcroutine');
    mpciterations = 50;
    N             = 4;
    T             = 0.1;%���沽��
    tmeasure      = 0.0;
    xmeasure      = [2.0];
    u0            = zeros(1,N);

    [t, x, u] = nmpc(@runningcosts, @terminalcosts, @constraints, ...
         @terminalconstraints, @linearconstraints, @system, ...
         mpciterations, N, T, tmeasure, xmeasure, u0)
    rmpath('./nmpcroutine');%�����path�������

%     size(u)
%     size(x)
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Definition of the NMPC functions
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function cost = runningcosts(t, x, u)%���ۺ���
    cost = 0;

function cost = terminalcosts(t, x)
    cost = 0;

function [c,ceq] = constraints(t, x, u)%��Լ��
    c   = [];%���״̬���Ϳ�����
    ceq = [];

function [c,ceq] = terminalconstraints(t, x)%���ն�Լ��
    c   = 0;%�ն�Լ��ֵ���״̬����
    ceq = [];

function [A, b, Aeq, beq, lb, ub] = linearconstraints(t, x, u)
    A   = [];%ֻ������Լ��ô��
    b   = [];
    Aeq = [];
    beq = [];
    lb  = [];
    ub  = [];

function y = system(t, x, u, T)
    y = u;%ϵͳ��x��y֮��Ĺ�ϵ