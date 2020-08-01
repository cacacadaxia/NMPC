












function test_fuck


    atol_ode  = 1e-4;
    rtol_ode  = 1e-4;
    t0=0;
    T=10;
    x0=randn(6,1);
    u=randn(3,1);


options = odeset('AbsTol', atol_ode, 'RelTol', rtol_ode);
[t_intermediate,x_intermediate] = ode45(system, ...
    [t0, t0+T], x0, options);




end



function dx=system(t,x)
A=randn(6,6);
B=randn(6,3);
dx=A*x;
end