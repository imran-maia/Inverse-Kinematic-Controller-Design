% Function for caculating the trajectory (position, velocity, and acceleration at time, t)

function [s_t,ds_t, dds_t] = Trapezoid(si, sf, ddsc, ti, tf, t)
% Input:
%       si   -> initial value
%       sf   -> final value
%       ddsc -> acceleration
%       ti   -> initial time
%       tf   -> final time
%       t    -> total time
% Output: 
%       s_t  -> position at t
%       ds_t -> velocity at t
%       dds_t -> acceleration at t

% Total duration of trajectory
delta = tf - ti;

% If the input parameters satisfy the condition 
if(abs(ddsc)>= 4*abs(sf-si)/delta^2)
    

% If the input parameters doesnot satisfy the condition
else 
    min_absddsc = 4*abs(sf-si)/delta^2;
    error(['Parameters are wrong! Absolute value of acceleration has to be greater than:', num2str(min_absddsc)]);
end

% Compute the cruise time
tc = (delta)/2-1/2*sqrt((delta^2*ddsc-4*(sf-si))/ddsc); 

% Calculate the tajectory for different time phase
if(t<ti)
    s_t = si;
    ds_t = 0;
    dds_t = 0;

elseif(t>tf)
    s_t = sf;
    ds_t = 0;
    dds_t = 0;

elseif(t>=ti & t<ti+tc)         % acceleration phase
    s_t = si+0.5*ddsc*(t-ti)^2;
    ds_t = ddsc*(t-ti);
    dds_t = ddsc;

elseif(t>=ti+tc & t<tf-tc)      % cruise phase
    s_t = si + ddsc*tc*(t-ti-tc/2);
    ds_t = ddsc*tc;
    dds_t = 0;

elseif(t>=tf-tc  & t<= tf)      % decceleration phase
    s_t = sf-0.5*ddsc*(tf-t)^2;
    ds_t = ddsc*(tf-t);
    dds_t = -ddsc;
end   

end

% End of the function for caculating the trajectory.