clear all;
clc;
close all;
A=[0 1;0 0];
B=[0;1/0.5];
C=[1 0];
D=0;
zd=10;
%PID design
Kp = 10; Ki = 5; Kd = 2;
num_c = [Kd, Kp, Ki];
den_c = [1,  0];

Cz_tf   = tf(num_c, den_c);
num_p = 2;          
den_p = [1, 0, 0];   

plant_tf = tf(num_p, den_p);
sys_pid = minreal(ss(feedback(Cz_tf * plant_tf, 1)));

%State Feedback Control
poles = [-2.83+2.1j, -2.83-2.1j];
K=place(A,B,poles);
A_cl=A-B*K;
sys_cl=ss(A_cl,B,C,D);
full=dcgain(sys_cl);
kd=inv(full);
sys_sf=ss(A_cl,B*kd,C,D);

%Simulating
t=0:0.01:20;
[y_pid,t_pid,x_pid]=lsim(sys_pid,ones(length(t),1)*zd,t,zeros(order(sys_pid),1));
[y_sf,t_sf,x_sf]=lsim(sys_sf,ones(length(t),1)*zd,t,zeros(order(sys_sf),1));

%System Info
info_pid=stepinfo(y_pid,t_pid,zd);
info_sf=stepinfo(y_sf,t_sf,zd);
Results=table([info_pid.Overshoot;info_sf.Overshoot],...
    [info_pid.Peak;info_sf.Peak],...
    [info_pid.RiseTime;info_sf.RiseTime],...
    'VariableNames',["Overshoot","Peak","RiseTime"],...
     'RowNames', ["PID Controller", "State Feedback"]);
disp(Results)
% %Control Input U
error_pid=zd-y_pid;
e_integral=cumtrapz(t,error_pid);
e_derivative=gradient(error_pid);

U_pid=Kp*error_pid+Ki*e_integral+Kd*e_derivative;
U_sf=(-K*x_sf')';
%Instaneous Power
P_pid = U_pid .*gradient(y_pid,0.01); 
P_sf = U_sf .* gradient(y_sf,0.01);
%Total Energy
E_pid=trapz(t,U_pid.^2);
E_sf=trapz(t,U_sf.^2);
Energy_results=table([E_pid;E_sf],...
   'VariableNames',{'TotalEnergyConsumption'},...
    'RowNames',{'PID','State Feedback'});
disp(Energy_results)
%Plot
figure;
subplot(2,2,1);
plot(t_pid,y_pid,Linewidth=0.5);
ylabel("Altitude (m)");
xlabel("Time (s)");
title('PID controller Response');
subplot(2,2,2);
plot(t_sf,y_sf,Linewidth=0.5);
ylabel("Altitude (m)");
xlabel("Time (s)");
title('State Feedback Controller Response');

subplot(2,2,3);
plot(t,P_pid,LineWidth=0.5);
ylabel("Power (W)");
xlabel("Time (s)");
title('Energy usuage at every instant by PID controller');
subplot(2,2,4);
plot(t,P_sf,Linewidth=0.5);
ylabel("Power (W)");
xlabel("Time (s)");
title('Energy usuage at every instant by state feedback controller');