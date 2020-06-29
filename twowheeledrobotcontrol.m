% simulation of cascaded control
clear;clc;
LineWidth=1;

%% Model Process

% State Space model parameters
K = 0.24;%Nm/A
m_c = 3.0;%Kg
l = 0.1;%m
r=0.04825;%m
I_c = 0.0125;%Kg.m^2
g=9.81;%m/s^2
R=3;%?
m_w=0.113;%kg
I_w=0.00001548;%kg.m^2
J=0.0003;
W_1=2*((-1*50^2*J)/(r^2) - m_w)-m_c+(m_c^2*l^2)/(I_c+m_c*l^2);
W_2=W_1 * ((I_c)/(m_c*l)+l);
A_22=(2*50*K^2)/(r^2*R*W_1);
A_42=(2*50*K^2)/(r^2*R*W_2);
A_23=(-1*g*m_c^2*l^2)/(W_1*(I_c+m_c*l^2));
A_43=(-1*g*m_c^2*l^2)/(W_2*((I_c+m_c*l^2)+g*W_1));
A=[0,1,0,0;
    0,A_22,A_23,0;
    0, 0, 0, 1;
    0, A_43,A_43,0];
B2= (-100*K)/(r*R*W_1);
B4= (-100*K)/(r*R*W_2);
B=[0,B2,0,B4]';
C=[1, 0, 0, 0;
    0, 0, 1, 0];
D=[0,0]';
%create a state space model with the matricies A&B
Ts=0.01;
%sys=ss(A,B,C,D,Ts,'StateName',{'vel','acc','p_ang','p_ang_vel'},'InputName','Force');
%step(sys);
[NUM,DEN] = ss2tf(A,B,C,D);
%set sampling time and
Fs=100;
dt=1/Fs;
N=200;
t=dt*(0:N-1);
u = [1 zeros(1,N-1)];
X=[0;0;1.15;0];
y = [zeros(1,N);
    zeros(1,N)];

F=[ r/2, r/2;
    r/l, -r/l];


%% Define Cascaded controller Parameters
open_system('twowheeledrobot')
%Controller1
PID_P1=0.1;
PID_I1=0.01;
PID_D1=0.9;
PID_N1=20;
%Controller2
PID_P2=0.1;
PID_I2=0.05;
PID_D2=10;
PID_N2=200;
%Controller3
PID_P3=10;
PID_I3=0.05;
PID_D3=10;
PID_N3=20;
%check the setpoint behaviour of the control system
%set initial and final values of the step input
i_value=0;
ref_value=0;

i_value1=0;
ref_value1=0;
sim('twowheeledrobot')

figure(1)
plot(ans.Position.Time,ans.Position.Data,'b','LineWidth',LineWidth)
grid on
title('Position Response')
ylabel('position (cm)')
xlabel('time (s)')

figure(2)
plot(ans.pitch_angle.Time,ans.pitch_angle.Data,'b','LineWidth',LineWidth)
grid on
title('Pitch Angle Response')
ylabel('pitch angle (deg)')
xlabel('time (s)')

