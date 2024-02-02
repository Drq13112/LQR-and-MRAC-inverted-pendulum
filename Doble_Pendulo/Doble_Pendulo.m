%Initialization

clear all
close all
clc


%System declaration
syms F x dx ddx th1 dth1 ddth1 th2 dth2 ddth2
syms l1 l2 g m0 m1 m2 b

m1 = 0.4;
m2 = 0.5;
m0=1;
l1 = 0.5;
L1=0.5;
L2=0.5 ;
l2 = 0.5;
g = 9.8;
b=0.1;

D = [m0+m1+m2 (m1+m2)*l1*cos(th1) (l2*m2*cos(th2));
    (m1+m2)*l1*cos(th1) (m1+m2)*l1^2 m2*l1*l2*cos(th1-th2);
    m2*l2*cos(th2) m2*l1*l2*cos(th1-th2) m2*l2^2];

C = [b -(m1+m2)*l1*sin(th1)*dth1 -m2*l2*sin(th2)*dth2;
   0 0 m2*l1*l2*sin(th1-th2)*dth2;
   0 -m2*l1*l2*sin(th1-th2)*dth1 0];
G = [0; 
    -(m1+m2)*l1*g*sin(th1); 
    -m2*g*l2*sin(th2)];

H = [1; 0; 0];

dG= [0 0 0;
     0 -l1*g*(m1+m2) 0;
     0 0 -l2*g*m2];

A_=-inv(D)*C;
C_=-inv(D)*dG;
B_=inv(D)*H;

A1 = [0 0 0 1 0 0;
      0 0 0 0 1 0;
      0 0 0 0 0 1;
      C_(1,1) C_(1,2) C_(1,3) A_(1,1) A_(1,2) A_(1,3);
      C_(2,1) C_(2,2) C_(2,3) A_(2,1) A_(2,2) A_(2,3);
      C_(3,1) C_(3,2) C_(3,3) A_(3,1) A_(3,2) A_(3,3)];

B1=[0; 0; 0; B_(1); B_(2); B_(3)];

A = double(subs(A1,[x dx th1 dth1 th2 dth2],[0 0 0 0 0 0]))
B = double(subs(B1,[x dx th1 dth1 th2 dth2],[0 0 0 0 0 0]))

C=[1 0 0 0 0 0;
   0 1 0 0 0 0;
   0 0 1 0 0 0;
   0 0 0 1 0 0;
   0 0 0 0 1 0;
   0 0 0 0 0 1];

D = [0];
Co = ctrb(A,B);
unco = length(A) - rank(Co) % Rank = 6(Full Rank) Hence Controlable.
Ob=obsv(A,C)
unobsv = length(A) - rank(Ob)

%%
% Selecting appropriate Q and R Values
Q = [1 0 0 0 0 0;
     0 1 0 0 0 0;
     0 0 1 0 0 0;
     0 0 0 1 0 0;
     0 0 0 0 1 0;
     0 0 0 0 0 1];

R = 1;


% Gain Matrix
K = lqr(A, B, Q, R)
%K = lqr(A_, B_, Q, R)
disp(eig(A - B*K))
% Lyapunov Indirect Method: All eigen Values are on the left hand side, so system is stable.
 
figure();
[Output] = sim('Doble_Pendulo_LQR',10);
x0_ = Output.yout{1}.Values.Data;
th1_ = Output.yout{3}.Values.Data;
th2_ = Output.yout{5}.Values.Data;
hold on
plot(Output.tout,th1_,'blue')
plot(Output.tout,th2_,'blue')

%%
Q = [1 0 0 0 0 0;
     0 1 0 0 0 0;
     0 0 1 0 0 0;
     0 0 0 10 0 0;
     0 0 0 0 1 0;
     0 0 0 0 0 1];
R = 1;
K = lqr(A, B, Q, R);
    disp(eig(A - B*K))
    [Output] = sim('Doble_Pendulo_LQR',10);
    x0_ = Output.yout{1}.Values.Data;
    th1_ = Output.yout{3}.Values.Data;
    th2_ = Output.yout{5}.Values.Data;
    hold on
    plot(Output.tout,th1_,'red')
    plot(Output.tout,th2_,'red')
    %%
Q = [1 0 0 0 0 0;
     0 1 0 0 0 0;
     0 0 1 0 0 0;
     0 0 0 10 0 0;
     0 0 0 0 10 0;
     0 0 0 0 0 10];
R = 1;
K = lqr(A, B, Q, R);
    disp(eig(A - B*K))
    [Output] = sim('Doble_Pendulo_LQR',10);
    x0_ = Output.yout{1}.Values.Data;
    th1_ = Output.yout{3}.Values.Data;
    th2_ = Output.yout{5}.Values.Data;
    hold on
    plot(Output.tout,th1_,'green')
    plot(Output.tout,th2_,'green')
    %%
Q = [0.1 0 0 0 0 0;
     0 1 0 0 0 0;
     0 0 1 0 0 0;
     0 0 0 1 0 0;
     0 0 0 0 10 0;
     0 0 0 0 0 10];
R = 10;
K = lqr(A, B, Q, R);
    disp(eig(A - B*K))
    [Output] = sim('Doble_Pendulo_LQR',10);
    x0_ = Output.yout{1}.Values.Data;
    th1_ = Output.yout{3}.Values.Data;
    th2_ = Output.yout{5}.Values.Data;
    hold on
    plot(Output.tout,th1_,'black')
    plot(Output.tout,th2_,'black')
