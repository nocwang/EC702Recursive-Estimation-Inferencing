
clc;clear;
%close all;
%%
function [x,P] =KF_update(x,P,C,R,nu)
    K=P*C'*inv(C*P*C'+R);
    x=x+K*nu;
    P=P-K*C*P;
end
function [x,P] =KF_predict(x,P,A,G,Q)
    x=A*x;
    P=A*P*A'+G*Q*G';
end

%%
dt = 1;

% parameters
% process noise q covariance
q = 1;

% measurement noises vr, va (in radians)
vr = 4; va = .01;  %  2 units in range, 6 degrees in angle.  

A1 = [1 dt 0 0;
    0  1  0 0;
    0 0 1 dt;
    0 0 0 1]; %  position velocity for object;

A = [A1 zeros(4,3);zeros(3,4) eye(3)]; % plus position and angle biases;

G = eye(7);

Q1 = [dt^3/3  dt^2/2;
      dt^2/2  dt];
  
Q = q*[Q1 zeros(2,5);
     zeros(2,2) Q1 zeros(2,3);
     zeros(3,4) 0.01*eye(3)];
Q(7,7) = 0.0009;

  
R = diag([vr va vr va]);
Rsqrt = sqrtm(R);

%% def C_symb in C_symb.m
 % get C=dh/dx given numeric x (7-dim state)
% measurements r
syms X x1_sym Y y1_sym x2_sym xb2 y2_sym yb2 ab2
R1 = sqrt((X - x1_sym )^2+ (Y - y1_sym )^2);
R2 = sqrt((X - x2_sym - xb2)^2+ (Y - y2_sym -yb2 )^2);
A1 = atan((Y - y1_sym )/(X - x1_sym));
A2 = atan((Y - y2_sym - yb2)/(X - x2_sym - xb2))+ab2;
R1x1 = diff(R1,X);
R1y1 = diff(R1,Y);
A1x1 =  diff(A1,X);
A1y1 = diff(A1,Y);

R2x2 = diff(R2,X);
R2xb2 = diff(R2,xb2);
R2y2 = diff(R2,Y);
R2yb2 = diff(R2,yb2);
A2x2 = diff(A2,X);
A2xb2 = diff(A2,xb2);
A2y2 = diff(A2,Y);
A2yb2 = diff(A2,yb2);
% observations [R1,A1,R2,A2]';  No bias state 
% C = [R1x1 0 R1y1 0 0 0  0;
%      A1x1 0 A1y1 0 0 0  0;
%      R2x2 0 R2y2 0 R2xb2 R2yb2  0;
%      A2x2 0 A2y2 0 A2xb2 A2yb2  1];
% (X,x1_sym,Y,y1_sym,x2_sym,xb2,y2_sym,yb2)
C_sym = [R1x1 0 R1y1 0 0 0  0;
     A1x1 0 A1y1 0 0 0  0;
     R2x2 0 R2y2 0 R2xb2 R2yb2  0;
     A2x2 0 A2y2 0 A2xb2 A2yb2  1]; 
 %%
% Assume sensor 1 is at (0,0), no bias. Sensor 2 is at (50,0) and has bias
% the state are the px, vx, py, vy, xbt2,ybt2, abt2 (radians)
x1 = 0; y1 = 0; ybt1 = 0; xbt1 = 0;
x2 = 50; y2 = 0; xbt2 = 5; ybt2 = 0; abt2 = 0.3;

xb1 = 0; yb1 = 0; % needed for linearization.

T = 50;
%Trajectory:  Initial position between the two sensors, moving straight up,
%no process noise is added here...but your filter won't know that!!!
x0 = zeros(7,T);
x0(:,1) = [25 0 10 1 xbt2 ybt2 abt2]';
for t = 2:T
    x0(:,t) = A*x0(:,t-1);
end

e_ekf = zeros(7,T);

% MONTE CARLO LOOP
MC = 40%1%
randn('state',0);
for mc=1:MC
    mc
    % Generate measurements, stored as a matrix with each column a
    % different time vector:
%     Y1 = zeros(4,T);
    V = randn(4,T);

    Y1 =   [sqrt((x0(1,:) - x1).^2+ (x0(3,:) - y1 ).^2);
        atan2((x0(3,:) - y1),(x0(1,:) - x1));
        sqrt((x0(1,:)  - x2 - x0(5,:)).^2+ (x0(3,:) - y2 - x0(6,:)).^2);
        x0(7,:)+ atan2((x0(3,:) - y2 - x0(6,:)),(x0(1,:)  - x2 - x0(5,:)))];
    Y1 = Y1 + 1*Rsqrt*V;

   % Insert your favorite EKF here...The predicted estimates will come out
   % in variable xp, the uptdated estimates in variable xu
   %%% NOTE: The commente code below is to give you a hint as to what to
   %%% do.  One issue you need to worry about is that angles wrap around,
   %%% so you need to be consistent in defining angles: the difference
   %%% between pi and -pi is zero, not 2pi!  The commented code below is to
   %%% help you write your EKF...Here is what you have to do: if the
   %%% predicted innovations angle has magnitude greater than pi, map it
   %%% into [-pi,pi].  You can choose to put this into a function if you
   %%% want
 
   xp = zeros(7,T);
   xu = zeros(7,T);
%    Pp= zeros(7,7);
   Pu = zeros(7,7);
   xp(:,1) = [25 0 0 1 0  0 0]';
   Pp = diag([100, 100, 100, 100, 100, 100, 0.4]);
   yp = zeros(4,T);
   
   for t = 1:T
       % update cycle
       yp(:,t) = [sqrt((xp(1,t) - x1).^2+ (xp(3,t) - y1 ).^2);
           atan2((xp(3,t) - y1),(xp(1,t) - x1));
           sqrt((xp(1,t)  - x2 - xp(5,t)).^2+ (xp(3,t) - y2 - xp(6,t)).^2);
           xp(7,t)+ atan2(xp(3,t) - y2 - xp(6,t),xp(1,t)- x2 - xp(5,t))];
       
       % correct sensor 2 innovations for wrap-around of 2 pi
%        Use Matlab/Octave function atan2 in the measurement model instead of
% atan to directly get an answer at range [−π, π].
       nu = Y1(:,t) - yp(:,t);
       if (abs(nu(2))>pi),
           nu(2) = nu(2) - 2*sign(nu(2))*pi;
       end
       if (abs(nu(4))>pi),
           nu(4) = nu(4) - 2 * sign(nu(4))*pi;
       end
       
       % finish an update cycle...
       %% by WTY:   C_symb.m, KF_update, KF_predict.
%        C =C_symb(xp(:,t));%def C_symb in C_symb.m
        x=xp(:,t);
        C =subs(C_sym, {X,x1_sym,Y,y1_sym,x2_sym,xb2,y2_sym,yb2}, {x(1),0,x(3),0,50,x(5),0,x(6)});
        C = double(C);% or it's slow!!!
        % C =C_sym(x(1),0,x(3),0,50,x(5),0,x(6));
       [xu(:,t),Pu] =KF_update(xp(:,t),Pp,C,R,nu);
       
       % Predict cycle...
       [xp(:,t+1),Pp] =KF_predict(xu(:,t),Pu,A,G,Q);
   end

%   This is the key output: it computes the absolute value of the error at
%   each time:
    e_ekf = e_ekf + abs(xu - x0); % this is the real error, since x0 is the truth 
end
e_ekf = e_ekf/MC;
figure;
subplot(2,2,1);
plot([e_ekf(1,:)' e_ekf(3,:)']);
ylim([0,10]);
subplot(2,2,2)
plot(e_ekf(5:6,:)');
ylim([0,10]);
subplot(2,2,3)
plot(e_ekf(7,:)');
ylim([0,0.5]);

save('hw3_4.mat')
