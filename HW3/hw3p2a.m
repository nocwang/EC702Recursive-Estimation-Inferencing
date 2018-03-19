clc;clear;
%close all;
%%
% function [x,P] =KF_update(x,P,C,R,nu)
%     K=P*C'*inv(C*P*C'+R);
%     x=x+K*nu;
%     P=P-K*C*P;
% end
% function [x,P] =KF_predict(x,P,A,G,Q)
%     x=A*x;
%     P=A*P*A'+G*Q*G';
% end
%%
% parameters
% process noise q covariance
q = 1;
% measurement noises vr, va (in radians)
R = (3*pi/180)^2;%diag([vr va vr va]);%Rsqrt = sqrtm(R);
A = [1 1 ;
    0  1 ]; %  position velocity for object;
G = [0.5;1];
Q = q;
%% def C_symb in C_symb.m
% get C=dh/dx given numeric x (7-dim state)
% measurements r
% Y = atan(X) returns the Inverse Tangent (tan-1) of the elements of X. The atan function operates element-wise on arrays. For real elements of X, atan(X) returns values in the interval [-pi/2,pi/2]. For complex values of X, atan(X) returns complex values. All angles are in radians.
syms X x1_sym %Y y1_sym x2_sym xb2 y2_sym yb2 ab2
A1 = atan((20 )/(X - x1_sym));
A1x1 =  diff(A1,X);
% latex(A1x1)
% observations [R1,A1,R2,A2]';  No bias state
% (X,x1_sym,Y,y1_sym,x2_sym,xb2,y2_sym,yb2)
C_sym = [A1x1,0];
%%
% the state are the x,s
% x1 = 0; y1 = 0; ybt1 = 0; xbt1 = 0;
% x2 = 50; y2 = 0; xbt2 = 5; ybt2 = 0; abt2 = 0.3;

% xb1 = 0; yb1 = 0; % needed for linearization.
y_all=load('track.ascii');
Y1=y_all';
T = length(y_all);%50;
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

xp = zeros(2,T);
xu = zeros(2,T);
Pu = zeros(2,2);
xp(:,1) = [60 0]';
Pp = diag([400,1]);
yp = zeros(1,T);

for t = 1:T
    % update cycle
    yp(:,t) = double(subs(A1, {X,x1_sym}, {xp(1,t),4*t}));
    % correct sensor 2 innovations for wrap-around of 2 pi
    %        Use Matlab/Octave function atan2 in the measurement model instead of
    % atan to directly get an answer at range [−π, π].
    nu = Y1(:,t) - yp(:,t);
    if (abs(nu)>pi),
        nu = nu - 2*sign(nu)*pi;
    end  
    % finish an update cycle...
    %% by WTY:    KF_update, KF_predict.
    x=xp(:,t);
    C =subs(C_sym, {X,x1_sym}, {x(1),4*t});
    C = double(C);% or it's slow!!!
    % C =C_sym(x(1),0,x(3),0,50,x(5),0,x(6));
    [xu(:,t),Pu] =KF_update(xp(:,t),Pp,C,R,nu);
    
    % Predict cycle...
    [xp(:,t+1),Pp] =KF_predict(xu(:,t),Pu,A,G,Q);
end

plot(yp);
hold on;
plot(Y1)
% figure;
% subplot(2,2,1);
% plot([e_ekf(1,:)' e_ekf(3,:)']);
% ylim([0,10]);
% subplot(2,2,2)
% plot(e_ekf(5:6,:)');
% ylim([0,10]);
% subplot(2,2,3)
% plot(e_ekf(7,:)');
% ylim([0,0.5]);
%
% save('hw3_2.mat')
