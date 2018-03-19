function C =C_symb(x)%(X,x1_sym,Y,y1_sym,x2_sym,xb2,y2_sym,yb2)
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
C(X,x1_sym,Y,y1_sym,x2_sym,xb2,y2_sym,yb2) = [R1x1 0 R1y1 0 0 0  0;
     A1x1 0 A1y1 0 0 0  0;
     R2x2 0 R2y2 0 R2xb2 R2yb2  0;
     A2x2 0 A2y2 0 A2xb2 A2yb2  1];   
 C =C(x(1),0,x(3),0,50,x(5),0,x(6));
 %x=x0(:,1)
end    