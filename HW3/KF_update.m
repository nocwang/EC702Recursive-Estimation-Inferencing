function [x,P] =KF_update(x,P,C,R,nu)
    K=P*C'*inv(C*P*C'+R);
    x=x+K*nu;
    P=P-K*C*P;
end