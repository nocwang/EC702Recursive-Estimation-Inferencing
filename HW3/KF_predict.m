function [x,P] =KF_predict(x,P,A,G,Q)
    x=A*x;
    P=A*P*A'+G*Q*G';
end