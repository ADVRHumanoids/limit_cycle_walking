clear all; clc;

syms a01 a11 a21 a31 a02 a12 a22 a32 th1 th1d th2 th3 dth1 dth2 dth3 D C G th

th = [th1 th2 th3];
thd(1)= th1d;

%     
q=[th1;th2;th3];
dq=[dth1;dth2;dth3];


a_leg = [a02 a12 a22 a32];
a_waist = [a01 a11 a21 a31];

f =[eye(3)*dq;inv(D)*(-C*dq-G)]

Ha=[th3-(a01+a11*th1+a21*th1^2+a31*th1^3);
    th2-(-th1+(a02+a12*th1+a22*th1^2+a32*th1^3)*(th1-th1d)*(th1+th1d))];

LfHa1 = jacobian(Ha,q);
LfHa= jacobian(Ha,q)*dq;


dLfHa=jacobian(LfHa,[q;dq]); % need to multiply this quantity times


% Ha matrix
H=sym(zeros(2,1));
H(1,1)=th3-a01-a11*th1-a21*th1^2-a31*th1^3;
H(2,1)=th2+th1-(a02+a12*th1+a22*th1^2+a32*th1^3)*(th1-th1d)*(th1+th1d);

% LfHa matrix
LfH=sym(zeros(2,1));
LfH(1,1)=(-a11-2*a21*th1-3*a31*th1^2)*dth1+dth3;
LfH(2,1)=(1-(a12+2*a22*th1+3*a32*th1^2)*(th1-th1d)*(th1+th1d)-(a02+a12*th1+a22*th1^2+a32*th1^3)*(th1+th1d)-(a02+a12*th1+a22*th1^2+a32*th1^3)*(th1-th1d))*dth1+dth2;

% dLfHa matrix
dLfH=sym(zeros(2,6));
dLfH(1,1)=(-2*a21-6*a31*th1)*dth1;
dLfH(1,4)=-a11-2*a21*th1-3*a31*th1^2;
dLfH(1,6)=1;
dLfH(2,1)=(-(2*a22+6*a32*th1)*(th1-th1d)*(th1+th1d)-2*(a12+2*a22*th1+3*a32*th1^2)*(th1+th1d)-2*(a12+2*a22*th1+3*a32*th1^2)*(th1-th1d)-2*a02-2*a12*th1-2*a22*th1^2-2*a32*th1^3)*dth1;
dLfH(2,4)=1-(a12+2*a22*th1+3*a32*th1^2)*(th1-th1d)*(th1+th1d)-(a02+a12*th1+a22*th1^2+a32*th1^3)*(th1+th1d)-(a02+a12*th1+a22*th1^2+a32*th1^3)*(th1-th1d);
dLfH(2,5)=1;






%output function h
y = [th2-(-th1+(a02 + a12*th1 + a22*th1^2+a32*th1^3)*(th1-th1d)*(th1+th1d));%leg virtual constraint
                                  th3-(a01 + a11*th1 + a21*th1^2 + a31*th1^3)]; %waist virtual constraint

logical(simplify(Ha - flip(y)) == 0) %OK                    
h_dq = jacobian(y,q);
myLfh = h_dq * dq;
logical(simplify(LfH - flip(myLfh)) == 0) %OK
h_dq_dq = [jacobian(myLfh, q), h_dq]; %RIGHT

logical(simplify(dLfH - flip(h_dq_dq)) == 0) %OK
myL2fh  = h_dq_dq * inv(D)*(-C* q_dot(1:n_link) - G);
myLgLfh =  h_dq * inv(D) * B;

