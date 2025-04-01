function [K,rho,feas]=LMI_DT_DeDicont_H2(F,G,H,N,ContStruc,q,r)
% Computes, using LMIs, the distributed "state feedback" control law for the discrete-time system, with reference to the control
% information structure specified by 'ContStruc'. It minimizes the H2 norm of the Gzw transfer function. 
%
% Inputs:
% - F: system matrix.
% - G: input matrices (i.e., G{1},..., G{N} are the input matrices of the decomposed system, one for each channel).
% - H: output matrices  (i.e., H{1},..., H{N} are the output matrices of the decomposed system, one for each channel, where [Hdec{1}',...,
% Hdec{N}']=I).
% - N: number of subsystems.
% - ContStruc: NxN matrix that specifies the information structure
% constraints (ContStruc(i,j)=1 if communication is allowed between channel
% j to channel i, ContStruc(i,j)=0 otherwise).
%
% Output:
% - K: structured control gain
% - rho: spectral radius of matrix (F+G*K) - note that [H{1}',...,
% H{N}']=I
% - feas: feasibility of the LMI problem (=0 if yes)

Gtot=[];
for i=1:N
    m(i)=size(G{i},2);
    n(i)=size(H{i},1);
    Gtot=[Gtot,G{i}];
end
ntot=size(F,1);
mtot=sum(m);

yalmip clear

S=sdpvar(ntot*2);

if ContStruc==ones(N,N)
    % Centralized design
    P=sdpvar(ntot);
    L=sdpvar(mtot,ntot);
else
    % Dentralized/distributed design
    P=[];
    L=sdpvar(mtot,ntot);
    minc=0;
    for i=1:N
        P=blkdiag(P,sdpvar(n(i)));
        ninc=0;
        for j=1:N
            if ContStruc(i,j)==0
                L(minc+1:minc+m(i),ninc+1:ninc+n(j))=zeros(m(i),n(j));
            end
            ninc=ninc+n(j);
        end
        minc=minc+m(i);
    end
end

H = [sqrt(q) 0 0 0;
    0 sqrt(q) 0 0;
    0 0 sqrt(q) 0;
    0 0 0 sqrt(q);
    0 0 0 0;
    0 0 0 0;
    0 0 0 0;
    0 0 0 0];

I = [0 0 0 0;
    0 0 0 0;
    0 0 0 0;
    0 0 0 0;
    sqrt(r) 0 0 0;
    0 sqrt(r) 0 0;
    0 0 sqrt(r) 0;
    0 0 0 sqrt(r)];

LMIconstr=[[S H*P+I*L;
    (H*P+I*L)' P]>=1e-2*eye(ntot*3)]+[[P-F*P*F'-F*L'*Gtot'-Gtot*L*F'-eye(4)*eye(4) Gtot*L; 
                                        (Gtot*L)' P]>=1e-2*eye(ntot*2)];
options=sdpsettings('solver','sedumi');
J=optimize(LMIconstr,trace(S),options);
feas=J.problem;
L=double(L);
P=double(P);

K=L/P;
rho=max(abs(eig(F+Gtot*K)));