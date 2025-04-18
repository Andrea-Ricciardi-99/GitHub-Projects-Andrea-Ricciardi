function [K,rho,feas]=LMI_CT_DeDicont_sector(A,B,C,N,ContStruc, theta)
% Computes, using LMIs, the distributed "state feedback" control law for the continuous-time system, with reference to the control
% information structure specified by 'ContStruc'. It constrains the eigenvalues to stay in a sector, with angle theta, in the 2nd and 3rd
% quadrant.
%
% Inputs:
% - A: system matrix.
% - B: input matrices (i.e., B{1},..., B{N} are the input matrices of the decomposed system, one for each channel).
% - C: output matrices  (i.e., C{1},..., C{N} are the output matrices of the decomposed system, one for each channel, where [Cdec{1}',...,
% Cdec{N}']=I).
% - N: number of subsystems.
% - ContStruc: NxN matrix that specifies the information structure
% constraints (ContStruc(i,j)=1 if communication is allowed between channel
% j to channel i, ContStruc(i,j)=0 otherwise).
% - theta: angle of the constraining sector.
%
% Output:
% - K: structured control gain
% - rho: spectral abscissa of matrix (A+B*K) - note that [C{1}',...,
% C{N}']=I
% - feas: feasibility of the LMI problem (=0 if yes)

Btot=[];
for i=1:N
    m(i)=size(B{i},2);
    n(i)=size(C{i},1);
    Btot=[Btot,B{i}];
end
ntot=size(A,1);
mtot=sum(m);



yalmip clear

gamma_L = sdpvar(1);
alpha_y = sdpvar(1);

if ContStruc==ones(N,N)
    % Centralized design
    Y=sdpvar(ntot);
    L=sdpvar(mtot,ntot);
else
    % Decentralized/distributed design
    Y=[];
    L=sdpvar(mtot,ntot);
    minc=0;
    for i=1:N
        Y=blkdiag(Y,sdpvar(n(i)));
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

LMIconstr=[[sin(theta)*(A*Y+Y*A'+Btot*L+L'*Btot') cos(theta)*(A*Y-Y*A'+Btot*L-L'*Btot');
    cos(theta)*(Y*A'-A*Y+L'*Btot'-Btot*L) sin(theta)*(A*Y+Y*A'+Btot*L+L'*Btot')]<=-1e-2*eye(ntot*2)]+[Y>=1e-2*eye(ntot)]+[Y*A' + A*Y + Btot*L + L'*Btot' + 2 * 0.004 * Y <= -1e-2 * eye(ntot)]+ [[gamma_L*eye(ntot) L'; L eye(ntot)]>=1e-2*eye(ntot*2)] + [[alpha_y*eye(ntot) eye(ntot); eye(ntot) Y]>=1e-2*eye(ntot*2)];
options=sdpsettings('solver','sedumi');
J=optimize(LMIconstr,gamma_L+alpha_y,options);
feas=J.problem;
L=double(L);
Y=double(Y);

K=L/Y;
rho=max(real(eig(A+Btot*K)));