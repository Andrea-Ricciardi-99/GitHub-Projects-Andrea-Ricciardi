function [kglo,rglo]=loc2glo_molle(alfa,kx,ky,kt,rx,ry,rt);


[kloc]=kappaloc(kx,ky,kt);
[rloc]=kappaloc(rx,ry,rt);

cosalfa=cos(alfa);
sinalfa=sin(alfa);
lam=[cosalfa sinalfa 0;-sinalfa cosalfa 0; 0 0 1];
lambda=zeros(6,6);
lambda(1:3,1:3)=lam;
lambda(4:6,4:6)=lam;

kglo=lambda'*kloc*lambda;
rglo=lambda'*rloc*lambda;
