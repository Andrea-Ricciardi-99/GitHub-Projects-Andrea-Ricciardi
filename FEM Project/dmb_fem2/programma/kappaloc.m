function [kloc]=kappaloc(kx,ky,kt);

kloc=zeros(6,6);
kloc(1,1)=kx;
kloc(1,4)=-kx;
kloc(4,1)=-kx;
kloc(4,4)=kx;
kloc(2,2)=ky;
kloc(2,5)=-ky;
kloc(5,2)=-ky;
kloc(5,5)=ky;
kloc(3,3)=kt;
kloc(3,6)=-kt;
kloc(6,3)=-kt;
kloc(6,6)=kt;
