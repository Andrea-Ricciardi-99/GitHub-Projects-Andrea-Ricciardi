function [K]=add_spring(igdl_xi,igdl_xj,K,kappa);


K(igdl_xi,igdl_xi)=K(igdl_xi,igdl_xi)+kappa;        %termine diag.principale nodo i
if igdl_xj>0
    K(igdl_xj,igdl_xj)=K(igdl_xj,igdl_xj)+kappa;        %termine diag.principale nodo j
    K(igdl_xi,igdl_xj)=K(igdl_xi,igdl_xj)-kappa;        %termine extradiagonale i-j
    K(igdl_xj,igdl_xi)=K(igdl_xj,igdl_xi)-kappa;        %termine extradiagonale j-i
end