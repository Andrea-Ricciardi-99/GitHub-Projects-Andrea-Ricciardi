  function [mG,kG] = el_fun (l,m,EA,T,alfa)

  % matrice di massa nel s.d.r. locale
  mL = m*l*[ 1./3.   0.   1./6.    0.
               0   1./3.    0    1./6.
             1./6.   0.   1./3.    0.  
               0   1./6.    0    1./3. ] ;

  % matrice di rigidezza nel s.d.r. locale
  % contributo della def. assiale
  kL_ax = EA/l* [ 1 0 -1 0
                  0 0  0 0 
                 -1 0  1 0 
                  0 0  0 0 ] ;

  % contributo della def. flessionale
  kL_tr = T/l * [ 0  0  0  0
                  0  1  0 -1 
                  0  0  0  0 
                  0 -1  0  1 ] ;

   kL = kL_ax+kL_tr ;

  % trasformazione s.d.r. locale --> s.d.r. globale
  % costruzione matrice lambda traf. 3x3
   lambda = [ cos(alfa) sin(alfa)
             -sin(alfa) cos(alfa)] ;

  % costruzione matrice Lambda traf. 6x6
  Lambda = [ lambda     zeros(2)
             zeros(2)   lambda      ] ;

 mG = Lambda' * mL * Lambda ;
 kG = Lambda' * kL * Lambda ;
