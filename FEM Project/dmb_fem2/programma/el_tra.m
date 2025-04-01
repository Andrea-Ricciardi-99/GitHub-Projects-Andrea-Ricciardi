  function [mG,kG] = el_tra (l,m,EA,EJ,alfa)

  % matrice di massa nel s.d.r. locale
  mL = m*l*[ 1./3. 0.          0.          1./6.  0.         0.
	        0.    13./35.     11.*l/210. 0.     9./70.     -13*l/420.
	        0.    11.*l/210. l^2/105.    0.     13*l/420. -l^2/140.
	        1./6. 0.          0.          1./3.  0.         0.
	        0.    9./70.      13*l/420.  0.     13./35.    -11.*l/210.
	        0.    -13*l/420.   -l^2/140.   0.     -11.*l/210. l^2/105.   ] ;

  % matrice di rigidezza nel s.d.r. locale
  % contributo della def. assiale
  kL_ax = EA/l* [ 1 0 0 -1 0 0
		 0 0 0  0 0 0 
		 0 0 0  0 0 0 
		-1 0 0  1 0 0 
		 0 0 0  0 0 0 
		 0 0 0  0 0 0 ] ; 

  % contributo della def. flessionale
  kL_fl = EJ * [ 0.    0.       0.      0.    0.       0.     
		         0.  +12./l^3  6./l^2   0.  -12./l^3  6./l^2
                 0.   6./l^2  +4./l     0.   -6./l^2  +2./l
		         0.    0.       0.      0.    0.       0. 
		         0.  -12./l^3  -6./l^2  0.  +12./l^3  -6./l^2
		         0.   6./l^2  +2./l    0.   -6./l^2  +4./l    ] ;
  kL = kL_ax+kL_fl ;

  % trasformazione s.d.r. locale --> s.d.r. globale
  % costruzione matrice lambda traf. 3x3
   lambda = [ cos(alfa) sin(alfa) 0. 
             -sin(alfa) cos(alfa) 0.
	               0.      0.     1. ] ;

  % costruzione matrice Lambda traf. 6x6
  Lambda = [ lambda     zeros(3)
             zeros(3)   lambda      ] ;

 mG = Lambda' * mL * Lambda ;
 kG = Lambda' * kL * Lambda ;