close all
alfa=0.1; beta=20.0e-4
f=[1:0.01:6];
rrc=alfa/4/pi./f+beta/pi.*f;
plot(f,rrc);grid