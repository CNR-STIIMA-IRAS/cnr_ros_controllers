function [y_r,Dy_r]=soh(t,y,Dy,DDy,st_low)

dt=mod(t,st_low);

y_r=y+Dy*dt+0.5*DDy*dt^2;
Dy_r=Dy+DDy*dt;
