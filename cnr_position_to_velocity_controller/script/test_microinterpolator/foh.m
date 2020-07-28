function [y_r,Dy_r]=foh(t,y,Dy,st_low)

dt=mod(t,st_low);

y_r=y+Dy*dt;
Dy_r=Dy;
