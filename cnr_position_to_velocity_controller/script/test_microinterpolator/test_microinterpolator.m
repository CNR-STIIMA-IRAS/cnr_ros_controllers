clear all;close all;clc;


st_fast=1e-3;
st_low=8e-3;



Ttot=3;
t=(0:st_fast:Ttot)';

acc_max=5;
vel_max=0.5;
    

if 0
    freq=1;
    w=2*pi*freq;
    A=min(vel_max/w,acc_max/w^2);
    y=A*sin(w*t);
    Dy=A*w*cos(w*t);
    DDy=-A*w^2*sin(w*t);
    
else
    tacc=vel_max/acc_max;
    
    DDy=acc_max*(t<=tacc)-acc_max*(t>=(Ttot-tacc));
    Dy=cumtrapz(t,DDy);
    y=cumtrapz(t,Dy);
end


td=(t(1):st_low:t(end));
yd=interp1(t,y,td,'previous');
Dyd=interp1(t,Dy,td,'previous');
DDyd=interp1(t,DDy,td,'previous');


y_zoh=interp1(td,yd,t,'previous');
Dy_zoh=interp1(td,Dyd,t,'previous');
DDy_zoh=interp1(td,DDyd,t,'previous');

for idx=1:length(t)
    [y_foh(idx,1),Dy_foh(idx,1)]=foh(t(idx),y_zoh(idx),Dy_zoh(idx),st_low);
    [y_soh(idx,1),Dy_soh(idx,1)]=soh(t(idx),y_zoh(idx),Dy_zoh(idx),DDy_zoh(idx),st_low);
end

h(1)=subplot(2,1,1);
plot(t,y,'k')
hold on
stairs(t,y_zoh,'b')
stairs(t,y_foh,'r')
stairs(t,y_soh,'m')
ylabel('Position')
legend('real','zoh','foh','soh')
grid on
h(2)=subplot(2,1,2);
plot(t,Dy,'k')
hold on
stairs(t,Dy_zoh,'b')
stairs(t,Dy_foh,'r')
stairs(t,Dy_soh,'m')
linkaxes(h,'x');
xlabel('Time');
ylabel('Velocity');
grid on


fprintf('ZOH (Zero-Order-Hold)  : Maximum position error = %5.4e, maximum velocity error = %5.4e\n',max(abs(y-y_zoh)),max(abs(Dy-Dy_zoh)));
fprintf('FOH (First-Order-Hold) : Maximum position error = %5.4e, maximum velocity error = %5.4e\n',max(abs(y-y_foh)),max(abs(Dy-Dy_foh)));
fprintf('SOH (Second-Order-Hold): Maximum position error = %5.4e, maximum velocity error = %5.4e\n',max(abs(y-y_soh)),max(abs(Dy-Dy_soh)));

