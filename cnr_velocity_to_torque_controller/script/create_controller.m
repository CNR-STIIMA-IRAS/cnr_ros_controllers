
function create_controller(controller,integral_controller,vel_filter,target_vel_filter,antiwindup_ratio,st);

fid=1;

% s=tf('s');
% st=1e-3;
% controller=1/(s+1);
% integral_controller=1/s;
% vel_filter=1/(0.001*s+1);
% target_vel_filter=1;

indent=4;

controller_d=c2d(ss(controller),st);
integral_controller_d=c2d(ss(integral_controller),st);
vel_filter_d=c2d(ss(vel_filter),st);
target_vel_filter_d=c2d(ss(target_vel_filter),st);

fprintf(fid,'%santiwindup_ratio: %f\n',antiwindup_ratio);

fprintf(fid,'%scontroller:\n',repmat(' ',1,indent));
init_matrix=zeros(order(controller_d),1);
print_controller(fid,controller_d,init_matrix,indent+2);


fprintf(fid,'%sintegral_controller:\n',repmat(' ',1,indent));
init_matrix=inv(obsv(integral_controller_d))*ones(size(integral_controller_d.A,1),1);
print_controller(fid,integral_controller_d,init_matrix,indent+2);


fprintf(fid,'%svel_filter:\n',repmat(' ',1,indent));
if order(vel_filter_d)>0
  init_matrix=inv(obsv(vel_filter_d))*ones(size(vel_filter_d.A,1),1);
else
  init_matrix=[];
end
print_controller(fid,vel_filter_d,init_matrix,indent+2);

fprintf(fid,'%starget_vel_filter:\n',repmat(' ',1,indent));
if order(target_vel_filter_d)>0
  init_matrix=inv(obsv(target_vel_filter_d))*ones(size(target_vel_filter_d.A,1),1);
else
  init_matrix=[];
end
print_controller(fid,target_vel_filter_d,init_matrix,indent+2);


