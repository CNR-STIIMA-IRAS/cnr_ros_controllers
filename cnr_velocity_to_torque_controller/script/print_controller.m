function print_controller(fid,sys,init_matrix,indent)
spaces=repmat(' ',1,indent);
[A,B,C,D]=ssdata(sys);
if size(A,1)==0
  A=0;
  B=0;
  C=0;
  init_matrix=0;
end

ub=1e4*ones(size(C,1),1);
lb=-1e4*ones(size(C,1),1);


Baw=zeros(size(A,1),size(C,1));

x0=zeros(size(A,1),1);
fprintf(fid,'%sA:\n',spaces);
fprintf(fid,'%s',save_matrix_to_yaml(A,indent));
fprintf(fid,'%sB:\n',spaces);
fprintf(fid,'%s',save_matrix_to_yaml(B,indent));
fprintf(fid,'%sBaw:\n',spaces);
fprintf(fid,'%s',save_matrix_to_yaml(Baw,indent));
fprintf(fid,'%sC:\n',spaces);
fprintf(fid,'%s',save_matrix_to_yaml(C,indent));
fprintf(fid,'%sD:\n',spaces);
fprintf(fid,'%s',save_matrix_to_yaml(D,indent));
fprintf(fid,'%smax_output:\n',spaces);
fprintf(fid,'%s',save_matrix_to_yaml(ub,indent));
fprintf(fid,'%smin_output:\n',spaces);
fprintf(fid,'%s',save_matrix_to_yaml(lb,indent));
fprintf(fid,'%sinitial_state:\n',spaces);
fprintf(fid,'%s',save_matrix_to_yaml(x0,indent));
fprintf(fid,'%sinitialization_matrix:\n',spaces);
fprintf(fid,'%s',save_matrix_to_yaml(init_matrix,indent));
end