%计算vector_f在当前位姿下的jacobian矩阵
function jacobian_f=Jaco(pose,A,Bp)
   position=pose(1:3)';
   orientation=pose(4:6);
   R=eul2rotm(orientation,'XYZ');
   vector_PB=zeros(3,8);
   B=zeros(3,8);
   BA=zeros(3,8);
   uBA=zeros(3,8);
   wrench_BA=zeros(3,8);
   for i=1:8
   %在O-XYZ系中表示向量PBi
   vector_PB(1:3,i)=R*Bp(1:3,i);
   %将attach points表示在O-XYZ坐标系中：
   B(1:3,i)=vector_PB(1:3,i)+position;
   %表示cable单位方向向量
   BA(1:3,i)=A(1:3,i)-B(1:3,i);
   uBA(1:3,i)=BA(1:3,i)/norm(BA(1:3,i));
   wrench_BA(1:3,i)=cross(vector_PB(1:3,i),uBA(1:3,i));
   end
   jacobian_f=-1*[uBA' wrench_BA'];
end