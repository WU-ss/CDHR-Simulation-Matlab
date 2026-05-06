%约束函数向量vector_f在当前位姿pose下的值
function vector_f=FunctionVector_f(pose,A,Bp,length_given)
   position=pose(1:3)';
   orientation=pose(4:6);
   R=eul2rotm(orientation,'XYZ');
   vector_f=zeros(8,1);
   for i=1:8
       vector_f(i,1)=sqrt((A(1:3,i)-position-R*Bp(1:3,i))'*(A(1:3,i)-position-R*Bp(1:3,i)))-length_given(i);
   end
end