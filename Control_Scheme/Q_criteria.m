%更新Q，Q用于damped parameter u的更新
function Q=Q_criteria(vector_f,vector_f_new,h_lm,u,g)
   delta_F=1/2*(vector_f'*vector_f-vector_f_new'*vector_f_new);
   delta_L=1/2*h_lm'*(u*h_lm-g);
   Q=delta_F/delta_L;
end
