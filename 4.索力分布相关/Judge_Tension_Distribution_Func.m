function [bool] = Judge_Tension_Distribution_Func(t,tm,tM)
% 判断该索力分布是否合理 tm<=t<=tM
% 输出为1代表合理，输出为0代表不合理
bool=1;
for i=1:size(t,1)
    if t(i,1)<tm | t(i,1)>tM
        bool=0;
        return
    else
        bool=1;
        return
    end
end
end

