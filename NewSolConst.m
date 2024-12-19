% New solutions construction
function [NewAnt,Nfitx]= NewSolConst(ModelInfor, x, PopSize, Dimension, Probability, flag,zeta, AgentIndex, pRepair,flag_uniform)
ell=RouletteWheelSelection(Probability);
StandardDeviation=zeta.*sum(abs(x(ell,:)-x))./(PopSize-1);  % Dimension *PopSize   标准差
Step=StepSelection(ell,flag, Dimension);
NewAnt=x(ell,:)+StandardDeviation.*Step;
pp=repair1(NewAnt',ModelInfor,AgentIndex, pRepair, flag_uniform);
NewAnt=pp';
Nfitx=SingleCostFunction(NewAnt', ModelInfor,AgentIndex);      
end

function Step=StepSelection(ell,flag, Dimension)
flag=flag(ell,:);
Step=randn(1,Dimension).*(1-flag)+trnd(1, 1, Dimension).*flag;   %  (高斯变异+柯西变异）
% Gaussian randn(1,Dimension) ; Cauchy trnd(1, 1, Dimension)
end