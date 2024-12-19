function Return = gradFunction(SingleCostFunction, TModelInfor, seed, AgentIndex)
%
% numerical computation of gradient
% this allows automatic gradient computation
% 
% seed: the seed (Dimension,1)
% first forward finite difference
% hstep = 0.001; - programmed in
%
hstep = 0.0000001;
Dimension = length(seed);
f= SingleCostFunction(seed, TModelInfor,AgentIndex);
for i = 1:Dimension
   xs = seed;
   xs(i) = xs(i) + hstep;
   %CostFunction(seed, ModelInfor)
   gradx(i)= (SingleCostFunction(xs, TModelInfor) -f)/hstep;
end
Return = gradx'; % (Dimension,1)