% revised at 8 pm 28.8.2018
% this function is for roulette wheel selection
% it is called from BSO.m, ABC.m, GSO.m
% P: the probability
% j: the selected individual
function SelectedIndex=RouletteWheelSelection(Prob)
    r=rand.*sum(Prob);
    CumProb=cumsum(Prob,2);
    SelectedIndex=find(r<=CumProb,1,'first');
    
    %Prob=Prob/(sum(Prob)+realmin);
    %SelectedIndex=randsrc(1,1,[1:numel(Prob);Prob]); % requires
    %sum(Prob)=1;
    
end

% 1. SelectedIndex=RouletteWheelSelection(Prob)：这是一个MATLAB函数的定义，它接受一个输入参数 Prob，其中 Prob 是一个包含选择概率的向量。
% 
% 2.r=rand.*sum(Prob)：这一行生成一个随机数 r，它的范围是 [0, sum(Prob)]，其中 rand 函数用于生成一个均匀分布的随机数。这个步骤相当于在轮盘上旋转，以随机选择一个位置。
% 
% 3.CumProb=cumsum(Prob,2)：这一行计算 Prob 中每个元素的累积和（cumulative sum），并将结果存储在 CumProb 中。cumsum 函数用于计算向量 Prob 的累积和。第二个参数 2 表示按行累积。
% 
% 4.SelectedIndex=find(r<=CumProb,1,'first')：这一行通过比较 r 和 CumProb 中的元素，找到第一个满足条件 r<=CumProb 的索引，并将其存储在 SelectedIndex 变量中。这就是轮盘赌选择的核心部分，它选择了一个索引，该索引对应的概率越大，被选择的概率就越高

% 如果 Prob 是 [1, 2, 3]，那么我们可以使用轮盘赌选择算法来计算 SelectedIndex，其中每个选项的选择概率与其在概率分布中的权重相关。
% 首先，生成一个随机数 r，范围在 [0, sum(Prob)] 内，其中 sum(Prob) 等于 1 + 2 + 3 = 6。然后，比较 r 与累积概率分布 CumProb，找到第一个满足条件 r <= CumProb 的索引。
% CumProb 是累积概率分布，由 [1, 3, 6] 构成，每个元素表示前面元素的累积和。
% 随机数 r 会在区间 [0, 6] 中生成。
% 根据随机数 r 的不同值，SelectedIndex 的值将有不同的可能性：
% 如果 r 落在 [0, 1] 的范围内，那么 SelectedIndex 将是 1。
% 如果 r 落在 (1, 3] 的范围内，那么 SelectedIndex 将是 2。
% 如果 r 落在 (3, 6] 的范围内，那么 SelectedIndex 将是 3