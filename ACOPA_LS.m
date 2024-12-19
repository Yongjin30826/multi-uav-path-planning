%"Yongjin Wang, Pengkai Chen, Yifan Wu, Weixuan Chen, Lijing Tan,
% Modified Continuous Ant Colony Optimisation with Local Search for Multiple Unmanned Aerial Vehicle Path Planning.
% In: Proceeding of the International Conference on Data-driven
% Optimization of Complex Systems. IEEE, pp. 557-564, 2024. (EI)"

function [TGbest, TGbestValue, FEvBestFitness]= ACOPA_LS (MaximumFEs, SwarmSize, InitPos, ModelInfor, pRepair)
flag_restart=1;  
zeta=0.6;   % 用于高斯、柯西变异
q=0.2;      % 用于计算粒子权重
win=10;
AgentIndex=1;
eval_agent(AgentIndex)=1;  % fitness values evaluations (FEs) count for each robot(agent)  每个机器人（代理程序）的适应值评估次数（FEs）计数
TModelInfor=CordinateTransformation(ModelInfor, AgentIndex); % thansfer the coordinate system

TaskNumber=TModelInfor.TaskNumber;  % number of tasks/robots
Dimension=TModelInfor.Num_WayPoints; 
Bound=TModelInfor.Bound;
xmin=Bound(:,1);
xmax=Bound(:,2);
Penalty=3 ; %ModelInfor.Task(1,5)*(Dimension+1)*2;

y=zeros(Dimension, TaskNumber);  % y coordinates    存储每个任务的最佳路径
SMaximumFEs=ceil(MaximumFEs/TaskNumber);  % maximum FEs for each task    每项任务的最大评价次数（向上取整）
TGbest=[];
TGbestValue=[];

flag_agent=0;  % if the optimisation of one agent'path is finished       如果一个代理的路径优化已经完成

%% parameters for Adaptive waypoints repair method    自适应拐点修复方法的参数
n_Oper= 3; % number of moving directions      移动方向的数量 （随机、向上、向下游走）
win_size=3*win; % the window size before probabilities are updated    在概率更新之前的窗口大小
n_win=2;  % none of the strategies improved the solutions in the m previous W windows, reinitialise   如果在过去的m个W窗口中策略没有对解进行提高或者改进，重新初始化
n_Ants=SwarmSize; % number of new solutions
Experience_Oper=ones(1, n_Oper);    % 公式(27)（28）中的三个修复方法的E值（经验值）

ConvergenceData = ones(1, MaximumFEs+1)*10^5;    % best fitness found    发现的最优解

TrialIndex=1;
current_eval=1; %%% fitness function evaluations counter   适应函数评估计数器
%previous_eval=0;
iter=0;

%% Start initialization in the archive (PopSize, Dimension)   在归档中进行初始化（种群大小，维度）
xant=InitPos';
fitx=SingleCostFunction(xant', TModelInfor, AgentIndex);          % 计算x的适应度值

%% Sort the population based on fitx
[fitx, indecies ] = sort( fitx );   % [fitx,indecies]前一个fitx是对初始化种群进行排序，后一个indecies是对排序后的元素在原fitx中的索引值
xant = xant( indecies, : );         % 对种群完成排序
ConvergenceData(1)=fitx(1);         % 每个种群里最好的解保存在ConvergenceData中

%StandardDeviation=zeros(PopSize, Dimension);
NewAnt= zeros(n_Ants, Dimension);

NoImprove=0;
SolutionWeights=1/(q*SwarmSize*sqrt(2*pi))*exp(-0.5*(((1:SwarmSize)-1)/(q*SwarmSize)).^2);  % 公式（22)计算wi
Probability=SolutionWeights./sum(SolutionWeights);                                           % 公式）（21）计算每个粒子被选中的概率                                

t=SwarmSize:-1:1;                                                                       % 计算两种变异的概率
Pci=0.5+0.4*(exp(10*(t-1)/(SwarmSize-1))-1)/(exp(10)-1);

nn=1;  % 在局部搜索时使用

while current_eval<MaximumFEs
    if flag_agent==1   % 下一个任务
        xant(SwarmSize/2+1:end,:)=xmin'+rand(SwarmSize/2, Dimension).*(xmax'-xmin');

        pp=repair1(xant(SwarmSize/2+1:end,:)',TModelInfor,AgentIndex, pRepair, flag_uniform);
        xant(SwarmSize/2+1:end,:)=pp';
        fitx=SingleCostFunction(xant', TModelInfor,AgentIndex);
        [fitx, indecies ] = sort( fitx );
        xant = xant( indecies, : );
        eval_agent(AgentIndex)=current_eval+1;
        current_eval=current_eval+n_Ants;

        ConvergenceData(current_eval-n_Ants+1:current_eval)=fitx(1); 

        flag_agent=0;
        
        
    else
        iter=iter+1;

%% ---------------------Update individuals------------------------------
        if mod(iter,win_size)==1        % iter 每满足一个Nwin之后，就要进行 p_r的更新（注意：经验值每次迭代都要更新，概率值则在预定窗口Nwin大小后进行更新）
            Prob_Oper=Experience_Oper/(sum(Experience_Oper)+realmin);  % realmin是处理非常小的浮点数值时，以确定可以接受的最小阈值。这可以用来避免舍入误差或处理接近零的数值
            Experience_Oper=ones(1, n_Oper);
        end

        % generate new population
        % Prob_Oper
        flag_moving=RouletteWheelSelection(Prob_Oper);   % 第一次时，当输入Prob_Oper=[0.333,0.333,0.333]时，随机数落在0-0.333,0.333-0.666,0.666-1.flag_moving分别是1，2，3 
        flag_uniform=[0 flag_moving];   % 在生成新解函数（NewSolConst）中，当新解生成后，即（高斯变异+柯西变异），又使用flag_uniform进行解的修复
        old_fitx=fitx(1);   %  计算E的值时，会用到，论文中的公式（27）
        ttemp=rand(SwarmSize,Dimension);
        flag=(ttemp>Pci')*1; %将 40x1 的 Pci 扩展以匹配 40x20 的 ttemp，然后执行逐元素比较。如果真，则赋值1，如果假，则赋值0；flag用于高斯变异和柯西变异的选择

        for i=1:n_Ants

            [NewAnt(i,:), Nfitx(i)]= NewSolConst(TModelInfor, xant, SwarmSize,Dimension, Probability, flag,zeta, AgentIndex, pRepair, flag_uniform);
            %新解的构建
        end
%% ---------------------Evaluation----------------------------------------   评估   

        allSwarm=[xant; NewAnt];   % 两个种群进行合并
        allFitnessValue= [fitx Nfitx];   % 两个种群对应的适应度值进行合并

        % sort
        [allFitnessValue, SortIndex]=sort(allFitnessValue);   % 对两个种群适应度值进行排序
        allSwarm=allSwarm(SortIndex,:);                       % 排序后的粒子构成的种群

        current_eval=current_eval+n_Ants;     %41 81 121 161...
        ConvergenceData(current_eval-n_Ants+1:current_eval)=allFitnessValue(1);  %2-41，42-81，82-121，122-161...

        % record the number without impovement      如果没有提高
        if fitx(1)<=allFitnessValue(1)
            NoImprove=NoImprove+1;
        else
            NoImprove=0;
        end

        %remove duplicates     删除重复项
        old_current_eval=current_eval;
        [allSwarm,allFitnessValue,current_eval] = hanDuplicate(allSwarm,allFitnessValue,xmin, xmax, TModelInfor,current_eval, AgentIndex);
        ConvergenceData(old_current_eval+1:current_eval)=allFitnessValue(1);

        xant=allSwarm(1:SwarmSize,:);   % 种群筛选到原规模大小
        fitx=allFitnessValue(1:SwarmSize);  %  适应度值筛选到原规模大小
        if NoImprove<=n_win*win_size
            Experience_Oper(flag_moving)=Experience_Oper(flag_moving)+abs((old_fitx-fitx(1)));  % 更新经验值
        else
            Experience_Oper=ones(1, n_Oper);      % 超出m个Nwin没有提高，则将E和p重新进行初始化
            Prob_Oper=Experience_Oper/(sum(Experience_Oper)+realmin);
        end

        Gbest=xant(1,:);
        GbestValue=fitx(1);
        
        
        IMaximumFEs=10000;
        temp_current=8000;
        if current_eval > temp_current + IMaximumFEs*(nn-1)
            LSMaximumFEs=2000;
            [xopt,fopt,k, Convergence,X] = BFGS(Gbest',@SingleCostFunction,TModelInfor, LSMaximumFEs,AgentIndex);  % 将Gbest输入到局部搜索中
            k;
            [Gbest, GbestValue]=GreedySelection(xopt,fopt,Gbest,GbestValue);
            ConvergenceData((current_eval+1):(current_eval+k))=Convergence(1:k);
            current_eval=  IMaximumFEs*nn + 1;
            nn = nn + 1;
            iter=iter + 50;
            xant = X';
        end
           

    end
    
    
    if flag_restart==1
        if current_eval>=SMaximumFEs*(AgentIndex-1)+ TrialIndex*SMaximumFEs/10 && GbestValue>=Penalty  % restart  超出多少代仍然是不可行路径，则重新开始
            TrialIndex=TrialIndex+1;
            xant=xmin'+rand(SwarmSize, Dimension).*(xmax'-xmin');
            pp=repair1(xant',TModelInfor,AgentIndex, pRepair, flag_uniform);
            xant=pp';
            fitx=SingleCostFunction(xant', TModelInfor,AgentIndex);
            [fitx, indecies ] = sort( fitx );
            xant = xant( indecies, : );
            current_eval=current_eval+n_Ants;
            ConvergenceData(current_eval-n_Ants+1:current_eval)=fitx(1); 
        end
    end

    if current_eval>=SMaximumFEs*(AgentIndex)   % the next agent  下一个路径
        TrialIndex=1;
        fitx=SingleCostFunction(Gbest', TModelInfor,AgentIndex);
        [path, Gbest] = CordinatesRecover(TModelInfor, Gbest',ModelInfor, AgentIndex); % transfer it to the original coordinate system
        Gbest=Gbest';

        y(:, AgentIndex)=Gbest';
        ModelInfor.y=y;
        TGbestValue=[TGbestValue ; GbestValue]; % the total cost of all tasks
        TGbest=[TGbest ;  path(:,1); path(:,2)];   % the best solutions of all tasks
        flag_agent=1;
        AgentIndex=AgentIndex+1;
        if AgentIndex<= TaskNumber    % 对下一个任务，重新更新地理坐标
          TModelInfor=CordinateTransformation(ModelInfor, AgentIndex);
          Bound=TModelInfor.Bound;
          xmin=Bound(:,1);
          xmax=Bound(:,2);
        end
    end

end
        
FEvBestFitness=ConvergenceData(1:MaximumFEs+1);
for l=1:MaximumFEs   % 避免报错，空集会报错
    if isnan(FEvBestFitness( l))   % 如果FEvBestFitness(1） 为NaN
        FEvBestFitness(l)=10^10;
    end
  
    if  ismember(l, eval_agent)% l==MaximumFEs/TaskNumber+1   检查变量 l 是否包含在数组 eval_agent 中。如果 l 包含在 eval_agent 中，就执行条件语句中的操作，否则不执行
        % do nothing
    else
        if FEvBestFitness(l)<FEvBestFitness(l+1)
            FEvBestFitness(l+1)=FEvBestFitness(l); % 收敛
        end   
    end
end

TotalCost=sum(TGbestValue);    % 三个任务的全部成本
TGbestValue=[TotalCost; TGbestValue];
disp([ ' best fitness = ', num2str(TGbestValue')]);      % 输出的是三个值，三个任务的总值，每个任务各自的值                        

end

%备注：
% 42 对种群进行初始化
% 43 计算x的适应度值
% 46-48 对种群里的粒子进行排序，并储存在ConvergenceData中
% 54-55 根据p_s筛选哪些粒子被选择，论文中的（21）（22） 1*40
% 57-58 根据p_r计算高斯和柯西变异的概率，论文中（26） 1*40
% 79-82 对p_w进行更新