function mainPathPlanning (DimensionSet, TaskSet)
if nargin==0
    DimensionSet=20;   % the number of waypoints
    TaskSet=1;
end
% nargin==0 的条件检查表示如果 nargin 的值等于0，那么说明没有传递参数给 mainPathPlanning 函数，nargin 等于0，就表示没有传递参数给 mainPathPlanning 函数，因此它会在函数内为 DimensionSet 和 TaskSet 分别分配默认值

%clear;
%clc;
AlgorithmName={'ACOPA_LS'};
%TrialTimes=30;
TrialTimes=1;
% algorithm parameters 
SwarmSize=40;
pRepair=1;  % probability of repair  修复概率
flag_uniform=0;   %% 先放在这里，暂时不知道有什么作用
TimeUsed=zeros(1, TrialTimes);

for Dimension=[DimensionSet]  % the number of waypoints      for i = d 是从i逐步增加到 d,但是 for i=[d]只执行一次
       
file_result=strcat('ACOPA4PathPlanning/','results/', 'Size',int2str(SwarmSize),'Dim',int2str(Dimension),'Result.txt'); % record results %这是一个创建一个字符串，包括路径和名称 
%strcat
%是一个MATLAB函数，用于将多个字符串连接成一个字符串。而前两个表示文件路径，后边Size是一个字符串，是文件名的一部分，int2str是整数转字符串，SwarmSize是一个数值
find_file_result=fopen(file_result,'a+'); %'a+'：这是文件打开模式，其中 'a' 表示追加（append）模式，'+' 表示允许读取和写入。追加模式意味着文件将以附加的方式打开，如果文件不存在，它将被创建，允许在文件末尾写入数据，而不会覆盖现有数据

for TaskIndex=[TaskSet]  %1:12
    [TaskInfor, ThreatInfor, ObstacleInfor, MovingObstacleInfor ]=EnvironmentInfor(TaskIndex);
    ModelInfor=ModelSetup(TaskInfor, ThreatInfor, ObstacleInfor,MovingObstacleInfor, Dimension); 
    TaskNumber=ModelInfor.TaskNumber;
    MaximumFEs=(100+(Dimension-20)*2500)*TaskNumber;
    %MaximumFEs=(50000+(Dimension-20)*2500)*TaskNumber;
    Penalty=3;
    % Initialise positions
    BestPathAndCost=zeros(Dimension*TaskNumber+TaskNumber+1, TrialTimes);
    
    for AlgorithmIndex=[1]  %
        ConvergenceData=zeros(TrialTimes,MaximumFEs+1);  %record the convergence data for each run 
        file_convergence_data=strcat('ACOPA4PathPlanning/','results/', char(AlgorithmName(AlgorithmIndex)),'Prob',int2str(TaskIndex),'Dim',int2str(Dimension),'Data.txt'); %record the convergence data
        find_file_convergence_data=fopen(file_convergence_data,'w');
        file_eachTrialResult_data=strcat('ACOPA4PathPlanning/','results/',char(AlgorithmName(AlgorithmIndex)),'Prob',int2str(TaskIndex),'Dim',int2str(Dimension),'Result.txt'); %record the best position and fitness values found by each trial
        find_file_eachTrialResult_data=fopen( file_eachTrialResult_data,'w');
        file_bestpath=strcat('ACOPA4PathPlanning/','results/',char(AlgorithmName(AlgorithmIndex)), 'Prob',int2str(TaskIndex), 'Dim',int2str(Dimension),'Path.txt'); % record the best path among all the trials by all algorithms
        find_file_bestpath=fopen(file_bestpath,'w');
        
        Algorithm=str2func(char(AlgorithmName(AlgorithmIndex)));   % 这段代码的作用是从字符串数组中选择一个函数名，然后将其转换为一个可以调用的函数句柄。这在需要根据条件选择要执行的函数时很有用
        
        for TrialIndex=1:TrialTimes
            TimeStart=tic; % record the running time
            
            TModelInfor=CordinateTransformation(ModelInfor, 1);
            Bound=TModelInfor.Bound;
            InitPos=Bound(:,1)+rand(Dimension, SwarmSize).*(Bound(:,2)-Bound(:,1));    % 初始化种群（在垂直轴上进行随机抽取）
            InitPos(:,SwarmSize/2+1:end)=repair(InitPos(:,SwarmSize/2+1:end),TModelInfor,1, pRepair, flag_uniform);       % 这里是对初始化种群的修复，之修复种群的后半部分
            [Gbest, GbestValue, GbestHistory]=feval(Algorithm, MaximumFEs, SwarmSize, InitPos, ModelInfor,pRepair);
            
            BestPathAndCost(1:TaskNumber+1, TrialIndex)=GbestValue; % Optimal cost
            BestPathAndCost(TaskNumber+2:(Dimension+2)*TaskNumber*2+TaskNumber+1, TrialIndex)= Gbest; % 第一行：三个任务的总适应度值；第2-4行：三个任务分别的适应度值；5-26是第一条任务的x值，27-48是第一条任务的y值，往后以此类推。
            ConvergenceData(TrialIndex,:)=GbestHistory(1:MaximumFEs+1);
            
            TimeUsed(TrialIndex)=toc(TimeStart);
        end
        AverageTime=mean(TimeUsed);
        StdTime=std(TimeUsed);
        MeanOfBestCost=mean(BestPathAndCost(1,:));
        StdOfBestCost=std(BestPathAndCost(1,:));
        MaxOfBestCost=max(BestPathAndCost(1,:));
        [MinOfBestCost, BestIndex]=min(BestPathAndCost(1,:));
        BBestPath=BestPathAndCost(TaskNumber+2:TaskNumber+(Dimension+2)*TaskNumber*2+1, BestIndex);
        
        A=sum(BestPathAndCost(2:1+TaskNumber,:)<=Penalty,1);    % 三条路线全部为可行路径的则为1，相加为3
        FeasibleCostIndex=find(A==TaskNumber);        
        FeasibleCost=BestPathAndCost(1, FeasibleCostIndex);     % 运行30次，每次最好的结果集合
        SuccessfulRate=length(FeasibleCostIndex)/TrialTimes;    % 运行30次的成功率
        MeanOfFeasible=mean(FeasibleCost);
        StdOfFeasible=std(FeasibleCost);
        
        
        MedianOfBestCost=median(BestPathAndCost(1,:));
        %MedianIndex = find(BestPathAndCost(1,:) == MedianOfBestCost);
        %MedianY=BestPathAndCost(2:Dimension+1, MedianIndex);
        %MedianBestPath=CordinatesRecover(ModelInfor, MedianY, TaskInfor);
        
        AverageConvergenceData=sum(ConvergenceData(:,1:end),1)/TrialTimes; % for the convergence graph    30次的150001次迭代，每一次150001的平均值（1*150001）
        
        fprintf('%i  %s  %.2f  %.2f  %.2f  %.2f  %.2f  %.2f  %.2f  %.2f  %.2f  %.2f\r\n', TaskIndex, char(AlgorithmName(AlgorithmIndex)), SuccessfulRate, MinOfBestCost, MaxOfBestCost, MedianOfBestCost, MeanOfBestCost, StdOfBestCost, MeanOfFeasible, StdOfFeasible, AverageTime, StdTime);   % 输出在命令行窗口
        fprintf(find_file_result, '%i  %s  %.4f  %.4f  %.4f  %.4f  %.4f  %.4f  %.4f %.4f  %.4f  %.4f\r\n', TaskIndex, char(AlgorithmName(AlgorithmIndex)), SuccessfulRate, MinOfBestCost, MaxOfBestCost, MedianOfBestCost, MeanOfBestCost, StdOfBestCost, MeanOfFeasible, StdOfFeasible, AverageTime, StdTime);
        fprintf(find_file_bestpath, '%.4f\t', BBestPath);
        
        fprintf(find_file_convergence_data, '%12.4e', AverageConvergenceData); %record the convergence data
        fprintf(find_file_eachTrialResult_data, '%12.4e', BestPathAndCost);
        fclose(find_file_convergence_data);
        fclose(find_file_eachTrialResult_data);
        fclose(find_file_bestpath);
    end
    
end
end
fclose ('all');
end 
