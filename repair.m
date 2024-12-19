function pos=repair(pos,ModelInfor,AgentIndex,prob, flag_uniform)
      flag_uniform=0;
      AgentIndex=1;
      [Dimension, SwarmSize]=size(pos);   % 获取矩阵的维度，包括行数和列数
      %maxGap=ModelInfor.maxgap;
      feasNum=ModelInfor.feasibleNum; %(TaskNumber, Num_WayPoints);         % 每条路线上每个点的垂线上的可行路径的段数
      feasibleGap=ModelInfor.feasible;  %(TaskNumber,Num_WayPoints, ObsNum+1, 2);    %  每条路线上每个点的垂线上的可行路径的每个段的两个端点
      %flag=0;   %the position is infeasible
      %Infeasible=ModelInfor.Tinfeasible;
      Bound=ModelInfor.Bound;
      minrange=Bound(:,1);
      maxrange=Bound(:,2);
     
      
      for i=1:SwarmSize
          if rand<prob
              if flag_uniform==1
                 pos(:,i)=pos(:,i).*(maxrange-minrange)+minrange; % decoding    
              end
              for j=1:Dimension
                  %num
                  flag=0;   %the position is infeasible
                  gap=abs(feasibleGap(AgentIndex, j, 1:feasNum(AgentIndex, j), 2)-feasibleGap(AgentIndex, j, 1:feasNum(AgentIndex, j), 1));   % 是一个1*1*2double，路径上的点的垂线的可行路径的总长度
                  %p=gap./sum(gap); 
                  ggap=zeros(1,feasNum(AgentIndex, j));
                  for k=1:feasNum(AgentIndex, j)
                      if pos(j,i)<=feasibleGap(AgentIndex, j, k, 2) && feasibleGap(AgentIndex, j, k, 1)<=pos(j,i)   % 两个条件都为真才为真，有一个是假则为假  可行路径上每个线段的长度
                          flag=1;    % feasible                   
                      end
                      ggap(k)=gap(1,1,k);   % 每个可行区域的长度
                  end
                  if flag==0   %  如果点在不可行区域内
                      index=RouletteWheelSelection(ggap);  %根据轮盘赌进行选择，如果随机生成的
                      pos(j,i)=feasibleGap(AgentIndex, j, index, 1)+(feasibleGap(AgentIndex, j, index, 2)-feasibleGap(AgentIndex, j, index, 1))*rand;
                      %pos(j,i)=maxGap(1)+(maxGap(2)-maxGap(1))*rand;
                      %随机生成新的解（在可行区间内）
                  end
              end
              if flag_uniform==1
                 pos(:,i)=(pos(:,i)-minrange)./(maxrange-minrange) ;
              end
          end
      end
end