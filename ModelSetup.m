function  Infor= ModelSetup(TaskInfor, ThreatInfor,ObstacleInfor,MovingObstacleInfor, Num_WayPoints)
      
      [TaskNumber,~] =size(TaskInfor);  %将获取矩阵的行数和列数，将行数返回给TaskNumber,列数则是不关心变量
      BoundInfor=TaskInfor(1,6);
      Bound=repmat([0, BoundInfor],Num_WayPoints,1);
      
      StartPoint=TaskInfor(:,1:2);
      TargetPoint=TaskInfor(:,3:4);
      %d=dist(StartPoint, TargetPoint');
      
      k=1:Num_WayPoints;
      x=StartPoint(:,1)+k.*(TargetPoint(:,1)-StartPoint(:,1))/(1+Num_WayPoints);
      
      % calculate the itersection with obstacles of each verticle line
      [ObsNum,~]=size(ObstacleInfor);
      obsx=x; %[StartPoint(:,1) x TargetPoint(:, 1)];
      Tfeasible=TaskInfor(1,6)*ones(TaskNumber,Num_WayPoints, ObsNum+1, 2);
      feasNum=zeros(TaskNumber, Num_WayPoints);
      %TInfeasible=TaskInfor(1,6)*ones(TaskNumber,Num_WayPoints, ObsNum, 2);
      maxgap=ones(TaskNumber,Num_WayPoints+2, 1, 2);  % 书架上两本书，每本书只有一页，每页上有一个矩阵3×22
      for ii=1:TaskNumber    % 总共有3条路径
      for i=1:Num_WayPoints % for each bin in x direction
         % intersectionNum=zeros(1,Num_WayPoints);
          infeasible=[];
          feasible=[];
          for j=1:ObsNum      
              %syms yy
              a=ObstacleInfor(j,1);
              b=ObstacleInfor(j,2);
              r=ObstacleInfor(j,3);
              if abs(obsx(ii,i)-a)<r    
                   obsy1=(r^2-(obsx(ii,i)-a)^2)^0.5+b;
                   obsy2=-(r^2-(obsx(ii,i)-a)^2)^0.5+b;
                   %eq=(obsx(i)-a)^2+(yy-b)^2==r^2;
                   %s=solve(eq,yy);
                   infeasible=[infeasible; obsy2 obsy1];
                   %intersectionNum(i)=intersectionNum(i)+1;
                   %%当路径上的点进入与圆相切的两条直线内时，进行判断，此时的可行路径是穿过路径上这点的圆对应的y1和y2的值。此二值比在图中的虚线上
              end
          end
          if ~isempty(infeasible)          % 如果infeasible不为空的话，则执行以下的条件
              [~,sortindex]=sort(infeasible(:,1));  %sortindex是对升序后的索引
              sinfeasible=infeasible(sortindex,:);  %sinfeasible 是将infeasible的值按照升序进行排列
              ssinfeasible=[0 reshape(sinfeasible',1,[]) TaskInfor(1,6)];   % reshape 是将矩阵原本向量变成行向量

              %if sinfeasible(1,1)>0
              %   feasible=[feasible; 0 sinfeasible(1,1)];
              %end
              ifNum=length(ssinfeasible);  % 获取ssinfeasible的元素的个数
              for k=2:2:ifNum
                  if ssinfeasible(k)>ssinfeasible(k-1)
                     feasible=[feasible; ssinfeasible(k-1) ssinfeasible(k)];  % 这是无人机路线在此直线上的可行路径（圆的上部分和下部分）
                  end
              end
              gap=feasible(:,2)-feasible(:,1);
              [~, maxindex]=max(gap);
              maxgap(ii,i,1,:)=feasible(maxindex,:);    % 可以将四维数组理解为一个书架，二维数组是一页纸，三维数组是一本书，四维数组是书架  每本书的第一页的ii行和i列元素值分别为feasible的第一个值和第二个值
              [fNum,~]=size(feasible);
              Tfeasible(ii, i, 1:fNum,:)=feasible;
              feasNum(ii,i)=fNum;
              %[iNum,~]=size(sinfeasible);
              %TInfeasible(ii, i, 1:iNum,:)=sinfeasible;
          else
              Tfeasible(ii, i, 1,:)=[0 TaskInfor(1,6)];
              maxgap(ii,i,1,:)=[0 TaskInfor(1,6)];    % 这里的四维数组表示，前两个表示第几条路线上的第几个点，第三个表示此点垂线上的线段编号，第四个表示两个端点。第三个数字如果表示没有Infeasible，其端点就直设置为（0,100);如果有feasible,但是编号没有达到最高的obsnum+1的话，此时两个端点都被设置成100
              feasNum(ii,i)=1;
          end
      end
      end
      
      Infor.maxgap=maxgap;
      %Infor.Tinfeasible=TInfeasible;
      Infor.feasible=Tfeasible;
      Infor.feasibleNum=feasNum;
      
      Infor.Task=TaskInfor;
      Infor.Threat=ThreatInfor;
      Infor.Obstacle=ObstacleInfor;
      Infor.MObstacle=MovingObstacleInfor;
      Infor.x=x';   %Num_WayPoints,TaskNumber
      Infor.Bound=Bound;
      Infor.Num_WayPoints=Num_WayPoints;
      Infor.TaskNumber=TaskNumber;
      
      Infor.y=[];  % the variables (Num_WayPoints,TaskNumber)
      
end