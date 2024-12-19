% transform cordinate
function  Infor= CordinateTransformation(ModelInfor, TaskIndex)
      Infor=ModelInfor;
      
      OTaskInfor=ModelInfor.Task;
      TaskInfor=OTaskInfor(TaskIndex,:);

      ThreatInfor=ModelInfor.Threat;
      ObstacleInfor=ModelInfor.Obstacle;
      Num_WayPoints=ModelInfor.Num_WayPoints;
      
      StartPoint=TaskInfor(1:2);
      TargetPoint=TaskInfor(3:4);
      %UpperBound=[0, TaskInfor(5)];
      %LowerBound=[TaskInfor(6), 0];
      d=dist(StartPoint, TargetPoint');   %  欧氏距离，也就是起始点和终点之间的距离
      %Theta=asin((TargetPoint(2)-StartPoint(2))/d);
      Theta=atan((TargetPoint(2)-StartPoint(2))/(TargetPoint(1)-StartPoint(1)));
      a=[cos(Theta) sin(Theta); -sin(Theta), cos(Theta)];
      % transform task cordinates
      TStartPoint=a*(StartPoint-StartPoint)';
      TTargetPoint=a*(TargetPoint-StartPoint)'; % (2,1)
      TTaskInfor(1:2)=TStartPoint';
      TTaskInfor(3:4)=TTargetPoint';
      TTaskInfor(5:6)=TaskInfor(5:6);
      % transform threat cordinates
      [n, ~]=size(ThreatInfor);
      TThreatInfor=ThreatInfor;
      for i=1:n
          TThreatInfor(i,1:2)=(a*(ThreatInfor(i, 1:2)-StartPoint)')';
      end
      
      k=1:Num_WayPoints;
      if TargetPoint(1)>StartPoint(1)    %如果targetpoint的横坐标值>startpoint的横坐标的值
            x=k.*(d/(Num_WayPoints+1)); % (1, Num_WayPoints)
      else
           x=-k.*(d/(Num_WayPoints+1));
      end         
      
      %x=k.*((TTargetPoint(1)-TStartPoint(1))/(Num_WayPoints+1));
      
      % transform obstable cordinates
      [m, ~]=size(ObstacleInfor);
      TObstacleInfor=ObstacleInfor;
      for i=1:m
          TObstacleInfor(i,1:2)=(a*(ObstacleInfor(i, 1:2)-StartPoint)')';
      end
      ObstacleInfor=TObstacleInfor;   %转换之后的坐标
      
      % calculate the transformed bound
      kk=(TargetPoint(2)-StartPoint(2))/(TargetPoint(1)-StartPoint(1));  % ???   、
      % 判断起点和终点的四种情况，分别为正正，正负，负正，负负。论文中只出现了两种情况，分别是正正，负正；即终点在起始点的右上方和右下方
      if kk>0
          % y=k*(x-StartPoint(1))+StartPoint(2)   
          for j=1:Num_WayPoints  
              %xx(j)=StartPoint(1)+cos(Theta)*x(j); % the x  in the original cordinate
              %yy(j)=kk*(xx(j)-StartPoint(1))+StartPoint(2);
              point=[x(j) 0];
              Opoint=(a\point'+StartPoint')';
              xx(j)=Opoint(1);
              yy(j)=Opoint(2);
              b(j)=yy(j)+(1/kk)*xx(j);     % the intersection with y axis    所围区域是一个矩形，这是与原y轴交点
              bb(j)= kk*b(j);       % the intersection with x axis           所围区域是一个矩形，这是与原x轴交点
              bbb(j)=-(TaskInfor(6)-b(j))*kk; % the intersection with y=TaskInfor(6)           所围区域是一个矩形，这是与x轴平行边的交点
              bbbb(j)=-(1/kk)*TaskInfor(5)+b(j);  % the itersection with x=TaskInfor(5)        所围区域是一个矩形，这是与y轴平行边的交点
              if b(j)<=TaskInfor(6)          %如果与y轴的交点值<100，则第j个点的上界值是b(j)
                  UpperBound(j)=b(j);
                  UpperPoint(j,:)=[0, UpperBound(j)];     %  和71行一起给出了点的纵坐标最大的范围
              else
                  UpperPointX(j)=bbb(j); %-(TaskInfor(6)-b(j))*kk;    %如果与y轴的交点值<100，则第j个点的上界值是bbb(j),即与x轴平行边的交点
                  UpperPoint(j,:)=[UpperPointX(j), TaskInfor(6)];
              end
              TUpperPoint(j,:)=a*(UpperPoint(j,:)-StartPoint)';   %  在新的坐标轴下，每个x轴坐标点正方向的y值最大范围（最大值）

              bb(j)= kk*b(j);       % the intersection with x axis     所围区域是一个矩形，这是与原x轴交点
              if bb(j)<=TaskInfor(5)                                        %如果与x轴的交点值<100，则第j个点的上界值是bb(j)
                  LowerBound(j)=bb(j);
                  LowerPoint(j,:)=[LowerBound(j), 0];                        %  和81行一起给出了点的纵坐标最大的范围
              else
                  LowerPointY(j)=bbbb(j); %-(1/kk)*TaskInfor(5)+b(j);
                  LowerPoint(j,:)=[TaskInfor(5), LowerPointY(j)];
              end
              TLowerPoint(j,:)=a*(LowerPoint(j,:)-StartPoint)';              %  在新的坐标轴下，每个x轴坐标点负方向的y值最大范围（最小值）
          end 
      else                % 在论文中此时是终点在目标点右下方
          % y=k*(x-StartPoint(1))+StartPoint(2)
          for j=1:Num_WayPoints
              %xx(j)=StartPoint(1)+cos(Theta)*x(j); % the x  in the original cordinate
              %yy(j)=kk*(xx(j)-StartPoint(1))+StartPoint(2);
              point=[x(j) 0];
              Opoint=(a\point'+StartPoint')';
              xx(j)=Opoint(1);
              yy(j)=Opoint(2);
              b(j)=yy(j)+(1/kk)*xx(j);     % the intersection with y axis
              bb(j)= kk*b(j);       % the intersection with x axis
              bbb(j)=-(TaskInfor(6)-b(j))*kk; % the intersection with y=TaskInfor(6)
              bbbb(j)=-(1/kk)*TaskInfor(5)+b(j);  % the itersection with x=TaskInfor(5)
              if b(j)>=0
                  LowerBound(j)=b(j);                       %和kk>0，UpperBound变成了LowerBound 
                  LowerPoint(j,:)=[0, LowerBound(j)];
              else
                  LowerPointX(j)=bb(j);
                  LowerPoint(j,:)=[LowerPointX(j), 0];
              end
              TLowerPoint(j,:)=a*(LowerPoint(j,:)-StartPoint)';

              
              if bbbb(j)<=TaskInfor(6)
                  UpperBound(j)=bbbb(j);
                  UpperPoint(j,:)=[TaskInfor(5),UpperBound(j)];
              else
                  UpperPointY(j)=bbb(j);
                  UpperPoint(j,:)=[UpperPointY(j), TaskInfor(6)];
              end
              TUpperPoint(j,:)=a*(UpperPoint(j,:)-StartPoint)';
          end
          
      end
      
      % calculate the itersection with obstacles of each verticle line    计算每条垂直线与障碍物的交点
      TaskNumber=1;                       %  这里的TaskNum 为何只有1
      [ObsNum,~]=size(ObstacleInfor);
      obsx=x; 
      Tfeasible=TaskInfor(1,6)*ones(TaskNumber,Num_WayPoints, ObsNum+1, 2);
      feasNum=zeros(TaskNumber, Num_WayPoints);
      %TInfeasible=TaskInfor(1,6)*ones(TaskNumber,Num_WayPoints, ObsNum, 2);
      maxgap=ones(TaskNumber,Num_WayPoints+2, 1, 2);
      for ii=1:TaskNumber
      for i=1:Num_WayPoints % for each bin in x direction
         % intersectionNum=zeros(1,Num_WayPoints);
          infeasible=[];
          feasible=[];
          for j=1:ObsNum
              %syms yy
              aa=ObstacleInfor(j,1);    %已经是转换后的坐标
              b=ObstacleInfor(j,2);
              r=ObstacleInfor(j,3);
              if abs(obsx(ii,i)-aa)<r
                   obsy1=(r^2-(obsx(ii,i)-aa)^2)^0.5+b;
                   obsy2=-(r^2-(obsx(ii,i)-aa)^2)^0.5+b;
                   %eq=(obsx(i)-a)^2+(yy-b)^2==r^2;
                   %s=solve(eq,yy);
                   infeasible=[infeasible; obsy2 obsy1];
                   %intersectionNum(i)=intersectionNum(i)+1; 
              end
          end
          if ~isempty(infeasible)
              [~,sortindex]=sort(infeasible(:,1));
              sinfeasible=infeasible(sortindex,:);
              ssinfeasible=[TLowerPoint(i,2) reshape(sinfeasible',1,[]) TUpperPoint(i,2)];

              %if sinfeasible(1,1)>0
              %   feasible=[feasible; 0 sinfeasible(1,1)];
              %end
              ifNum=length(ssinfeasible);
              lastBound=TLowerPoint(i,2);
              for k=2:2:ifNum
                  LastEdge=max(lastBound,ssinfeasible(k-1));    % ! different situations
                  if ssinfeasible(k)>LastEdge %ssinfeasible(k-1)
                     feasible=[feasible; LastEdge  ssinfeasible(k)];  %ssinfeasible(k-1)
                  else
                     lastBound=LastEdge;
                  end
              end
              gap=feasible(:,2)-feasible(:,1);
              [~, maxindex]=max(gap);
              maxgap(ii,i,1,:)=feasible(maxindex,:);
              [fNum,~]=size(feasible);
              Tfeasible(ii, i, 1:fNum,:)=feasible;    %最有疑问的是这个Tfeasible
              feasNum(ii,i)=fNum;                   
              %[iNum,~]=size(sinfeasible);
              %TInfeasible(ii, i, 1:iNum,:)=sinfeasible;
          else
              Tfeasible(ii, i, 1,:)=[TLowerPoint(i,2) TUpperPoint(i,2)];
              maxgap(ii,i,1,:)=[TLowerPoint(i,2) TUpperPoint(i,2)];
              feasNum(ii,i)=1;
          end
      end
      end
    
      Infor.maxgap=maxgap;
      %Infor.Tinfeasible=TInfeasible;
      Infor.feasible=Tfeasible;
      Infor.feasibleNum=feasNum;
      
      %b=a*(UpperBound-StartPoint)';
      %c=a*(LowerBound-StartPoint)';
      Bound=[TLowerPoint(:,2),TUpperPoint(:,2)]; % lowerBound, UpperBound  (D,2)
      %Bound=[-100, 100];
      Infor.Task=TTaskInfor;
      Infor.Threat=TThreatInfor;
      Infor.Obstacle=TObstacleInfor;
      Infor.x=x';
      Infor.Bound=Bound;
      Infor.Num_WayPoints=Num_WayPoints;
      
      Infor.a=a;
end