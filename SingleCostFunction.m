
% ParticleSwarm: (Dimension, SwarmSize)
function FitValue=SingleCostFunction(ParticleSwarm, ModelInfor, AgentIndex)

%AgentIndex=1;
%flag_collisionUAV=0;  % 0 do not calculate the collision among UAVs; 1 otherwise
flag_threat=0; % one way to calculate it
[Dimension, SwarmSize]=size(ParticleSwarm); % Dimension is the number of waypoints
X=ModelInfor.x; % Dimension, AgentNumber
[~, AgentNumber]=size(X);
if AgentNumber==1   % means the coordinate system has been transformed for each agent 每个代理的坐标都已转换
    AgentIndex=1;
end
Y=ModelInfor.y; % Dimension, AgentNumber, the y axis of waypoints of all agents
SX=X(:,AgentIndex);
Threat=ModelInfor.Threat;
Obstacle=ModelInfor.Obstacle;
Task=ModelInfor.Task;
Penalty=3;
%TimeSafe=0.1;
%Velocity=10;
%d_safe=1;

if Task(5)<=100
    MaximumLength=300;
else
    MaximumLength=500;
end
    
StartPoint=Task(AgentIndex,1:2);
TargetPoint=Task(AgentIndex,3:4);
ST=dist(StartPoint, TargetPoint');  % the length of the straight line connencting the starting point and the target point

for i=1: SwarmSize
   SY=ParticleSwarm(:, i);
   if ~isreal (SY)    %  如果SY存在虚数部分，则37行将保留只包含实数部分，去掉虚数部分
       SY=real(SY);
       %FitValue(i)=nan;
       %break;
   end
   Waypoints=[SX SY];
   Path=[StartPoint; SX SY; TargetPoint]; % (D+2,2)  the path of the UAV 
   
   %% Calculate the cost associated with the total length \in [0,1]
   d=Path(2:Dimension+2,:)-Path(1:Dimension+1,:);
   PathLength=sum((sum(d.*d,2)).^0.5);
   CLength=1-(ST/PathLength);
   
   %% Calculate the cost associated with turning \in [0,1] 
   for ii=2:Dimension+1
       %Theta=d(ii,:)*d(ii-1,:)'/(d(ii,:)*d(ii,:)'^0.5)*((d(ii,:)*d(ii,:)'^0.5));
       turning(ii-1)=dot(d(ii,:),d(ii-1,:))/(norm(d(ii,:))*norm(d(ii-1,:)));
   end
   CTurning=(1-mean(turning))/2;  
   
    %% Calculate the cost associated with the threats \in [0,1] 
    if  ~isempty (Threat) 
       [n, ~]=size(Threat);  % n: the number of Threats 
       dThreat=pdist2(Path, Threat(:,1:2)); %dd(i,j) is the distance between i (D+2) point and j Threat
       dDanger=0;
       for j=1:n  % for each Threat
           for k=2:Dimension+2  % for each point. Note: the start point is collision free
              x1=Path(k-1,1); y1=Path(k-1,2);
              x2=Path(k,1); y2=Path(k,2);
              x3=Threat(j,1);y3=Threat(j,2);r=Threat(j,3);
              A=(x2-x1)^2+(y2-y1)^2; %A>0
              B=2*((x2-x1)*(x1-x3)+(y2-y1)*(y1-y3));
              C=x3^2+y3^2+x1^2+y1^2-2*(x3*x1+y3*y1)-r^2;
              delta=B^2-4*A*C;

              if delta<=0 %&& dThreat(k,j)>=Threat(j,3)     圆与线段没有交点
                  % do nothing, no intersection
              else
                  mu1=(-B+delta^0.5)/(2*A); mu2=(-B-delta^0.5)/(2*A); % mu2<mu1
                  x_int1=x1+mu1*(x2-x1); y_int1= y1+mu1*(y2-y1);   % the point of intersection 
                  x_int2=x1+mu2*(x2-x1); y_int2= y1+mu2*(y2-y1);            % mu1和mu2  分别是交点到端点的线段长度占线段总长度的比例
                  if dThreat(k,j)>=Threat(j,3) % x2 the point is outside the Threat   第二个点在圆外
                     if dThreat(k-1,j)>=Threat(j,3)                                  % 第一个点也在圆外
                         if (mu1<1 && mu1>0 ) && (mu2<1 && mu2>0)
                         d=((x_int1-x_int2)^2+(y_int1-y_int2)^2)^0.5;
                         dDanger=dDanger+d;
                         else
                             % do nothing
                         end
                     else                                                           %   第二个点在圆内
                         d=((x1-x_int1)^2+(y1-y_int1)^2)^0.5;
                         dDanger=dDanger+d;
                     end
                  else                                                               %第二个点在圆内
                     if dThreat(k-1,j)>=Threat(j,3)                                  %第一个点在圆外
                         d=((x2-x_int2)^2+(y2-y_int2)^2)^0.5;
                         dDanger=dDanger+d;
                     else                                                            %第一个点也在圆内
                         d=A^0.5;
                         dDanger=dDanger+d;
                     end
                  end

              end

           end
       end
       if dDanger>0
           CDanger=dDanger/PathLength;
       else
           CDanger=0;
       end

       if CDanger>1
           CDanger=1;
       end
    else
        CDanger=0;
    end
   
    %% Another way to calculate the cost associated with the threats \in [0,1]
    if flag_threat==1  
    if ~isempty (Threat)
       [m, ~]=size(Threat);  % m: the number of threats 
       dThreat=pdist2(Path, Threat(:,1:2)); % dThreat(i,j) is the distance between i (D+2) point and j threat

       for j=1:m  % for each threat
           for k=2:Dimension+2  % for each point/line segment
              if dThreat(k,j)<=Threat(j,3) 
                   if dThreat(k-1,j)<=Threat(j,3)  % the segment is in the circle
                       TS(k)=norm(Path(k,:)-Path(k-1,:));
                   else                            % intersect 
                       x1=Path(k-1,1); y1=Path(k-1,2);
                       x2=Path(k,1); y2=Path(k,2);
                       x3=Threat(j,1);y3=Threat(j,2);r=Threat(j,3);
                       A=(x2-x1)^2+(y2-y1)^2;
                       B=2*((x2-x1)*(x1-x3)+(y2-y1)*(y1-y3));
                       C=x3^2+y3^2+x1^2+y1^2-2*(x3*x1+y3*y1)-r^2;
                       delta=B^2-4*A*C;
                       mu1=(-B+delta^0.5)/(2*A); mu2=(-B-delta^0.5)/(2*A);
                       if  0<=mu1&& mu1<=1
                           TS(k)=norm(Path(k,:)-Path(k-1,:))*(1-mu1);
                       else
                           if  0<=mu2 && mu2<=1
                               TS(k)=norm(Path(k,:)-Path(k-1,:))*(1-mu2);
                           else
                               TS(k)=0;
                           end
                       end

                   end
              else 
                  if dThreat(k-1,j)<=Threat(j,3)   % intersect
                      x1=Path(k-1,1); y1=Path(k-1,2);
                      x2=Path(k,1); y2=Path(k,2);
                      x3=Threat(j,1);y3=Threat(j,2);r=Threat(j,3);
                      A=(x2-x1)^2+(y2-y1)^2;
                      B=2*((x2-x1)*(x1-x3)+(y2-y1)*(y1-y3));
                      C=x3^2+y3^2+x1^2+y1^2-2*(x3*x1+y3*y1)-r^2;
                      delta=B^2-4*A*C;
                      mu1=(-B+delta^0.5)/(2*A); mu2=(-B-delta^0.5)/(2*A);
                      if  0<=mu1&& mu1<=1
                           TS(k)=norm(Path(k,:)-Path(k-1,:))*mu1;
                       else
                           if  0<=mu2 && mu2<=1
                               TS(k)=norm(Path(k,:)-Path(k-1,:))*mu2;
                           else
                               TS(k)=0;
                           end
                       end
                  else
                      x1=Path(k-1,1); y1=Path(k-1,2);
                      x2=Path(k,1); y2=Path(k,2);
                      x3=Threat(j,1);y3=Threat(j,2);r=Threat(j,3);
                      A=(x2-x1)^2+(y2-y1)^2;
                      B=2*((x2-x1)*(x1-x3)+(y2-y1)*(y1-y3));
                      C=x3^2+y3^2+x1^2+y1^2-2*(x3*x1+y3*y1)-r^2;
                      delta=B^2-4*A*C;
                      if delta<=0      % no intersection
                          TS(k)=0;
                      else             % two intersections 
                          mu1=(-B+delta^0.5)/(2*A); mu2=(-B-delta^0.5)/(2*A);
                         TS(k)=norm(Path(k,:)-Path(k-1,:))*(mu1-mu2);
                      end
                  end
              end
           end
       end
       CDanger=sum(TS)/sum(Threat(:,3));
       if CDanger>1
           CDanger=1;
       end
    else
        CDanger=0;
    end
    end
   

   %% Calculate the cost associated with the collision with obstacles \in [p,p+1]
   [n, ~]=size(Obstacle);  % n: the number of obstacles 
   dObstacle=pdist2(Path, Obstacle(:,1:2)); %dd(i,j) is the distance between i (D+2) point and j obstacle
   dCollision=0;
   for j=1:n  % for each obstacle
       for k=2:Dimension+2  % for each point. Note: the start point is collision free
          %if dObstacle(k,j)<Obstacle(j,3)
          %     Collision=Collision+1;
          %else 
          x1=Path(k-1,1); y1=Path(k-1,2);
          x2=Path(k,1); y2=Path(k,2);
          x3=Obstacle(j,1);y3=Obstacle(j,2);r=Obstacle(j,3);
          A=(x2-x1)^2+(y2-y1)^2; %A>0
          B=2*((x2-x1)*(x1-x3)+(y2-y1)*(y1-y3));
          C=x3^2+y3^2+x1^2+y1^2-2*(x3*x1+y3*y1)-r^2;
          delta=B^2-4*A*C;
          
          if delta<=0 %&& dObstacle(k,j)>=Obstacle(j,3)
              % do nothing, no intersection
          else
              mu1=(-B+delta^0.5)/(2*A); mu2=(-B-delta^0.5)/(2*A); % mu2<mu1
              x_int1=x1+mu1*(x2-x1); y_int1= y1+mu1*(y2-y1);   % the point of intersection
              x_int2=x1+mu2*(x2-x1); y_int2= y1+mu2*(y2-y1);
              if dObstacle(k,j)>=Obstacle(j,3) % x2 the point is outside the obstacle
                 if dObstacle(k-1,j)>=Obstacle(j,3)
                     if (mu1<1 && mu1>0 ) && (mu2<1 && mu2>0)
                     d=((x_int1-x_int2)^2+(y_int1-y_int2)^2)^0.5;
                     dCollision=dCollision+d;
                     else
                         % do nothing
                     end
                 else
                     d=((x1-x_int1)^2+(y1-y_int1)^2)^0.5;
                     dCollision=dCollision+d;
                 end
              else
                 if dObstacle(k-1,j)>=Obstacle(j,3)
                     d=((x2-x_int2)^2+(y2-y_int2)^2)^0.5;
                     dCollision=dCollision+d;
                 else
                     d=A^0.5;
                     dCollision=dCollision+d;
                 end
              end    
          end
       end
   end
   if dCollision>0
       CCollision=Penalty*(1+dCollision); %/PathLength; !!!!
   else
       CCollision=0;
   end
   
    
   %% Calculate the cost associated with the fuel represented by fly length \in [p,p+1]
   if PathLength<=MaximumLength
       CFuel=0;
   else
       CFuel=Penalty+(PathLength-MaximumLength)/MaximumLength;
   end
   
   FitValue(i)=CLength+CDanger+CTurning+ CCollision+CFuel;
   
end

end