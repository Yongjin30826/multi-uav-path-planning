function [Cost]=MultiCoordinationCost(Path, PathLength, Speed, Num_Agent, SafetyDistance, SafetyTime,Num_WayPoints)
[Dimension, SwarmSize]=size(Speed);

for ii=1:SwarmSize
    
    NTask=Num_Agent;
    for i=1:Num_Agent
        time(:,i)=PathLength(:,i)/Speed(i,ii);  % record the time spending on each line segment beween two waypoints; Num_WayPoints+1
        %N_InsertPoint=ceil(PathLength(:,i)/1);
        %此时SwarmSize是20个速度粒子，此时第一次time是第一个速度粒子在PathLength上的时间
    end
    Ttime=sum(time,1);   % 对矩阵的列进行求和
    % calculate the ETA
    [ETA,~]=max(Ttime);
    %[~,index]=sort(Ttime, 'descend');
    % check the collisions
    N_collision=0;

    for j=2:NTask
        for jj=1:j-1   % the previous i-1 UAV
            distance=dist(Path(:,(j-1)*2+1:j*2),Path(:,(jj-1)*2+1:jj*2)');     % 64*64，A矩阵第一行与B矩阵第一列距离，A矩阵第一行与B矩阵第二列距离...（遍历，总共计算64*64次）
            Collision=find(distance<=SafetyDistance);        % 满足条件的索引值；按列来累计索引值
            if ~isempty(Collision)   % there is collision
                nc=length(Collision);
                for iii=1:nc
                    w2=ceil(Collision(iii)/(Num_WayPoints+2)); % the w2 point of the UAV jj among Num_WayPoints+2 points    出现在矩阵diatance的第几列
                    % w1=mod(Collision(iii),(Num_WayPoints+2)); % the w1 point of UAV j
                    w1=Collision(iii)-(w2-1)*(Num_WayPoints+2); % the w1 point of UAV j                                     出现在矩阵diatance的第几行
                    % calculate the time 
                    if ~(w1==1 && w2==1)
                        time_j=sum(time(1:w1-1,j));
                        time_jj=sum(time(1:w2-1,jj));
                        TimeInteval=abs(time_j-time_jj);  
                        if TimeInteval < SafetyTime  
                            N_collision=N_collision+1;

                        end
                    end
                end
            end
        end
    end

    Cost(ii)=ETA + 100000* N_collision;
end


end