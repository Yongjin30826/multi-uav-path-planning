function pathplot(Dimension, TaskSet)
if nargin==0
    Dimension=40;   % the number of waypoints
    TaskSet=4:6;
end

% path planning task information and parameters
AlgorithmName={'ACOPA'};
%aa=['-*', '-p' ,'-+', '-d' ,'-s', '->', '-<', '--','-o','-^', '-v'];
OutputName={'MACOR-LS'};

for TaskIndex=[TaskSet]
    
    [TaskInfor, ThreatInfor, ObstacleInfor,  ~ ]=EnvironmentInfor(TaskIndex);
    [Ntask, ~]=size(TaskInfor);
    [NT,~]=size(ThreatInfor);
    [NO,~]=size(ObstacleInfor);
    SizeEn=TaskInfor(1,5);
    
    figure(TaskIndex);
    for AlgorithmIndex=[1]  
        %figure(AlgorithmIndex);
        FileName=strcat('ACOPA4PathPlanning/','results/',char(AlgorithmName(AlgorithmIndex)), 'Prob',int2str(TaskIndex), 'Dim',int2str(Dimension),'Path.txt');
        FindFile=fopen(FileName, 'r');
        Data=fscanf(FindFile,'%50f',[(Dimension+2),inf]);
        
        for i=1:Ntask
            X=Data(:,(i-1)*2+1)';
            Y=Data(:,i*2)';
            if AlgorithmIndex==10  % MFACODE4, the axis is not transformed 
                %StartPoint=TaskInfor(i, 1:2);
                %TargetPoint=TaskInfor(i, 3:4);
                d=dist(StartPoint, TargetPoint');
                k=1:Dimension;
                if TargetPoint(1)>StartPoint(1)
                     x=k.*(d/(Dimension+1)); % (1, Num_WayPoints)
                else
                     x=-k.*(d/(Dimension+1));
                end
                y=Y';
                Waypoints=[x' y(2:1+Dimension)];
                d=dist(StartPoint, TargetPoint');
                Theta=atan((TargetPoint(2)-StartPoint(2))/(TargetPoint(1)-StartPoint(1)));
                a=[cos(Theta) sin(Theta); -sin(Theta), cos(Theta)];
                for ii=1:Dimension
                    RWaypoints(ii,:)=(a\Waypoints(ii,:)')';
                end
                RWaypoints=RWaypoints+StartPoint;
                Path=[StartPoint; RWaypoints; TargetPoint];  
                X=Path(:,1);
                Y=Path(:,2);
            end
            %aa(AlgorithmIndex*2-1:AlgorithmIndex*2)
           
            plot(X,Y,'-o', 'MarkerSize',4);
            hold on
             plot(X(1),Y(1),'o','MarkerFaceColor','k','MarkerEdgeColor','b', 'MarkerSize',6); % plot the start and 
    hold on
    plot( X(end), Y(end),'s','MarkerFaceColor','k','MarkerEdgeColor','b', 'MarkerSize',6); % plot the start and 
    
        end
    end
    
    xlabel('x');
    ylabel('y');
    
    for ThreatIndex=1:NT
        r=ThreatInfor(ThreatIndex,3);
        xt=ThreatInfor(ThreatIndex,1);
        yt=ThreatInfor(ThreatIndex,2);
        theta = linspace(0,2*pi);
        x = r*cos(theta) + xt;
        y = r*sin(theta) + yt;
        plot(x,y,'--b');
        
        r1=r-1;
        x1 = r1*cos(theta) + xt;
        y1 = r1*sin(theta) + yt;
        plot(x1,y1,'b');
    end
    
    for ObstacleIndex=1:NO
        r=ObstacleInfor(ObstacleIndex,3);
        xt=ObstacleInfor(ObstacleIndex,1);
        yt=ObstacleInfor(ObstacleIndex,2);
        theta = linspace(0,2*pi);
        x = r*cos(theta) + xt;
        y = r*sin(theta) + yt;
        plot(x,y,'--r');
        
        r1=r-1;
        x1 = r1*cos(theta) + xt;
        y1 = r1*sin(theta) + yt;
        plot(x1,y1,'r');
    end 
    
   xlim([0 SizeEn])
   ylim([0 SizeEn])
   TitleName=strcat(char(OutputName(AlgorithmIndex)), '\_D', int2str(Dimension), '\_Case',int2str(TaskIndex));
   title(TitleName);
   FigureName=strcat('ACOPA4PathPlanning/','results/', char(OutputName(AlgorithmIndex)),'D', int2str(Dimension), 'P',int2str(TaskIndex));
   print(FigureName,'-dpng');
   fclose ('all');
end
end
