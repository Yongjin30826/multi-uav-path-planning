%Recover the transformed cordinates of waypoints to get the path
% x: the x axis of waypoints
% y: the y axis of waypoints
% RWaypoints: the recovered waypoints
function [Path,Gbest] = CordinatesRecover(Modelinfor, y, OModelinfor, AgentIndex)
a=Modelinfor.a;

x=Modelinfor.x; % (n,1)  Num_WayPoints,TaskNumber
%[Num_WayPoints,TaskNumber]=size(wx); 
Taskinfor=OModelinfor.Task;
StartPoint=Taskinfor(AgentIndex,1:2);
TargetPoint=Taskinfor(AgentIndex,3:4);
Waypoints=[x y]; %(n,2)
[n, ~]=size(Waypoints);

for i=1:n
    RWaypoints(i,:)=(a\Waypoints(i,:)')';
end
RWaypoints=RWaypoints+StartPoint;
Path=[StartPoint; RWaypoints; TargetPoint];   % ?D+2? 2?
Gbest=Path(2:1+n,2);
end