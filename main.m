%"Yongjin Wang, Pengkai Chen, Yifan Wu, Weixuan Chen, Lijing Tan,
% Modified Continuous Ant Colony Optimisation with Local Search for Multiple Unmanned Aerial Vehicle Path Planning.
% In: Proceeding of the International Conference on Data-driven
% Optimization of Complex Systems. IEEE, pp. 557-564, 2024. (EI)"
% Contact: (qq3082622022@gmail.com)

for Dimension=40   % the number of waypoints  20 30 40 60
    %for Task=1:4   % 1:12
    for Task=1 % 1:12
        %% Path plannning
        mainPathPlanning (Dimension, Task)  %论文中用ACOPA_LS解决单UGV的路径规划
        
        %% Multi-agent coordination
        mainMultiagentCoordination (Dimension, Task)   %论文中用ACOR解决多个无人车协调问题

        %% plot
        pathplot (Dimension, Task)
    end
end