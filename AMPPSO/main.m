clc, close all

%--- 算法选择 1：APSO算法  2：PSO算法  3：CS-PSO算法 4：MP_PSO算法 5:A-PSO算法 6:GA算法
options = 1;

%--- 算法参数设置
SearchAgents = 50;       % 种群数量
Max_iter = 200;          % 最大搜索步数

%--- 协同无人船设置
USV = USV_SetUp1;        % 在 USV_SetUp.m 文件进行设置

%--- 选择算法
if options == 1
    solution = AMP_PSO(USV, SearchAgents, Max_iter);  % PSO算法
elseif options == 2
    solution = PSO(USV, SearchAgents, Max_iter);  % CAMP-PSO算法
elseif options == 3
    solution = CS_PSO(USV, SearchAgents, Max_iter);  % CS-PSO算法
elseif options == 4
    solution = MP_PSO(USV, SearchAgents, Max_iter);  % MP_PSO算法
elseif options == 5
    solution = A_PSO(USV, SearchAgents, Max_iter);  % A-PSO算法
elseif options == 6
    solution = A_GA(USV, SearchAgents, Max_iter);   % GA算法
end

%--- 绘图
IMGPlot(solution, USV) % 自适应绘图（全自动绘图，但不如手动的好看）