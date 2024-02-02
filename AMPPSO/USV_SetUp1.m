function USV = USV_SetUp5

USV.S = [         1,           3,       16;
                 3,        1,          16;
                
                ];     % 起点位置 (x,y)或(x,y,z)

USV.G = [         18,       19,      4;
                  19,       19,      5;
                
                   ];      % 目标位置 (x,y)或(x,y,z)

USV.PointNum = [                 30;
                                 30;
                                 
                                  ];                 % 每个无人船导航点个数（起始点之间点的个数）

USV.PointDim = size(USV.S, 2);        % 坐标点维度 （由 起点 坐标决定）
USV.num = size(USV.S, 1);                 % UAV数量 （由 起点 个数决定）


% 威胁点设置 (x,y,r) 或 (x,y,z,r)
% （每行为一个威胁的坐标和半径）
USV.Menace.radar = [    10,10,10,3;  
                     % 15,    7,    10,     2;  
                      % 17,    18,   8,      2; 
                      %  8,      8,     0,      6.8;  
                     % 8,      17,   8,      3;      
                      ];   %硬威胁

USV.Menace.other = [ 8,2,12,2;
   6,8, 4, 1.5;
    18,10,4,2; 
                      %  11,   13,    13,     2.3;
                      %  13,   14,     9,      1.8;  
                     %  18,   13,    10,     2.2; 
                     % 16,   16,     0,      4;    
                      ];   %软威胁


% 无人机约束设置（min,max)
% （可单独为每个无人机设置，每行为一个无人机约束的上下限）
USV.limt.v = 0.002*repmat([0.3, 0.7], USV.num, 1);                 % 速度约束 
USV.limt.phi = deg2rad(repmat([-35, 35], USV.num, 1));      % 转角约束
USV.limt.theta = deg2rad(repmat([-20, 20], USV.num, 1));   % 倾角约束 
USV.limt.h = repmat([0.02, 20], USV.num, 1);                         % 高度约束 
USV.limt.x = repmat([0, 20], USV.num, 1);                            % 位置x约束 
USV.limt.y = repmat([0, 20], USV.num, 1);                            % 位置y约束 
USV.limt.z = USV.limt.h;                                                            % 位置z约束 （忽略地球弧度）
USV.limt.L = zeros(USV.num, 2);                                              % 航程约束 （最短航迹片段2km，最大航程1.5倍起始距离）
for i =1:USV.num
    zz.max = 1.6 * norm(USV.G(i, :) - USV.S(i, :));
    zz.min = 0.5;
    USV.limt.L(i, :) = [zz.min, zz.max];
end

USV.tc = 21000;         % 协同时间 （单位s）
USV.ds = 0.5;          % 安全距离 （单位km）


% 报错
ErrorCheck(USV)
end





%% 程序自检
function ErrorCheck(UAV)

dim = UAV.PointDim; 
if dim ~= size(UAV.G,2) || dim ~= size(UAV.Menace.radar,2)-1 || dim ~= size(UAV.Menace.other,2)-1
    if dim ~= size(UAV.G,2)
        error('仿真维度为%d，但目标点坐标为%d维', dim, size(UAV.G,2))
    else
        error('仿真维度为%d，但威胁点坐标为%d维', dim, size(UAV.Menace.radar,2)-1)
    end
end

num = UAV.num;
if num ~= size(UAV.G,1) || num ~= size(UAV.limt.v,1) || num ~= size(UAV.limt.phi,1) ...
        || num ~= size(UAV.limt.theta,1) || num ~= size(UAV.limt.h,1) || num ~= size(UAV.limt.x,1) ...
        || num ~= size(UAV.limt.y,1) || num ~= size(UAV.limt.z,1) || num ~= size(UAV.limt.L,1)
    if num ~= size(UAV.G,1)
        error('无人船个数为%d, 但目标点有%d个', num, size(UAV.G,1))
    else
        error('约束条件个数与无人船个数不一致')
    end
end

if num ~= size(UAV.PointNum, 1)
    error('无人船个数为%d, 但为%d个无人船设置了导航点', num, size(UAV.PointNum, 1))
end

MaxPoint = floor(UAV.limt.L(:,2) ./ UAV.limt.L(:,1)) - 1;   % 每个无人船支持的最大航迹点数量
for i = 1 : UAV.num
    if UAV.PointNum(i) > MaxPoint(i)
        error('%d号无人船导航点个数超出任务需求，请尝试减少导航点个数', i)
    end
end

end
