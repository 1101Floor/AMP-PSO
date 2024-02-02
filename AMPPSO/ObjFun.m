function [F, subF, Data] = ObjFun(Track, USV)
%OBJFUN 目标函数、适应度函数（一个agent的）

% 多目标优化权重设置（必需为行向量）
weight = [ 0.2, 0.1, 0.3, 0.1, 0.3 ]; % 默认权重


% 表达式系数调整（将各项指标无量纲化）
p1 = 10;   % 燃料项（已除以最大航程）
p21 = 1; % 高度项
p22 = 10; % 低度项
p31 = 5000; % 硬威胁
p32 =1; % 软威胁
p4 = 1.1;   % 时间同步项
p5 = 10;   % 碰撞项


% 进行航迹检测
report = TrackDetect(Track, USV);  % Track 为 struck结构

% 路径
ZZ = sum(USV.limt.L);
MaxL_mt = ZZ(2);
f_o = p1 * report.L_mt / MaxL_mt;

% 高度
dim = USV.PointDim;
if dim < 3
    f_h = 0;
else
    f_h = 0;
    for i = 1 : USV.num
        Hmax = USV.limt.h(i, 2);
        Hmin = USV.limt.h(i, 1); 
        for k = 1 : USV.PointNum(i)
            z = Track.P{i}(3, k);
            if z>Hmax
                fk = p21 * (z - Hmax);
            elseif z >= Hmin
                fk = 0;
            else
                fk = p22 * (Hmin - z);
            end
            f_h = f_h + fk;
        end
    end
end

% 威胁
O_r = USV.Menace.radar(: ,1:end-1);                        
O_o = USV.Menace.other(: ,1:end-1);              
f_t = 0;  % 威胁代价
for i = 1 : USV.num
    for k = 1 : USV.PointNum(i)
        P = Track.P{i}(:, k)' ;  % 转置成 1*dim
        for m = 1 : size(O_r, 1)
            fk = p31 / (norm(P - O_r(m, :)))^4;
            f_t = f_t + fk;
        end
        for m = 1 : size(O_o, 1)
            fk = p32 / norm(P - O_o(m, :));
            f_t = f_t + fk;
        end
    end
end


f_m = 0;
for i = 1 : USV.num
    Li = report.L(i);
    tmax = Li / USV.limt.v(i,1);
    tmin = Li / USV.limt.v(i,2);
    ti = report.time(i);
    tc = USV.tc;
    if tc <= tmax && tc >= tmin
        fk = 0;
    else
        fk = p4 * abs(ti - tc);
    end
    f_m = f_m + fk;
end


f_c = p5 * report.col_times;


% 目标函数分量
subF = [ f_o; f_h; f_t; f_m; f_c ]; % 5*1


% 加权目标函数
F = weight * subF ;

% 输出信息
Data.ProbPoint = report.ProbPoint;      % 所有有问题的点
Data.AngleProb = report.AngleProb;    % 不满足角度约束的点
Data.TrajProb = report.TrajProb;           % 不满足最小航迹间隔的点
Data.Threat = report.Threat;                  % 受威胁的点

Data.L = report.L;                                      % 每个无人船的航程
Data.t = report.time;                                 % 每个无人船的时间
Data.c = report.col_times;                         % 所有无人船总碰撞次数


end

