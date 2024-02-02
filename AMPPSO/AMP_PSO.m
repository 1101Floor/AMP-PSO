function solution = AMP_PSO(USV, SearchAgents, Max_iter)
%多种群PSO优化算法

PopsInitnum = 20;          %跟随着种群数

% 算法初始化
[PSOPops, ~] = PopsInit(USV, SearchAgents, false);   % 随机生成初始种群
ClassPops = PopsCluster(PSOPops, USV);                    % 进行初始聚类
dim = PSOPops.PosDim;                                                 % 状态变量维度
cSearchAgents = ClassPops.SearchAgents;                    % 搜索智能体个数（子种群数量）
SearchAgents = cSearchAgents * ClassPops.k;              % 对所有智能体个数进行修正（k的整数倍）
PSOPops.Pos = PSOPops.Pos(1:SearchAgents, :);       % 对种群进行修正

% 报错
if cSearchAgents < 4
    error('搜索智能体个数过少')
end 

% 初始化解
Glbest = zeros(ClassPops.k, dim);          % 全局最优解
Glbest_score = 1 ./ zeros(ClassPops.k, 1);     % 解适应度

Plbest = zeros(ClassPops.k, dim);            % 局部最优解
Plbest_score = 1 ./ zeros(ClassPops.k, 1);       % 解适应度

Gbest = zeros(ClassPops.k, dim);           % 跟随着全局
Gbest_score = 1 ./ zeros(ClassPops.k, 1);      % 解适应度

Fitness_list = zeros(ClassPops.k, Max_iter);  % 适应度曲线

Pops.PosDim = PSOPops.PosDim;  % 子种群
Pops.lb = PSOPops.lb;
Pops.ub = PSOPops.ub;

% 迭代求解
tic
fprintf('>>AMP-PSO 优化中    00.00%%')
for iter = 1 : Max_iter

    % ①  更新参数
    a = 2 - iter * 2 / Max_iter;               % 线性递减
    %a = 2 * cos((iter/Max_iter)*pi/2);   % 非线性递减
    c1 = -iter / Max_iter + 2;
    c2 = iter / Max_iter + 1;
    c3 = -iter / Max_iter + 2;
    
    % ②  聚类
    if iter > 1 
    ClassPops = PopsCluster(PSOPops, USV);
    end
    for k = 1 : ClassPops.k
        Positions = ClassPops.Pos{k};  

        % ③  寻找 α、β、δ 
        for i = 1 : cSearchAgents
            % 读取目标函数
            fitness = ClassPops.Fitness(k, i);

            % 更新解
            if fitness <= Glbest_score(k)  % 适应能力最强
                Glbest_score(k) = fitness;
                Glbest(k, :) = Positions(i, :);
            end 
            if fitness > Glbest_score(k) && fitness <= Plbest_score(k)
                Plbest_score(k) = fitness;
                Plbest(k, :) = Positions(i, :);
            end
            if fitness > Glbest_score(k) && fitness > Plbest_score(k) && fitness <= Gbest_score(k)
                Gbest_score(k) = fitness;
                Gbest(k, :) = Positions(i, :);
            end
        end

        % ④  更新位置和速度
        totalRandomNumbers = cSearchAgents * dim * 3; % 每个循环迭代需要3个随机数
        randomNumbers = rand(totalRandomNumbers, 3); % 为每个维度生成随机数
        idx = 0; % 用于访问随机数数组的索引

        for i = 1 : cSearchAgents
            for j = 1 : dim
                % 从预生成的随机数中获取值
                r1 = randomNumbers(idx + 1, 1);
                r2 = randomNumbers(idx + 1, 2);
                r3 = randomNumbers(idx + 1, 2);

                A1 = 2 * a * r1 - a;
                C1 = 2 * r2;
                D_Glbest = abs(C1 * Glbest(k, j) - Positions(i, j));
                velocityx = Glbest(k, j) - A1 * D_Glbest;
                
                A2 = 2 * a * r2 - a; 
                C2 = 2 * r2;
                D_Plbest = abs(C2 * Plbest(k, j) - Positions(i, j));
                velocityy = Plbest(k, j) - A2 * D_Plbest;

                A3 = 2 * a * r3 - a; % 同上
                C3 = 2 * r3;
                D_Gbest = abs(C3 * Gbest(k, j) - Positions(i, j));
                velocityz = Gbest(k, j) - A3 * D_Gbest;
                
                Positions(i, j) = (velocityx + velocityy + velocityz) / 3;

                idx = idx + 1; % 更新随机数索引
            end
        end

    % ⑤  调整不符合要求的状态变量
    Pops.Pos = Positions;
    ProbPoints =  ClassPops.ProbPoints{k};
    [Pops, ~] = BoundAdjust(Pops, ProbPoints, USV);

    % ⑥  存储适应度
    Fitness_list(k, iter) = Glbest_score(k);

    % ⑦  合并种群
    PSOPops.Pos(cSearchAgents*(k-1)+1:cSearchAgents*k, :) = Pops.Pos;

    end

    if iter/Max_iter*100 < 10
        fprintf('\b\b\b\b\b%.2f%%', iter/Max_iter*100)
    else
        fprintf('\b\b\b\b\b\b%.2f%%', iter/Max_iter*100)
    end
end
fprintf('\n\n>>计算完成！\n\n')
toc


% 寻找 α β δ 位置
n = 3; 
A = ClassPops.Fitness; 
t = findmin(A, n);
index = cSearchAgents * (t(:, 1) - 1) + t(:, 2);

real_Glbest_no = index(1);
real_Plbest_no = index(2);
real_Gbest_no = index(3);
Pbest_Data = ClassPops.Data{t(1, 1)}{t(1, 2)}  ;

% 输出值
solution.method = 'CAMP-PSO';                                 % 算法
% solution.ClassPops = ClassPops;                               % 分类信息
solution.WolfPops = PSOPops;                                % 所有解种群信息
solution.Tracks = Pops2Tracks(PSOPops, USV);    % 所有解航迹信息
solution.Fitness_list = mean(Fitness_list, 1);            % 所有α解的平均适应度曲线
solution.Glbest_Data = Pbest_Data;                       
solution.Glbest_no = real_Glbest_no;                          %位置
solution.Plbest_no = real_Plbest_no;                               % 位置
solution.Gbest_no = real_Gbest_no;                            % 位置

end



%% 寻找A矩阵中最小n个数的位置
function t = findmin(A, n)
    t = sort(A(:));
    [x, y] = find(A <= t(n), n);
    t = [x, y];         % 前n个最小项在矩阵A中的位置[行,列]
    B = zeros(n, 1);
    for i = 1 : n
        B(i) = A(t(i, 1), t(i, 2));
    end
    [~, index] = sort(B);
    t = t(index, :); % 前n个从小到大排序的位置
end



%% 对种群进行聚类
function [ClassPops] = PopsCluster(PSOPops, USV)

SearchAgents = size(PSOPops.Pos, 1);  % 智能体个数 
Dim = PSOPops.PosDim;                        % 智能体维度
Tracks = Pops2Tracks(PSOPops, USV); % 智能体转换成航迹信息

% 计算适应度
o_Fitness = zeros(SearchAgents, 1); % 60*1
o_subF = []; % 5*60
o_ProbPoints = cell(SearchAgents, 1);  % 60*1
o_Data = cell(SearchAgents, 1); % 60*1

% parfor 并行计算适应度
for i = 1:SearchAgents
    [fitness, subF, Data] = ObjFun(Tracks{i}, USV);
    o_ProbPoints{i} = Data.ProbPoint;  %cell-cell
    o_Data(i) = {Data}; %cell-struct
    o_Fitness(i) = fitness; %vector-var
    o_subF = [o_subF, subF]; 
end

% 分类
k = size(subF, 1);                                              % 分 k 类（由objfun决定）
cSearchAgents = floor(SearchAgents / k);    % 并行智能体个数
cFitness = zeros(k, cSearchAgents);               % 保存每类的适应度
cPositions = cell(k, 1);                                     % 保存每类的位置信息
cTracks = cell(k, 1);                                          % 保存每类的航迹信息
cProbPoints = cell(k, 1);                                  % 保存每类的有问题航迹点
cData = cell(k, 1);                                             % 存储每类的检测报告

% 排序
[~, Index] = sort(o_subF, 2, 'ascend') ;     % 沿维度2升序排序，返回新矩阵和序号
                                                                       % fitness越小越好
% 聚类
for i = 1:k
    Positions = zeros(cSearchAgents, Dim);
    batchTrack = cell(cSearchAgents, 1);
    batchProbPoints = cell(cSearchAgents, 1);
    batchData = cell(cSearchAgents, 1);
    for j = 1:cSearchAgents
        idx = Index(i, j);
        cFitness(i, j) = o_Fitness(idx); %mat-vector
        Positions(j, :) = PSOPops.Pos(idx, :); %mat-mat
        batchTrack{j} = Tracks{idx}; %cell-cell
        batchProbPoints{j} = o_ProbPoints{idx}; %cell-cell
        batchData{j} = o_Data{idx}; %cell-cell
    end
    cPositions(i) = {Positions}; %cell-mat
    cTracks{i} = batchTrack; %cell-cell
    cProbPoints{i} = batchProbPoints; %cell-cell
    cData{i} = batchData; %cell-cell
end

% 输出
ClassPops.Pos = cPositions;
ClassPops.Tracks = cTracks;
ClassPops.ProbPoints = cProbPoints;
ClassPops.Data = cData;
ClassPops.Fitness = cFitness;
ClassPops.SearchAgents = cSearchAgents;
ClassPops.k = k;
end
