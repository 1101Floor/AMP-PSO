function solution = AMP_PSO(USV, SearchAgents, Max_iter)
%����ȺPSO�Ż��㷨

PopsInitnum = 20;          %��������Ⱥ��

% �㷨��ʼ��
[PSOPops, ~] = PopsInit(USV, SearchAgents, false);   % ������ɳ�ʼ��Ⱥ
ClassPops = PopsCluster(PSOPops, USV);                    % ���г�ʼ����
dim = PSOPops.PosDim;                                                 % ״̬����ά��
cSearchAgents = ClassPops.SearchAgents;                    % �������������������Ⱥ������
SearchAgents = cSearchAgents * ClassPops.k;              % �������������������������k����������
PSOPops.Pos = PSOPops.Pos(1:SearchAgents, :);       % ����Ⱥ��������

% ����
if cSearchAgents < 4
    error('�����������������')
end 

% ��ʼ����
Glbest = zeros(ClassPops.k, dim);          % ȫ�����Ž�
Glbest_score = 1 ./ zeros(ClassPops.k, 1);     % ����Ӧ��

Plbest = zeros(ClassPops.k, dim);            % �ֲ����Ž�
Plbest_score = 1 ./ zeros(ClassPops.k, 1);       % ����Ӧ��

Gbest = zeros(ClassPops.k, dim);           % ������ȫ��
Gbest_score = 1 ./ zeros(ClassPops.k, 1);      % ����Ӧ��

Fitness_list = zeros(ClassPops.k, Max_iter);  % ��Ӧ������

Pops.PosDim = PSOPops.PosDim;  % ����Ⱥ
Pops.lb = PSOPops.lb;
Pops.ub = PSOPops.ub;

% �������
tic
fprintf('>>AMP-PSO �Ż���    00.00%%')
for iter = 1 : Max_iter

    % ��  ���²���
    a = 2 - iter * 2 / Max_iter;               % ���Եݼ�
    %a = 2 * cos((iter/Max_iter)*pi/2);   % �����Եݼ�
    c1 = -iter / Max_iter + 2;
    c2 = iter / Max_iter + 1;
    c3 = -iter / Max_iter + 2;
    
    % ��  ����
    if iter > 1 
    ClassPops = PopsCluster(PSOPops, USV);
    end
    for k = 1 : ClassPops.k
        Positions = ClassPops.Pos{k};  

        % ��  Ѱ�� �����¡��� 
        for i = 1 : cSearchAgents
            % ��ȡĿ�꺯��
            fitness = ClassPops.Fitness(k, i);

            % ���½�
            if fitness <= Glbest_score(k)  % ��Ӧ������ǿ
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

        % ��  ����λ�ú��ٶ�
        totalRandomNumbers = cSearchAgents * dim * 3; % ÿ��ѭ��������Ҫ3�������
        randomNumbers = rand(totalRandomNumbers, 3); % Ϊÿ��ά�����������
        idx = 0; % ���ڷ�����������������

        for i = 1 : cSearchAgents
            for j = 1 : dim
                % ��Ԥ���ɵ�������л�ȡֵ
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

                A3 = 2 * a * r3 - a; % ͬ��
                C3 = 2 * r3;
                D_Gbest = abs(C3 * Gbest(k, j) - Positions(i, j));
                velocityz = Gbest(k, j) - A3 * D_Gbest;
                
                Positions(i, j) = (velocityx + velocityy + velocityz) / 3;

                idx = idx + 1; % �������������
            end
        end

    % ��  ����������Ҫ���״̬����
    Pops.Pos = Positions;
    ProbPoints =  ClassPops.ProbPoints{k};
    [Pops, ~] = BoundAdjust(Pops, ProbPoints, USV);

    % ��  �洢��Ӧ��
    Fitness_list(k, iter) = Glbest_score(k);

    % ��  �ϲ���Ⱥ
    PSOPops.Pos(cSearchAgents*(k-1)+1:cSearchAgents*k, :) = Pops.Pos;

    end

    if iter/Max_iter*100 < 10
        fprintf('\b\b\b\b\b%.2f%%', iter/Max_iter*100)
    else
        fprintf('\b\b\b\b\b\b%.2f%%', iter/Max_iter*100)
    end
end
fprintf('\n\n>>������ɣ�\n\n')
toc


% Ѱ�� �� �� �� λ��
n = 3; 
A = ClassPops.Fitness; 
t = findmin(A, n);
index = cSearchAgents * (t(:, 1) - 1) + t(:, 2);

real_Glbest_no = index(1);
real_Plbest_no = index(2);
real_Gbest_no = index(3);
Pbest_Data = ClassPops.Data{t(1, 1)}{t(1, 2)}  ;

% ���ֵ
solution.method = 'CAMP-PSO';                                 % �㷨
% solution.ClassPops = ClassPops;                               % ������Ϣ
solution.WolfPops = PSOPops;                                % ���н���Ⱥ��Ϣ
solution.Tracks = Pops2Tracks(PSOPops, USV);    % ���н⺽����Ϣ
solution.Fitness_list = mean(Fitness_list, 1);            % ���Ц����ƽ����Ӧ������
solution.Glbest_Data = Pbest_Data;                       
solution.Glbest_no = real_Glbest_no;                          %λ��
solution.Plbest_no = real_Plbest_no;                               % λ��
solution.Gbest_no = real_Gbest_no;                            % λ��

end



%% Ѱ��A��������Сn������λ��
function t = findmin(A, n)
    t = sort(A(:));
    [x, y] = find(A <= t(n), n);
    t = [x, y];         % ǰn����С���ھ���A�е�λ��[��,��]
    B = zeros(n, 1);
    for i = 1 : n
        B(i) = A(t(i, 1), t(i, 2));
    end
    [~, index] = sort(B);
    t = t(index, :); % ǰn����С���������λ��
end



%% ����Ⱥ���о���
function [ClassPops] = PopsCluster(PSOPops, USV)

SearchAgents = size(PSOPops.Pos, 1);  % ��������� 
Dim = PSOPops.PosDim;                        % ������ά��
Tracks = Pops2Tracks(PSOPops, USV); % ������ת���ɺ�����Ϣ

% ������Ӧ��
o_Fitness = zeros(SearchAgents, 1); % 60*1
o_subF = []; % 5*60
o_ProbPoints = cell(SearchAgents, 1);  % 60*1
o_Data = cell(SearchAgents, 1); % 60*1

% parfor ���м�����Ӧ��
for i = 1:SearchAgents
    [fitness, subF, Data] = ObjFun(Tracks{i}, USV);
    o_ProbPoints{i} = Data.ProbPoint;  %cell-cell
    o_Data(i) = {Data}; %cell-struct
    o_Fitness(i) = fitness; %vector-var
    o_subF = [o_subF, subF]; 
end

% ����
k = size(subF, 1);                                              % �� k �ࣨ��objfun������
cSearchAgents = floor(SearchAgents / k);    % �������������
cFitness = zeros(k, cSearchAgents);               % ����ÿ�����Ӧ��
cPositions = cell(k, 1);                                     % ����ÿ���λ����Ϣ
cTracks = cell(k, 1);                                          % ����ÿ��ĺ�����Ϣ
cProbPoints = cell(k, 1);                                  % ����ÿ��������⺽����
cData = cell(k, 1);                                             % �洢ÿ��ļ�ⱨ��

% ����
[~, Index] = sort(o_subF, 2, 'ascend') ;     % ��ά��2�������򣬷����¾�������
                                                                       % fitnessԽСԽ��
% ����
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

% ���
ClassPops.Pos = cPositions;
ClassPops.Tracks = cTracks;
ClassPops.ProbPoints = cProbPoints;
ClassPops.Data = cData;
ClassPops.Fitness = cFitness;
ClassPops.SearchAgents = cSearchAgents;
ClassPops.k = k;
end
