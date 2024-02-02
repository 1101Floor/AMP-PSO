function [PSOPops, Tracks] = PopsInit(USV, SearchAgents, GreedyInit)
%POPSINIT ��ʼ����Ⱥ"λ��"
if (nargin<3)
GreedyInit = false;
end

% �����Լ�
if SearchAgents < 4
    error('�����������������')
end


% �� �����ʼ����ռ�
dim = USV.PointDim;  % ������ά�� 

% �ٶ� n�����˴�
lb_v = USV.limt.v(:, 1)' ;    % 1*n
ub_v = USV.limt.v(:, 2)' ;   % 1*n
V0 = rand(SearchAgents, USV.num) .* repmat(ub_v - lb_v, SearchAgents, 1) + repmat(lb_v, SearchAgents, 1);
%  ����USV��ÿһ�б�ʾÿ�Ҵ��Ĳ���
%  ����Agent��ÿһ�б�ʾÿ��������Ĳ������б�ʾ���Ĳ���ƽ��

% ������ 
P0 = [];
for i = 1 : USV.num
    lb = [USV.limt.x(i, 1), USV.limt.y(i, 1), USV.limt.z(i, 1)] ;     % 1*3 ά
    ub = [USV.limt.x(i, 2), USV.limt.y(i, 2), USV.limt.z(i, 2)] ;    % 1*3 ά
    if dim < 3
        lb = lb(1:2);      % 1*2 ά
        ub = ub(1:2);    % 1*2 ά
    end
    PointNum = USV.PointNum(i);                  % ���������
    P_i = rand(SearchAgents, dim*PointNum) .* repmat(ub - lb, SearchAgents, PointNum) + repmat(lb, SearchAgents, PointNum);
   
    % ̰�����ɷ����ӵڶ��������㿪ʼ����̰���㣩
    if GreedyInit  
        for k = 2 : PointNum    
            ep = 0.2;      % ��ֵ
            m = rand();  % ������
            if m >= ep   % ���������������ֵʱ����k��������̰�����ɣ������������
            % ��������һ�������㣬�������յ�
                lb = P_i(:, dim*(k-1)-dim+1:dim*(k-1));         % N*2ά
                ub = repmat(USV.G(i, :), SearchAgents, 1);   % N*2ά
                P_i(:, dim*k-dim+1:dim*k) = rand(SearchAgents, dim) .* (ub-lb) + lb;
            end
        end
    end

    P0 = [P0, P_i];
end

PSOPops.Pos = [P0, V0]; % ��Ⱥλ��


% �� ��Ⱥλ��������
lb = [];
ub = [];
for i = 1 : USV.num
    lb_i = [USV.limt.x(i, 1), USV.limt.y(i, 1), USV.limt.z(i, 1)] ;     % 1*3 ά
    ub_i = [USV.limt.x(i, 2), USV.limt.y(i, 2), USV.limt.z(i, 2)] ;    % 1*3 ά
    if dim < 3
        lb_i = lb_i(1:2);      % 1*2 ά
        ub_i = ub_i(1:2);    % 1*2 ά
    end
    PointNum = USV.PointNum(i);
    lb_i = repmat(lb_i, 1, PointNum);
    ub_i = repmat(ub_i, 1, PointNum);
    
    lb = [lb, lb_i];
    ub = [ub, ub_i];
end
lb = [lb, lb_v];
ub = [ub, ub_v];

PSOPops.PosDim = size(lb, 2);
PSOPops.lb = lb;
PSOPops.ub = ub;


% �� ���ɳ�ʼ����
Tracks = Pops2Tracks(PSOPops, USV); 

clc
fprintf('>>�㷨��ʼ����ɣ�\n\n')

end

