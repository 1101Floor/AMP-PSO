clc, close all

%--- �㷨ѡ�� 1��APSO�㷨  2��PSO�㷨  3��CS-PSO�㷨 4��MP_PSO�㷨 5:A-PSO�㷨 6:GA�㷨
options = 1;

%--- �㷨��������
SearchAgents = 50;       % ��Ⱥ����
Max_iter = 200;          % �����������

%--- Эͬ���˴�����
USV = USV_SetUp1;        % �� USV_SetUp.m �ļ���������

%--- ѡ���㷨
if options == 1
    solution = AMP_PSO(USV, SearchAgents, Max_iter);  % PSO�㷨
elseif options == 2
    solution = PSO(USV, SearchAgents, Max_iter);  % CAMP-PSO�㷨
elseif options == 3
    solution = CS_PSO(USV, SearchAgents, Max_iter);  % CS-PSO�㷨
elseif options == 4
    solution = MP_PSO(USV, SearchAgents, Max_iter);  % MP_PSO�㷨
elseif options == 5
    solution = A_PSO(USV, SearchAgents, Max_iter);  % A-PSO�㷨
elseif options == 6
    solution = A_GA(USV, SearchAgents, Max_iter);   % GA�㷨
end

%--- ��ͼ
IMGPlot(solution, USV) % ����Ӧ��ͼ��ȫ�Զ���ͼ���������ֶ��ĺÿ���