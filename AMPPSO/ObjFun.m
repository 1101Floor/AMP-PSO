function [F, subF, Data] = ObjFun(Track, USV)
%OBJFUN Ŀ�꺯������Ӧ�Ⱥ�����һ��agent�ģ�

% ��Ŀ���Ż�Ȩ�����ã�����Ϊ��������
weight = [ 0.2, 0.1, 0.3, 0.1, 0.3 ]; % Ĭ��Ȩ��


% ���ʽϵ��������������ָ�������ٻ���
p1 = 10;   % ȼ����ѳ�����󺽳̣�
p21 = 1; % �߶���
p22 = 10; % �Ͷ���
p31 = 5000; % Ӳ��в
p32 =1; % ����в
p4 = 1.1;   % ʱ��ͬ����
p5 = 10;   % ��ײ��


% ���к������
report = TrackDetect(Track, USV);  % Track Ϊ struck�ṹ

% ·��
ZZ = sum(USV.limt.L);
MaxL_mt = ZZ(2);
f_o = p1 * report.L_mt / MaxL_mt;

% �߶�
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

% ��в
O_r = USV.Menace.radar(: ,1:end-1);                        
O_o = USV.Menace.other(: ,1:end-1);              
f_t = 0;  % ��в����
for i = 1 : USV.num
    for k = 1 : USV.PointNum(i)
        P = Track.P{i}(:, k)' ;  % ת�ó� 1*dim
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


% Ŀ�꺯������
subF = [ f_o; f_h; f_t; f_m; f_c ]; % 5*1


% ��ȨĿ�꺯��
F = weight * subF ;

% �����Ϣ
Data.ProbPoint = report.ProbPoint;      % ����������ĵ�
Data.AngleProb = report.AngleProb;    % ������Ƕ�Լ���ĵ�
Data.TrajProb = report.TrajProb;           % ��������С��������ĵ�
Data.Threat = report.Threat;                  % ����в�ĵ�

Data.L = report.L;                                      % ÿ�����˴��ĺ���
Data.t = report.time;                                 % ÿ�����˴���ʱ��
Data.c = report.col_times;                         % �������˴�����ײ����


end

