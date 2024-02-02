function [report] = TrackDetect(Track, USV)
%TRACKDETECT �жϺ����Ƿ��������(һ��agent��)

%% ���˻��������

% ��в���
dim = USV.PointDim;                                         % ���滷��ά��
M = [USV.Menace.radar; USV.Menace.other]; % ��в��

Threat = cell(USV.num, 1);                                 % ��в�����
Angle = cell(USV.num, 1);                                  % ת�Ǽ������
MiniTraj = cell(USV.num, 1);                              % ��С����Ƭ�μ������
ProbPoint = cell(USV.num, 1);                           % �����  

L = cell(USV.num, 1);                                          % ����Ƭ�������ۼӽṹ��
Time = cell(USV.num, 1);                                    % ���������ʱ����                                          
      
L_mt = 0;                                                               % �������˴�����֮��
totalTime = zeros(USV.num, 1);                         % ����ʱ��
totalL = zeros(USV.num, 1);                                % ÿ�����˴����о���

for i = 1 : USV.num
      PointNum = USV.PointNum(i);   
      Judge = zeros(1, PointNum+1);
      L_i = zeros(1, PointNum+1);
      Time_i = zeros(1, PointNum+1);
      Angle_i = zeros(1, PointNum+1);
      Traj_i = zeros(1, PointNum+1);
      ProbPoint_i = zeros(1, PointNum+1);
      % ���м��
      l = 0;
      V = Track.V(i);
      for k = 1 : PointNum
            % ǰ����
            P2 = Track.P{i}(:, k)' ;   % ת�ó� 1*dim
            if k == 1
                P1 = USV.S(i, :);
                ZZ = P2 - P1;
                phi0 = atan2(ZZ(2), ZZ(1));   % ת�Ǽ��
                phi1 = phi0;
                d_phi = phi1 - phi0;
                if dim > 2                                % ��Ǽ��
                    theta0 = atan(ZZ(3) / sqrt(ZZ(1)^2 + ZZ(2)^2));
                    theta1 = theta0;
                    d_theta = theta1 - theta0;
                else
                    d_theta = 0;
                end
            else
                P1 = Track.P{i}(:, k-1)' ;  % ת�ó� 1*dim
                ZZ = P2 - P1;
                phi1 = atan2(ZZ(2), ZZ(1));
                d_phi = phi1 - phi0;
                phi0 = phi1;
                if dim > 2                                   
                    theta1 = atan(ZZ(3) / sqrt(ZZ(1)^2 + ZZ(2)^2));
                    d_theta = theta1 - theta0;
                    theta0 = theta1;
                else
                    d_theta = 0;
                end
            end
         
            [across, ~] = CheckThreat(P1, P2, M); % ��в���
            Judge(k) = across;
            
            dl = norm(P1 - P2);
            l = l + dl; % �ۼӾ���
            t = l / V;  % ʱ���
            L_i(k) = l;
            Time_i(k) = t;

            if abs(d_phi) > USV.limt.phi(i, 2)  ||  abs(d_theta) > USV.limt.theta(i, 2)
                Angle_i(k) = true;
            else
                Angle_i(k) = false;
            end

            if dl < USV.limt.L(i, 1)
                Traj_i(k) = true;
            else
                Traj_i(k) = false;
            end

            ProbPoint_i(k) = Angle_i(k) | Traj_i(k) | Judge(k); % �����

            % ���һ�μ��
            if k == PointNum
                P1 = USV.G(i, :);
                [across, ~] = CheckThreat(P2, P1, M);
                Judge(k+1) = across;
                
                dl = norm(P1 - P2);
                l = l + dl;
                t = l / V;
                L_i(k+1) = l;
                Time_i(k+1) = t;

                ZZ = P1-P2;
                phi1 = atan2(ZZ(2), ZZ(1));
                d_phi = phi1 - phi0;
                if dim>2      
                    theta1 = atan(ZZ(3) / sqrt(ZZ(1)^2 + ZZ(2)^2));
                    d_theta = theta1 - theta0;
                else
                    d_theta = 0;
                end

                if abs(d_phi) > USV.limt.phi(i, 2)  ||  abs(d_theta) > USV.limt.theta(i, 2)
                    Angle_i(k+1) = true;
                else
                    Angle_i(k+1) = false;
                end

                if dl < USV.limt.L(i, 1)
                    Traj_i(k+1) = true;
                else
                    Traj_i(k+1) = false;
                end

                ProbPoint_i(k+1) = Angle_i(k+1) | Traj_i(k+1) | Judge(k+1);

            end
      end   
      Threat(i) = {Judge};                % ����������ȱȵ����Ŀ��һ��
      Angle(i) = {Angle_i};               % ת��
      MiniTraj(i) = {Traj_i};              % ����Ƭ��
      ProbPoint(i) = {ProbPoint_i}; % �����

      L(i) = {L_i};                 % ·�����ȣ��ۼӣ�
      Time(i) = {Time_i};    % ʱ�䣨�ۼӣ�
      L_mt = L_mt + l;           % �ܳ���
      totalTime(i) = t;            % ʱ��
      totalL(i) = l;                   % ����
end

% �����˻���ײ���
d_safe = USV.ds; % ��ȫ����
CollideTimes = 0; % ��ײ����
for i = 2 : USV.num
    PointNum_i = USV.PointNum(i);
    for k = 1 : PointNum_i
        P1 = Track.P{i}(:, k)';  % Kʱ�� i ���˴�λ��
        t_i = Time{i, 1}(k);                  % Kʱ�� i ���˴�ʱ��
        for j = 1 : i-1
            PointNum_j = USV.PointNum(j);
            flag = false; % 
            % ����ͬһʱ�̵ĵ�
            for kj = 1 : PointNum_j
                if kj ==1
                    t_j_l = 0;
                    P_l =  USV.S(j, :); 
                else
                    t_j_l = Time{j}(kj-1);
                    P_l =  Track.P{j}(:, kj-1)'; 
                end
                t_j_r = Time{j}(kj);
                P_r =  Track.P{j}(:, kj)'; 

                if t_i <= t_j_r  &&  t_i >= t_j_l
                    flag = true;
                    P2 =  P_l + (t_i - t_j_l) / (t_j_r - t_j_l) * (P_r - P_l);% Kʱ�� J ���˴�λ��
                    % Kʱ�� j ���˴�λ��
                end
            end
            % ��������������

            if flag  % ���ҵ�P2λ�ã�������ײ���
                collide = CheckCollide(P1, P2, d_safe);
            else     % û��P2λ�ã�������
                collide = false;
            end
            if collide
                CollideTimes = CollideTimes + 1;
            end
        end
    end
end

% ���ɼ�ⱨ��
report.L_mt = L_mt;                       %���г�֮��
report.Threat = Threat;                 %����в�ĺ�����λ��
report.AngleProb = Angle;            % ת�ǲ�����ĵ�
report.TrajProb = MiniTraj;           % ��������С��������ĵ�
report.ProbPoint = ProbPoint;     % ������ĵ�
report.L = totalL;                            %���о���
report.time = totalTime;                %����ʱ��
report.col_times = CollideTimes;  %��ײ����
end



%% ��Խ��в�����
function [across, across_num] = CheckThreat(P1, P2, M)
    % ��в�������Բ���򣬲��ʺ�Բ������
    O = M(:,1:end-1);  % Բ��
    R = M(:, end);        % �뾶 
    
    % ����߶��Ƿ񴩹�ĳ���ϰ���
    total = 0;
    for i = 1 : size(O, 1)
         a = norm(P1 - P2);
         b = norm(P2 - O(i, :));
         c = norm(P1 - O(i, :));
         % P1���Ƿ���Բ��
         if c < R(i)         
             isHit = true;   
         % P2���Ƿ���Բ��
         elseif b < R(i)  
             isHit = true;   
         % P1 P2������Բ��
         else               
             dim = size(O, 2);  % P1�� 1*dim ά
             if dim < 3
                 % ƽ�����   
                 A = P1(2) - P2(2);
                 B = P2(1) - P1(1);
                 C = P1(1)*P2(2) - P2(1)*P1(2);
                 x = O(i, 1);
                 y = O(i, 2);
                 d = abs(A*x + B*y + C) / sqrt(A^2 + B^2); 
             else
                 % �ռ����
                 PP = P2 - P1;
                 PO = O(i, :) - P1;
                 d = norm(cross(PP, PO)) / a;
             end
             % �����о�
             if d >= R(i)
                 isHit = false;  % ���ཻ
             % �Ƕ��о�(������������ʱ�����Ƕ�Ϊ���ʱ�ཻ)
             elseif d > 0
                 cosP1 = a^2 + c^2 - b^2 / (2*c*a);
                 cosP2 = a^2 + b^2 - c^2 / (2*b*a);
                 if cosP1 > 0  &&  cosP2 > 0
                     isHit = true;
                 else
                     isHit = false;
                 end
             % �������⣬����Ϊ0��������������
             else
                 if a > b  &&  a > c
                    isHit = true;
                 else
                    isHit = false;
                 end
             end
         end

         if isHit
            total = total + 1; %�ܹ���ײ����
         end
    end

    if total > 0
        across = true;
    else
        across = false;          % �Ƿ�Խ����
    end
    across_num = total;     % ���������ĸ���
end



%% ��ײ���
function [collide] = CheckCollide(P1, P2, d_safe)
    if norm(P1 - P2) >= d_safe
        collide = false;
    else
        collide = true;
    end
end
