function [PSOPops, Tracks] = BoundAdjust(PSOPops, ProbPoints, USV)
%BOUNDADJUST Լ������
Tracks = Pops2Tracks(PSOPops, USV); 

% ��  ������Լ����"ɾ��"�����㣬��ȡǰ��ƽ��ֵ
dim = USV.PointDim;                                    % ά��
for agent = 1 : size(PSOPops.Pos, 1)
    Track = Tracks{agent};                               % �켣 (struct�ṹ)
    ProbPoint = ProbPoints{agent};               % �����
    Position = [];                                               % ����Ⱥ����
    for i =1:USV.num
          PointNum = USV.PointNum(i);
          % ɾ��һ�������ϵ������
          for k = 1 : PointNum
                flag = ProbPoint{i}(k);
                if flag == 1
                    if k == 1
                        P1 = USV.S(i, :)' ;
                    else
                        P1 = Track.P{i}(:, k-1);
                    end
                    if k == PointNum
                        P2 = USV.G(i, :)' ;
                    else
                        P2 = Track.P{i}(:, k+1);
                    end
                    Track.P{i}(:, k) = (P1+P2) / 2;  % ɾ��������
                end
          end
    
          % ת��Ϊ��Ⱥ������ʽ
          p = Track.P{i} ;
          p = reshape(p, 1, dim*PointNum);
          Position = [Position, p];
    end
    V = Track.V';
    Position = [Position, V];
    
    % �µ�λ����Ϣ
    PSOPops.Pos(agent, :) = Position;
end

% ��  �߽紦��Խ��ȡ�߽�ֵ
PSOPops.Pos = BoundClamp(PSOPops.Pos, PSOPops.lb, PSOPops.ub);

% �����º���
Tracks = Pops2Tracks(PSOPops, USV); 

end



% �߽�ü�
function x = BoundClamp(x, lb, ub)
    Flag4ub = x > ub;
    Flag4lb = x < lb;
    x = x .* ( ~(Flag4ub + Flag4lb) ) + ub .* Flag4ub + lb .* Flag4lb;
end