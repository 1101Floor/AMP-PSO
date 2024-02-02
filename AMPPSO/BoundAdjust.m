function [PSOPops, Tracks] = BoundAdjust(PSOPops, ProbPoints, USV)
%BOUNDADJUST 约束处理
Tracks = Pops2Tracks(PSOPops, USV); 

% ①  不满足约束："删除"航迹点，即取前后平均值
dim = USV.PointDim;                                    % 维度
for agent = 1 : size(PSOPops.Pos, 1)
    Track = Tracks{agent};                               % 轨迹 (struct结构)
    ProbPoint = ProbPoints{agent};               % 问题点
    Position = [];                                               % 新种群编码
    for i =1:USV.num
          PointNum = USV.PointNum(i);
          % 删除一条航迹上的问题点
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
                    Track.P{i}(:, k) = (P1+P2) / 2;  % 删除航迹点
                end
          end
    
          % 转换为种群编码形式
          p = Track.P{i} ;
          p = reshape(p, 1, dim*PointNum);
          Position = [Position, p];
    end
    V = Track.V';
    Position = [Position, V];
    
    % 新的位置信息
    PSOPops.Pos(agent, :) = Position;
end

% ②  边界处理：越界取边界值
PSOPops.Pos = BoundClamp(PSOPops.Pos, PSOPops.lb, PSOPops.ub);

% 生成新航迹
Tracks = Pops2Tracks(PSOPops, USV); 

end



% 边界裁剪
function x = BoundClamp(x, lb, ub)
    Flag4ub = x > ub;
    Flag4lb = x < lb;
    x = x .* ( ~(Flag4ub + Flag4lb) ) + ub .* Flag4ub + lb .* Flag4lb;
end