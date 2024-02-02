function Tracks = Pops2Tracks(PSOPops, USV)

SearchAgents = size(PSOPops.Pos, 1);          % 种群数量
UAVnum = USV.num;                                      % 无人船个数
dim = USV.PointDim;                                      % 仿真维度
v = PSOPops.Pos(:, end-UAVnum+1:end);   % 协同无人船速度
P = PSOPops.Pos(:, 1:end-UAVnum);       % 协同无人船航迹 xy

Tracks = cell(SearchAgents, 1);
for agent = 1 : SearchAgents
    a.V = v(agent, :)';
    P_a = P(agent, :);
    a.P = cell(UAVnum, 1);
    for i =1:UAVnum
        PointNum = USV.PointNum(i);
        P_ai = P_a(1 : PointNum*dim);
        P_ai = reshape(P_ai, dim, PointNum);
        P_a = P_a(PointNum*dim+1 : end);
        a.P(i) =  {P_ai};
    end
    Tracks(agent) = {a};
end

end

