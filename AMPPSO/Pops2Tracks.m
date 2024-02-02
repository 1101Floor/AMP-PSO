function Tracks = Pops2Tracks(PSOPops, USV)

SearchAgents = size(PSOPops.Pos, 1);          % ��Ⱥ����
UAVnum = USV.num;                                      % ���˴�����
dim = USV.PointDim;                                      % ����ά��
v = PSOPops.Pos(:, end-UAVnum+1:end);   % Эͬ���˴��ٶ�
P = PSOPops.Pos(:, 1:end-UAVnum);       % Эͬ���˴����� xy

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

