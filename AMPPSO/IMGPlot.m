function IMGPlot(solution, USV)
close all; 

% ��
Tracks = solution.Tracks;                    % ������
Data = solution.Glbest_Data;               % ���ź�����Ϣ
Fitness_list = solution.Fitness_list;     % ��Ӧ������
Glbest_no = solution.Glbest_no;          % �����
Plbest_no = solution.Plbest_no;               % �����
Gbest_no = solution.Gbest_no;             % �����
agent_no = Glbest_no;                          % Ҫ���ƵĽ�����


% ����ͼ
if USV.PointDim<3

    %%%%%%%%% �������� 2D���� �������� %%%%%%%%%
    
    figure(1)
    for i = 1:USV.num
        Byangtiao = [USV.S(i,1),Tracks{agent_no, 1}.P{i, 1}(1,:),USV.G(i,1)];
        y = [USV.S(i,2),Tracks{agent_no, 1}.P{i, 1}(2,:),USV.G(i,2)];
        plot(Byangtiao,y,'LineWidth',2)   
        hold on
    end
    for i = 1:USV.num
        plot(USV.S(i,1),USV.S(i,2),'ko','LineWidth',1,'MarkerSize',9)
        hold on
        plot(USV.G(i,1),USV.G(i,2),'p','color','k','LineWidth',1, 'MarkerSize',10)
        hold on
    end
    for i = 1:size(USV.Menace.radar,1)
        rectangle('Position',[USV.Menace.radar(i,1)-USV.Menace.radar(i,3),USV.Menace.radar(i,2)-USV.Menace.radar(i,3),2*USV.Menace.radar(i,3),2*USV.Menace.radar(i,3)],'Curvature',[1,1],'EdgeColor','k','FaceColor','g')
        hold on
    end
    for i = 1:size(USV.Menace.other,1)
        rectangle('Position',[USV.Menace.other(i,1)-USV.Menace.other(i,3),USV.Menace.other(i,2)-USV.Menace.other(i,3),2*USV.Menace.other(i,3),2*USV.Menace.other(i,3)],'Curvature',[1,1],'EdgeColor','k','FaceColor','c')
        hold on
    end
    for i = 1:USV.num
        leg_str{i} = ['Track',num2str(i)];  
    end
    leg_str{USV.num+1} = 'Start';
    leg_str{USV.num+2} = 'End';
    legend(leg_str)
    grid on
    axis equal
    dx = (max(USV.limt.x(:,2))-min(USV.limt.x(:,1)))*0.06;
    dy = (max(USV.limt.y(:,2))-min(USV.limt.y(:,1)))*0.06;
    xlim([min(USV.limt.x(:,1))-dx,max(USV.limt.x(:,2))+dx])
    ylim([min(USV.limt.y(:,1))-dy,max(USV.limt.y(:,2))+dy])
    xlabel('x(km)')
    ylabel('y(km)')
    title('·���滮ͼ')
  
else

    %%%%%%%%% �������� 3D���� �������� %%%%%%%%%

    figure('color',[1 1 1])
    for i = 1:USV.num
        Byangtiao = [USV.S(i,1),Tracks{agent_no, 1}.P{i, 1}(1,:),USV.G(i,1)];
        y = [USV.S(i,2),Tracks{agent_no, 1}.P{i, 1}(2,:),USV.G(i,2)];
        z = [USV.S(i,3),Tracks{agent_no, 1}.P{i, 1}(3,:),USV.G(i,3)];
        plot3(Byangtiao,y,z,'LineWidth',2)   
        hold on
    end
    for i = 1:USV.num
        plot3(USV.S(i,1),USV.S(i,2),USV.S(i,3),'ko','LineWidth',1.3,'MarkerSize',12)
        hold on
        plot3(USV.G(i,1),USV.G(i,2),USV.G(i,3),'p','color','k','LineWidth',1.3,'MarkerSize',13)
        hold on
    end
    for i = 1:size(USV.Menace.radar,1)
        drawsphere(USV.Menace.radar(i,1),USV.Menace.radar(i,2),USV.Menace.radar(i,3),USV.Menace.radar(i,4),true)
        hold on
    end
    for i = 1:size(USV.Menace.other,1)
        drawsphere(USV.Menace.other(i,1),USV.Menace.other(i,2),USV.Menace.other(i,3),USV.Menace.other(i,4))
        hold on
    end
    for i = 1:USV.num
        leg_str{i} = ['Track',num2str(i)];  
    end
    leg_str{USV.num+1} = 'Start';
    leg_str{USV.num+2} = 'End';
    legend(leg_str)
    grid on
    axis square
    %axis equal
    dx = (max(USV.limt.x(:,2))-min(USV.limt.x(:,1)))*0.06;
    dy = (max(USV.limt.y(:,2))-min(USV.limt.y(:,1)))*0.06;
    dz = (max(USV.limt.z(:,2))-min(USV.limt.z(:,1)))*0.06;
    xlim([min(USV.limt.x(:,1))-dx,max(USV.limt.x(:,2))+dx])
    ylim([min(USV.limt.y(:,1))-dy,max(USV.limt.y(:,2))+dy])
    zlim([min(USV.limt.z(:,1))-dz,max(USV.limt.z(:,2))+dz])
    xlabel('x (km)')
    ylabel('y (km)')
    zlabel('z (km)')
    
  
end



% ��Ӧ��
disp(Fitness_list)
figure('color',[1 1 1])
plot(Fitness_list,'k','LineWidth',1)
grid on
xlabel('iteration')
ylabel('fitness')




% ��Ļ�����Ϣ
fprintf('\n���˴�������%d', USV.num)
fprintf('\n���˴������������')
fprintf('%d,  ', USV.PointNum)
fprintf('\n���˴����о��룺')
fprintf('%.2fkm,  ', Data.L/10)
fprintf('\n���˴�����ʱ�䣺')
fprintf('%.2fs,  ', Data.t/10)
fprintf('\n���˴�����ƽ���ٶȣ�')
fprintf('%.2fm/s,  ', Data.L./Data.t*1e3)
fprintf('\n���˴�����ײ������%d', Data.c)
fprintf('\nĿ�꺯������ֵ��%.2f', Fitness_list(end))
% fprintf('\n�����¡��� ���ţ�%d,  %d,  %d', Alpha_no, Beta_no, Delta_no)
fprintf('\n\n')

end



%% ��������
function drawsphere(a,b,c,R,useSurf)
    % ��(a,b,c)Ϊ���ģ�RΪ�뾶
    if (nargin<5)
        useSurf = false;
    end

    % ��������
    [x,y,z] = sphere(30);

    % �����뾶
    x = R*x; 
    y = R*y;
    z = R*z;

    % ��������
    x = x+a;
    y = y+b;
    z = z+c;
   
     if useSurf
        % ʹ��surf����
        axis equal;
        surf(x,y,z);
        hold on
    else
        % ʹ��mesh����
        axis equal;
        mesh(x,y,z);
        hold on
    end
end