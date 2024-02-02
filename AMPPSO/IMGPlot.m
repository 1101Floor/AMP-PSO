function IMGPlot(solution, USV)
close all; 

% 解
Tracks = solution.Tracks;                    % 航迹们
Data = solution.Glbest_Data;               % 最优航迹信息
Fitness_list = solution.Fitness_list;     % 适应度曲线
Glbest_no = solution.Glbest_no;          % 解序号
Plbest_no = solution.Plbest_no;               % 解序号
Gbest_no = solution.Gbest_no;             % 解序号
agent_no = Glbest_no;                          % 要绘制的解的序号


% 航迹图
if USV.PointDim<3

    %%%%%%%%% ———— 2D仿真 ———— %%%%%%%%%
    
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
    title('路径规划图')
  
else

    %%%%%%%%% ———— 3D仿真 ———— %%%%%%%%%

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



% 适应度
disp(Fitness_list)
figure('color',[1 1 1])
plot(Fitness_list,'k','LineWidth',1)
grid on
xlabel('iteration')
ylabel('fitness')




% 屏幕输出信息
fprintf('\n无人船数量：%d', USV.num)
fprintf('\n无人船导航点个数：')
fprintf('%d,  ', USV.PointNum)
fprintf('\n无人船航行距离：')
fprintf('%.2fkm,  ', Data.L/10)
fprintf('\n无人船航行时间：')
fprintf('%.2fs,  ', Data.t/10)
fprintf('\n无人船航行平均速度：')
fprintf('%.2fm/s,  ', Data.L./Data.t*1e3)
fprintf('\n无人船总碰撞次数：%d', Data.c)
fprintf('\n目标函数收敛值：%.2f', Fitness_list(end))
% fprintf('\nα、β、δ 解编号：%d,  %d,  %d', Alpha_no, Beta_no, Delta_no)
fprintf('\n\n')

end



%% 绘制球面
function drawsphere(a,b,c,R,useSurf)
    % 以(a,b,c)为球心，R为半径
    if (nargin<5)
        useSurf = false;
    end

    % 生成数据
    [x,y,z] = sphere(30);

    % 调整半径
    x = R*x; 
    y = R*y;
    z = R*z;

    % 调整球心
    x = x+a;
    y = y+b;
    z = z+c;
   
     if useSurf
        % 使用surf绘制
        axis equal;
        surf(x,y,z);
        hold on
    else
        % 使用mesh绘制
        axis equal;
        mesh(x,y,z);
        hold on
    end
end