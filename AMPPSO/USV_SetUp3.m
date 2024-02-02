function USV = USV_SetUp1

USV.S = [   1,           3,       16;
                 3,        1,          16;
                 1,           1,          17;
                  5,        1,          15;
                 1,           5,       15;
               
                ];     % ���λ�� (x,y)��(x,y,z)

USV.G = [       18,       19,      4;
                  19,       19,      5;
                   18,       18,      3;
                  19,       18,      3; 
                  18,       19,      3;
                 
                   ];      % Ŀ��λ�� (x,y)��(x,y,z)

USV.PointNum = [            %      25;
                            %      28;
                             %     31;  
                             %     34;  
                             %`     37;
                                  
                                  30;
                                  30;
                                  30;
                                  30;
                                  30;  
                                  ];                 % ÿ�����˴��������������ʼ��֮���ĸ�����

USV.PointDim = size(USV.S, 2);        % �����ά�� ���� ��� ���������
USV.num = size(USV.S, 1);                 % UAV���� ���� ��� ����������


% ��в������ (x,y,r) �� (x,y,z,r)
% ��ÿ��Ϊһ����в������Ͱ뾶��
USV.Menace.radar = [  10,10,10,3; 
    5, 5, 17, 2.5;
    17,14,4,1.5; 
           15,4,10,2;   
          7,15,9,2;       ];   % Ӳ��в

USV.Menace.other = [   8,2,12,2;
    6,8, 4, 1.5;
    18,10,4,2; 
    14,4,6,1.5;
    13, 16, 15, 2;
    2,14,8,2;
    8,2,6,2;   ];   % ����в


USV.limt.v = 0.002*repmat([0.3, 0.7], USV.num, 1);                 % �ٶ�Լ��
USV.limt.phi = deg2rad(repmat([-35, 35], USV.num, 1));      % ת��Լ�� ��-60�� ~ 60�㣩
USV.limt.theta = deg2rad(repmat([-20, 20], USV.num, 1));   % ���Լ�� ��-45�� ~ 45�㣩
USV.limt.h = repmat([0.02, 20], USV.num, 1);                         % �߶�Լ�� 
USV.limt.x = repmat([0, 20], USV.num, 1);                            % λ��xԼ�� 
USV.limt.y = repmat([0, 20], USV.num, 1);                            % λ��yԼ�� 
USV.limt.z = USV.limt.h;                                                            % λ��zԼ�� �����Ե��򻡶ȣ�
USV.limt.L = zeros(USV.num, 2);                                              % ����Լ�� ����̺���Ƭ��2km����󺽳�1.5����ʼ���룩
for i =1:USV.num
    zz.max = 1.6 * norm(USV.G(i, :) - USV.S(i, :));
    zz.min = 0.5;
    USV.limt.L(i, :) = [zz.min, zz.max];
end

USV.tc = 27000;         % Эͬʱ�� ����λs��
USV.ds = 0.5;          % ��ȫ���� ����λkm��


% ����
ErrorCheck(USV)
end





%% �����Լ�
function ErrorCheck(UAV)

dim = UAV.PointDim; 
if dim ~= size(UAV.G,2) || dim ~= size(UAV.Menace.radar,2)-1 || dim ~= size(UAV.Menace.other,2)-1
    if dim ~= size(UAV.G,2)
        error('����ά��Ϊ%d����Ŀ�������Ϊ%dά', dim, size(UAV.G,2))
    else
        error('����ά��Ϊ%d������в������Ϊ%dά', dim, size(UAV.Menace.radar,2)-1)
    end
end

num = UAV.num;
if num ~= size(UAV.G,1) || num ~= size(UAV.limt.v,1) || num ~= size(UAV.limt.phi,1) ...
        || num ~= size(UAV.limt.theta,1) || num ~= size(UAV.limt.h,1) || num ~= size(UAV.limt.x,1) ...
        || num ~= size(UAV.limt.y,1) || num ~= size(UAV.limt.z,1) || num ~= size(UAV.limt.L,1)
    if num ~= size(UAV.G,1)
        error('���˴�����Ϊ%d, ��Ŀ�����%d��', num, size(UAV.G,1))
    else
        error('Լ���������������˴�������һ��')
    end
end

if num ~= size(UAV.PointNum, 1)
    error('���˴�����Ϊ%d, ��Ϊ%d�����˴������˵�����', num, size(UAV.PointNum, 1))
end

MaxPoint = floor(UAV.limt.L(:,2) ./ UAV.limt.L(:,1)) - 1;   % ÿ�����˻�֧�ֵ���󺽼�������
for i = 1 : UAV.num
    if UAV.PointNum(i) > MaxPoint(i)
        error('%d�����˴�����������������������볢�Լ��ٵ��������', i)
    end
end

end
