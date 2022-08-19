% 以UR5E为参考，设置SDH参数
L(1) = Link([0   162.5   0       pi/2],  'standard') ;
L(2) = Link([0   0      -425     0   ],  'standard') ;
L(3) = Link([0   0      -392.2   0   ],  'standard') ;
L(4) = Link([0   133.3   0       pi/2],  'standard') ;
L(5) = Link([0   99.7    0      -pi/2],  'standard') ;
L(6) = Link([0   99.6    0       0   ],  'standard') ;

% 构建模型
six_link = SerialLink([L(1) L(2) L(3) L(4) L(5) L(6)]) ;

% 构建开头结尾以及中间矩阵
T1 = transl(300, 400, 100) * troty(50) ; 
T2 = transl(0, 300, 250) * trotx(130) ;
T3 = transl(-300, -300, 500) * trotz(100) ;

% 获得转角
rpy1 = tr2rpy(T1)/pi*180 ; 
rpy2 = tr2rpy(T2)/pi*180 ; 
rpy3 = tr2rpy(T3)/pi*180 ; 
% 此处转动复杂，原因是函数超过了范围，范围属于【-pi， pi】之间

% 构建位置线性插值
wp = [300, 400, 100; 0, 300, 250; -300, -300, 500] ;
a = 2 ;  % 第一段轨迹时间
b = 2 ;  % 第一段轨迹时间
t = 0.05 ;  % 插值间隔
ac = 0.5 ;  % 转角加速度
P_traj = mstraj(wp, [], [a b], [], t, ac) ;  % 位置线性插值
T_traj_transl = transl(P_traj) ;

% 构建角度线性插值
wr = [rpy1; rpy2; rpy3] ;
rpy_traj = mstraj(wr, [], [a b], [], t, ac) ;  % 角度线性插值
T_traj_rot = rpy2tr(rpy_traj) ;

n = (a + b)/t ;  % 总插值数
T_traj = zeros(4, 4, n) ;  % 初始化所有插值点对应的SE3

% 给所有SE3赋值,并检测是否处于障碍物内
x0 = 50 ;
y0 = 50 ;
z0 = 50 ;
r0 = 200 ;
for i = 1:n
    T_traj(:, :, i) = T_traj_transl(:, :, i) * T_traj_rot(:, :, i) ; 
    r = sqrt((T_traj(1, 4, i) - x0) ^ 2 + ...
        (T_traj(2, 4, i) - y0) ^ 2 + ...
         (T_traj(3, 4, i) - z0) ^ 2) ;
    if r ^ 2 <= r0 ^ 2
        T_traj(1, 4, i) = T_traj(1, 4, i) + (r0 - r) * (T_traj(1, 4, i) - x0) / r + 5 ;
        T_traj(2, 4, i) = T_traj(2, 4, i) + (r0 - r) * (T_traj(2, 4, i) - x0) / r + 5 ;
        T_traj(3, 4, i) = T_traj(3, 4, i) + (r0 - r) * (T_traj(3, 4, i) - x0) / r + 5 ;
    end
end

% 做IK
Qtraj = six_link.ikunc(T_traj) ;  

% 设置奇异区域精度
eps_angel = 0.5 ;  
eps_position = 3 ;

% 若达到奇异点，则停止
for i = 1:n
    for j = 1:6
        if (j == 5) && (abs(Qtraj(i, j)) <= eps_angel)   % 关节五为0
                fprintf('wrist sigularity in %d position!\n', i) ; 
%                 error('motor collapse!')  % 单纯停止
        elseif (j == 2) && (abs(Qtraj(i, j) - Qtraj(i, j+1)) <= eps_angel)  % 关节二三角度一致
                fprintf('elbow sigularity in %d position!\n', i) ;
%                 error('motor collapse!')  % 单纯停止
        elseif (j == 4) && (abs(T_traj_transl(1, 4, i)) <= eps_position) && ...
                    abs(T_traj_transl(2, 4, i)) <= eps_position && ...
                      (abs((Qtraj(i, j) - pi/2) <= eps_angel ||...
                        abs(Qtraj(i, j) - pi/2*3) <= eps_angel))  % 四角度在原点上方且四关节垂直
                fprintf('shoulder sigularityin %d position!\n', i) ;
%                 error('motor collapse!')  % 单纯停止
        end
    end
end

% 画出示教器轨迹
six_link.plot(Qtraj) ;
