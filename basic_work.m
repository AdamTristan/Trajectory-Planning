% 一维五次插值
t = linspace(0, 2, 51) ; 
[P dP ddP] = tpoly(0, 3, t) ;
plot(t, P);
hold on
plot(t, dP);
hold on
plot(t, ddP);

% 一维线性插值
t = linspace(0, 2, 51) ; 
[P dP ddP] = lspb(0, 3, t) ;
plot(t, P);
hold on
plot(t, dP);
hold on
plot(t, ddP);

% 二维多项式插值
t = linspace(0, 2, 51) ; 
[P dP ddP] = mtraj(@lspb, [0 0], [3 4], t) ;
plot(t, P);
hold on
plot(t, dP);
hold on
plot(t, ddP);

%%
% 二维多段轨迹插值
wp = [0, 0; 3, 4; 1, 2] ;
P1 = mstraj(wp, [], [2 1], [], 0.04, 0) ; % 轨迹位置 最大速度 时间 初始位置 插值间隔 加速时间
plot(P1);
hold on
P2 = mstraj(wp, [], [2 1], [], 0.04, 0.5) ;
plot(P2);
hold on

%%
% 三维多段轨迹插值
wp = [0, 0, 0; 3, 4, 1; 1, 2, 5] ;
P1 = mstraj(wp, [], [2 1], [], 0.04, 0) ; % 轨迹位置 最大速度 时间 初始位置 插值间隔 加速时间
plot(P1);
hold on ;
P2 = mstraj(wp, [], [2 1], [], 0.04, 0.5) ;
plot(P2);
legend('p1', 'p2')

%%
% 实际案例
L(1) = Link([0   162.5   0       pi/2],  'standard');
L(2) = Link([0   0      -425     0   ],  'standard');
L(3) = Link([0   0      -392.2   0   ],  'standard');
L(4) = Link([0   133.3   0       pi/2],  'standard');
L(5) = Link([0   99.7    0      -pi/2],  'standard');
L(6) = Link([0   99.6    0       0   ],  'standard');

six_link = SerialLink([L(1) L(2) L(3) L(4) L(5) L(6)]);

P1 = [300 400 300] ;
P2 = [100 -500 400] ;

t = linspace(0, 2, 51) ;
Traj = mtraj(@lspb, P1, P2, t) ;

n = size(Traj, 1) ; 
T = zeros(4, 4, n) ; 

for i = 1:n
    T(:, :, i) = transl(Traj(i, :)) ; 
end

Qtraj = six_link.ikunc(T) ; 
% six_link.plot(Qtraj) ;
six_link.plot(Qtraj, 'trail', 'r') ;
% six_link.plot(Qtraj, 'movie', 'trail.gif') ;

%%
% 绘制XYZ坐标变化曲线

hold on
grid on 

plot(t, Traj(:, 1), '.-', 'linewidth', 1) ;
plot(t, Traj(:, 2), '.-', 'linewidth', 1) ;
plot(t, Traj(:, 3), '.-', 'linewidth', 1) ;

legend('x', 'y', 'z') ;
xlabel('time') ; 
ylabel('posiiton') ;

% 直接用工具箱求解（线性插值）
% trinterp(T1， T2， N )

%%
% 加上RPY角度
T1 = transl(300, 400, 300)  ; 
T2 = transl(400, 300, 400) * troty(200) ;
 
q1 = six_link.ikunc(T1) ;
q2 = six_link.ikunc(T2) ;

six_link.plot(q1) ;  
% pause ;    
% six_link.plot(q2) ; 

%%
% 旋转实现

L(1) = Link([0   162.5   0       pi/2],  'standard');
L(2) = Link([0   0      -425     0   ],  'standard');
L(3) = Link([0   0      -392.2   0   ],  'standard');
L(4) = Link([0   133.3   0       pi/2],  'standard');
L(5) = Link([0   99.7    0      -pi/2],  'standard');
L(6) = Link([0   99.6    0       0   ],  'standard');

six_link = SerialLink([L(1) L(2) L(3) L(4) L(5) L(6)]);

T1 = transl(300, 400, 300) * troty(50) ; 
T2 = transl(0, 0, 400) * trotx(130) ;

rpy1 = tr2rpy(T1)/pi*180 ; % 获得转角
rpy2 = tr2rpy(T2)/pi*180 ; 
% 此处转动复杂，原因是函数超过了范围，范围属于【-pi， pi】之间

tend = 2 ;
times = 51 ;

t = linspace(0, tend, times) ;
rpy_traj = mtraj(@lspb, rpy1, rpy2, t) ; % 转角线性插值
T_traj_rot = rpy2tr(rpy_traj) ;

P1 = transl(T1) ; 
P2 = transl(T2) ; 
P_traj = mtraj(@lspb, P1', P2', t) ; % 位置线性插值
T_traj_transl = transl(P_traj) ;

n = length(t) ; 
T_traj = zeros(4, 4, n) ; 

for i = 1:n
    T_traj(:, :, i) = T_traj_transl(:, :, i) * T_traj_rot(:, :, i) ; 
end

eps_angel = 0.1 ; % 设置奇异区域精度
eps_position = 10 ;
amax = 10 ; 

q1 = six_link.ikunc(T1) ;
q2 = six_link.ikunc(T2) ;
[q dq ddq] = jtraj(q1, q2, t) ;

Qtraj = six_link.ikunc(T_traj) ;

% 停止法
% for i = 1:51
%     for j = 1:6
%         if (j == 5) % 关节五为0
%             if(abs(Qtraj(i, j)) <= eps_angel) 
%                 fprintf('wrist sigularity in %d position!\n', i) ; % Qtraj(i, j) = Qtraj(i, j) - 0.2 ; % 改变路径无意义，就是要到达那个点，故尝试限制加速度或者速度
%                 if ddq(i, j) >= amax
%                     error('motor collapse!') % 单纯停止
%                 end      
%             end
%         elseif (j == 2) % 关节二三角度一致
%             if (abs(Qtraj(i, j) - Qtraj(i, j+1)) <= eps_angel)
%                 fprintf('elbow sigularity in %d position!\n', i) ; % Qtraj(i, j) = Qtraj(i, j) + 0.2 ; % 改变路径无意义，就是要到达那个点，故尝试限制加速度或者速度
%                 if ddq(i, j) >= amax
%                     error('motor collapse!') % 单纯停止
%                 end
%             end
%         elseif  (j == 4)  % 四角度在原点上方且四关节垂直
%             if (abs(T_traj_transl(1, 4, i)) <= eps_position) && abs(T_traj_transl(2, 4, i)) <= eps_position && abs((Qtraj(i, j) - pi/2) <= eps_angel || abs(Qtraj(i, j) - pi/2*3) <= eps_angel)
%                 fprintf('shoulder sigularityin %d position!\n', i) ; % Qtraj(i, j-2) = Qtraj(i, j-2) - 0.2 ; % 改变路径无意义，就是要到达那个点，故尝试限制加速度或者速度
%                 if ddq(i, j) >= amax
%                     error('motor collapse!') % 单纯停止
%                 end
%             end 
%         end
%     end
% end

% % 角度直接改变法
% for i = 1:51
%     for j = 1:6
%         if (j == 5) % 关节五为0
%             if(abs(Qtraj(i, j)) <= eps_angel) 
%                 fprintf('wrist sigularity in %d position!\n', i) ; % Qtraj(i, j) = Qtraj(i, j) - 0.2 ; % 改变路径无意义，就是要到达那个点，故尝试限制加速度或者速度
%                 if ddq(i, j) >= amaz
%                     if Qtraj(i, j) < 0 
%                         Qtraj(i, j) = -0.1 + Qtraj(i, j) ;
%                     end
%                     if Qtraj(i, j) > 0 
%                         Qtraj(i, j) = 0.1 + Qtraj(i, j) ;
%                     end
%                 end      
%             end
%         elseif (j == 2) % 关节二三角度一致
%             if (abs(Qtraj(i, j) - Qtraj(i, j+1)) <= eps_angel)
%                 fprintf('elbow sigularity in %d position!\n', i) ; % Qtraj(i, j) = Qtraj(i, j) + 0.2 ; % 改变路径无意义，就是要到达那个点，故尝试限制加速度或者速度
%                 if ddq(i, j) >= amaz
%                     Qtraj(i, j) = Qtraj(i, j) + 0.1
%                     Qtraj(i, j+1) = Qtraj(i, j+1) - 0.1
%                 end
%             end
%         elseif  (j == 4)  % 四角度在原点上方且四关节垂直
%             if (abs(T_traj_transl(1, 4, i)) <= eps_position) && abs(T_traj_transl(2, 4, i)) <= eps_position && abs((Qtraj(i, j) - pi/2) <= eps_angel || abs(Qtraj(i, j) - pi/2*3) <= eps_angel)
%                 fprintf('shoulder sigularityin %d position!\n', i) ; % Qtraj(i, j-2) = Qtraj(i, j-2) - 0.2 ; % 改变路径无意义，就是要到达那个点，故尝试限制加速度或者速度
%                 if ddq(i, j) >= amax
%                     Qtraj(i, j-1) = Qtraj(i, j-1) - 0.1
%                 end
%             end 
%         end
%     end
% end

% 改变速度法
% for i = 1:51
%     for j = 1:6
%         if (j == 5) % 关节五为0
%             if(abs(Qtraj(i, j)) <= eps_angel) 
%                 fprintf('wrist sigularity in %d position!\n', i) ; % Qtraj(i, j) = Qtraj(i, j) - 0.2 ; % 改变路径无意义，就是要到达那个点，故尝试限制加速度或者速度
%                 if ddq(i, j) >= amaz
%                     tend = tend + 1 ; 
%                     tbegin = (tend - 0)/(times - 1) * (i - 1) ;
%                     times = fix(times/2) ; 
%                     
%                     T11 = T_traj(:, :, i) ;
%                     T22 = T2 ;
%                     
%                     rpy11 = tr2rpy(T21)/pi*180 ; % 获得转角
%                     rpy22 = tr2rpy(T22)/pi*180 ; 
%                     
%                     t_cv = linspace(ten, tend, times) ;
%                     rpy_traj_cv = mtraj(@lspb, rpy11, rpy22, t) ; % 转角线性插值
%                     T_traj_rot_cv = rpy2tr(rpy_traj_cv) ;
% 
%                     P11 = transl(T11) ; 
%                     P22 = transl(T22) ; 
%                     P_traj_cv = mtraj(@lspb, P11', P22', t) ; % 位置线性插值
%                     T_traj_transl_cv = transl(P_traj_cv) ;
% 
%                     n = length(t) ; 
%                     T_traj_cv = zeros(4, 4, n) ; 
%                         
%                     for i = 1:n
%                         T_traj(:, :, i) = T_traj_transl(:, :, i) * T_traj_rot(:, :, i) ; 
%                     end
%                     
%                     Qtraj_cv = six_link.ikunc(T_traj_cv) ;
%                     
%                     break ;
%                 end      
%             end
%         elseif (j == 2) % 关节二三角度一致
%             if (abs(Qtraj(i, j) - Qtraj(i, j+1)) <= eps_angel)
%                 fprintf('elbow sigularity in %d position!\n', i) ; % Qtraj(i, j) = Qtraj(i, j) + 0.2 ; % 改变路径无意义，就是要到达那个点，故尝试限制加速度或者速度
%                 if ddq(i, j) >= amaz
%                      tend = tend + 1 ; 
%                     tbegin = (tend - 0)/(times - 1) * (i - 1) ;
%                     times = fix(times/2) ; 
%                     
%                     T11 = T_traj(:, :, i) ;
%                     T22 = T2 ;
%                     
%                     rpy11 = tr2rpy(T21)/pi*180 ; % 获得转角
%                     rpy22 = tr2rpy(T22)/pi*180 ; 
%                     
%                     t_cv = linspace(ten, tend, times) ;
%                     rpy_traj_cv = mtraj(@lspb, rpy11, rpy22, t) ; % 转角线性插值
%                     T_traj_rot_cv = rpy2tr(rpy_traj_cv) ;
% 
%                     P11 = transl(T11) ; 
%                     P22 = transl(T22) ; 
%                     P_traj_cv = mtraj(@lspb, P11', P22', t) ; % 位置线性插值
%                     T_traj_transl_cv = transl(P_traj_cv) ;
% 
%                     n = length(t) ; 
%                     T_traj_cv = zeros(4, 4, n) ; 
%                         
%                     for i = 1:n
%                         T_traj(:, :, i) = T_traj_transl(:, :, i) * T_traj_rot(:, :, i) ; 
%                     end
%                     
%                     Qtraj_cv = six_link.ikunc(T_traj_cv) ;
%                     
%                     break ;
%                 end
%             end
%         elseif  (j == 4)  % 四角度在原点上方且四关节垂直
%             if (abs(T_traj_transl(1, 4, i)) <= eps_position) && abs(T_traj_transl(2, 4, i)) <= eps_position && abs((Qtraj(i, j) - pi/2) <= eps_angel || abs(Qtraj(i, j) - pi/2*3) <= eps_angel)
%                 fprintf('shoulder sigularityin %d position!\n', i) ; % Qtraj(i, j-2) = Qtraj(i, j-2) - 0.2 ; % 改变路径无意义，就是要到达那个点，故尝试限制加速度或者速度
%                 if ddq(i, j) >= amax
%                      tend = tend + 1 ; 
%                     tbegin = (tend - 0)/(times - 1) * (i - 1) ;
%                     times = fix(times/2) ; 
%                     
%                     T11 = T_traj(:, :, i) ;
%                     T22 = T2 ;
%                     
%                     rpy11 = tr2rpy(T21)/pi*180 ; % 获得转角
%                     rpy22 = tr2rpy(T22)/pi*180 ; 
%                     
%                     t_cv = linspace(ten, tend, times) ;
%                     rpy_traj_cv = mtraj(@lspb, rpy11, rpy22, t) ; % 转角线性插值
%                     T_traj_rot_cv = rpy2tr(rpy_traj_cv) ;
% 
%                     P11 = transl(T11) ; 
%                     P22 = transl(T22) ; 
%                     P_traj_cv = mtraj(@lspb, P11', P22', t) ; % 位置线性插值
%                     T_traj_transl_cv = transl(P_traj_cv) ;
% 
%                     n = length(t) ; 
%                     T_traj_cv = zeros(4, 4, n) ; 
%                         
%                     for i = 1:n
%                         T_traj(:, :, i) = T_traj_transl(:, :, i) * T_traj_rot(:, :, i) ; 
%                     end
%                     
%                     Qtraj_cv = six_link.ikunc(T_traj_cv) ;
%                     
%                     break ;
%                 end
%             end 
%         end
%     end
% end

% six_link.plot(Qtraj) ;
% pause ;
% six_link.plot(Qtraj_cv) ;

%%
% 位置速度加速度图表
q1 = six_link.ikunc(T1) ;
q2 = six_link.ikunc(T2) ;
[q dq ddq] = jtraj(q1, q2, t) ;
subplot(1, 3, 1) ;
plot(t, q) ;
grid on
xlabel('time') ;
xlabel('q') ;
legend('joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6') ;

subplot(1, 3, 2) ;
plot(t, dq) ;
grid on
xlabel('time') ;
xlabel('dq') ;
legend('joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6') ;

subplot(1, 3, 3) ;
plot(t, ddq) ;
grid on
xlabel('time') ;
xlabel('ddq') ;
legend('joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6') ;

%%
% 两段轨迹

L(1) = Link([0   162.5   0       pi/2],  'standard');
L(2) = Link([0   0      -425     0   ],  'standard');
L(3) = Link([0   0      -392.2   0   ],  'standard');
L(4) = Link([0   133.3   0       pi/2],  'standard');
L(5) = Link([0   99.7    0      -pi/2],  'standard');
L(6) = Link([0   99.6    0       0   ],  'standard');

six_link = SerialLink([L(1) L(2) L(3) L(4) L(5) L(6)]);

T1 = transl(300, 400, 300) * troty(50) ; 
T2 = transl(0, 0, 400) * trotx(130) ;
T3 = transl(-100, 300, 500) * trotz(100) ;

rpy1 = tr2rpy(T1)/pi*180 ; % 获得转角
rpy2 = tr2rpy(T2)/pi*180 ; 
rpy3 = tr2rpy(T3)/pi*180 ; 

% 此处转动复杂，原因是函数超过了范围，范围属于【-pi， pi】之间

wp = [300, 400, 300; 0, 0, 400; -100, 300, 500] ;
a = 2 ;
b = 2 ;
t = 0.04 ;
ac = 0.05 ;
P_traj = mstraj(wp, [], [a b], [], t, ac) ;  % 位置线性插值
T_traj_transl = transl(P_traj) ;

wr = [rpy1; rpy2; rpy3] ;
rpy_traj = mstraj(wr, [], [a b], [], t, ac) ;  % 位置线性插值
T_traj_rot = rpy2tr(rpy_traj) ;

n = (a + b)/t ; 
T_traj = zeros(4, 4, n) ; 

for i = 1:n
    T_traj(:, :, i) = T_traj_transl(:, :, i) * T_traj_rot(:, :, i) ; 
end

Qtraj = six_link.ikunc(T_traj) ;

eps_angel = 0.1 ; % 设置奇异区域精度
eps_position = 10 ;
amax = 1 ; 

q = zeros(n, 6) ; 
dq = zeros(n, 6) ; 
ddq = zeros(n, 6) ; 

for i = 1:n-2
    q1 = six_link.ikunc(T_traj(:, :, i)) ;
    q2 = six_link.ikunc(T_traj(:, :, i+1)) ;
    q3 = six_link.ikunc(T_traj(:, :, i+2)) ;
    dq(i, :) = (q3-q2)/t-(q2-q1)/t ; 
    ddq(i, :) = ((q3-q2)/t-(q2-q1)/t)/t/180*pi ;
end

for i = 1:n
    q(i, :) = six_link.ikunc(T_traj(:, :, i)) ;
end

t1 = linspace(0, 4, 100) ;

subplot(1, 3, 1) ;
plot(t1, q) ;
grid on
xlabel('time') ;
xlabel('q') ;
legend('joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6') ;

subplot(1, 3, 2) ;
plot(t1, dq) ;
grid on
xlabel('time') ;
xlabel('dq') ;
legend('joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6') ;

subplot(1, 3, 3) ;
plot(t1, ddq) ;
grid on
xlabel('time') ;
xlabel('ddq') ;
legend('joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6') ;

% 停止法
% for i = 1:n
%     for j = 1:6
%         if (j == 5) % 关节五为0
%             if(abs(Qtraj(i, j)) <= eps_angel) 
%                 fprintf('wrist sigularity in %d position!\n', i) ; % Qtraj(i, j) = Qtraj(i, j) - 0.2 ; % 改变路径无意义，就是要到达那个点，故尝试限制加速度或者速度
%                 if abs(ddq(i, 5)) >= amax
%                     error('motor collapse!') % 单纯停止
%                 end      
%             end
%         elseif (j == 2) % 关节二三角度一致
%             if (abs(Qtraj(i, j) - Qtraj(i, j+1)) <= eps_angel)
%                 fprintf('elbow sigularity in %d position!\n', i) ; % Qtraj(i, j) = Qtraj(i, j) + 0.2 ; % 改变路径无意义，就是要到达那个点，故尝试限制加速度或者速度
%                 if abs(ddq(5)) >= amax
%                     error('motor collapse!') % 单纯停止
%                 end
%             end
%         elseif  (j == 4)  % 四角度在原点上方且四关节垂直
%             if (abs(T_traj_transl(1, 4, i)) <= eps_position) && abs(T_traj_transl(2, 4, i)) <= eps_position && abs((Qtraj(i, j) - pi/2) <= eps_angel || abs(Qtraj(i, j) - pi/2*3) <= eps_angel)
%                 fprintf('shoulder sigularityin %d position!\n', i) ; % Qtraj(i, j-2) = Qtraj(i, j-2) - 0.2 ; % 改变路径无意义，就是要到达那个点，故尝试限制加速度或者速度
%                 if abs(ddq(5)) >= amax
%                     error('motor collapse!') % 单纯停止
%                 end
%             end 
%         end
%     end
% end
% 
% six_link.plot(Qtraj) ;












