L(1) = Link([0   162.5   0       pi/2],  'standard');
L(2) = Link([0   0      -425     0   ],  'standard');
L(3) = Link([0   0      -392.2   0   ],  'standard');
L(4) = Link([0   133.3   0       pi/2],  'standard');
L(5) = Link([0   99.7    0      -pi/2],  'standard');
L(6) = Link([0   99.6    0       0   ],  'standard');

six_link = SerialLink([L(1) L(2) L(3) L(4) L(5) L(6)]);

T1 = transl(300, 400, 100) * troty(50) ; 
T2 = transl(0, 0, 600) * trotx(130) ;
T3 = transl(-100, 0, 500) * trotz(100) ;

rpy1 = tr2rpy(T1)/pi*180 ; % 获得转角
rpy2 = tr2rpy(T2)/pi*180 ; 
rpy3 = tr2rpy(T3)/pi*180 ; 

% 此处转动复杂，原因是函数超过了范围，范围属于【-pi， pi】之间

wp = [300, 400, 100; 0, 0, 600; -100, 0, 500] ;
a = 2 ;
b = 2 ;
t = 0.05 ;
ac = 1.8 ;
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

tx = 1 ;

for i = 1:n-1
    dq(i, :) = (Qtraj(i+1, :) - Qtraj(i, :))/tx ;
end

for i = 1:n-2
    ddq(i, :) = (dq(i+1, :) - dq(i, :))/tx ;
end

% t1 = linspace(0, a + b, n) ;
% 
% subplot(1, 3, 1) ;
% plot(t1, Qtraj) ;
% grid on
% xlabel('time') ;
% xlabel('q') ;
% legend('joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6') ;
% 
% subplot(1, 3, 2) ;
% plot(t1, dq) ;
% grid on
% xlabel('time') ;
% xlabel('dq') ;
% legend('joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6') ;
% 
% subplot(1, 3, 3) ;
% plot(t1, ddq) ;
% grid on
% xlabel('time') ;
% xlabel('ddq') ;
% legend('joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6') ;

% 停止法
for i = 1:n
    for j = 1:6
        if (j == 5) % 关节五为0
            if(abs(Qtraj(i, j)) <= eps_angel) 
                fprintf('wrist sigularity in %d position!\n', i) ; % Qtraj(i, j) = Qtraj(i, j) - 0.2 ; % 改变路径无意义，就是要到达那个点，故尝试限制加速度或者速度
                if abs(ddq(i, 5)) >= amax
                    error('motor collapse!') % 单纯停止
                end      
            end
        elseif (j == 2) % 关节二三角度一致
            if (abs(Qtraj(i, j) - Qtraj(i, j+1)) <= eps_angel)
                fprintf('elbow sigularity in %d position!\n', i) ; % Qtraj(i, j) = Qtraj(i, j) + 0.2 ; % 改变路径无意义，就是要到达那个点，故尝试限制加速度或者速度
                if abs(ddq(i, 5)) >= amax
                    error('motor collapse!') % 单纯停止
                end
            end
        elseif  (j == 4)  % 四角度在原点上方且四关节垂直
            if (abs(T_traj_transl(1, 4, i)) <= eps_position) && abs(T_traj_transl(2, 4, i)) <= eps_position && abs((Qtraj(i, j) - pi/2) <= eps_angel || abs(Qtraj(i, j) - pi/2*3) <= eps_angel)
                fprintf('shoulder sigularityin %d position!\n', i) ; % Qtraj(i, j-2) = Qtraj(i, j-2) - 0.2 ; % 改变路径无意义，就是要到达那个点，故尝试限制加速度或者速度
                if abs(ddq(i, 5)) >= amax
                    error('motor collapse!') % 单纯停止
                end
            end 
        end
    end
end

six_link.plot(Qtraj);

%%
% 实际案例
L(1) = Link([0   162.5   0       pi/2],  'standard');
L(2) = Link([0   0      -425     0   ],  'standard');
L(3) = Link([0   0      -392.2   0   ],  'standard');
L(4) = Link([0   133.3   0       pi/2],  'standard');
L(5) = Link([0   99.7    0      -pi/2],  'standard');
L(6) = Link([0   99.6    0       0   ],  'standard');

six_link = SerialLink([L(1) L(2) L(3) L(4) L(5) L(6)]);

P1 = [-450 -450 600] ;
P2 = [450 450 600] ;

% P1 = [-450 -450 600] ;
% P2 = [500 300 600] ;

t = linspace(0, 2, 101) ;
Traj = mtraj(@tpoly, P1, P2, t) ;

n = size(Traj, 1) ; 
T = zeros(4, 4, n) ; 

for i = 1:n
    T(:, :, i) = transl(Traj(i, :)) ; 
end

Qtraj = six_link.ikunc(T) ; 

% six_link.plot(Qtraj) ;

q1 = six_link.ikunc(T(:, :, 1)) ;
q2 = six_link.ikunc(T(:, :, n)) ;
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
