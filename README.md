# Trajectory-Planning
baisc poly and linear algorithm trajectory planning based on matlab

轨迹规划与运动规划
之前讲了正运动学，逆运动学，接下来讲一下实际的应用：运动规划和轨迹规划。所有Matlab代码均放在：https://github.com/AdamTristan/Trajectory-Planning
1.基础知识
1.1运动规划和轨迹规划的区别：
1.1.1 Path planning: generate a pure geometric path, from an initial to a final point.
1.1.2 Trajectory planning: a path function related to time with given path.
1.2关节空间和笛卡尔空间的区别：
1.2.1 Cartesian space: obtain the trajectory of end effect and calculate the joint angle using IK.
1.2.2 Joint space：obtain the initial and final joint angle using IK then calculate the joint trajectory
![image](https://user-images.githubusercontent.com/76904279/185567180-4f20eca4-8585-44fa-9bd4-9260231e28c7.png)
2.常见的规划插值
对于常见的点与点之间的规划，有多项式插值和线性插值。
2.1单段三次项插值（以关节空间为例）
我们把两点之间的距离写成和时间t有关的三次项（其中，ai为所求变量）：
![image](https://user-images.githubusercontent.com/76904279/185567201-d2b93f3a-ec1f-49cb-b675-3c5eb95b1729.png)
已知初位置、初速度，末位置、末速度，故可得ai之间的关系矩阵：
![image](https://user-images.githubusercontent.com/76904279/185567213-c269d98b-54f8-43bd-b95f-94b691da0205.png)
易得ai的数值，故得轨迹。
2.2二段三次项插值
此处分为，单调和变化（对于斜率来说），以变化为例：
![image](https://user-images.githubusercontent.com/76904279/185567233-c1111331-8181-44fe-8728-db76a75f1171.png)
已知初位置、初速度，末位置、末速度，又知道中间速度与加速度左极限应当和中间速度与加速度右极限一一相等，故得到ai之间的关系矩阵：
![image](https://user-images.githubusercontent.com/76904279/185567256-f283a339-29c4-42e3-b950-161800ba8abd.png)
2.3 线性插值（以笛卡尔空间为例）
线性插值是由中间线性，两端二次项构成，如图：
![image](https://user-images.githubusercontent.com/76904279/185567264-b7ada1d1-f770-4ffe-b015-c7bd40a6975d.png)
下面还是回到我们的老朋友，二连杆，末端是一个单自由度的抓手，已知四点：
![image](https://user-images.githubusercontent.com/76904279/185567275-a05d424f-62fd-459f-81af-e7cb91d8039f.png)
首先求速度：
![image](https://user-images.githubusercontent.com/76904279/185567291-b0399c02-a75a-4179-bf4c-de72900b1819.png)
然后求加速度：
![image](https://user-images.githubusercontent.com/76904279/185567301-e7e05b1d-8fb1-46db-9028-4ada3e66b434.png)
已知所有运动参数，反求曲线得（以X为例）：
![image](https://user-images.githubusercontent.com/76904279/185567319-8f10f327-47a0-441b-8f9e-4c90a7021017.png)
根据自己想要插值的时间，带入对应的t，就可以求得此时的X/Y/Z/θ。
3.奇异点分析（Singularity analysis）
3.1奇异点的定义
当末端位于奇异点时，一个末端位置会有无限多组解。因为我们用Jacobian矩阵来联系轴角度及机械手臂末端速度的关系，当机械手臂中的两轴共线时，矩阵内存在线性无关得列向量，造成Jacobian矩阵的秩减少，其行列式值为零，使Jacabian矩阵无逆矩阵。同时，若角度改变，因为Jacobian的行列式为零，所以角速度趋向于无穷。
3.2三种奇异类型（以六轴机械臂为例）
3.2.1腕关节奇异点
四六轴共线，转动角度有无穷组组合：
![image](https://user-images.githubusercontent.com/76904279/185567354-98a26969-c742-4076-819b-a0c6c42bc1bd.png)
3.2.2肘关节奇异点
二三轴和腕关节中心共线（或者共面平行）：
![image](https://user-images.githubusercontent.com/76904279/185567369-3334e63c-0cd6-45f4-8503-bd808f1806fb.png)
3.2.3肩关节奇异点
腕关节中心在一轴正上方，且五轴竖直：
![image](https://user-images.githubusercontent.com/76904279/185567382-e07775f6-b460-4e3e-b39a-4855eb341843.png)
3.3控制速度与加速度
实际上，因为轨迹规划的点是离散的，生活中达不到完全的奇异点，但是接近奇异点角速度仍然会很大，会烧坏电机，甚至造成伤亡。在过奇异点附近时，若是不加以限制，你的速度加速度（左二左三）图表将会是：
![image](https://user-images.githubusercontent.com/76904279/185567403-3c32fbf5-289f-4302-94ef-fecd449bccce.png)
若是你加以限制，则速度加速度（左二左三）图像可以是：
![image](https://user-images.githubusercontent.com/76904279/185567428-bc8c8c44-15c3-4fd5-8d4a-d19addf460b3.png)
通过改变采样频率和二次项加速度可以改变加速度，但是要从根本上改变，还是要轨迹规划时避开奇异点。
4.避免碰撞（Obstacle avoidance）
再回到我们的老朋友，二连杆在平面中，充满了障碍，如图：
![image](https://user-images.githubusercontent.com/76904279/185567439-ccec184f-3c21-4c5a-aef0-a8d1bb55a381.png)
左三图为他的C-空间（位形空间，有几个自由度就是几维的，图像在每个维度上的周期是2Π），若起点和中点均在白色空间（Cfree）中且可以画出不碰到灰黑色空间的不间断轨迹，就证明它可以从开始点运动到中点。
轨迹算法有很多种，这次我采用的是如下代码：
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
End
即，如果发现有规划的轨迹点在障碍物之中，则将该轨迹点先沿着以障碍物中心，障碍物外为方向的射线移动到障碍物边缘，再加5mm的距离防止碰撞。

参考资料：
1.《现代机器人学：机构、规划与控制》Frank C. Park等 机械工业出版社 （初学者最好的书，需要矩阵分析的知识）
2.https://www.bilibili.com/video/BV1oa4y1v7TY?spm_id_from=333.337.search-card.all.click&vd_source=8863b28d80f3287a2bc09e716e5f6988（台大机器人运动学）
3.Khatib, O. (1985). [Institute of Electrical and Electronics Engineers 1985 IEEE International Conference on Robotics and Automation - St. Louis, MO, USA (March 1985)] Proceedings. 1985 IEEE International Conference on Robotics and Automation - Real-time obstacle avoidance for manipulators and mobile robots. , 2(0), 500–505. doi:10.1109/robot.1985.1087247 （不太看得懂的大牛论文）
4.《机器人学、机器视觉与控制》Peter Corke 电子工业出版社 （广而不深）
5.https://zhuanlan.zhihu.com/p/63149294
6.https://zhuanlan.zhihu.com/p/94879910
7.https://zhuanlan.zhihu.com/p/143098261
8.https://cloud.tencent.com/developer/article/1744929
9.https://aitechtogether.com/article/32101.html
10.还有篇印尼兄弟的论文找不到了lol
