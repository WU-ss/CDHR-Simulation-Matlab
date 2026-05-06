%标准DH
%Link(DH,option):
%Link(DH,option):DH = [THETAi Di Ai-1 ALPHAi-1 SIGMA]
L1 = Link([0 0 0 0 0],'modified');
L2 = Link([0 0 4 0 0],'modified');
L3 = Link([0 0 3 0 0],'modified');
robot = SerialLink([L1 L2 L3]); %建立连杆机器人
% robot.plot([deg2rad(10) deg2rad(20) deg2rad(30)]) %画机器人运动示意图图
TRRR=robot.fkine([deg2rad(10) deg2rad(20) deg2rad(30)]) %求正运动学的变换矩阵

q_solutions = robot.ikine(TRRR,'mask',[1,1,1,0,0,0])
deg2rad(10)
