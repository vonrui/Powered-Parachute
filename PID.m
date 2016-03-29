function ctlu = PID(x, xIdeal)
%
%
%
%
global tf xIdeal
disp('=======================================================')
disp('enter in PID')
disp('=======================================================')
%计算x偏差量,微分，积分
errx= xIdeal - x;
errxHis=[];
errxHis= [errxHis, errx];
errxInte=errx* tf;          %误差积分
ctlu  = [0;0;0];                %初始化控制量

%高度控制回路
Kp_height=0.012;
Ti_height= 0.01;
Td_height=0.025;
%最终给出俯仰角theta控制量
thetactl = Kp_height*(-1)*errx(6)+Ti_height*(-1)*errxInte(6)+Td_height*x(3)   


%计算theta偏差量，微分，积分
errtheta= thetactl - x(11);
errthetaHis=[];
errthetaHis= [errthetaHis, errtheta];
errthetaInte=errtheta* tf;          %误差积分

%俯仰角控制回路（作为高度控制回路的内环）
Kp_pitch=1;
Ti_pitch= 0.001;
Td_pitch=0.2;
%最终给出推力控制量
%ctlu(3) = Kp_pitch* errtheta+errthetaInte* Ti_pitch-x(8)*Td_pitch


%偏航角控制回路
Kp_yaw=0.01;
Ti_yaw= 0;         
Td_yaw =0;    
%最终给出滚转角（phi）控制量
phictl = Kp_yaw*errx(12)+Ti_yaw*errxInte(6)-Td_yaw*x(9)  
x(10)
%计算phi偏差量，微分，积分
errphi= phictl - x(10);
errphiHis=[];
errphiHis= [errphiHis, errphi];
errphiInte=errphi* tf;          %误差积分

%横滚角控制回路（作为偏航角控制回路的内环）
Kp_roll=0.02;
Ti_roll= 0;
Td_roll=0.02;
errxHis=[];

%ctlu(1) = 
ctlu(2) = Kp_roll* errphi+ Ti_roll*errphiInte-Td_roll*x(7)





end


