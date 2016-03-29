function   xdot = EOM(t,x)
%	FLIGHT Equations of Motion
%   EOM描述运动微分方程
%   2016/3/5
%====================================================================
%disp('***************** ENTER IN EOM *******************')

global CRoll CPitch Slat Slong emax Lz Sb
global m Ixx Iyy Izz Ixz S b cBar  CONHIS u tuHis deluHis uInc  RUNNING
%       S                   机翼面积
%       b                   机翼展长
%       cbar              机翼平均气动弦长
%       CONHIS        是否记录控制历史
%       u                   控制量
%       tuHis            机翼面积
%       deluHis         差值
%       uInc               控制增量
%       RUNNING      是否在运行




%       姿态角
%		x(10)   =		Roll angle of body WRT Earth,   phir,     rad
%		x(11)   =		Pitch angle of body WRT Earth, thetar,  rad
%		x(12)   =		Yaw angle of body WRT Earth,   psir,     rad
%       Earth-to-Body-Axis Transformation Matrix
%       计算变换矩阵
          HEB		=	DCM(x(10),x(11),x(12));
    
%		x(6)    =		Negative of c.m. altitude WRT Earth, ze = -h, m
%       Atmospheric State（大气状态）
          x(6)    =   min(x(6),0);        % 约束Z方向高度代数值x(6)小于0
          [airDens,airPres,temp,soundSpeed]	=	Atmos(-x(6));
    
    
%       Body-Axis Wind Field　（体轴系风场）
           windbody	=	WindField(-x(6),x(10),x(11),x(12));

%   	Body-Axis Gravity Components　（体轴系重力组成）
          gb	=	HEB * [0;0;9.80665];
          
%       Air-Relative Velocity Vector
%       考虑风场:计算体轴系相对于地面坐标系的相对速度矢量
           x(1)    =   max(x(1),0);                         %   Limit axial velocity to >= 0 m/s
           Va		=	[x(1);x(2);x(3)] + windbody;      %计算相对速度
        	V		=	sqrt(Va' * Va);                  %计算速度模值 
           alphar	=	atan(Va(3) / abs(Va(1)));  %计算攻角
           
           alphar  =   min(alphar, (pi/2 - 1e-6));  %避免攻角出现奇异
%       Limit angle of attack to <= 90 deg       %限制攻角小于９０度
           alpha 	=	57.2957795 * alphar ;      %将单位转换为度
           
           
           betar	= 	asin(Va(2) / V);                   %计算侧滑角
           beta	= 	57.2957795 * betar;               %将单位转化为度
           Mach	= 	V / soundSpeed;                   %计算马赫数
           qbar	=	0.5 * airDens * V^2;               %计算动压
           
  
   
%       Force and Moment Coefficients; Thrust
%       调用函数AeroModel()计算 力与力矩系数; 推力
          [CD,CL,CY,Cl,Cm,Cn,Thrust]	=	AeroModel(x,u,Mach,alphar,betar,V);
           
          
           qbarS	=	qbar * S;

           CX	=	-CD * cos(alphar) + CL * sin(alphar);	% Body-axis X coefficient
           CZ	= 	-CD * sin(alphar) - CL * cos(alphar);	% Body-axis Z coefficient
           
%      	State Accelerations 
           Xb =	(CX * qbarS + Thrust) / m;          %X轴加速度(推力线与X轴共线)
           Yb =	CY * qbarS / m;
           Zb =	CZ * qbarS / m;
           
           %计算气动阻尼力矩大小
           Ldamp = 1/2*1*airDens*Slat*x(7)^2*(Lz-CRoll)^3+1/2*1*airDens*Sb*x(7)^2*CRoll^3;
           Mdamp = 1/2*1*airDens*Slong*x(8)^2*(Lz-CRoll)^3+1/2*1*airDens*Sb*x(8)^2*CRoll^3;
           Ndamp = 0.1*1/64*1*airDens*emax*x(9)^2*b^4;
           
           %重力回复力矩大小
           gb(3);
           LG= m*gb(3)*CRoll*sin(x(10));
           MG= m*gb(3)*CPitch*sin(x(11));
           
            %X轴滚转力矩 考虑重力回复力矩
           Lb =	Cl * qbarS * b  %-LG-Ldamp        %正方向：从本体后方看顺时针滚转
           %Y轴俯仰力矩  考虑推力力矩 重力回复力矩  气动阻尼力矩
           Mb =	Cm * qbarS * cBar+Thrust*CPitch-MG -Mdamp;
           %Z轴偏航力矩 考虑副翼阻力产生的力矩  空气阻尼力矩
           Cn * qbarS * b;
           Nb =	Cn * qbarS * b +Ndamp  ;              

           nz	=	-Zb / 9.80665;							% Normal load factor(Z轴向过载)
           
           
%        Dynamic Equations
%        四组动力学方程:分别计算x(1)~x(10)的微分xd1~xd10

%        计算X Y Z 轴加速度
            xd1 = Xb + gb(1) + x(9) * x(2) - x(8) * x(3);
        	xd2 = Yb + gb(2) - x(9) * x(1) + x(7) * x(3);
            xd3 = Zb + gb(3) + x(8) * x(1) - x(7) * x(2);
            
%        计算X Y Z轴在体轴系中的速度
        	y	=	HEB' * [x(1);x(2);x(3)];
        	xd4	=	y(1);
            xd5	=	y(2);
            xd6	=	y(3);
            
%         计算X Y Z轴角加速度
        	xd7	= 	(Izz * Lb + Ixz * Nb - (Ixz * (Iyy - Ixx - Izz) * x(7) + ...
                        (Ixz^2 + Izz * (Izz - Iyy)) * x(9)) * x(8)) / (Ixx * Izz - Ixz^2);
            
            xd8 = 	(Mb - (Ixx - Izz) * x(7) * x(9) - Ixz * (x(7)^2 - x(9)^2)) / Iyy;
    
        	xd9 =	(Ixz * Lb + Ixx * Nb - (Ixz * (Iyy - Ixx - Izz) * x(9) + ...
				(Ixz^2 + Ixx * (Ixx - Iyy)) * x(7)) * x(8)) / (Ixx * Izz - Ixz^2);

%         避免俯仰角奇异            
            cosPitch	=	cos(x(11));         %计算俯仰角
            if abs(cosPitch)	<=	0.00001
            cosPitch	=	0.00001 * sign(cosPitch);
            end
            tanPitch	=	sin(x(11)) / cosPitch;
                            
%         计算角速度           
            xd10	=	x(7) + (sin(x(10)) * x(8) + cos(x(10)) * x(9)) * tanPitch;                
            xd11	=	cos(x(10)) * x(8) - sin(x(10)) * x(9);
            xd12	=	(sin(x(10)) * x(8) + cos(x(10)) * x(9)) / cosPitch;

%         EOM函数最终返回x(1)~x(12)的微分xd1~xd12           
            xdot	=	[xd1;xd2;xd3;xd4;xd5;xd6;xd7;xd8;xd9;xd10;xd11;xd12];

            
            
    