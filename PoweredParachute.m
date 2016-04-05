      %   PowerParachute 动力伞飞行仿真模型

      tic     %设置程序运行起始时间
      clear
      global  u x xIdeal V CONHIS tuHis deluHis uInc TrimHist RUNNING tf  PidCtrl tol
      global m Ixx Iyy Izz Ixz S b cBar r Mtheta lWing xcm  xcp CWing CBody CRoll CPitch Slat Slong emax Lz Sb
      global AlphaTable CDTable0 CDTable45 CDTable90 CLTable0 CLTable45 CLTable90 CYBetaTable ClBetaTable CldATable CmTable CndATable CnBetaTable
      load ('/home/ldp/app/R2013b_UNIX/ldp workspace/PoweredParachute/InerGeo.mat')
      load ('/home/ldp/app/R2013b_UNIX/ldp workspace/PoweredParachute/DataTable.mat')
      %CONHIS  是否记录(Control history)
      %tuHis   tu的历史记录
      %deluHis delta u(t)的历史记录
      %uInc    u的增量记录
      %TrimHist trim的历史记录


      
     
      
      
      disp('** 6-DOF FLIGHT Simulation **')
      date
      warning('off')
      
      %	This is the SCRIPT FILE.  It contains the Main Program, which:
      %		Defines initial conditions 定义初始化条件
      %		Contains aerodynamic data tables (if required)　包含空气动力学数据表
      %		Calculates longitudinal trim condition　计算纵向配平条件
      %		Calculates stability-and-control derivatives　计算稳定和控制导数
      %		Simulates flight path using nonlinear equations of motion
      %		利用非线性方程模拟飞行路径
      
      
      %	DEFINITION OF THE STATE VECTOR
      %       体轴系的惯性速度(相对于地面坐标系)
      %		x(1)    = 		Body-axis x inertial velocity, ub, m/s
      %		x(2)    =		Body-axis y inertial velocity, vb, m/s
      %		x(3)    =		Body-axis z inertial velocity, wb, m/s
      %       质心在地面坐标系中的位置
      %		x(4)    =		North position of center of mass WRT Earth, xe, m
      %		x(5)    =		East position of center of mass WRT Earth, ye, m
      %		x(6)    =		Negative of c.m. altitude WRT Earth, ze = -h, m
      %       角速度
      %		x(7)    =		Body-axis roll rate, pr, rad/s
      %		x(8)    =		Body-axis pitch rate, qr, rad/s
      %		x(9)    =		Body-axis yaw rate, rr,rad/s
      %       地面坐标系中的姿态角
      %		x(10)   =		Roll angle of body WRT Earth, phir, rad
      %		x(11)   =		Pitch angle of body WRT Earth, thetar, rad
      %		x(12)   =		Yaw angle of body WRT Earth, psir, rad
	
      %	DEFINITION OF THE CONTROL VECTOR
      %       舵面偏角定义
      %		u(1)    = 		Aileron1, dAr, rad, positive: left trailing edge down
      %   u(2)    =     Aileron2, dAr, rad, positive: right trailing edge down
      %		u(3)    = 		Throttle, dT, %           油门百分数

      %   ======================================================================
      %	USER INPUTS
      %	======================================================================

      %	FLIGHT Flags (1 = ON, 0 = OFF)

      TRIM    = 	1;          % Trim flag (= 1 to calculate trim @ I.C.)
      LINEAR  =  1;		% Linear model flag (= 1 to calculate and store F and G)
      SIMUL   =	1;		% Flight path flag (= 1 for nonlinear simulation)
      PidCtrl  =  0;            %设置ＰＩＤ控制器（１为加入控制）
      tol         =   500;
      xIdeal = [ 0;0;0
                      0;0;-9980
                      0;0;0
                      0;0;0.1];     %注意x(6)<0
    
    
      %	Initial Altitude (ft), Indicated Airspeed (kt), 
      %   Dynamic Pressure (N/m^2), and True Airspeed (m/s0
      
      %定义初始高度, 空速
      hm          =   10000;        % Altitude above Sea Level, m
      VmsIAS   =   7 ;           % Indicated Airspeed, m/s
      
      %计算空气性质：　密度　压力　温度　声速
      [airDens,airPres,temp,soundSpeed] = Atmos(hm) ;  
      
      % Dynamic Pressure at sea level, N/m^2（海平面动压）
      qBarSL  =   0.5*1.225*VmsIAS^2; 
      
      % True Airspeed, TAS, m/s
      V   =   sqrt(2*qBarSL/airDens);	
      TASms   =   V;
       
      
      %	Alphabetical List of Initial Conditions（初始条件表）

      alpha   =	0;      % Angle of attack, deg (relative to air mass)　攻角
      beta    =	0;      % Sideslip angle, deg (relative to air mass)　侧滑角
      dA1     =0 ;      % Aileron angle, deg　　　　　　　　　   左侧副翼角度
      dA2      = 0;        % Aileron angle, deg　　　　　　　　　　　右侧副翼角度
      dT      =  0;   % Throttle setting, %(<1)　　　　　　油门百分数
      hdot    =	0;      % Altitude rate, m/s　　　　　　　　　高度率
      p       =	0;      % Body-axis roll rate, deg/s　　　　　　体轴系滚转角速率
      phi     =	0;      % Body roll angle wrt earth, deg　　　　滚转角（地面坐标系）
      psi     =	0;      % Body yaw angle wrt earth, deg　　　　偏航角（地面坐标系）
      q       =	0;      % Body-axis pitch rate, deg/sec　　　　　俯仰角（体轴系）
      r       =	0;      % Body-axis yaw rate, deg/s　　　　　　　偏航角（体轴系)
      tf      =	0.1;    % Final time for simulation, sec　　　　　总时间
      ti      = 	0;      % Initial time for simulation, sec　　　　　初始时间
      theta   =	alpha;  % Body pitch angle wrt earth, deg [theta = alpha if hdot = 0]
      xe      =	0;      % Initial longitudinal position, m　　　　初始纵向位置
      ye      = 	0;      % Initial lateral position, m　　　　　　初始横向位置
      ze      = 	-hm;    % Initial vertical position, m [h: + up, z: + down]　初始垂直方向位置（注意Ｚ向下为正）
      
      % Initial Conditions Depending on Prior Initial Conditions

      % Inertial Vertical Flight Path Angle, deg
      gamma = 57.2957795 * atan(hdot / sqrt(V^2 - hdot^2));
      % Dynamic Pressure, N/m^2（动压）
      qbar  =   0.5 * airDens * V^2 ;
      % Indicated Air Speed, m/s(声速)
      IAS   = sqrt(2 * qbar / 1.225);
      % Mach Number(马赫数)
      Mach  =   V / soundSpeed  ;
      

      % Initial State Perturbation（初始状态扰动） (Test Inputs: m, m/s, rad, or rad/s)
      delx  = [0;0;0
                   0;0;0
                   0;0;0
                   0;0;0];

      % Initial Control Perturbation （初始控制摄动）(Test Inputs: rad or 100%)     
      delu  = [0;0;0];

      % State Vector and Control Initialization, rad  (转化为弧度单位)
      phir  = phi * .01745329;
      thetar  = theta * .01745329;
      psir  = psi * .01745329;
      
      pr =  p * 0.01745329;       %转化为弧度单位
      qr =    q * 0.01745329;
      rr =    r * 0.01745329;

      windbody = WindField(-ze,phir,thetar,psir);  %计算初始风场
      alphar  = alpha * .01745329;
      betar = beta * .01745329;


      % DEFINITION OF THE STATE VECTOR
      %   x(1)    =     Body-axis x inertial velocity,                ub, m/s
      %   x(2)    =   Body-axis y inertial velocity,                  vb, m/s
      %   x(3)    =   Body-axis z inertial velocity,                  wb, m/s
      %   x(4)    =   North position of center of mass WRT Earth,     xe, m
      %   x(5)    =   East position of center of mass WRT Earth,      ye, m
      %   x(6)    =   Negative of c.m. altitude WRT Earth,            ze = -h, m
      %   x(7)    =   Body-axis roll rate,                             pr, rad/s
      %   x(8)    =   Body-axis pitch rate,                           qr, rad/s
      %   x(9)    =   Body-axis yaw rate,                             rr,rad/s
      %   x(10)   =   Roll angle of body WRT Earth,                   phir, rad
      %   x(11)   =   Pitch angle of body WRT Earth,                  thetar, rad
      %   x(12)   =   Yaw angle of body WRT Earth,                    psir, rad
    
      x = [V * cos(alphar) * cos(betar) - windbody(1)
           V * sin(betar) - windbody(2)
           V * sin(alphar) * cos(betar) - windbody(3)
           xe
           ye
           ze
            pr
            qr
            rr
           phir
           thetar
           psir];
  
      % DEFINITION OF THE CONTROL VECTOR
      %       舵面偏角定义
      %   u(1)    =     Aileron1, dAr, rad, positive: left trailing edge down
      %   u(2)    =     Aileron2, dAr, rad, positive: right trailing edge down
      %   u(3)    =     Throttle, dT, %           油门百分数
       
        
      u = [dA1 * 0.01745329     %左副翼偏角(弧度)
           dA2 * 0.01745329       %右副翼偏角(弧度)
           dT                                %推力百分数(<1)
           ];

       %	Linear Model Stability-and-Control Derivative Calculation
       %   ============================================
       
       if LINEAR >= 1
		disp('Generate and Save LINEAR MODEL')
		thresh	=	[.1;.1;.1;.1;.1;.1;.1;.1;.1;.1;.1;.1;.1;.1;.1];
		xj		=	[x;u];
		xdotj		=	LinModel(ti,xj);
		[dFdX,fac]	=	numjac('LinModel',ti,xj,xdotj,thresh,[],0);
		Fmodel		=	dFdX(1:12,1:12)
		Gmodel		=	dFdX(1:12,13:15)
		save ('/home/ldp/app/R2013b_UNIX/ldp workspace/PoweredParachute/Fmodel','Fmodel','TASms','hm')
		save ('/home/ldp/app/R2013b_UNIX/ldp workspace/PoweredParachute/Gmodel','Gmodel')
         end

       
      % Flight Path Calculation
      %   =======================
      
      xrecord = [];         %x历史记录
      urecord =[];          %u历史记录
      trecord = [];          %t历史记录
      xo    = x + delx;     %xo初始条件
      kHis= 1;
      ctlu =[];                 %创建ctlu数组
      tolHis =[1:1:tol];    %循环次数
      
      %==========================================
      %循环求解EOM微分方程组
      %==========================================
      for i=[1:1:tol]

         tspan = [ti tf];
         xo    = x + delx;
            %是否使用ＰＩＤ控制器
            if PidCtrl >=1
            ctlu = PID(x, xIdeal);
            u   =   u + ctlu;
            u = uconfine(u);
            end
         options =   odeset('Events',@event,'RelTol',1e-4,'AbsTol',1e-4);
         [t,x] = ode15s(@EOM,tspan,xo,options);      %求解运动学方程

         %记录控制历史
         xrecord = [xrecord;  x];
         urecord =[urecord, u];
         if i ==1
         trecord = [trecord ; t];
         else 
         trecord = [trecord; t+(i-1)*tf];
         end
         x = x(end, :);
         x = x';
         
         disp('i=')
         disp(i)
         end
         
         kHis  =kHis+ length(t);
        
         

      %========================================================================
      %   Plot 
      %========================================================================
        %3D轨迹图
        figure
        subplot(3,2,1)
        plot3(xrecord(:,4),xrecord(:,5),-xrecord(:,6))
		xlabel('North, m'), ylabel('East, m'), zlabel('Altitude, m'), grid
        title('3D Flight Path')
        
        %欧拉角
        %figure
        subplot(3,2,2)
        plot(trecord,xrecord(:,10) * 57.29578,trecord,xrecord(:,11) * 57.29578,trecord,xrecord(:,12) * 57.29578)
		xlabel('Time, s'), ylabel('phi (blue), theta (green), psi (red), deg'), grid
        title('Euler Angles')
        legend('Roll angle, phi', 'Pitch angle, theta', 'Yaw angle, psi')
        
        %三轴速度(注意转化成了度/秒)
        %figure
        subplot(3,2,3)
        plot(trecord,xrecord(:,1) ,trecord,xrecord(:,2) ,trecord,xrecord(:,3) )
		xlabel('Time, s'), ylabel('Vx (blue), Vy(green), Vz(red), m/s'), grid
        title('Velocity')
        legend('Vx,x(1)', 'Vy,x(2)', 'Vz, x(3)')
        
         %三轴角速度
        %figure
        subplot(3,2,4)
        plot(trecord,xrecord(:,7)* 57.29578 ,trecord,xrecord(:,8)* 57.29578 ,trecord,xrecord(:,9)* 57.29578 )
		xlabel('Time, s'), ylabel('Wx (blue), Wy(green), Wz(red), m/s'), grid
        title('Angular Velocity')
        legend('Wx,x(7)', 'Wy,x(8)', 'Wz, x(9)')
        
        %控制量u
        %figure
        subplot(3,2,5)
        %在作图的时候将tolHis*tf以便时间刻度与其他曲线图统一
        plot(tolHis*tf, urecord(1,:)*57.29578,tolHis*tf, urecord(2,:)*57.29578,tolHis*tf, urecord(3,:))
        xlabel('tolHis, s'), ylabel('dA1 (blue), dA2 (green), Thrust (red), deg'), grid
        title('urecord')
        legend('Left Alieon angle, dA1', 'Right Alieon angle, dA2', 'Thrust, dT')
        
        disp('END OF SIMULATION')
        toc
       

        
        
        
        
        
      
