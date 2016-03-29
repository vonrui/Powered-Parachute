%   Supporting Calculations for Geometric, Inertial, and Aerodynamic 
%   Properties of Powered Parachute
%   为AeroModel函数准备几何,惯性,气动数据



    clear
    disp('==============================================================')
    %disp('***************** ENTER IN PGEOMASSAERO *******************')
    disp('==============================================================')
    Date    =   date
    
    
    
    %结构质量    单位: KG
    WingSys = 5;                         %翼伞质量
    BodySys = 135;                     %载荷仓质量
    m  = WingSys + BodySys;
    %计算质心
    Lz = 7;
    CWing = Lz*BodySys/m;            %质心与翼伞距离
    CBody = Lz*WingSys/m;            %质心与载荷仓距离
    CRoll = 3.5;                                  %滚转中心
    CPitch = 3.5;                                %俯仰中心
   
    
    %几何特征
    S = 37.5                                      %伞翼面积
    Sb = 0.5                                      %载荷仓面积
    cBar = 3.75                             %伞翼平均气动弦长
    b = 10                                      %展长
    xcp = -1.5                                %机翼压心在本体系中的位置
    xcm     =   xcp + 0.45*cBar    %   (计算质心)Center of mass from nose (USER-specified), m
    lWing   =   xcm - xcp              % Horizontal distance between c.m and wing c.p., m
    zWing   =   -3                    % Vertical distance between c.m and wing c.p., m
    emax= 0.18*cBar            %伞翼最大厚度
    Slong = emax*b         %伞翼纵向投影面积
    Slat = emax* cBar      %翼伞横向投影面积
    
    
    %惯性积（考虑了附加质量）
    %   Moments and Product of Inertia
    Jp= WingSys*[  1/12*b*b+CWing*CWing              0             1/4*cBar*CWing;
                               0       7/48*cBar*cBar+CWing*CWing     0        ;
                               1/4*cBar*CWing      0              1/12*b*b+7/48*cBar*cBar
                          ]
                      
    Jb= BodySys*[ 1/6*Sb+CBody*CBody    0             0;
                                0       1/6*Sb+CBody*CBody       0;
                                 0                   0                       1/6*Sb
                            ]
    Jr= Jp+ Jb
    
    
    
    Ixx     =  Jr(1,1)
    Iyy     =   Jr(2,2)
    Izz     =   Jr(3,3)
    Ixz     =   Jr(1,3)
    
    
    %   Maximum Aileron Deflection is  35 deg
    dAmax   =   35 * 0.01745329 
    
    save('/home/ldp/app/R2013b_UNIX/ldp workspace/powered parachute/InerGeo.mat','m','xcm','Ixx','Iyy','Izz','Ixz','cBar','b','S', ...
       'xcp','Lz','lWing','CWing','CBody','CRoll','CPitch','emax', 'Slong','Slat','Sb')
    
    
    
    
    disp('===================================')
    disp('Aerodynamic Properties for Power Parachute')
    disp('===================================')
    
    %   Power Parachute Aero Properties    
    AlphaTable	=	[-10 -8 -6 -4 -2 0 2 4 6 8 10 11 12 13 14 15 16 17 18 19 20 ...
					21 22 23 24 25 30 35 40 45 50 55 60 65 70 75 80 85 90];
    Points      =   length(AlphaTable);
    AlphaRad    =   0.0174533*AlphaTable;       %将角度转化为弧度
    SinAlpha    =   sin(AlphaRad);
    CosAlpha    =   cos(AlphaRad);
    
  
    %   Longitudinal Aerodynamics
    %   =========================
    %   Lift(升力系数表)
  
    figure
    %CLTable         =   [-1.10414493484302,-0.883315947874415,-0.662486960905811,-0.441657973937207,-0.220828986968604,0,0.220828986968604,0.441657973937207,0.662486960905811,0.883315947874415,1.10414493484302,1.21170652584920,1.28853623371075,1.33463405842769,1.35000000000000,1.33463405842769,1.28853623371075,1.21170652584920,1.10414493484302,0.965851460692216,0.796826103396791,0.730000000000000,0.730000000000000,0.740000000000000,0.760000000000000,0.780000000000000,0.972831540445641,1.21091619655235,1.42218430518409,1.58862675882105,1.69489278019590,1.72936655653370,1.68499212712194,1.55979005609270,1.35703040737636,1.08504994405072,0.756725704861244,0.388640515367069,-3.02492158249372e-06]
    CLTable            =   [0.350, 0.430, 0.510, 0.590, 0.670, 0.760, 0.808, 0.856, 0.904, 0.952, 1.000, 0.978, 0.957, 0.935, 0.914, 0.893, 0.871, 0.8782, 0.8855, 0.8928, 0.900, 0.876, 0.852, 0.828, 0.804, 0.780, 0.700, 0.636, 0.573, 0.510, 0.446, 0.383, 0.320, 0.240, 0.160, 0.080      0      0      0];
    %                            [-10         -8      -6         -4        -2        0         2         4         6         8        10        11      12       13       14       15       16       17         18       19         20         21          22    23       24        25      30       35        40      45         50      55      60      65        70       75      80    85    90]
    plot(AlphaTable,CLTable),grid, title('CLTable')
    
    
    %   Drag(阻力系数表)
  
    figure
    CDTable         =   [0.0813550435243690,0.0613436742181042,0.0457792758687873,0.0346618484764181,0.0279913920409966,0.0257679065625227,0.0279913920409966,0.0346618484764181,0.0457792758687873,0.0613436742181042,0.0705452253508198,0.0811270091534428,0.0932960605264592,0.107290469605428,0.118019516565971,0.129821468222568,0.142803615044825,0.157083976549307,0.172792374204238,0.190071611624662,0.209078772787128,0.229986650065841,0.252985315072425,0.278283846579668,0.306112231237634,0.339165775094203,0.561664842825890,0.847893121418590,1.19335505107119,1.58862782829305,2.01989609406167,2.46979356322061,2.91849499945190,3.34498481591431,3.72841647598934,4.04947060702661,4.29161974980414,4.44221394754024,4.49331350328450];
    plot(AlphaTable,CDTable), grid, title('CDTable')
    
    
    %   Lateral-Directional Aerodynamics
%   ================================
%   CYBetaTable(侧向力-侧滑角 系数表)
%   Side Force Sensitivity to Sideslip Angle
      
    figure
    CYBetaTable = [-0.648870630191512,-0.652468328898195,-0.655271094769849,-0.657275513065066,-0.658479141706816,-0.658880514257748,-0.658479141706816,-0.657275513065066,-0.655271094769849,-0.652468328898195,-0.648870630191512,-0.646775013095032,-0.644482381895247,-0.641993434950428,-0.639308930418575,-0.636429686026474,-0.633356578820611,-0.630090544900014,-0.626632579131105,-0.622983734844660,-0.619145123514947,-0.615117914421165,-0.610903334291269,-0.606502666928293,-0.601917252819295,-0.597148488727031,-0.570607189478796,-0.539723221256627,-0.504731629908286,-0.465898722800555,-0.423520042057638,-0.377918115305215,-0.329440001038360,-0.278454647294573,-0.225350083734032,-0.170530468497050,-0.114413012313954,-0.0574248032767895,4.43561724861952e-07];
    plot(AlphaTable, CYBetaTable), grid, title('CYBetaTable')
    
    
    
    %   CldATable, Roll Moment Sensitivity to Aileron Deflection(滚转力矩-副翼表)
    figure
    CldATable   =   [0.109879723720884,0.110488957860214,0.110963578108538,0.111303006213698,0.111506828634477,0.111574797044439,0.111506828634477,0.111303006213698,0.110963578108538,0.110488957860214,0.109879723720884,0.109524852014766,0.109136617949138,0.108715139783957,0.108260545905667,0.107772974788084,0.107252574950222,0.106699504911049,0.106113933141201,0.105496038011663,0.104846007739441,0.104164040330220,0.103450343518059,0.102705134702107,0.101928640880382,0.101121098580627,0.0966265961437879,0.0913967063356224,0.0854712318095601,0.0788952690432736,0.0717188651269599,0.0639966368744384,0.0557873551558634,0.0471534976155316,0.0381607731788775,0.0288776219674458,0.0193746944277950,0.00972431363847895,-7.51127228036274e-08];
    plot(AlphaTable, CldATable), grid, title('CldATable')
    
    
    %   ClBetaTable, Roll Moment Sensitivity to Sideslip Angle(滚转力矩-侧滑表)
    
    figure
    ClBetaTable =   [0.00273315190164909,-0.00680028067390678,-0.0164190961285897,-0.0260882584501516,-0.0357725845744313,-0.0454368157631068,-0.0550456891433348,-0.0645640092872919,-0.0739567197095453,-0.0831889741602776,-0.0922262075926827,-0.0965387017675830,-0.0994779349013999,-0.101055015325220,-0.101283471161002,-0.100179235757755,-0.0977606313023011,-0.0940483506151446,-0.0890654371430348,-0.0828372631607971,-0.0753915061960500,-0.0721769929672119,-0.0716824605966475,-0.0715680284690889,-0.0718247374747178,-0.0720471697343258,-0.0761367779685029,-0.0805317068221790,-0.0823773597096791,-0.0811784394392741,-0.0767769348108589,-0.0693734822286305,-0.0595056793767253,-0.0479859475222406,-0.0358064165918617,-0.0240222834136508,-0.0136276888400436,-0.00543906084564649,3.05882073447030e-08];
    plot(AlphaTable, ClBetaTable), grid, title('ClBetaTable')
    
    
    
    
    %   Pitching Moment (c.m. @ wing c.p.)(俯仰力矩表)
    figure
    CmTable         =   [0.146681149175663,0.117619086803684,0.0883741535184620,0.0589921826682437,0.0295189087697684,0,-0.0295189087697684,-0.0589921826682437,-0.0883741535184620,-0.117619086803684,-0.146431183983874,-0.160453916164061,-0.170421615607425,-0.176385633035138,-0.178235051816793,-0.176145295187666,-0.170182791654346,-0.160422460252220,-0.146947792845151,-0.129850946412764,-0.109232846970114,-0.0983095622731028,-0.0884786060457925,-0.0796307454412133,-0.0716676708970920,-0.0754664135070912,-0.122278914880603,-0.194012618843902,-0.253668919760856,-0.306974466289298,-0.360280006538578,-0.411965877257355,-0.460461628075459,-0.504293738884495,-0.542130392038978,-0.572821938996161,-0.595435831835118,-0.609284958277656,-0.613948519265042];
    plot(AlphaTable,CmTable), grid, title('CmTable')
    
    
    %   CndATable, Yaw Moment Sensitivity to Aileron Deflection
    CndATable   =   [0.00273315190164909,-0.00680028067390678,-0.0164190961285897,-0.0260882584501516,-0.0357725845744313,-0.0454368157631068,-0.0550456891433348,-0.0645640092872919,-0.0739567197095453,-0.0831889741602776,-0.0922262075926827,-0.0965387017675830,-0.0994779349013999,-0.101055015325220,-0.101283471161002,-0.100179235757755,-0.0977606313023011,-0.0940483506151446,-0.0890654371430348,-0.0828372631607971,-0.0753915061960500,-0.0721769929672119,-0.0716824605966475,-0.0715680284690889,-0.0718247374747178,-0.0720471697343258,-0.0761367779685029,-0.0805317068221790,-0.0823773597096791,-0.0811784394392741,-0.0767769348108589,-0.0693734822286305,-0.0595056793767253,-0.0479859475222406,-0.0358064165918617,-0.0240222834136508,-0.0136276888400436,-0.00543906084564649,3.05882073447030e-08];
    figure
    plot(AlphaTable, CndATable), grid, title('CndATable')
    
   CnBetaTable = [0.0945055283646115,0.0958882732428614,0.0971626169766501,0.0983249099584958,0.0993716313890952,0.100299396131161,0.101104961404247,0.101785233309260,0.102337273171598,0.102758303692017,0.103045714894635,0.103127547319115,0.103057106287996,0.102835492808735,0.102464034245359,0.101944282974941,0.101278014877257,0.100467227658586,0.0995141390106920,0.0984211846061589,0.0971910159312839,0.0963138449097102,0.0956539349843548,0.0950010360520145,0.0943545381182128,0.0936781838176769,0.0901702901792574,0.0860557173597463,0.0811120592884932,0.0753336588599997,0.0687494618486525,0.0614246096214750,0.0534581303371169,0.0449769648814814,0.0361270026640453,0.0270621601883803,0.0179327688827380,0.00887461987852139,-6.75220573667606e-08]
   figure
   plot(AlphaTable, CnBetaTable), grid, title('CnBetaTable')
    
    save('/home/ldp/app/R2013b_UNIX/ldp workspace/powered parachute/DataTable.mat','AlphaTable','CLTable','CDTable','CmTable', ...
        'CYBetaTable','ClBetaTable','CldATable','CndATable' ,'CnBetaTable')
    
    
    
    