gammai = pi/16;         %将翼伞分为8段，每段对应的圆心角
B = cell(1, 8);                %创建元胞数组

for i = [1: 1 :8]
    B{i} = [ 1                  0                              0
                0  cos((i-4)*gammai)    sin((i-4)*gammai)   
                0  -1*sin((i-4)*gammai)   cos((i-4)*gammai)   
              ]
end