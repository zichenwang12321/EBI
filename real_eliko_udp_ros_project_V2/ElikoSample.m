
function [ComplexOut] = ElikoSample()
    while 1
        try
            [ZModule, ZPhase] = ElikoRead();
        catch 
            [ZModule, ZPhase] = ElikoRead();
        end

        ComplexOut = zeros(size(ZModule));
        for row = 1:size(ZModule, 1)
            for col = 1:size(ZModule, 2)
                mod = ZModule(row, col);
                ang = ZPhase(row, col);
                [X, Y] = pol2cart(deg2rad(ang), mod);
                ComplexOut(row, col) = complex(X, Y);
            end
        end
        break
    end
end
%% 虚拟测试
% 模拟一个 3x3 的复数矩阵
   % realPart = rand(3,3) * 10;
    %imagPart = rand(3,3) * 10;
    %ComplexOut = complex(realPart, imagPart);
%end
%% 
 % 返回一个 4x4 模拟复数矩阵
   % realPart = rand(4, 4) * 100;     % 实部
   % imagPart = rand(4, 4) * 100;     % 虚部
   % ComplexOut = complex(realPart, imagPart);
%end