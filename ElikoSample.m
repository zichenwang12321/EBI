function [ComplexOut] = ElikoSample()
    while true
        try
            [ZModule, ZPhase] = ElikoRead();
        catch
            [ZModule, ZPhase] = ElikoRead();
        end

        tempComplex = zeros(size(ZModule));
        for row = 1:size(ZModule, 1)
            for col = 1:size(ZModule, 2)
                mod = ZModule(row, col);
                ang = ZPhase(row, col);
                [X, Y] = pol2cart(deg2rad(ang), mod);
                tempComplex(row, col) = complex(X, Y);
            end
        end

        % 不做discard 0/负值的处理
        break;
    end

    ComplexOut = tempComplex; % 只保存第3次数据
end
