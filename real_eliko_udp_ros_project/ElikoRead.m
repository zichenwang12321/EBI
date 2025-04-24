
function [ZModule, ZPhase] = ElikoRead()
    ZModule = single.empty(0);
    ZPhase = single.empty(0);
    try
        PicometerControl('Start');
    catch
        PicometerControl('Stop');
        PicometerControl('Start');
    end

    Data = PicometerControl('GetData', 149);
    ZModule = vertcat(ZModule, Data.ImpedanceModule(51:149,:));
    ZPhase = vertcat(ZPhase, Data.ImpedancePhase(51:149,:));

    PicometerControl('Stop');
end
