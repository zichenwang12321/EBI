function ElikoReconnect()
    try
        PicometerControl('Disconnect');
    catch exception

    end

    PicometerControl('Connect');
    PicometerControl('SetSamplingDivider', 1);
    PicometerControl('SetCompensation', 'Off', 'Off');
    PicometerControl('SetExcitationLevel', 120);
    PicometerControl('SetInputGain', '5x', '1x');
end