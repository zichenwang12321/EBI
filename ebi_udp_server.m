function ebi_udp_server()
    % 参数配置
    DESIRED_HZ = 100;
    DELAY = 1.0 / DESIRED_HZ;
    UDP_IP = '192.168.1.134';
    UDP_PORT = 1235;
    SEQ_NUM = 0;

    % 设备连接参数
    MAX_INIT_RETRIES = 5;
    INIT_RETRY_DELAY = 1; % 秒

    DEBUG = true;

    % UDP 初始化
    u = udpport("datagram", "IPV4");

    % 日志文件初始化
    logFolder = 'code/src/teleop_enhancer_pkg/Log/ebi/';
    if ~exist(logFolder, 'dir')
        mkdir(logFolder);
    end
    logFile = fullfile(logFolder, 'ebi_data_server.csv');
    fid = fopen(logFile, 'w');
    fprintf(fid, 'seq;timestamp;data\n');

    % 设备初始化
    disp('Initializing device connection...');
    init_success = false;

    for retry = 1:MAX_INIT_RETRIES
        try
            try
                PicometerControl('Disconnect');
                pause(0.5);
            catch
            end

            ElikoReconnect();
            pause(0.5);

            testData = ElikoSample();
            if DEBUG
                disp('Initial test data sample:');
                disp(testData);
            end

            if ~isempty(testData) && ismatrix(testData) && ~isreal(testData)
                init_success = true;
                disp(['Device connected successfully on attempt ', num2str(retry)]);
                break;
            else
                error('Received invalid test data format');
            end

        catch ME
            fprintf('Initialization attempt %d failed: %s\n', retry, ME.message);
            if retry < MAX_INIT_RETRIES
                pause(INIT_RETRY_DELAY);
            end
        end

        numCycles = 3; % 读取3次
        for cycle = 1:numCycles
            ElikoSample();
        end
    end

    if ~init_success
        error('Failed to initialize device after %d attempts. Aborting.', MAX_INIT_RETRIES);
    end

    disp('EBI UDP server started... Press Ctrl+C to stop.');

    try
        while true
            tic;

            try
                complexData = ElikoSample();

                if DEBUG && mod(SEQ_NUM, 10) == 0
                    disp(['Sample ', num2str(SEQ_NUM), ' raw complex data (no scaling):']);
                    disp(complexData);
                end

                if isempty(complexData) || ~ismatrix(complexData) || isreal(complexData)
                    error('Received invalid data format');
                end
            catch ME
                fprintf('Sampling error: %s\n', ME.message);
                try
                    PicometerControl('Disconnect');
                    pause(0.5);
                    ElikoReconnect();
                    pause(0.5);
                    complexData = ElikoSample();
                    disp('Device reconnected successfully.');
                    if DEBUG
                        disp('Post-reconnection sample:');
                        disp(complexData);
                    end
                catch ME
                    error('Reconnection failed: %s', ME.message);
                end
            end

            % 不缩放，保留原始数据
            normalizedData = complexData;

            firstRow = normalizedData(1, :);

            % 防止不足15个元素，补零
            if length(firstRow) < 15
                firstRow(end+1:15) = complex(0, 0);
            end

            realParts = real(firstRow(1:15));
            imagParts = imag(firstRow(1:15));

            finalVector = [realParts, imagParts];

            % 格式化成字符串
            sendStr = sprintf('%.6f ', finalVector);
            sendStr = strtrim(sendStr);

            % 构造消息
            timestamp_sec = posixtime(datetime('now'));
            message = sprintf('%d;%.6f;%s', SEQ_NUM, timestamp_sec, sendStr);

            % 发送 UDP 消息
            write(u, uint8(message), "string", UDP_IP, UDP_PORT);

            if DEBUG && mod(SEQ_NUM, 10) == 0
                disp(['Sent UDP message (SEQ=', num2str(SEQ_NUM), '):']);
                disp(message);
            end

            % 写入日志
            fprintf(fid, '%d;%.6f;%s\n', SEQ_NUM, timestamp_sec, sendStr);

            % 控制频率
            SEQ_NUM = SEQ_NUM + 1;
            elapsed = toc;
            pause(max(0, DELAY - elapsed));
        end
    catch ME
        disp('Interrupted. Stopping...');
        disp(getReport(ME));
    end

    try
        disp('Closing device connection...');
        PicometerControl('Disconnect');
        disp('Device disconnected successfully.');
    catch ME
        warning('Error while disconnecting device: %s', ME.message);
    end

    fclose(fid);
    clear u;
    disp('Server stopped.');
end
