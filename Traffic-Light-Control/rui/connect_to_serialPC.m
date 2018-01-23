function serialPC = connect_to_serialPC(comNum,BaudRate)
if exist('serialPC','var') == 0
    %initialize serialPC
    try
        fprintf('\n Open serial PC terminal (%s - change if this is not the case).\n\tThis may take 1-2 minutes...\n',comNum);
        serialPC=serial(comNum,'BaudRate', BaudRate, 'DataBits',8, 'StopBits',1, 'Timeout',1);    %setting up serial communication with #1
        fopen(serialPC);
        fprintf('serialPC opened\n');
    catch
        fprintf('\nERROR - serialPC Communication link not openned.\n');
    end
end
end