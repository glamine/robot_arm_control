function initCommunication(port,baudRate)
    global serialObj
    serialObj = serial(port);
    set(serialObj,'BaudRate',baudRate);
    fopen(serialObj);
end

