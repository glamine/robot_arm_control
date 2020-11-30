function initSerial(port, baudRate)

    global serialObj
    serialObj = serial(port);
    set(serialObj,'BaudRate',baudRate);
    fopen(serialObj);

end

function transmitData(data) %data = 1?

    global serialObj
    
    fprintf(serialObj,'%c',data);
    fprintf(serialObj,'cmd');
%useful functions : 
% fprintf
  

end

function [data] = readData()

    global serialObj

    data = fscanf(serialObj);

end

function status = handleSerial(instruction)

    global serialObj
    
    if(strcmp(instruction,'grip1Status'))
        fprintf(serialObj,'%c',1);
    elseif(strcmp(instruction,'prox1Sensor'))
        fprintf(serialObj,'%c',2);
    end
    status = fscanf(serialObj);
    
end

function endSerial()

    global serialObj
    fclose(serialObj);
    delete(serialObj);
    clear serialObj;

end
