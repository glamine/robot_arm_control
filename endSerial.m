function endSerial()

    global serialObj
    fclose(serialObj);
    delete(serialObj);
    clear serialObj;

end
