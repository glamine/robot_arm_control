function transmitData(data)

    global serialObj
    
	Stringtot = strcat(num2str(data(1)),',',num2str(data(2)),',',num2str(data(3)),',',num2str(data(4)),',',num2str(data(5)),'\n');
	%Stringtot = strcat(num2str(data(1)),',',num2str(data(2)));
	
    %fprintf(serialObj,'%c',Stringtot);
	fprintf(serialObj,Stringtot);%,'async');
    %fprintf(serialObj,'cmd');

	%useful functions : 
	%fprintf
  

end

