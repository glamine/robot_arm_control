function [data] = readData()

	global serialObj

    Chain = fscanf(serialObj);
	Chain1 = strsplit(Chain,',');
	data = str2double(Chain1);
	%if(length(data) ~= 3)
	%	data = [0 0 0];
	%end
	%filtering the non-ok signals

end

