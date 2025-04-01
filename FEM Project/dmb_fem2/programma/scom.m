function line=scom(fid)

line=fgetl(fid);
while(line(1:1)=='!')
    if feof(fid)
        error('Impossibile individuare riga non commentata')
    end
    line=fgetl(fid);
end 

