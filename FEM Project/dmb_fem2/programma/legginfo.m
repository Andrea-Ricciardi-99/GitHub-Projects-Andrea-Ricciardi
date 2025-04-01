function f=legginfo(nome,handles)
eval(['fid=fopen(''',nome,''',''r'');']);

cont='y';
while(cont~='n')
line=fgetl(fid);
if(line(1:5)=='#page')
print_stringa(' ',handles);
print_stringa('Premere Invio per continuare ',handles);
pause
elseif(line(1:4)=='#END')
cont='n';
else
print_stringa(line,handles);
end
end
fclose(fid);
f=0.;
