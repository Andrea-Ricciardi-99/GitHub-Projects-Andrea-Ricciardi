function [nf,iexit]=assign_nf(lingua_it);

if lingua_it
    lb1='Numero delle componenti delle forze: ';
    dlgTitle='Assegna il numero delle componenti delle forze agenti sulla struttura';
else
    lb1='Number of forces components: ';
    dlgTitle='Assign number of forces components acting on the structure';
end

def_1=''; 
prompt={lb1};
default={def_1};   
lineNo=1;
answer=inputdlg(prompt,dlgTitle,lineNo,default,'on');
if isempty(answer)
    iexit=true;
    nf=[];
else
    nf=str2double(answer(1)); 
    iexit=false;
end 
