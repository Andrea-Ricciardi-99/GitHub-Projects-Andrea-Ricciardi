function [fout,iexit,iout]=set_frequency(fmin,fmax,np,df,fre,lingua_it);

if lingua_it
    lb1='Assegna la frequenza [Hz]: ';
    str_msg_inf=['ATTENZIONE: la frequenza impostata è inferiore al campo di frequenza studiato, la deformata è visualizzata a f= ' num2str(fmin) ' Hz'];
    str_msg_sup=['ATTENZIONE: la frequenza impostata è superiore al campo di frequenza studiato, la deformata è visualizzata a f= ' num2str(fmax) ' Hz'];
else
    lb1='Assign frequency of motion [Hz]: ';
    str_msg_inf=['WARNING: Frequency of motion below the studied frequency range, motion displayed at f= ' num2str(fmin) ' Hz'];
    str_msg_sup=['WARNING: Frequency of motion above the studied frequency range, motion displayed at f= ' num2str(fmax) ' Hz'];
end

def_1=''; 
prompt={lb1};
default={def_1};   
dlgTitle=' ';
lineNo=1;
answer=inputdlg(prompt,dlgTitle,lineNo,default,'on');
if isempty(answer)
    iexit=true;
    fout=[];
else
    fout=str2double(answer(1)); 
    iexit=false;
    
    if fout < fmin
        h=msgbox(str_msg_inf);
        waitfor(h);
        iout=1;
    elseif fout > fmax
        h=msgbox(str_msg_sup);
        waitfor(h);
        iout=np;
    else
        iout=round((fout-fmin)/df)+1;
        fre(iout)
    end
end 
