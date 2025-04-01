function [node,idir,iexit]=assign_FRFout_data(nnod,lingua_it);

if lingua_it
    lb1='Nodo da plottare: ';
    lb2='Componente di spostamento da plottare 1=x, 2=y, 3=theta: ';
else
    lb1='Output node: ';
    lb2='Output displacement component 1=x, 2=y, 3=theta: ';
end

leggi_dati=true;
while leggi_dati
    def_1=''; 
    def_2=''; 
    prompt={lb1,lb2};
    default={def_1,def_2};   
    dlgTitle=' ';
    lineNo=1;
    answer=inputdlg(prompt,dlgTitle,lineNo,default);
    if isempty(answer)
        iexit=true;
    else
        iexit=false;
        node=str2double(answer(1)); 
        idir=str2double(answer(2)); 
        if node>=1 & node<=nnod & idir>=1 & idir<=3 
            leggi_dati=false;
        end
    end 
end
