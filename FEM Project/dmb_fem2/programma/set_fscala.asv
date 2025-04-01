function set_fscala(fscala,defo,str_title,caso,nm_conc,incid_masse,idb,xy,nfig,lingua_it,beams,Tbeams,funi);

if lingua_it
    str_title=[str_title ' - fattore di scala ' ];
    if caso=='sta'
        str_menu1='Visulizzazione deformata statica';
    else
        str_menu1='Visulizzazione deformata dinamica';
    end
    str_menu2='Cambia il fattore di scala';
    str_menu3='Esci';
    lb1='Assegna il nuovo valore del fattore di scala ';
else
    str_title=[str_title ' - scale factor ' ];
    if caso=='sta'
        str_menu1='Display of static deflection';
    else
        str_menu1='Display of dynamic deflection';
    end
    str_menu2='Change scale factor';
    str_menu3='Exit';
    lb1='Input new value for the scale factor ';
end


while fscala ~= 0
    figure(nfig);
    diseg(defo,fscala,nm_conc,incid_masse,idb,xy,beams,Tbeams,funi);
    title([str_title  num2str(fscala)])
    
    ico1 = menu(str_menu1,str_menu2,str_menu3);
    if ico1==1
        def_1=num2str(fscala); 
        dlgTitle='';
        prompt={lb1};
        default={def_1};   
        lineNo=1;
        answer=inputdlg(prompt,dlgTitle,lineNo,default);
        if isempty(answer)==true
            fscala=0;
        else
            fscala=str2double(answer(1)); 
        end 
        close (nfig)
    else
        fscala=0;
    end
end
