function [force,idir,vf,iexit]=assign_forces_data_distrib(idb,ngdl,nf,lingua_it,beams,Tbeams,nbeam,nbeamT);

force=zeros(ngdl,1);
for i=1:nf
    leggi_dati=true;
    while leggi_dati
        if lingua_it
            lb1='Direzione della componente della forza 1=x, 2=y, 3=theta: ';
            lb2='Ampiezza della forza ([N]): ';
            dlgTitle=['Componente di forza n. ' int2str(i)];
        else
            lb1='Forced displacement component 1=x, 2=y, 3=theta: ';
            lb2='Force amplitude ([N]): ';
            dlgTitle=['Force component n. ' int2str(i)];
        end
        def_1=''; 
        def_2=''; 
        prompt={lb1,lb2};
        default={def_1,def_2};   
        lineNo=1;
        answer=inputdlg(prompt,dlgTitle,lineNo,default,'on');
        if isempty(answer)
            iexit=true;
            idir=[]; 
            vf=[]; 
            return
        else
            iexit=false;
            idir=str2double(answer(1)); 
            vf=str2double(answer(2)); 
            if idir>=1 & idir<=3 
                leggi_dati=false;
            end
        end 
    end    
    
    for tr=1:nbeam
        nodoi=beams.incid(tr,1);
        nodoj=beams.incid(tr,2);
        gdyi=idb(nodoi,2);
        gdthi=idb(nodoi,3);
        gdyj=idb(nodo,j2);
        gdthj=idb(nodoj,3);
        elle=beams.l(tr);
        force(gdyi)=force(gdyi)+vf*elle/2;
        force(gdthi)=force(gdthi)+vf*elle^2/12;
        force(gdyj)=force(gdyj)+vf*elle/2;
        force(gdthj)=force(gdthj)-vf*elle^2/12;
    end
    for tr=1:nbeamT
        nodoi=Tbeams.incid(tr,1);
        nodoj=Tbeams.incid(tr,2);
        gdyi=idb(nodoi,2);
        gdthi=idb(nodoi,3);
        gdyj=idb(nodoj,2);
        gdthj=idb(nodoj,3);
        elle=Tbeams.l(tr);
        if gdyi<=ngdl; force(gdyi)=force(gdyi)+vf*elle/2;end
        if gdthi<=ngdl; force(gdthi)=force(gdthi)+vf*elle^2/12;end
        if gdyj<=ngdl; force(gdyj)=force(gdyj)+vf*elle/2;end
        if gdthj<=ngdl; force(gdthj)=force(gdthj)-vf*elle^2/12;end
    end
end 




