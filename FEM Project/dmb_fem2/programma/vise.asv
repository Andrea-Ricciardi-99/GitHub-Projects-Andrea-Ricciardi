function vise(idb,ngdl,spo,handles)

[nnod,p]=size(idb);
spo_n=zeros(nnod,3) ;
if handles.lingua_it
    str_display=' --- Spostamenti nodali : ';
    str_display=strvcat(str_display,' ','  nodo n.            x              y              theta');
else
    str_display=' --- Nodal displacements : ';
    str_display=strvcat(str_display,' ','  node n.            x              y              theta');
end

for (kk=1:nnod) 
    for (kj=1:3) 
        ki=idb(kk,kj) ;
        if (ki <= ngdl ) , spo_n(kk,kj) = spo(ki) ;,end
    end
    str=sprintf('   %i      %e      %e      %e',kk,spo_n(kk,:));
    str_display=strvcat(str_display,str);
end
str_display=strvcat(str_display,' ');


testo_old=get(handles.LBOX_testo,'String');
set(handles.LBOX_testo,'Value',1,'String',strvcat(testo_old,str_display));
disp(str_display)
