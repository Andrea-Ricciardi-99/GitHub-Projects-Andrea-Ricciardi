function [file_i,xy,nnod,size,idb,ngdl,nbeam,nbeamT,alfa,beta,beams,Tbeams,funi,...
        incid_masse,masse,nm_conc,incid_molle,molle,nk_conc,pathinput,error,...
        nstring,alfa_molle]=loadstructure(handles,default_directory_prog)

% Caricamento file input DA modificare con uigetfile
error=false;
[xy,nnod,size,idb,ngdl,nbeam,nbeamT,alfa,beta,beams,Tbeams,funi,incid_masse,masse,nm_conc,incid_molle,molle,nk_conc,alfa_molle]=azzera;

if ~isempty(handles)
    if handles.lingua_it
        str_fileinp='Nome file *.inp di input della struttura';
        str_err_nodi='Errore: nodi non numerati in ordine progressivo';
        str_print_nodi='Numero nodi struttura ';
        str_print_gdl='Numero g.d.l. struttura ';
        str_print_travi='Numero elementi trave ';
        str_print_traviT='Numero elementi trave tesata';
        str_print_funi='Numero elementi fune ';
        str_print_masse='Numero masse concentrate ';
        str_print_molle='Numero molle concentrate ';
        str_print_mtot='Massa totale [kg] ';
    else
        str_fileinp='Select the file name *.inp of the structure';
        str_err_nodi='Error: nodes number is not progressive';
        str_print_nodi='Total nodes number ';
        str_print_gdl='Number of d.o.f. ';
        str_print_travi='Number of beam elements ';
        str_print_traviT='Number of tensile beam elements ';
        str_print_funi='Number of string elements ';
        str_print_masse='Number of concentrated masses ';
        str_print_molle='Number of concentrated springs ';
        str_print_mtot='Total mass [kg] ';
    end
else
    str_fileinp='Nome file *.inp di input della struttura';
    str_err_nodi='Errore: nodi non numerati in ordine progressivo';
    str_print_nodi='Numero nodi struttura ';
    str_print_gdl='Numero g.d.l. struttura ';
    str_print_travi='Numero elementi trave ';
    str_print_traviT='Numero elementi trave tesata';
    str_print_funi='Numero elementi fune ';
    str_print_masse='Numero masse concentrate ';
    str_print_molle='Numero molle concentrate ';
    str_print_mtot='Massa totale [kg] ';
end

main_dir=pwd;
cd(default_directory_prog)

[file_i, pathinput] = uigetfile('*.inp', str_fileinp);

if file_i==0
    cd(main_dir)
    error=true;
    return
else
    cd(pathinput)
    % Apro il file.
    nchr=length(file_i);
    eval(['fid_i=fopen(''',file_i(1:nchr-4),'.inp'',''r'');']);
    cd(main_dir)
end

% Lettura card nodi
facoltativa=false;
leggi_card=findcard(fid_i,'*NODES',facoltativa);
% line=scom(fid_i)
% finewhile=findstr(line,'*ENDNODES')
iconta=0;
while leggi_card
    line=scom(fid_i);
    leggi_card=isempty(findstr(line,'*ENDNODES'));
    if leggi_card
        tmp=sscanf(line,'%d %d %d %d %f %f')';
        iconta=iconta+1;
        if iconta ~=tmp(1)
            msgbox(str_err_nodi)
            error=true;
            return  
        end
        ivinc(iconta,:)=tmp(2:4);
        xy(iconta,:)=tmp(5:6);
    end
end
nnod=iconta;
print_stringa([str_print_nodi,int2str(nnod)],handles);
size=sqrt((max(xy(:,1))-min(xy(:,1)))^2+(max(xy(:,2))-min(xy(:,2)))^2);
% fine lettura card nodi

%keyboard

% costruzione matrice IDB di numerazione gdl nodi
% 1 g.d.l. liberi
igdl=0;
for i=1:nnod
    for j=1:3
        if ivinc(i,j) == 0
            igdl=igdl+1;
            idb(i,j)=igdl;
        end
    end
end
ngdl=igdl;
print_stringa([str_print_gdl,int2str(ngdl)],handles);
% 2 g.d.l. vincolati
for i=1:nnod
    for j=1:3
        if ivinc(i,j) == 1
            igdl=igdl+1;
            idb(i,j)=igdl;
        end
    end
end
% 3 g.d.l. slaves
for i=1:nnod
    for j=1:3
        if ivinc(i,j) < 0
            nodo_master=-ivinc(i,j);
            idb(i,j)=idb(nodo_master,j);
        end
    end
end


%keyboard

% Lettura card travi
Mtot=0;
facoltativa=true;
leggi_card=findcard(fid_i,'*BEAMS',facoltativa);
iconta=0;
while leggi_card
    line=scom(fid_i);
    leggi_card=isempty(findstr(line,'*ENDBEAMS'));
    if leggi_card
        tmp=sscanf(line,'%d %d %d %f %f %f %f %f')';
        iconta=iconta+1;
        beams.incid(iconta,:)=tmp(2:3);
        beams.m(1,iconta)=tmp(4);
        beams.EA(1,iconta)=tmp(5);
        beams.EJ(1,iconta)=tmp(6);
        if length(tmp)==6
            beams.damp(1,iconta)=false;
        else
            beams.damp(1,iconta)=true;
            beams.mtx_alfa(1,iconta)=tmp(7);
            beams.mtx_beta(1,iconta)=tmp(8);
        end
        nodoi=beams.incid(iconta,1);
        nodoj=beams.incid(iconta,2);
        dx=xy(nodoj,1)-xy(nodoi,1);
        dy=xy(nodoj,2)-xy(nodoi,2);
        beams.l(1,iconta)=sqrt(dx^2+dy^2);
        beams.gamma(1,iconta)=atan2(dy,dx);
        beams.incidenze(iconta,:)=[idb(nodoi,:) idb(nodoj,:)];
        beams.posiz(iconta,:)=xy(nodoi,:);
        Mtot=Mtot+beams.m(1,iconta)*beams.l(1,iconta);
    end
end
nbeam=iconta;
print_stringa([str_print_travi,int2str(nbeam)],handles);
% fine lettura card travi

% Stefano 21/5/2007 funi tesate
% Lettura card *STRING
facoltativa=true;
leggi_card=findcard(fid_i,'*STRING',facoltativa);
iconta=0;
while leggi_card
    line=scom(fid_i);
    leggi_card=isempty(findstr(line,'*ENDSTRING'));
    if leggi_card
        tmp=sscanf(line,'%d %d %d %f %f %f %f %f')';
%         keyboard
        iconta=iconta+1;
        funi.incid(iconta,:)=tmp(2:3);
        funi.m(1,iconta)=tmp(4);
        funi.EA(1,iconta)=tmp(5);
        funi.T(1,iconta)=tmp(6);
        nodoi=funi.incid(iconta,1);
        nodoj=funi.incid(iconta,2);
        dx=xy(nodoj,1)-xy(nodoi,1);
        dy=xy(nodoj,2)-xy(nodoi,2);
        funi.l(1,iconta)=sqrt(dx^2+dy^2);
        funi.gamma(1,iconta)=atan2(dy,dx);
        funi.incidenze(iconta,:)=[idb(nodoi,1:2) idb(nodoj,1:2)];
        funi.posiz(iconta,:)=xy(nodoi,:);
        Mtot=Mtot+funi.m(1,iconta)*funi.l(1,iconta);
    end
end
nstring=iconta;
print_stringa([str_print_funi,int2str(nstring)],handles);
% fine lettura card funi

%Andrea 22/04/08 Lettura card travi tesate
facoltativa=true;
leggi_card=findcard(fid_i,'*T.BEAMS',facoltativa);
iconta=0;
while leggi_card
    line=scom(fid_i);
    leggi_card=isempty(findstr(line,'*ENDT.BEAMS'));
    if leggi_card
        tmp=sscanf(line,'%d %d %d %f %f %f %f %f %f')';
        iconta=iconta+1;
        Tbeams.incid(iconta,:)=tmp(2:3);
        Tbeams.m(1,iconta)=tmp(4);
        Tbeams.EA(1,iconta)=tmp(5);
        Tbeams.EJ(1,iconta)=tmp(6);
        Tbeams.T(1,iconta)=tmp(7);
        if length(tmp)==7
            Tbeams.damp(1,iconta)=false;
        else
            Tbeams.damp(1,iconta)=true;
            Tbeams.mtx_alfa(1,iconta)=tmp(8);
            Tbeams.mtx_beta(1,iconta)=tmp(9);
        end
        nodoi=Tbeams.incid(iconta,1);
        nodoj=Tbeams.incid(iconta,2);
        dx=xy(nodoj,1)-xy(nodoi,1);
        dy=xy(nodoj,2)-xy(nodoi,2);
        Tbeams.l(1,iconta)=sqrt(dx^2+dy^2);
        Tbeams.gamma(1,iconta)=atan2(dy,dx);
        Tbeams.incidenze(iconta,:)=[idb(nodoi,:) idb(nodoj,:)];
        Tbeams.posiz(iconta,:)=xy(nodoi,:);
        Mtot=Mtot+Tbeams.m(1,iconta)*Tbeams.l(1,iconta);
    end
end
nbeamT=iconta;
print_stringa([str_print_traviT,int2str(nbeamT)],handles);
% fine lettura card travi tesate

% Lettura card "Damping"
facoltativa=false;
leggi_card=findcard(fid_i,'*DAMPING',facoltativa);
if leggi_card
    line=scom(fid_i);
    tmp=sscanf(line,'%f %f')';
    alfa=tmp(1);
    beta=tmp(2);
end

% Lettura card masse concentrate
facoltativa=true;
leggi_card=findcard(fid_i,'*MASSES',facoltativa);
iconta=0;
while leggi_card
    line=scom(fid_i);
    leggi_card=isempty(findstr(line,'*ENDMASSES'));
    if leggi_card
        tmp=sscanf(line,'%d %d %f %f')';
        iconta=iconta+1;
        incid_masse(1,iconta)=tmp(2);
        masse(iconta,:)=tmp(3:4);
        Mtot=Mtot+masse(iconta,1);
    end
end
nm_conc=iconta;
print_stringa([str_print_masse,int2str(nm_conc)],handles);
% fine lettura card masse

%keyboard

% Lettura card molle concentrate
facoltativa=true;
leggi_card=findcard(fid_i,'*SPRINGS',facoltativa);
iconta=0;
while leggi_card
    line=scom(fid_i);
    leggi_card=isempty(findstr(line,'*ENDSPRINGS'));
    if leggi_card
        tmp=sscanf(line,'%d %d %d %f %f %f %f %f %f %f')';
        iconta=iconta+1;
        incid_molle(iconta,:)=tmp(2:3);
        molle(iconta,:)=tmp(4:9);
        if length(tmp)==9
            alfa_molle(iconta)=0;
        else
            alfa_molle(iconta)=tmp(10)/180*pi;
        end
        
    end
end
nk_conc=iconta;
print_stringa([str_print_molle,int2str(nk_conc)],handles);
print_stringa([str_print_mtot,num2str(Mtot)],handles);
% fine lettura card molle

% Chiudo il file della struttura.
fclose(fid_i) ;





