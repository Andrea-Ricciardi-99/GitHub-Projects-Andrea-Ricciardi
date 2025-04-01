function [M,R,K] = assem (incid_masse,masse,nm_conc,incid_molle,molle,nk_conc,idb,nbeam,...
    alfa,beta,nstring,nbeamT,beams,Tbeams,funi,alfa_molle);

% verifiche di consistenza dati in ingresso
%[n_el,nc]=size(incidenze);
% if nc ~= 6
%     disp('Errore: numero colonne matrici incidenze diverso da 6')
%     return
% end

n_gdl=max(max(idb));
% if n_gdl > 50
%     sprintf('Errore: numero g.d.l. > 50 non permesso')
%     return
% end

% assemblaggio marici M e K
M=zeros(n_gdl,n_gdl);
K=zeros(n_gdl,n_gdl);
R=zeros(n_gdl,n_gdl);

%assemblaggio elementi finiti trave
for k=1:nbeam
    [mG,kG] = el_tra (beams.l(k),beams.m(k),beams.EA(k),beams.EJ(k),beams.gamma(k));
    if beams.damp(k)
        alfa_k=beams.mtx_alfa(k);
        beta_k=beams.mtx_beta(k);
    else
        alfa_k=alfa;
        beta_k=beta;
    end
    rG=alfa_k*mG+beta_k*kG;
    for iri=1:6
        for ico=1:6
            i1=beams.incidenze(k,iri);
            i2=beams.incidenze(k,ico);
            M(i1,i2)=M(i1,i2)+mG(iri,ico);
            K(i1,i2)=K(i1,i2)+kG(iri,ico);
            R(i1,i2)=R(i1,i2)+rG(iri,ico);
        end
    end
end

% Stefano 21/05/2007 - Aggiunta elementi finiti fune
%assemblaggio elementi finiti fune
for k=1:nstring
    [mG,kG] = el_fun (funi.l(k),funi.m(k),funi.EA(k),funi.T(k),funi.gamma(k));
    rG=alfa*mG+beta*kG;
    for iri=1:4
        for ico=1:4
            i1=funi.incidenze(k,iri);
            i2=funi.incidenze(k,ico);
            M(i1,i2)=M(i1,i2)+mG(iri,ico);
            K(i1,i2)=K(i1,i2)+kG(iri,ico);
            R(i1,i2)=R(i1,i2)+rG(iri,ico);
        end
    end
end

%Andrea 22/04/08 assemblaggio elementi finiti trave tesata
for k=1:nbeamT
    [mG,kG] = el_trates (Tbeams.l(k),Tbeams.m(k),Tbeams.EA(k),Tbeams.EJ(k),Tbeams.T(k),Tbeams.gamma(k));
    if Tbeams.damp(k)
        alfa_k=Tbeams.mtx_alfa(k);
        beta_k=Tbeams.mtx_beta(k);
    else
        alfa_k=alfa;
        beta_k=beta;
    end
    rG=alfa_k*mG+beta_k*kG;
    for iri=1:6
        for ico=1:6
            i1=Tbeams.incidenze(k,iri);
            i2=Tbeams.incidenze(k,ico);
            M(i1,i2)=M(i1,i2)+mG(iri,ico);
            K(i1,i2)=K(i1,i2)+kG(iri,ico);
            R(i1,i2)=R(i1,i2)+rG(iri,ico);
        end
    end
end

% assemblaggio masse concentrate

for im=1:nm_conc
    nodo=incid_masse(im);
    igdl_x=idb(nodo,1);
    igdl_y=idb(nodo,2);
    igdl_t=idb(nodo,3);
    M(igdl_x,igdl_x)=M(igdl_x,igdl_x)+masse(im,1);
    M(igdl_y,igdl_y)=M(igdl_y,igdl_y)+masse(im,1);
    M(igdl_t,igdl_t)=M(igdl_t,igdl_t)+masse(im,2);
end

% assemblaggio molle concentrate

for im=1:nk_conc
    nodoi=incid_molle(im,1);
    nodoj=incid_molle(im,2);
    kx=molle(im,1);
    ky=molle(im,2);
    kt=molle(im,3);
    rx=molle(im,4);
    ry=molle(im,5);
    rt=molle(im,6);
    alfa=alfa_molle(im);
    igdl_xi=idb(nodoi,1);
    igdl_yi=idb(nodoi,2);
    igdl_ti=idb(nodoi,3);
    if nodoj>0
        igdl_xj=idb(nodoj,1);
        igdl_yj=idb(nodoj,2);
        igdl_tj=idb(nodoj,3);
    else
        igdl_xj=0;
        igdl_yj=0;
        igdl_tj=0;
    end
    [kglo,rglo]=loc2glo_molle(alfa,kx,ky,kt,rx,ry,rt);

    incidenze=[igdl_xi igdl_yi igdl_ti igdl_xj igdl_yj igdl_tj];
    for iri=1:6
        for ico=1:6
            i1=incidenze(iri);
            i2=incidenze(ico);
            if i1>0 & i2>0
                K(i1,i2)=K(i1,i2)+kglo(iri,ico);
                R(i1,i2)=R(i1,i2)+rglo(iri,ico);
            end
        end
    end
end
