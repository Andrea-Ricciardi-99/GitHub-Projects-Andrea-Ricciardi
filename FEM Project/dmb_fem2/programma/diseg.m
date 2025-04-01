function diseg(modo,fscala,nm_conc,incid_masse,idb,xy,beams,Tbeams,funi)

%keyboard

% verifiche di consistenza dati in ingresso
[n_el,nc]=size(beams.incidenze);
[n_elf,nc]=size(funi.incidenze);
[n_elt,nc]=size(Tbeams.incidenze);

n_gdl=length(modo);

hold on
% ciclo sugli elementi finiti trave
for k=1:n_el
% costruzione vettore spostamenti nodali dell'elemento nel s.d.r. globale
    for iri=1:6
        if beams.incidenze(k,iri) <= n_gdl
        xkG(iri,1)=modo(beams.incidenze(k,iri));
        else
        xkG(iri,1)=0.;
        end
    end
% applicazione fattore di scala
    xkG=fscala*xkG;
% proiezione da globale a locale vettore spostamenti nodali
    lambda = [ cos(beams.gamma(k)) sin(beams.gamma(k)) 0. 
              -sin(beams.gamma(k)) cos(beams.gamma(k)) 0.
	                0.      0.     1. ] ;
    Lambda = [ lambda     zeros(3)
              zeros(3)   lambda      ] ;
    xkL=Lambda*xkG;

% calcolo spostamenti assiali u e trasversali w mediante funzioni di forma
    csi=beams.l(k)*[0:0.05:1];
    fu=zeros(6,length(csi));
    fu(1,:)=1-csi/beams.l(k);
    fu(4,:)=csi/beams.l(k);
    u=(fu'*xkL)';
    fw=zeros(6,length(csi));
    fw(2,:)=2*(csi/beams.l(k)).^3-3*(csi/beams.l(k)).^2+1;
    fw(3,:)=beams.l(k)*((csi/beams.l(k)).^3-2*(csi/beams.l(k)).^2+csi/beams.l(k));
    fw(5,:)=-2*(csi/beams.l(k)).^3+3*(csi/beams.l(k)).^2;
    fw(6,:)=beams.l(k)*((csi/beams.l(k)).^3-(csi/beams.l(k)).^2);
    w=(fw'*xkL)';
% proiezione da s.d.r. locale a globale della deformata dell'elemento
    xyG=lambda(1:2,1:2)'*[u+csi;w];
    undef=lambda(1:2,1:2)'*[csi;zeros(1,length(csi))];
 % disegno traccia elemento deformato e indeformato
    plot(undef(1,:)+beams.posiz(k,1),undef(2,:)+beams.posiz(k,2),'b--','LineWidth',2)
    plot(xyG(1,:)+beams.posiz(k,1),xyG(2,:)+beams.posiz(k,2),'b')
end

% Stefano 21/05/2007 Elementi finiti fune tesata
% ciclo sugli elementi finiti fune tesata
clear xkG
clear fu
clear fw
clear lambda
clear Lambda
for k=1:n_elf
% costruzione vettore spostamenti nodali dell'elemento nel s.d.r. globale
    for iri=1:4
        if funi.incidenze(k,iri) <= n_gdl
        xkG(iri,1)=modo(funi.incidenze(k,iri));
        else
        xkG(iri,1)=0.;
        end
    end
% applicazione fattore di scala
    xkG=fscala*xkG;
% proiezione da globale a locale vettore spostamenti nodali
    lambda = [ cos(funi.gamma(k)) sin(funi.gamma(k))
              -sin(funi.gamma(k)) cos(funi.gamma(k))] ;
    Lambda = [ lambda     zeros(2)
              zeros(2)   lambda      ] ;
    xkL=Lambda*xkG;

% calcolo spostamenti assiali u e trasversali w mediante funzioni di forma
    csi=funi.l(k)*[0:1:1];
    fu=zeros(4,length(csi));
    fu(1,:)=1-csi/funi.l(k);
    fu(3,:)=csi/funi.l(k);
    u=(fu'*xkL)';
    fw=zeros(4,length(csi));
    fw(2,:)=1-csi/funi.l(k);
    fw(4,:)=csi/funi.l(k);
    w=(fw'*xkL)';
% proiezione da s.d.r. locale a globale della deformata dell'elemento
    xyG=lambda(1:2,1:2)'*[u+csi;w];
    undef=lambda(1:2,1:2)'*[csi;zeros(1,length(csi))];
 % disegno traccia elemento deformato e indeformato
    plot(undef(1,:)+funi.posiz(k,1),undef(2,:)+funi.posiz(k,2),'g--','LineWidth',2)
    plot(xyG(1,:)+funi.posiz(k,1),xyG(2,:)+funi.posiz(k,2),'g')
end

%Andrea 24/04/08 ciclo sugli elementi finiti trave tesata
for k=1:n_elt
% costruzione vettore spostamenti nodali dell'elemento nel s.d.r. globale
    for iri=1:6
        if Tbeams.incidenze(k,iri) <= n_gdl
        xkG(iri,1)=modo(Tbeams.incidenze(k,iri));
        else
        xkG(iri,1)=0.;
        end
    end
% applicazione fattore di scala
    xkG=fscala*xkG;
% proiezione da globale a locale vettore spostamenti nodali
    lambda = [ cos(Tbeams.gamma(k)) sin(Tbeams.gamma(k)) 0. 
              -sin(Tbeams.gamma(k)) cos(Tbeams.gamma(k)) 0.
	                0.      0.     1. ] ;
    Lambda = [ lambda     zeros(3)
              zeros(3)   lambda      ] ;
    xkL=Lambda*xkG;

% calcolo spostamenti assiali u e trasversali w mediante funzioni di forma
    csi=Tbeams.l(k)*[0:0.05:1];
    fu=zeros(6,length(csi));
    fu(1,:)=1-csi/Tbeams.l(k);
    fu(4,:)=csi/Tbeams.l(k);
    u=(fu'*xkL)';
    fw=zeros(6,length(csi));
    fw(2,:)=2*(csi/Tbeams.l(k)).^3-3*(csi/Tbeams.l(k)).^2+1;
    fw(3,:)=Tbeams.l(k)*((csi/Tbeams.l(k)).^3-2*(csi/Tbeams.l(k)).^2+csi/Tbeams.l(k));
    fw(5,:)=-2*(csi/Tbeams.l(k)).^3+3*(csi/Tbeams.l(k)).^2;
    fw(6,:)=Tbeams.l(k)*((csi/Tbeams.l(k)).^3-(csi/Tbeams.l(k)).^2);
    w=(fw'*xkL)';
% proiezione da s.d.r. locale a globale della deformata dell'elemento
    xyG=lambda(1:2,1:2)'*[u+csi;w];
    undef=lambda(1:2,1:2)'*[csi;zeros(1,length(csi))];
 % disegno traccia elemento deformato e indeformato
    plot(undef(1,:)+Tbeams.posiz(k,1),undef(2,:)+Tbeams.posiz(k,2),'b--','LineWidth',2)
    plot(xyG(1,:)+Tbeams.posiz(k,1),xyG(2,:)+Tbeams.posiz(k,2),'b')
end

% masse concentrate
for im=1:nm_conc
    nodo=incid_masse(im);
    xin=xy(nodo,1);
    yin=xy(nodo,2);
    igdl_x=idb(nodo,1);
    igdl_y=idb(nodo,2);
    if igdl_x>n_gdl         %gdl vincolato
        xtot=xin;
    else
        xtot=xin+fscala*modo(igdl_x);
    end
    if igdl_y>n_gdl
        ytot=yin;
    else
        ytot=yin+fscala*modo(igdl_y);
    end
    plot(xin,yin,'m.','MarkerSize',36)
    plot(xtot,ytot,'mo','MarkerSize',12)
end

grid on
axis equal

