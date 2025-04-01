function [gamma_txt]=direzione_txt(nr_nodo,beams,Tbeams,funi);

%calcola la direzione ottimale per inserire il numero del nodo:
% 1. trova le travi, funi e travi tesate che confluiscono al nodo
% 2. calcola gli angoli relativi
% 3. scrive sulla bisettrice dell'angolo relativo max. 
% 4. controlla se è meglio l'angolo calcolato o quello incrementato di pi

if isempty(beams.incid)
    travi1=[];
    travi2=[];
else
    travi1=find(nr_nodo==beams.incid(:,1));
    travi2=find(nr_nodo==beams.incid(:,2));
end
if isempty(funi.incid)
    funi1=[];
    funi2=[];
else
    funi1=find(nr_nodo==funi.incid(:,1));
    funi2=find(nr_nodo==funi.incid(:,2));
end
if isempty(Tbeams.incid)
    traviT1=[];
    traviT2=[];
else
    traviT1=find(nr_nodo==Tbeams.incid(:,1));
    traviT2=find(nr_nodo==Tbeams.incid(:,2));
end

gamma1=[beams.gamma(travi1) funi.gamma(funi1) Tbeams.gamma(traviT1)];        
gamma2=[beams.gamma(travi2) funi.gamma(funi2) Tbeams.gamma(traviT2)]-pi;            %beams.gamma è calcolato non dal nodo considerato
gamma_tmp=[gamma1 gamma2];
if isempty(gamma_tmp)               %nodo connesso solo a molle
    gamma_txt=[];
    return
else
    gamma_tmp=sort(gamma_tmp);
    gamma_tmp=[gamma_tmp gamma_tmp(1)];     %aggiunge il primo valore per calcolare automaticamente tutti i possibili delta_gamma
    delta_gamma=diff(gamma_tmp);
    delta_gamma(end)=delta_gamma(end)+2*pi;     %correzione ultima differenza per prendere l'angolo complementare all'angolo giro
    [gmax imax]=max(abs(delta_gamma));
    gamma_txt=mean(gamma_tmp(imax:imax+1));
    
    diff1=abs(gamma_txt-[gamma1 gamma2]);
    diff2=abs((gamma_txt+pi)-[gamma1 gamma2]);
    if min(diff1)<min(diff2)
        gamma_txt=gamma_txt+pi;
    end
end
