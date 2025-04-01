function dismat (m,k)

ngdl=length(m);

if length(m) ~= ngdl
    disp('Errore: matrici M e K non hanno stesso numero di righe')
    return
end

posi_piene=[];
posi_vuote=[];
for iri=1:ngdl
    for ico=1:ngdl
        if m(iri,ico) ~= 0
            posi_piene=[posi_piene ; [ico -iri]];
        else
            posi_vuote=[posi_vuote ; [ico -iri]];
        end
    end
end

figure
plot(posi_piene(:,1),posi_piene(:,2),'r*',posi_vuote(:,1),posi_vuote(:,2),'bo')
axis([0 ngdl+1 -(ngdl+1) 0]);
grid
title('Matrice M');
qq=get(gca,'YTick');
rr=int2str(abs(qq'));
set(gca,'YTickLabel',rr)
set(gca,'XAxisLocation','top')

posi_piene=[];
posi_vuote=[];
for iri=1:ngdl
    for ico=1:ngdl
        if k(iri,ico) ~= 0
            posi_piene=[posi_piene ; [ico -iri]];
        else
            posi_vuote=[posi_vuote ; [ico -iri]];
        end
    end
end

figure
plot(posi_piene(:,1),posi_piene(:,2),'r*',posi_vuote(:,1),posi_vuote(:,2),'bo')
axis([0 ngdl+1 -(ngdl+1) 0]);
grid
title('Matrice K');
qq=get(gca,'YTick');
rr=int2str(abs(qq'));
set(gca,'YTickLabel',rr)
set(gca,'XAxisLocation','top')