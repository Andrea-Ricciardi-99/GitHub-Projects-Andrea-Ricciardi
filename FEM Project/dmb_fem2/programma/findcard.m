function leggi_card=findcard(fid,card,facoltativa);
% Cerca all'interno del file specificato da fid la stringa 'stringa' e posiziona il puntatore di lettura file alla riga successiva.
% Restituisce 1 se la stringa e' stata trovata, 0 altrimenti.

leggi_card=false;
frewind(fid);
riga=fgets(fid);
while ~feof(fid)
    if ~isempty(findstr(riga,card))
        leggi_card=true;
        return
    end
    riga=fgets(fid);
end

if ~facoltativa
    error(['Impossibile trovare stringa ' card])
end

