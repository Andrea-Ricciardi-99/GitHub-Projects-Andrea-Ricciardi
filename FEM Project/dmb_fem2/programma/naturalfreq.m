function [freq,phi,fscala,vett_str,nfig_end]=naturalfreq(handles,nfig);

ngdl=handles.ngdl;
M=handles.M;
K=handles.K;
size=handles.size;
idb=handles.idb;
incid_masse=handles.incid_masse;
nm_conc=handles.nm_conc;
xy=handles.xy;
lingua_it=handles.lingua_it;
beams=handles.beams;
funi=handles.funi;
Tbeams=handles.Tbeams;


% Calculation of natural frequencies and modes of vibration
[v,d]=eig(inv(M)*K);
% natural frequencies and modes are extracted in ascending order of
% frequency; modes are stored in the modal matrix phi
[omega,I]=sort(sqrt(diag(d)));
freq=omega/2/pi;
for mode=1:ngdl
    phi(:,mode)=v(:,I(mode));
    fscala(mode)=0.2*size/max(abs(phi(:,mode)));
end

% Results are displayed
mode=1 ;
nfig_end=nfig;
vett_str=[];
while mode <= ngdl ,
    
    nfig_end=nfig_end+1;

    % Sottomenu' per scegliere il modo da visualizzare.
    figure(nfig_end);
    diseg(phi(:,mode),fscala(mode),nm_conc,incid_masse,idb,xy,beams,Tbeams,funi);
    
    if lingua_it
        str_title=['Modo n. ',int2str(mode),' - ',num2str(freq(mode)),' [Hz]'];
        str_print1=' --- Modo n.                 : %i';
        str_print2=' --- Frequenza naturale  [Hz]: %e';
        str_menu1='Visualizza i modi di vibrare';
        str_menu2='Modo successivo';
        str_menu3='Modo precedente';
        str_menu4='Cambia il fattore di scala';
        str_menu5='Esci';
        lb1='Assegna il nuovo valore del fattore di scala ';
    else
        str_title=['Mode n. ',int2str(mode),' - ',num2str(freq(mode)),' [Hz]'];
        str_print1=' --- Mode n.                : %i';
        str_print2=' --- Natural Frequency  [Hz]: %e';
        str_menu1='Display of Modes of Vibration';
        str_menu2='Next Mode';
        str_menu3='Previous Mode';
        str_menu4='Change scale factor';
        str_menu5='Exit';
        lb1='Input new value for the scale factor ';
    end

    vett_str=strvcat(vett_str,str_title);
    title(str_title)
    print_stringa(sprintf(str_print1,mode),handles);
    print_stringa(sprintf(str_print2,freq(mode)),handles);
    vise(idb,ngdl,phi(:,mode),handles) ;
    ico1 = menu(str_menu1,str_menu2,str_menu3,str_menu4,str_menu5);
    if ico1==1
        mode=mode+1;
    elseif ico1 == 2
        mode=max([1 mode-1]);
    elseif ico1 == 3
        def_1=num2str(fscala(mode)); 
        dlgTitle='';
        prompt={lb1};
        default={def_1};   
        lineNo=1;
        answer=inputdlg(prompt,dlgTitle,lineNo,default);
        if isempty(answer)
            fscala(mode)=0;
        else
            fscala(mode)=str2double(answer(1)); 
        end 
        close (nfig_end)
    elseif ico1 == 4
        mode=ngdl+1;
    end
end % end of while cicle

