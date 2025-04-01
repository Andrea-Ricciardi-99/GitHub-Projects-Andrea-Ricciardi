function dis_stru(xy,incid_masse,nm_conc,str_title,incid_molle,nk_conc,draw_node_label,beams,Tbeams,funi,axis_equal,dimtxt,nbeam,nbeamT,nstring)

% disegna la struttura indeformata

% keyboard

% passo 1: disegno posizioni nodali
plot(xy(:,1),xy(:,2),'ro')
hold on
grid
if axis_equal
    axis equal
else
    axis auto
end
title(str_title)    

if draw_node_label
    ax=axis;
    delta_x=abs(ax(2)-ax(1));
    delta_y=abs(ax(4)-ax(3));
%    dist=max(delta_x,delta_y)/30;
    dist=max(delta_x,delta_y)/30;
    for nr_nodo=1:length(xy);
        [gamma_txt]=direzione_txt(nr_nodo,beams,Tbeams,funi);
        if ~isempty(gamma_txt)
%             x=xy(nr_nodo,1)+dist*cos(gamma_txt);
%             y=xy(nr_nodo,2)+dist*sin(gamma_txt);
            x=xy(nr_nodo,1)+delta_x/30*cos(gamma_txt);
            y=xy(nr_nodo,2)+delta_y/30*sin(gamma_txt);
            text(x,y,num2str(nr_nodo),'FontWeight','bold','FontSize',dimtxt);
        end
    end
end

% passo 2: disegno le travi
for i=1:nbeam
    xin=beams.posiz(i,1);
    yin=beams.posiz(i,2);
    xfi=beams.posiz(i,1)+beams.l(i)*cos(beams.gamma(i));
    yfi=beams.posiz(i,2)+beams.l(i)*sin(beams.gamma(i));
    plot([xin xfi],[yin yfi],'b','LineWidth',2)
end

% Stefano 21/05/2007: Introduzione elementi fune tesata
% passo 2b: disegno le funi tesate
for i=1:nstring
    xin=funi.posiz(i,1);
    yin=funi.posiz(i,2);
    xfi=funi.posiz(i,1)+funi.l(i)*cos(funi.gamma(i));
    yfi=funi.posiz(i,2)+funi.l(i)*sin(funi.gamma(i));
    plot([xin xfi],[yin yfi],'g','LineWidth',2)
end

% Andrea 22/04/08: disegno le travi tesate
for i=1:nbeamT
    xin=Tbeams.posiz(i,1);
    yin=Tbeams.posiz(i,2);
    xfi=Tbeams.posiz(i,1)+Tbeams.l(i)*cos(Tbeams.gamma(i));
    yfi=Tbeams.posiz(i,2)+Tbeams.l(i)*sin(Tbeams.gamma(i));
    plot([xin xfi],[yin yfi],'r','LineWidth',2)
end

% passo 3: disegno le masse concentrate
for im=1:nm_conc
    nodo=incid_masse(im);
    xin=xy(nodo,1);
    yin=xy(nodo,2);
    plot(xin,yin,'m.','MarkerSize',36)
end

% passo 4: disegno le molle concentrate
for im=1:nk_conc
    nodo=incid_molle(im,:);
    xin=xy(nodo(1),1);
    yin=xy(nodo(1),2);
    if nodo(2)==0
        plot(xin,yin,'k*','MarkerSize',15,'LineWidth',3)
    else
        xfi=xy(nodo(2),1);
        yfi=xy(nodo(2),2);
        plot([xin xfi],[yin yfi],'k','LineWidth',3)
    end
end

