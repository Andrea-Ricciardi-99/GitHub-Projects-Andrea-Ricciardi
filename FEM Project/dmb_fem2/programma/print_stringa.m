function print_stringa(stringa,handles);

if ~isempty(handles)
    testo_old=get(handles.LBOX_testo,'String');
    set(handles.LBOX_testo,'Value',1,'String',strvcat(testo_old,stringa));
end
disp(stringa)
drawnow
