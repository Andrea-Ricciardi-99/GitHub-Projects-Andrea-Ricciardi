function imposta_lingua_menu(handles)

if handles.lingua_it
    set(handles.MENU_file_load,'Label','Carica struttura');
    set(handles.MENU_file_chiudi,'Label','Chiudi');
    set(handles.MENU_file_default,'Label','Directory di default');
    set(handles.MENU_grafica,'Label','Grafica');
    set(handles.MENU_grafica_strutt,'Label','Struttura indeformata');
    set(handles.MENU_grafica_showfig,'Label','Mostra le figure');
    set(handles.MENU_grafica_opzioni,'Label','Opzioni');
    set(handles.MENU_grafica_struttonly,'Label','Disegna solo la struttura');
    set(handles.MENU_grafica_strutt_nodes,'Label','Disegna la struttura e il numero dei nodi');
    set(handles.MENU_grafica_axisequal,'Label','Opzione axis equal');
    set(handles.MENU_grafica_axisauto,'Label','Opzione axis auto');
    set(handles.MENU_grafica_dimnodi,'Label',['Dimensioni nr. dei nodi (attuale: ' num2str(handles.dimtxt) ')']);
    set(handles.MENU_statica,'Label','Analisi statiche');
    set(handles.MENU_statica_gravita,'Label','Gravità');
    set(handles.MENU_statica_forze,'Label','Forze concentrate');
    set(handles.MENU_freq,'Label','Analisi nel dominio delle frequenze');
    set(handles.MENU_freq_modi,'Label','Frequenze naturali e modi di vibrare');
    set(handles.MENU_freq_FRF,'Label','Risposta in frequenza');
    set(handles.MENU_esporta,'Label','Esporta');
    set(handles.MENU_esporta_MKR,'Label','Matrici strutturali');
    set(handles.MENU_esporta_modi,'Label','Frequenze proprie e modi di vibrare');
    set(handles.MENU_esporta_FRF,'Label','Risposta in frequenza');
    set(handles.MENU_freq_FRF_x,'Label','Spostamento');
    set(handles.MENU_freq_FRF_xpp,'Label','Accelerazione');
    set(handles.MENU_utilities_lingua,'Label','Seleziona lingua');
    set(handles.MENU_utilities_lingua_IT,'Label','Italiano');
    set(handles.MENU_utilities_lingua_EN,'Label','Inglese');
    set(handles.MENU_utilities_clc,'Label','Pulisci la finestra di comando');
    set(handles.MENU_utilities_closefig,'Label','Chiudi le figure');
else
    set(handles.MENU_file_load,'Label','Load structure');
    set(handles.MENU_file_chiudi,'Label','Close');
    set(handles.MENU_file_default,'Label','Default directory');
    set(handles.MENU_grafica,'Label','Graphic');
    set(handles.MENU_grafica_strutt,'Label','Undeformed structure');
    set(handles.MENU_grafica_showfig,'Label','Show figures');
    set(handles.MENU_grafica_opzioni,'Label','Options');
    set(handles.MENU_grafica_struttonly,'Label','Plot only the structure');
    set(handles.MENU_grafica_strutt_nodes,'Label','Plot the structure and the nodes number');
    set(handles.MENU_grafica_axisequal,'Label','Option axis equal');
    set(handles.MENU_grafica_axisauto,'Label','Option axis auto');
    set(handles.MENU_grafica_dimnodi,'Label',['Dimensions of the nodes number (actual: ' num2str(handles.dimtxt) ')']);
    set(handles.MENU_statica,'Label','Static analysis');
    set(handles.MENU_statica_gravita,'Label','Structure weight');
    set(handles.MENU_statica_forze,'Label','Concentrated loads');
    set(handles.MENU_freq,'Label','Frequency domain analysis');
    set(handles.MENU_freq_modi,'Label','Natural frequencies and modes of vibration');
    set(handles.MENU_freq_FRF,'Label','Frequency response');
    set(handles.MENU_esporta,'Label','Export');
    set(handles.MENU_esporta_MKR,'Label','Structural matrices');
    set(handles.MENU_esporta_modi,'Label','Natural frequencies and modes of vibration');
    set(handles.MENU_esporta_FRF,'Label','Frequency response');
    set(handles.MENU_freq_FRF_x,'Label','Displacement');
    set(handles.MENU_freq_FRF_xpp,'Label','Acceleration');
    set(handles.MENU_utilities_lingua,'Label','Select language');
    set(handles.MENU_utilities_lingua_IT,'Label','Italian');
    set(handles.MENU_utilities_lingua_EN,'Label','English');
    set(handles.MENU_utilities_clc,'Label','Clear command window');
    set(handles.MENU_utilities_closefig,'Label','Close figures');
end


