function varargout = dmb_fem2(varargin)
% DMB_FEM2 M-file for dmb_fem2.fig
%      DMB_FEM2, by itself, creates a new DMB_FEM2 or raises the existing
%      singleton*.
%
%      H = DMB_FEM2 returns the handle to a new DMB_FEM2 or the handle to
%      the existing singleton*.
%
%      DMB_FEM2('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in DMB_FEM2.M with the given input arguments.
%
%      DMB_FEM2('Property','Value',...) creates a new DMB_FEM2 or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before dmb_fem2_OpeningFunction gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to dmb_fem2_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help dmb_fem2

% Last Modified by GUIDE v2.5 16-Dec-2009 13:19:37

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @dmb_fem2_OpeningFcn, ...
                   'gui_OutputFcn',  @dmb_fem2_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
                   'gui_Callback',   []);
if nargin & isstr(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT


% --- Executes just before dmb_fem2 is made visible.
function dmb_fem2_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to dmb_fem2 (see VARARGIN)

global vett_hfig_close vett_hfig_show nfig nfig_aperte

clc

main_dir=pwd;
vero=1;
falso=0;
leggi_dati_default=exist('dati_default.mat');
if leggi_dati_default==2
    load dati_default.mat
    if isdir(default_directory_prog)==falso
        default_directory_prog=main_dir;
    end
else
    default_directory_prog=main_dir;
    lingua_default=false;
end   

handles.main_dir=main_dir;
handles.default_directory_prog=default_directory_prog;
handles.Versione='1.6';
handles.lingua_it=lingua_default;
vett_hfig_close=[];
nfig=0;
nfig_aperte=0;

set(hObject,'Position', [20 10 175 40]);
if handles.lingua_it
    set(handles.MENU_utilities_lingua_IT,'Checked','on');
    set(handles.MENU_utilities_lingua_EN,'Checked','off');
else
    set(handles.MENU_utilities_lingua_IT,'Checked','off');
    set(handles.MENU_utilities_lingua_EN,'Checked','on');
end

set(handles.MENU_grafica_struttonly,'Checked','on');
set(handles.MENU_grafica_strutt_nodes,'Checked','off');
set(handles.MENU_grafica_axisequal,'Checked','on');
set(handles.MENU_grafica_axisauto,'Checked','off');
handles.draw_node_label=false;
handles.axis_equal=true;
handles.dimtxt=10;

imposta_lingua_menu(handles);
abilita_menu(handles,'off');
set(handles.MENU_freq_FRF_xd,'Enable','off');
set(handles.MENU_file_chiudi,'Enable','off');
set(handles.MENU_esporta_MKR,'Enable','off');
set(handles.MENU_esporta_modi,'Enable','off');
set(handles.MENU_esporta_FRF,'Enable','off');
set(handles.MENU_utilities_closefig,'Enable','off');
set(handles.MENU_grafica_showfig,'Enable','off');
set(handles.frm_dmb_fem2,'Name','DMB_FEM2');

% Choose default command line output for dmb_fem2
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes dmb_fem2 wait for user response (see UIRESUME)
% uiwait(handles.frm_dmb_fem2);


% --- Outputs from this function are returned to the command line.
function varargout = dmb_fem2_OutputFcn(hObject, eventdata, handles)
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --------------------------------------------------------------------
function MENU_file_Callback(hObject, eventdata, handles)
% hObject    handle to MENU_file (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)



% --------------------------------------------------------------------
function MENU_file_exit_Callback(hObject, eventdata, handles)
% hObject    handle to MENU_file_exit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

close(handles.frm_dmb_fem2);


% --------------------------------------------------------------------
function MENU_info_Callback(hObject, eventdata, handles)
% hObject    handle to MENU_info (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes during object creation, after setting all properties.
function LBOX_testo_CreateFcn(hObject, eventdata, handles)
% hObject    handle to LBOX_testo (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: listbox controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc
    set(hObject,'BackgroundColor','white');
else
    set(hObject,'BackgroundColor',get(0,'defaultUicontrolBackgroundColor'));
end


% --- Executes on selection change in LBOX_testo.
function LBOX_testo_Callback(hObject, eventdata, handles)
% hObject    handle to LBOX_testo (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = get(hObject,'String') returns LBOX_testo contents as cell array
%        contents{get(hObject,'Value')} returns selected item from LBOX_testo



% --------------------------------------------------------------------
function MENU_file_default_Callback(hObject, eventdata, handles)
% hObject    handle to MENU_file_default (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

default_directory_prog=handles.default_directory_prog;
main_dir=handles.main_dir;

default_directory_prog_old=default_directory_prog;
if handles.lingua_it
    default_directory_prog=uigetdir(default_directory_prog,'Seleziona la directory di progetto');
else
    default_directory_prog=uigetdir(default_directory_prog,'Select the project directory');
end
cd(main_dir);

if default_directory_prog~=0
    lingua_default=handles.lingua_it;
    save dati_default.mat default_directory_prog lingua_default
    handles.default_directory_prog=default_directory_prog;
    guidata(hObject, handles);
% 
% else
%     default_directory_prog=default_directory_prog_old;
end



% --------------------------------------------------------------------
function MENU_file_load_Callback(hObject, eventdata, handles)
% hObject    handle to MENU_file_load (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

default_directory_prog=handles.default_directory_prog;

[file_i,xy,nnod,size,idb,ngdl,nbeam,nbeamT,alfa,beta,beams,Tbeams,funi,...
        incid_masse,masse,nm_conc,incid_molle,molle,nk_conc,pathinput,error,nstring,alfa_molle]=loadstructure...
    (handles,default_directory_prog);

if ~error
    % costruzione matrice M, K  
    
    [Mtot,Rtot,Ktot] = assem (incid_masse,masse,nm_conc,incid_molle,molle,nk_conc,idb,nbeam,...
        alfa,beta,nstring,nbeamT,beams,Tbeams,funi,alfa_molle);
    M=Mtot(1:ngdl,1:ngdl);
    R=Rtot(1:ngdl,1:ngdl);
    K=Ktot(1:ngdl,1:ngdl);
    
    if handles.lingua_it
        print_stringa('Matrici M, R, K assemblate',handles);
    else
        print_stringa('Matrices M, R, K assembled',handles);
    end

    
    handles.file_i=file_i;
    handles.pathinput=pathinput;
    handles.xy=xy;
    handles.nnod=nnod;
    handles.size=size;
    handles.idb=idb;
    handles.ngdl=ngdl;
    handles.beams=beams;
    handles.Tbeams=Tbeams;
    handles.funi=funi;
    handles.nbeam=nbeam;
    handles.nbeamT=nbeamT;
    handles.nstring=nstring;
    handles.alfa=alfa;
    handles.beta=beta;
    handles.incid_masse=incid_masse;
    handles.masse=masse;
    handles.nm_conc=nm_conc;
    handles.incid_molle=incid_molle;
    handles.molle=molle;
    handles.nk_conc=nk_conc;
    handles.Mtot=Mtot;
    handles.Ktot=Ktot;
    handles.Rtot=Rtot;
    handles.M=M;
    handles.K=K;
    handles.R=R;
    
    abilita_menu(handles,'on');
    set(handles.MENU_file_chiudi,'Enable','on');
    set(handles.MENU_esporta_MKR,'Enable','on');
    set(handles.MENU_file_load,'Enable','off');
    set(handles.frm_dmb_fem2,'Name',file_i);
    
    guidata(hObject, handles);
end

% --------------------------------------------------------------------
function MENU_info_about_Callback(hObject, eventdata, handles)
% hObject    handle to MENU_info_about (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

ftmp=legginfo('ABOUT.TXT',handles);

% --------------------------------------------------------------------
function MENU_info_help_Callback(hObject, eventdata, handles)
% hObject    handle to MENU_info_help (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

ftmp=legginfo('USAGE.TXT',handles);

% --------------------------------------------------------------------
function MENU_utilities_Callback(hObject, eventdata, handles)
% hObject    handle to MENU_utilities (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --------------------------------------------------------------------
function MENU_utilities_clc_Callback(hObject, eventdata, handles)
% hObject    handle to MENU_utilities_clc (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

clc
set(handles.LBOX_testo,'Value',1,'String','');

% --------------------------------------------------------------------
function MENU_utilities_lingua_Callback(hObject, eventdata, handles)
% hObject    handle to MENU_utilities_lingua (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --------------------------------------------------------------------
function MENU_utilities_lingua_IT_Callback(hObject, eventdata, handles)
% hObject    handle to MENU_utilities_lingua_IT (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

set(handles.MENU_utilities_lingua_IT,'Checked','on');
set(handles.MENU_utilities_lingua_EN,'Checked','off');
handles.lingua_it=true;
imposta_lingua_menu(handles)

lingua_default=handles.lingua_it;
default_directory_prog=handles.default_directory_prog;
save dati_default.mat default_directory_prog lingua_default

guidata(hObject, handles);

% --------------------------------------------------------------------
function MENU_utilities_lingua_EN_Callback(hObject, eventdata, handles)
% hObject    handle to MENU_utilities_lingua_EN (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

set(handles.MENU_utilities_lingua_IT,'Checked','off');
set(handles.MENU_utilities_lingua_EN,'Checked','on');
handles.lingua_it=false;
imposta_lingua_menu(handles)

lingua_default=handles.lingua_it;
default_directory_prog=handles.default_directory_prog;
save dati_default.mat default_directory_prog lingua_default

guidata(hObject, handles);

% --------------------------------------------------------------------
function MENU_grafica_Callback(hObject, eventdata, handles)
% hObject    handle to MENU_grafica (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --------------------------------------------------------------------
function MENU_grafica_strutt_Callback(hObject, eventdata, handles)
% hObject    handle to MENU_grafica_strutt (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

global vett_hfig_close vett_hfig_show nfig hfig_closeall nfig_aperte

xy=handles.xy;
incid_masse=handles.incid_masse;
incid_molle=handles.incid_molle;
nm_conc=handles.nm_conc;
nk_conc=handles.nk_conc;
file_i=handles.file_i;
lingua_it=handles.lingua_it;
draw_node_label=handles.draw_node_label;
beams=handles.beams;
funi=handles.funi;
Tbeams=handles.Tbeams;
axis_equal=handles.axis_equal;
dimtxt=handles.dimtxt;
nbeam=handles.nbeam;
nbeamT=handles.nbeamT;
nstring=handles.nstring;

if lingua_it
    str_title='struttura indeformata';  
    str_close='Chiudi tutte le figure';
else
    str_title='undeformed structure';
    str_close='Close all figures';
end

nfig=nfig+1;
nfig_aperte=nfig_aperte+1;

figure(nfig)   
dis_stru(xy,incid_masse,nm_conc,str_title,incid_molle,nk_conc,draw_node_label,beams,Tbeams,funi,axis_equal,dimtxt,nbeam,nbeamT,nstring);

if nfig_aperte==1
    set(handles.MENU_grafica_showfig,'Enable','on')
    set(handles.MENU_utilities_closefig,'Enable','on')
    hfig_closeall = uimenu(handles.MENU_utilities_closefig,'Label',str_close,'Callback',{@MENU_utilities_closefig_closeall_Callback,handles});
end
str=[file_i ' - Fig. ' num2str(nfig) ': ' str_title];
vett_hfig_close(nfig) = uimenu(handles.MENU_utilities_closefig,'Label',str,'Callback',{@MENU_utilities_closefig_vettfig_Callback,handles});
vett_hfig_show(nfig) = uimenu(handles.MENU_grafica_showfig,'Label',str,'Callback',{@MENU_grafica_showfig_vettfig_Callback,handles});

% --------------------------------------------------------------------
function MENU_statica_Callback(hObject, eventdata, handles)
% hObject    handle to MENU_statica (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --------------------------------------------------------------------
function MENU_statica_gravita_Callback(hObject, eventdata, handles)
% hObject    handle to MENU_statica_gravita (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

global vett_hfig_close vett_hfig_show nfig hfig_closeall nfig_aperte


[str_title,nfig,nfig_aperte]=static('g',handles,nfig,nfig_aperte);

%keyboard

if nfig_aperte==1
    set(handles.MENU_grafica_showfig,'Enable','on')
    set(handles.MENU_utilities_closefig,'Enable','on')
    if handles.lingua_it
        str_close='Chiudi tutte le figure';
    else
        str_close='Close all figures';
    end
    hfig_closeall = uimenu(handles.MENU_utilities_closefig,'Label',str_close,'Callback',{@MENU_utilities_closefig_closeall_Callback,handles});
end
str=[handles.file_i ' - Fig. ' num2str(nfig) ': ' str_title];
vett_hfig_close(nfig) = uimenu(handles.MENU_utilities_closefig,'Label',str,'Callback',{@MENU_utilities_closefig_vettfig_Callback,handles});
vett_hfig_show(nfig) = uimenu(handles.MENU_grafica_showfig,'Label',str,'Callback',{@MENU_grafica_showfig_vettfig_Callback,handles});


% --------------------------------------------------------------------
function MENU_statica_forze_Callback(hObject, eventdata, handles)
% hObject    handle to MENU_statica_forze (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

global vett_hfig_close vett_hfig_show nfig hfig_closeall nfig_aperte

[str_title,nfig,nfig_aperte]=static('f',handles,nfig,nfig_aperte)

if nfig_aperte==1
    set(handles.MENU_grafica_showfig,'Enable','on')
    set(handles.MENU_utilities_closefig,'Enable','on')
    if handles.lingua_it
        str_close='Chiudi tutte le figure';
    else
        str_close='Close all figures';
    end
    hfig_closeall = uimenu(handles.MENU_utilities_closefig,'Label',str_close,'Callback',{@MENU_utilities_closefig_closeall_Callback,handles});
end
str=[handles.file_i ' - Fig. ' num2str(nfig) ': ' str_title];
vett_hfig_close(nfig) = uimenu(handles.MENU_utilities_closefig,'Label',str,'Callback',{@MENU_utilities_closefig_vettfig_Callback,handles});
vett_hfig_show(nfig) = uimenu(handles.MENU_grafica_showfig,'Label',str,'Callback',{@MENU_grafica_showfig_vettfig_Callback,handles});

% --------------------------------------------------------------------
function MENU_freq_Callback(hObject, eventdata, handles)
% hObject    handle to MENU_freq (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --------------------------------------------------------------------
function MENU_freq_modi_Callback(hObject, eventdata, handles)
% hObject    handle to MENU_freq_modi (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

global vett_hfig_close vett_hfig_show nfig hfig_closeall nfig_aperte


[freq,phi,fscala,vett_str,nfig_end]=naturalfreq(handles,nfig);

handles.freq=freq;
handles.phi=phi;
handles.fscala_modi=fscala;
guidata(hObject, handles);

set(handles.MENU_esporta_modi,'Enable','on');

for ifig=nfig+1:nfig_end
    nfig_aperte=nfig_aperte+1;
    if nfig_aperte==1
        set(handles.MENU_grafica_showfig,'Enable','on')
        set(handles.MENU_utilities_closefig,'Enable','on')
        if handles.lingua_it
            str_close='Chiudi tutte le figure';
        else
            str_close='Close all figures';
        end
        hfig_closeall = uimenu(handles.MENU_utilities_closefig,'Label',str_close,'Callback',{@MENU_utilities_closefig_closeall_Callback,handles});
    end
    str=[handles.file_i ' - Fig. ' num2str(ifig) ': ' vett_str(ifig-nfig,:)];
    vett_hfig_close(ifig) = uimenu(handles.MENU_utilities_closefig,'Label',str,'Callback',{@MENU_utilities_closefig_vettfig_Callback,handles});
    vett_hfig_show(ifig) = uimenu(handles.MENU_grafica_showfig,'Label',str,'Callback',{@MENU_grafica_showfig_vettfig_Callback,handles});
end
nfig=nfig_end;


% --------------------------------------------------------------------
function MENU_freq_FRF_Callback(hObject, eventdata, handles)
% hObject    handle to MENU_freq_FRF (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --------------------------------------------------------------------
function MENU_esporta_Callback(hObject, eventdata, handles)
% hObject    handle to MENU_esporta (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --------------------------------------------------------------------
function MENU_esporta_MKR_Callback(hObject, eventdata, handles)
% hObject    handle to MENU_esporta_MKR (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


M=handles.Mtot;
K=handles.Ktot;
R=handles.Rtot;
idb=handles.idb;
file_i=handles.file_i;
pathinput=handles.pathinput;

main_dir=pwd;
cd (pathinput);
nchr=length(file_i);
file_out=[file_i(1:nchr-4) '_mkr.mat'];
save(file_out,'M','R','K','idb');
cd (main_dir);

if handles.lingua_it
    msgbox(['Esportate le matrici strutturali nel file ' file_out]);
else
    msgbox(['Structural matrices exported in file ' file_out]);
end

% --------------------------------------------------------------------
function MENU_esporta_modi_Callback(hObject, eventdata, handles)
% hObject    handle to MENU_esporta_modi (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

freq=handles.freq;
phi=handles.phi;
fscala=handles.fscala_modi;
file_i=handles.file_i;
pathinput=handles.pathinput;

main_dir=pwd;
cd (pathinput);
nchr=length(file_i);
file_out=[file_i(1:nchr-4) '_fre.mat'];
save(file_out,'freq','phi','fscala');
cd (main_dir);

if handles.lingua_it
    msgbox(['Esportate le frequenze e i modi nel file ' file_out]);
else
    msgbox(['Frequencies and modes of vibration exported in file ' file_out]);
end

% --------------------------------------------------------------------
function MENU_esporta_FRF_Callback(hObject, eventdata, handles)
% hObject    handle to MENU_esporta_FRF (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


fre=handles.fre;
x=handles.x;
file_i=handles.file_i;
pathinput=handles.pathinput;

main_dir=pwd;
cd (pathinput);
nchr=length(file_i);
file_out=[file_i(1:nchr-4) '_frf.mat'];
save(file_out,'fre','x');
cd (main_dir);

if handles.lingua_it
    msgbox(['Esportate le risposte in frequenza nel file' file_out]);
else
    msgbox(['Frequency response exported in file ' file_out]);
end

% --------------------------------------------------------------------
function MENU_file_chiudi_Callback(hObject, eventdata, handles)
% hObject    handle to MENU_file_chiudi (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

handles = rmfield(handles,'file_i');
handles = rmfield(handles,'pathinput');
handles = rmfield(handles,'xy');
handles = rmfield(handles,'nnod');
handles = rmfield(handles,'size');
handles = rmfield(handles,'idb');
handles = rmfield(handles,'ngdl');
handles = rmfield(handles,'beams');
handles = rmfield(handles,'funi');
handles = rmfield(handles,'Tbeams');
handles = rmfield(handles,'nbeam');
handles = rmfield(handles,'nbeamT');
handles = rmfield(handles,'nstring');
handles = rmfield(handles,'alfa');
handles = rmfield(handles,'beta');
handles = rmfield(handles,'incid_masse');
handles = rmfield(handles,'masse');
handles = rmfield(handles,'nm_conc');
handles = rmfield(handles,'incid_molle');
handles = rmfield(handles,'nk_conc');
handles = rmfield(handles,'molle');
handles = rmfield(handles,'Mtot');
handles = rmfield(handles,'Ktot');
handles = rmfield(handles,'Rtot');
handles = rmfield(handles,'M');
handles = rmfield(handles,'K');
handles = rmfield(handles,'R');
if isfield(handles,'freq')
    handles = rmfield(handles,'freq');
    handles = rmfield(handles,'phi');
    handles = rmfield(handles,'fscala_modi');
end
if isfield(handles,'fre')
    handles = rmfield(handles,'fre');
    handles = rmfield(handles,'x');
end

abilita_menu(handles,'off');
set(handles.MENU_esporta_MKR,'Enable','off');
set(handles.MENU_esporta_modi,'Enable','off');
set(handles.MENU_esporta_FRF,'Enable','off');
set(handles.MENU_file_load,'Enable','on');
set(handles.MENU_file_chiudi,'Enable','off');
set(handles.frm_dmb_fem2,'Name','DMB_FEM2');

clc
set(handles.LBOX_testo,'Value',1,'String','');

guidata(hObject, handles);


% --------------------------------------------------------------------
function MENU_utilities_closefig_Callback(hObject, eventdata, handles)
% hObject    handle to MENU_utilities_closefig (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --------------------------------------------------------------------
function MENU_utilities_closefig_closeall_Callback(hObject, eventdata, handles)

%keyboard
global vett_hfig_close vett_hfig_show nfig hfig_closeall nfig_aperte

for ifig=1:nfig
    if vett_hfig_close(ifig)~=-1
        figure(ifig)
        close
        delete(vett_hfig_close(ifig));
        delete(vett_hfig_show(ifig));
        vett_hfig_close(ifig)=-1;
        vett_hfig_show(ifig)=-1;
    end
end

nfig_aperte=0;
delete(hfig_closeall);
set(handles.MENU_utilities_closefig,'Enable','off')
set(handles.MENU_grafica_showfig,'Enable','off')

% --------------------------------------------------------------------
function MENU_utilities_closefig_vettfig_Callback(hObject, eventdata, handles)

global vett_hfig_close vett_hfig_show nfig nfig_aperte hfig_closeall

%keyboard

ifig=find(hObject==vett_hfig_close);
figure(ifig)
close
delete(hObject);
delete(vett_hfig_show(ifig));

nfig_aperte=nfig_aperte-1;
vett_hfig_close(ifig)=-1;
vett_hfig_show(ifig)=-1;
    
if nfig_aperte==0
    delete(hfig_closeall);
    set(handles.MENU_utilities_closefig,'Enable','off')
    set(handles.MENU_grafica_showfig,'Enable','off')
end


% --------------------------------------------------------------------
function MENU_grafica_showfig_Callback(hObject, eventdata, handles)
% hObject    handle to MENU_grafica_showfig (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --------------------------------------------------------------------
function MENU_grafica_showfig_vettfig_Callback(hObject, eventdata, handles)

global vett_hfig_show   

%keyboard

ifig=find(hObject==vett_hfig_show);
figure(ifig)



% --------------------------------------------------------------------
function MENU_grafica_struttonly_Callback(hObject, eventdata, handles)
% hObject    handle to MENU_grafica_struttonly (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

set(handles.MENU_grafica_struttonly,'Checked','on');
set(handles.MENU_grafica_strutt_nodes,'Checked','off');
handles.draw_node_label=false;
guidata(hObject, handles);

% --------------------------------------------------------------------
function MENU_grafica_strutt_nodes_Callback(hObject, eventdata, handles)
% hObject    handle to MENU_grafica_strutt_nodes (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

set(handles.MENU_grafica_struttonly,'Checked','off');
set(handles.MENU_grafica_strutt_nodes,'Checked','on');
handles.draw_node_label=true;
guidata(hObject, handles);

% --------------------------------------------------------------------
function MENU_grafica_opzioni_Callback(hObject, eventdata, handles)
% hObject    handle to MENU_grafica_opzioni (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --------------------------------------------------------------------
function MENU_freq_FRF_x_Callback(hObject, eventdata, handles)
% hObject    handle to MENU_freq_FRF_x (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

global vett_hfig_close vett_hfig_show nfig hfig_closeall nfig_aperte


[fre,x,xpp,vett_str,nfig_end]=FRF(handles,nfig,'s');

set(handles.MENU_esporta_FRF,'Enable','on');

handles.fre=fre;
handles.x=x;
guidata(hObject, handles);

set(handles.MENU_esporta_modi,'Enable','on');

for ifig=nfig+1:nfig_end
    nfig_aperte=nfig_aperte+1;
    if nfig_aperte==1
        set(handles.MENU_grafica_showfig,'Enable','on')
        set(handles.MENU_utilities_closefig,'Enable','on')
        if handles.lingua_it
            str_close='Chiudi tutte le figure';
        else
            str_close='Close all figures';
        end
        hfig_closeall = uimenu(handles.MENU_utilities_closefig,'Label',str_close,'Callback',{@MENU_utilities_closefig_closeall_Callback,handles});
    end
    str=[handles.file_i ' - Fig. ' num2str(ifig) ': ' vett_str(ifig-nfig,:)];
    vett_hfig_close(ifig) = uimenu(handles.MENU_utilities_closefig,'Label',str,'Callback',{@MENU_utilities_closefig_vettfig_Callback,handles});
    vett_hfig_show(ifig) = uimenu(handles.MENU_grafica_showfig,'Label',str,'Callback',{@MENU_grafica_showfig_vettfig_Callback,handles});
end
nfig=nfig_end;

% --------------------------------------------------------------------
function MENU_freq_FRF_xpp_Callback(hObject, eventdata, handles)
% hObject    handle to MENU_freq_FRF_xpp (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

global vett_hfig_close vett_hfig_show nfig hfig_closeall nfig_aperte


[fre,x,xpp,vett_str,nfig_end]=FRF(handles,nfig,'a');

set(handles.MENU_esporta_FRF,'Enable','on');

handles.fre=fre;
handles.x=x;
guidata(hObject, handles);

set(handles.MENU_esporta_modi,'Enable','on');

for ifig=nfig+1:nfig_end
    nfig_aperte=nfig_aperte+1;
    if nfig_aperte==1
        set(handles.MENU_grafica_showfig,'Enable','on')
        set(handles.MENU_utilities_closefig,'Enable','on')
        if handles.lingua_it
            str_close='Chiudi tutte le figure';
        else
            str_close='Close all figures';
        end
        hfig_closeall = uimenu(handles.MENU_utilities_closefig,'Label',str_close,'Callback',{@MENU_utilities_closefig_closeall_Callback,handles});
    end
    str=[handles.file_i ' - Fig. ' num2str(ifig) ': ' vett_str(ifig-nfig,:)];
    vett_hfig_close(ifig) = uimenu(handles.MENU_utilities_closefig,'Label',str,'Callback',{@MENU_utilities_closefig_vettfig_Callback,handles});
    vett_hfig_show(ifig) = uimenu(handles.MENU_grafica_showfig,'Label',str,'Callback',{@MENU_grafica_showfig_vettfig_Callback,handles});
end
nfig=nfig_end;



% --------------------------------------------------------------------
function MENU_freq_FRF_xd_Callback(hObject, eventdata, handles)
% hObject    handle to MENU_freq_FRF_xd (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

global vett_hfig_close vett_hfig_show nfig hfig_closeall nfig_aperte


[fre,x,xpp,vett_str,nfig_end]=FRFd(handles,nfig,'s');

set(handles.MENU_esporta_FRF,'Enable','on');

handles.fre=fre;
handles.x=x;
guidata(hObject, handles);

set(handles.MENU_esporta_modi,'Enable','on');

for ifig=nfig+1:nfig_end
    nfig_aperte=nfig_aperte+1;
    if nfig_aperte==1
        set(handles.MENU_grafica_showfig,'Enable','on')
        set(handles.MENU_utilities_closefig,'Enable','on')
        if handles.lingua_it
            str_close='Chiudi tutte le figure';
        else
            str_close='Close all figures';
        end
        hfig_closeall = uimenu(handles.MENU_utilities_closefig,'Label',str_close,'Callback',{@MENU_utilities_closefig_closeall_Callback,handles});
    end
    str=[handles.file_i ' - Fig. ' num2str(ifig) ': ' vett_str(ifig-nfig,:)];
    vett_hfig_close(ifig) = uimenu(handles.MENU_utilities_closefig,'Label',str,'Callback',{@MENU_utilities_closefig_vettfig_Callback,handles});
    vett_hfig_show(ifig) = uimenu(handles.MENU_grafica_showfig,'Label',str,'Callback',{@MENU_grafica_showfig_vettfig_Callback,handles});
end
nfig=nfig_end;


% --------------------------------------------------------------------
function MENU_grafica_axisequal_Callback(hObject, eventdata, handles)
% hObject    handle to MENU_grafica_axisequal (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

set(handles.MENU_grafica_axisequal,'Checked','on');
set(handles.MENU_grafica_axisauto,'Checked','off');
handles.axis_equal=true;
guidata(hObject, handles);

% --------------------------------------------------------------------
function MENU_grafica_dimnodi_Callback(hObject, eventdata, handles)
% hObject    handle to MENU_grafica_dimnodi (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

if handles.lingua_it
    lb1='Dimensione del testo';
else
    lb1='Dimensions of the text';
end

def_1=num2str(handles.dimtxt);
prompt={lb1};
default={def_1};
dlgTitle='';
lineNo=1;
answer=inputdlg(prompt,dlgTitle,lineNo,default);
    
if isempty(answer)~=1
    handles.dimtxt=str2double(answer(1));
end
if handles.lingua_it
    set(handles.MENU_grafica_dimnodi,'Label',['Dimensioni nr. dei nodi (attuale: ' num2str(handles.dimtxt) ')']);
else
    set(handles.MENU_grafica_dimnodi,'Label',['Dimensions of the nodes number (actual: ' num2str(handles.dimtxt) ')']);
end
guidata(hObject, handles);

% --------------------------------------------------------------------
function MENU_grafica_axisauto_Callback(hObject, eventdata, handles)
% hObject    handle to MENU_grafica_axisauto (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

set(handles.MENU_grafica_axisequal,'Checked','off');
set(handles.MENU_grafica_axisauto,'Checked','on');
handles.axis_equal=false;
guidata(hObject, handles);

