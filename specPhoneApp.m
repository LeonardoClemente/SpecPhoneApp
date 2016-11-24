function varargout = specPhoneApp(varargin)
% SPECPHONEAPP MATLAB code for specPhoneApp.fig
%      SPECPHONEAPP, by itself, creates a new SPECPHONEAPP or raises the existing
%      singleton*.
%
%      H = SPECPHONEAPP returns the handle to a new SPECPHONEAPP or the handle to
%      the existing singleton*.
%
%      SPECPHONEAPP('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in SPECPHONEAPP.M with the given input arguments.
%
%      SPECPHONEAPP('Property','Value',...) creates a new SPECPHONEAPP or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before specPhoneApp_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to specPhoneApp_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help specPhoneApp

% Last Modified by GUIDE v2.5 23-Nov-2016 15:26:03

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @specPhoneApp_OpeningFcn, ...
                   'gui_OutputFcn',  @specPhoneApp_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
                   'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT


% --- Executes just before specPhoneApp is made visible.
function specPhoneApp_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to specPhoneApp (see VARARGIN)

% Choose default command line output for specPhoneApp
handles.output = hObject;
ABSPATH = '/Users/leonardo/Desktop/AIRDROP';

%CHANGE
k=dirList(ABSPATH);
n=length(k);

set(handles.browseEdit,'String',ABSPATH);
set(handles.directoryList,'String',k);
set(handles.cornerWarning,'String',strcat(num2str(n),'files detected.'));
try
    counterHolder=handles.browseCounter;
    counterHolder=counterHolder+1;
    holder=handles.browseHistory;
    holder{counterHolder}=ABSPATH;
    handles.browseCounter=counterHolder;
    handles.browseHistory=holder;
    holder{counterHolder}
catch error
    holder=cell(1,0);
    holder{1}=ABSPATH;
    handles.browseHistory=holder;
    handles.browseCounter=1; 
end


% Get info for reaction managing part

handles.nReactions = str2num(get(handles.nReactionsEdit,'String'));
set(handles.annotateAlarm,'String',strcat(num2str(handles.nReactions+1),' reactions needed.'));
handles.reactionsTable = zeros(handles.nReactions+1,handles.nReactions+1); % Change
handles.annotationCounter = 0;
handles.xSliderValue = 1173;
handles.ySliderValue = 1608;
handles.noteMatrix = [];
handles.noteCounter = 0;
handles.graphLength = 700;
set(handles.graphLengthEdit,'String',num2str(handles.graphLength));
% Experiment struct data
handles.intensities = [];
handles.absorbances = [];
handles.k = 0;
handles.alpha = 0;
handles.colors={[0 0 1], [0 0 0],[0 1 0],[1 0 0], [0 1 1],[1 0 1]};
handles.scale=700:300/800:400;
handles.ABSPATH=ABSPATH;
handles.click=clock;
handles.value=get(handles.directoryList,'value');
handles.excludeList=cell(1,0);
guidata(hObject, handles);
%CHANGE






% Update handles structure
guidata(hObject, handles);

% UIWAIT makes specPhoneApp wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = specPhoneApp_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in refreshButton.
function refreshButton_Callback(hObject, eventdata, handles)
% hObject    handle to refreshButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
try
    ABSPATH = handles.ABSPATH;
%CHANGE
    k=dirList(num2str(ABSPATH));
    n=length(k);
    display(k)
    set(handles.browseEdit,'String',ABSPATH);
    set(handles.directoryList,'Value',1);
    set(handles.directoryList,'String',k);
    set(handles.cornerWarning,'String',strcat(num2str(n),'files detected.'));
catch er
    set(handles.cornerWarning,'String','Not able to plot. Please check file has valid format');
    disp(er);
end
% Update handles structure
guidata(hObject, handles);


% --- Executes on button press in browseButton.
function browseButton_Callback(hObject, eventdata, handles)
ABSPATH=uigetdir();
k=dirList(ABSPATH);
n=length(k);

set(handles.browseEdit,'String',ABSPATH);
set(handles.directoryList,'String',k);
set(handles.cornerWarning,'String',strcat(num2str(n),'files detected.'));
try
    counterHolder=handles.browseCounter;
    counterHolder=counterHolder+1;
    holder=handles.browseHistory;
    holder{counterHolder}=ABSPATH;
    handles.browseCounter=counterHolder;
    handles.browseHistory=holder;
    holder{counterHolder}
catch error
    holder=cell(1,0);
    holder{1}=ABSPATH;
    handles.browseHistory=holder;
    handles.browseCounter=1; 
end


handles.ABSPATH=ABSPATH;
handles.click=clock;
handles.value=get(handles.directoryList,'value');
handles.excludeList=cell(1,0);
guidata(hObject, handles);
% hObject    handle to browseButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)



function browseEdit_Callback(hObject, eventdata, handles)
% hObject    handle to browseEdit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of browseEdit as text
%        str2double(get(hObject,'String')) returns contents of browseEdit as a double


% --- Executes during object creation, after setting all properties.
function browseEdit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to browseEdit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in directoryList.
function directoryList_Callback(hObject, eventdata, handles)
% hObject    handle to directoryList (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
try
    % Measuring for double click
    click=clock;
    val=get(handles.directoryList,'value');
    previousClick=handles.click;
    previousValue=handles.value;
    t=click-previousClick;
 
    list= get(handles.directoryList,'String');
    val = get(handles.directoryList,'Value');
    mainPath = handles.ABSPATH
    imagePath = list{val};
    
    % Plotting
    image = imread(strcat(mainPath,'/',imagePath));
    imshow(image, 'Parent', handles.imageOriginal);
    
    handles.imagePath = imagePath;
    handles.image = image;

    if t(6) < .5 && val == previousValue

        %Generating  new browser history
        handles.click=click;
        handles.value=val;
        m=get(handles.directoryList,'string');
        k=get(handles.directoryList,'value');
        
        
        
        
        path=handles.ABSPATH;
        newPath=strcat(path,'/',m{k});

        if isdir(newPath) == 1

            newList=dirList(newPath);
            listSize=size(newList,2);
            if listSize >= 1
                set(handles.directoryList,'value',1);
                set(handles.directoryList,'string',newList);
                browseHistory=handles.browseHistory;
                browseCounter=handles.browseCounter;
                browseHistory=[browseHistory newPath];
                browseCounter=browseCounter+1;
                handles.browseHistory=browseHistory;    
                handles.browseCounter=browseCounter;
                handles.ABSPATH=newPath;
                set(handles.alertText,'string','Success');

            else
                set(handles.alertText,'string','Empty folder selected')%ACTION
            end
        else
            set(handles.alertText,'string','Not a valid directory to open.')%ACTION

        end


    else
        handles.click=click;
        handles.value=val;
    end

k=handles.browseHistory;
k{handles.browseCounter}
    

catch m1
   set(handles.cornerWarning,'string','Directory list not initialized.'); 
end
guidata(hObject, handles);
% Hints: contents = cellstr(get(hObject,'String')) returns directoryList contents as cell array
%        contents{get(hObject,'Value')} returns selected item from directoryList


% --- Executes during object creation, after setting all properties.
function directoryList_CreateFcn(hObject, eventdata, handles)
% hObject    handle to directoryList (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: listbox controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes during object creation, after setting all properties.
function imageOriginal_CreateFcn(hObject, eventdata, handles)
% hObject    handle to imageOriginal (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: place code in OpeningFcn to populate imageOriginal


% --- If Enable == 'on', executes on mouse press in 5 pixel border.
% --- Otherwise, executes on mouse press in 5 pixel border or over directoryList.
function directoryList_ButtonDownFcn(hObject, eventdata, handles)
% hObject    handle to directoryList (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on key press with focus on directoryList and none of its controls.
function directoryList_KeyPressFcn(hObject, eventdata, handles)
% hObject    handle to directoryList (see GCBO)
% eventdata  structure with the following fields (see UICONTROL)
%	Key: name of the key that was pressed, in lower case
%	Character: character interpretation of the key(s) that was pressed
%	Modifier: name(s) of the modifier key(s) (i.e., control, shift) pressed
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in refButton.
function refButton_Callback(hObject, eventdata, handles)
% hObject    handle to refButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
try
    handles.refImagePath=handles.imagePath;
    handles.refImage=handles.image;
    imshow(handles.refImage, 'Parent', handles.imageRef);
    imageSizeX=size(handles.image,1);
    imageSizeY=size(handles.image,2);
    handles.refImageSizeX=imageSizeX;
    handles.refImageSizeY=imageSizeY;
   
    set(handles.xEdit,'String',num2str(imageSizeX));
    set(handles.xSlider,'Max',imageSizeX);
    set(handles.ySlider,'Max',imageSizeY);
    set(handles.yEdit,'String',num2str(imageSizeY));
%     
%     
    
catch er
    set(handles.cornerWarning,'String','Not able to plot ref Image. Please check file has valid format');
    disp(er);
end
guidata(hObject, handles);


% --- Executes on button press in analyzeButton.
function analyzeButton_Callback(hObject, eventdata, handles)
% hObject    handle to analyzeButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%try
try
    cla(handles.intensityGraph,'reset');
    cla(handles.absorbanceGraph,'reset');
    
    %Constraint values
    columnIndex = handles.xSliderValue;
    rowIndex = handles.ySliderValue;
    graphLength = str2num(get(handles.graphLengthEdit,'String'));
    %ORIGINAL = specPhone_getHorizontalIntensity(handles.image,xIndex);
    ORIGINAL = specPhone_getVerticalIntensity(handles.image,columnIndex);
    REFERENCE = specPhone_getVerticalIntensity(handles.refImage,columnIndex);
    display('si')
    handles.scale=linspace(700,400,handles.graphLength+1);
    
    hold(handles.intensityGraph,'on');
    %PLOTTING INTENSITIES
    
    handles.intensityOriginal = ORIGINAL.intensity;
        handles.refIntensity = REFERENCE.intensity;
    
    azul = plot(handles.intensityGraph,handles.scale, handles.intensityOriginal(rowIndex:rowIndex+graphLength),'LineWidth',2);
    
    
    rojo = plot(handles.intensityGraph,handles.scale, handles.refIntensity(rowIndex:rowIndex+graphLength),'red','LineWidth',2);
    
    h = legend([rojo,azul],{'Reference','Original'});
    hold(handles.intensityGraph, 'off');
    
    size(ORIGINAL.intensity)
    size(handles.refIntensity)
    handles.absorbance = specPhone_getAbsorbance(ORIGINAL.intensity,handles.refIntensity);
    absorbance = handles.absorbance;
    absorbance(isinf(absorbance)) = 0; 
    set(handles.averageAbsorbanceStatic,'String',num2str(mean(absorbance(200:600))));
    
    % PLOTTING ABSORBANCE
    mask = ones(1,20)/20;
    
    t = plot(handles.absorbanceGraph,handles.scale,conv(absorbance(rowIndex:rowIndex+graphLength),mask,'same'),'Color',[1 0 1],'LineWidth',2);
    
catch er
    disp(er);
end
guidata(hObject, handles);


% --- Executes during object creation, after setting all properties.
function intensityGraph_CreateFcn(hObject, eventdata, handles)
% hObject    handle to intensityGraph (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: place code in OpeningFcn to populate intensityGraph



function nReactionsEdit_Callback(hObject, eventdata, handles)
% hObject    handle to nReactionsEdit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of nReactionsEdit as text
%        str2double(get(hObject,'String')) returns contents of nReactionsEdit as a double


% --- Executes during object creation, after setting all properties.
function nReactionsEdit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to nReactionsEdit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function alphaConcentrationEdit_Callback(hObject, eventdata, handles)
% hObject    handle to alphaConcentrationEdit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of alphaConcentrationEdit as text
%        str2double(get(hObject,'String')) returns contents of alphaConcentrationEdit as a double


% --- Executes during object creation, after setting all properties.
function alphaConcentrationEdit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to alphaConcentrationEdit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function betaConcentrationEdit_Callback(hObject, eventdata, handles)
% hObject    handle to betaConcentrationEdit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of betaConcentrationEdit as text
%        str2double(get(hObject,'String')) returns contents of betaConcentrationEdit as a double


% --- Executes during object creation, after setting all properties.
function betaConcentrationEdit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to betaConcentrationEdit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in annotateButton.
function annotateButton_Callback(hObject, eventdata, handles)

matrix=handles.noteMatrix;
counter=handles.noteCounter;

absorbance = str2num(get(handles.averageAbsorbanceStatic,'String'));
alphaConc = str2num(get(handles.alphaConcentrationEdit,'String'));

newNote = [absorbance alphaConc];
matrix = [matrix
newNote];
counter = counter+1;

handles.noteMatrix = matrix;
handles.noteCounter = counter;
handles.absorbances = [handles.absorbances
handles.absorbance'];
handles.intensities = [handles.intensities
handles.intensityOriginal'];

set(handles.noteTable,'Data', matrix)

% if handles.annotationCounter < handles.nReactions + 1
%     absorbance = get(handles.averageAbsorbanceStatic,'String')    
%     alphaConc = get(handles.alphaConcentrationEdit,'String')
%     
%     if handles.nReactions == 1
%         betaConc = 0;
%     else
%         betaConc = get(handles.betaConcentrationEdit,'String')
%     end
%     
%     if strcmp(alphaConc, 'Please input') || strcmp(betaConc, 'Please input')
%         set(handles.annotateAlarm,'String','Please input values for alpha and beta concentrations');
%     else
%         
%         try
%             
%             reactionMatrix = handles.reactionsTable;
%             index = handles.annotationCounter + 1;
%             display(index)
%             reactionMatrix(index,1) = str2num(absorbance);
%             reactionMatrix(index,2) = str2num(alphaConc);
%             reactionMatrix(index,3) = str2num(betaConc);
%             handles.reactionsTable = reactionMatrix;
%             
%             set(handles.annotateAlarm,'String',strcat(num2str(handles.nReactions+1-index),' reactions needed.'));
%             handles.annotationCounter = index;
%             
%         catch er
%             set(handles.annotateAlarm,'String','Unable to fill matrix');
%             display(er)
%         end
%     end
%             
%         
% else
%     set(handles.annotateAlarm,'String','Enough.');
% end
guidata(hObject, handles);
    
% hObject    handle to annotateButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in regressionButton.
function regressionButton_Callback(hObject, eventdata, handles)
% hObject    handle to regressionButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
cla(handles.regressionGraph,'reset');
cla(handles.absorbanceStackGraph,'reset');
matrix = handles.noteMatrix;
concentrations = matrix(:,2);
REGRESSION = specPhone_getAbsorbanceRegression(matrix);
set(handles.kStatic,'String',num2str(REGRESSION.k));
set(handles.alphaStatic,'String',num2str(REGRESSION.alpha));

hold(handles.regressionGraph,'on');
plot(handles.regressionGraph,REGRESSION.logMatrix(:,1),REGRESSION.logMatrix(:,2),'o','LineWidth',2);
plot(handles.regressionGraph,REGRESSION.logMatrix(:,1),REGRESSION.logMatrix(:,2),'Color',[1 0 0],'LineWidth',2);
hold(handles.regressionGraph,'off');

% Calculating absorbances

colors ={[0 0 1], [0 0 0],[0 1 0],[1 0 0], [0 1 1],[1 0 1]}
absorbances = handles.absorbances;
convAbs = zeros(size(absorbances));

%absorbances=absorbances/max(absorbances(:));

n=size(absorbances,1);
l = cell(1,n);
c = cell(1,n);
nPointsAverage=30
mask = ones(1,nPointsAverage)/nPointsAverage;

% Calculating intensities
intensities = handles.intensities;
% intensities = intensities/max(intensities(:));
convInt = zeros(size(intensities));


for i = 1:n
    convInt(i,:)=conv(intensities(i,:),mask,'same');
    convAbs(i,:)=conv(absorbances(i,:),mask,'same');
end
s = convAbs;
s(isinf(s))=0;


handles.convAbs = (convAbs/max(s(:)));
handles.convInt = convInt


% Plotting absorbances
if get(handles.absorbanceRadio,'Value') ==1
    hold(handles.absorbanceStackGraph,'on');
    for i = 1:n  

        %l{i} = plot(handles.absorbanceStackGraph,conv(absorbances(i,handles.ySliderValue:handles.ySliderValue+1000),mask,'same'),'Color',colors{i},'LineWidth',2);
        l{i} = plot(handles.absorbanceStackGraph,handles.convAbs(i,handles.ySliderValue:handles.ySliderValue+1000),'Color',colors{i},'LineWidth',2);
        c{i} = strcat(num2str(concentrations(i)),' mg/ml');
    end
    legend([l{:}], c{:});
    hold(handles.absorbanceStackGraph,'off');
else
    display('entro a intensidades')
    % Plotting intensities
    hold(handles.absorbanceStackGraph,'on');
    for i = 1:n  

        %l{i} = plot(handles.absorbanceStackGraph,conv(absorbances(i,handles.ySliderValue:handles.ySliderValue+1000),mask,'same'),'Color',colors{i},'LineWidth',2);
        l{i} = plot(handles.absorbanceStackGraph,handles.convInt(i,handles.ySliderValue:handles.ySliderValue+1000),'Color',colors{i},'LineWidth',2);
        c{i} = strcat(num2str(concentrations(i)),' mg/ml');
    end
    legend([l{:}], c{:});
    hold(handles.absorbanceStackGraph,'off');
end

guidata(hObject, handles);

% if handles.nReactions+1 == handles.annotationCounter
%     handles.epsilon = str2num(get(handles.epsilonEdit,'String'));
%     handles.pathLength = str2num(get(handles.pathLengthEdit,'String'));
%     [alpha,beta,k] = specPhone_getReactionConstants(handles.reactionsTable,handles.nReactions,handles.epsilon,handles.pathLength);
%     handles.alpha = alpha;
%     handles.beta = beta;
%     handles.k = k;
%     set(handles.alphaStatic,'String',num2str(alpha));
%     set(handles.betaStatic,'String',num2str(beta));
%     set(handles.kStatic,'String',num2str(k))
%     
% end
   



function epsilonEdit_Callback(hObject, eventdata, handles)
% hObject    handle to epsilonEdit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of epsilonEdit as text
%        str2double(get(hObject,'String')) returns contents of epsilonEdit as a double


% --- Executes during object creation, after setting all properties.
function epsilonEdit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to epsilonEdit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function pathLengthEdit_Callback(hObject, eventdata, handles)
% hObject    handle to pathLengthEdit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of pathLengthEdit as text
%        str2double(get(hObject,'String')) returns contents of pathLengthEdit as a double


% --- Executes during object creation, after setting all properties.
function pathLengthEdit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to pathLengthEdit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on slider movement.
function ySlider_Callback(hObject, eventdata, handles)
% hObject    handle to ySlider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
value=floor(get(handles.ySlider,'Value'));
handles.ySliderValue = value;
set(handles.yEdit,'String',num2str(value));

xvalue = handles.xSliderValue;
matrix = handles.refImage;


% Displaying image
if value > 11
    value = value-1;
    matrix(value-10:value,:,:)= 255;
else
    display('no imprimi en y');
end

if xvalue > 11
    xvalue = xvalue-1;
    matrix(:,xvalue-10:xvalue,:)= 255;
else
    display('no imprimi en x ');
end
imshow(matrix, 'Parent', handles.imageRef);

guidata(hObject, handles);
% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function ySlider_CreateFcn(hObject, eventdata, handles)
% hObject    handle to ySlider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function xSlider_Callback(hObject, eventdata, handles)
% hObject    handle to xSlider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
value=floor(get(handles.xSlider,'Value'));
handles.xSliderValue = value;
set(handles.xEdit,'String',num2str(value));

yvalue = handles.ySliderValue;

matrix = handles.refImage;

if value > 11
    value = value-1;
    matrix(:,value-10:value,:)= 255;
else
    display('no imprimi');
end
if yvalue > 11
    yvalue = yvalue-1;
    matrix(yvalue-10:yvalue,:,:)= 255;
else
    display('no imprimi');
end

imshow(matrix, 'Parent', handles.imageRef);

guidata(hObject, handles);





% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function xSlider_CreateFcn(hObject, eventdata, handles)
% hObject    handle to xSlider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end



function xEdit_Callback(hObject, eventdata, handles)
% hObject    handle to xEdit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
value=floor(str2num((get(handles.xEdit,'String'))));

if value <= handles.refImageSizeX && value > 1;

    handles.xSliderValue = value;
    set(handles.xEdit,'String',num2str(value));

    yvalue = handles.ySliderValue;

    matrix = handles.refImage;

    if value > 11
        value = value-1;
        matrix(:,value-10:value,:)= 255;
    else
        display('no imprimi');
    end
    if yvalue > 11
        yvalue = yvalue-1;
        matrix(yvalue-10:yvalue,:,:)= 255;
    else
        display('no imprimi');
    end

    imshow(matrix, 'Parent', handles.imageRef);
else
    set(handles.xEdit,'String',num2str(handles.xSliderValue));
end

guidata(hObject, handles);
% Hints: get(hObject,'String') returns contents of xEdit as text

function xEdit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to xEdit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function yEdit_Callback(hObject, eventdata, handles)
% hObject    handle to yEdit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
value=floor(str2num(get(handles.yEdit,'String')));

if value <= handles.refImageSizeY && value >= 1 

    handles.ySliderValue = value;
    set(handles.yEdit,'String',num2str(value));

    xvalue = handles.xSliderValue;
    matrix = handles.refImage;


    % Displaying image
    if value > 11
        value = value-1;
        matrix(value-10:value,:,:)= 255;
    else
        display('no imprimi en y');
    end

    if xvalue > 11
        xvalue = xvalue-1;
        matrix(:,xvalue-10:xvalue,:)= 255;
    else
        display('no imprimi en x ');
    end
    imshow(matrix, 'Parent', handles.imageRef);
else
    set(handles.yEdit,'String',num2str(handles.ySliderValue))
end


guidata(hObject, handles);
% Hints: get(hObject,'String') returns contents of yEdit as text
%        str2double(get(hObject,'String')) returns contents of yEdit as a double


% --- Executes during object creation, after setting all properties.
function yEdit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to yEdit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes when entered data in editable cell(s) in noteTable.
function noteTable_CellEditCallback(hObject, eventdata, handles)
% hObject    handle to noteTable (see GCBO)
% eventdata  structure with the following fields (see UITABLE)
%	Indices: row and column indices of the cell(s) edited
%	PreviousData: previous data for the cell(s) edited
%	EditData: string(s) entered by the user
%	NewData: EditData or its converted form set on the Data property. Empty if Data was not changed
%	Error: error string when failed to convert EditData to appropriate value for Data
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in saveButton.
function saveButton_Callback(hObject, eventdata, handles)
% hObject    handle to saveButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
EXPERIMENT = struct();
EXPERIMENT.refImage = handles.refImage;
EXPERIMENT.AnalysisCoordinates = [handles.xSliderValue handles.ySliderValue];
EXPERIMENT.nPixels = handles.graphLength;
EXPERIMENT.epsilon = str2num(get(handles.epsilonEdit,'String'));
EXPERIMENT.pathLength = str2num(get(handles.pathLengthEdit,'String'));
EXPERIMENT.sampleIntensities = handles.intensities;
EXPERIMENT.sampleAbsorbances = handles.absorbances;
EXPERIMENT.noteTable = handles.noteMatrix;

save(strcat(handles.ABSPATH,'/specPhone_experimentInfo.mat'),'EXPERIMENT');



function graphLengthEdit_Callback(hObject, eventdata, handles)
% hObject    handle to graphLengthEdit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
graphLength=str2num(get(handles.graphLengthEdit,'String'));
try
    if graphLength < handles.refImageSizeY-handles.ySliderValue
        handles.graphLength = graphLength;
    else
        set(handles.graphLengthEdit,'String',num2str(handles.graphLength));
    end
catch er
end
guidata(hObject, handles);
        
% Hints: get(hObject,'String') returns contents of graphLengthEdit as text
%        str2double(get(hObject,'String')) returns contents of graphLengthEdit as a double


% --- Executes during object creation, after setting all properties.
function graphLengthEdit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to graphLengthEdit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes during object deletion, before destroying properties.
function noteTable_DeleteFcn(hObject, eventdata, handles)
% hObject    handle to noteTable (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in deleteNoteButton.
function deleteNoteButton_Callback(hObject, eventdata, handles)
% hObject    handle to deleteNoteButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
matrix = handles.noteMatrix
absorbances = handles.absorbances
s = size(matrix);
newMatrix = [];
newAbsorbances =[];
if s(1) > 1
    for i = 1:s(1)-1
        newMatrix = [newMatrix
matrix(i,:)];
newAbsorbances = [newAbsorbances
absorbances(i,:)];
    end
end
handles.noteMatrix = newMatrix
set(handles.noteTable,'Data',newMatrix);
guidata(hObject, handles);


% --- Executes on button press in pushbutton10.
function pushbutton10_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton10 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
close(figure(1));
figure(1);

matrix = handles.noteMatrix;
concentrations = matrix(:,2)*100;
handles.colors ={[0 0 1], [0 0 0],[0 1 0],[1 0 0], [0 1 1],[1 0 1]}


% Plotting absorbances
n = size(handles.absorbances,1);
l = cell(1,n);
c = cell(1,n);
hold on
if get(handles.absorbanceRadio,'Value') ==1
    path = strcat(handles.ABSPATH,'/absorbances.fig')
    
    for i = 1:n  

        %l{i} = plot(fig,conv(absorbances(i,handles.ySliderValue:handles.ySliderValue+1000),mask,'same'),'Color',colors{i},'LineWidth',2);
        l{i} = plot(handles.scale,handles.convAbs(i,handles.ySliderValue:handles.ySliderValue+handles.graphLength),'Color',handles.colors{i},'LineWidth',2);
        c{i} = strcat(num2str(concentrations(i)),' %');
    end
    ylabel('Absorbance (A.U.)','FontSize',20)
else
    display('entro a intensidades');
    path = strcat(handles.ABSPATH,'/intensities.fig')
    % Plotting intensities
    
    for i = 1:n  

        %l{i} = plot(handles.absorbanceStackGraph,conv(absorbances(i,handles.ySliderValue:handles.ySliderValue+1000),mask,'same'),'Color',colors{i},'LineWidth',2);
        l{i} = plot(handles.scale,handles.convInt(i,handles.ySliderValue:handles.ySliderValue+handles.graphLength),'Color',handles.colors{i},'LineWidth',2);
        c{i} = strcat(num2str(concentrations(i)),' %');
    end
    ylabel('Intensity (A.U.)','FontSize',20)
end
legend([l{:}], c{:});
xlabel('Wavelength','FontSize',20)
hold off
fig = figure(1)
savefig(fig,path);
