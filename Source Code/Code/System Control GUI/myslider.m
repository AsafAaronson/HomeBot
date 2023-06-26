function varargout = GuiCam(varargin)
% MYSLIDER MATLAB code for myslider.fig
%      MYSLIDER, by itself, creates a new MYSLIDER or raises the existing
%      singleton*.
%
%      H = MYSLIDER returns the handle to a new MYSLIDER or the handle to
%      the existing singleton*.
%
%      MYSLIDER('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in MYSLIDER.M with the given input arguments.
%
%      MYSLIDER('Property','Value',...) creates a new MYSLIDER or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before myslider_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to myslider_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help myslider

% Last Modified by GUIDE v2.5 12-Jun-2023 16:34:36

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @myslider_OpeningFcn, ...
                   'gui_OutputFcn',  @myslider_OutputFcn, ...
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
% End initialization code - DO NOT EDIT %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


% --- Executes just before myslider is made visible.
function myslider_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to myslider (see VARARGIN)

% Choose default command line output for myslider
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes myslider wait for user response (see UIRESUME)
% uiwait(handles.figure1);
set(handles.slider1,'Value',90); 
set(handles.slider2,'Value',90); 
set(handles.slider3,'Value',90); 
set(handles.slider4,'Value',90); 
set(handles.slider5,'Value',90); 
set(handles.slider6,'Value',90); 
set(handles.slider9,'Value',98); 

% --- Outputs from this function are returned to the command line.
function varargout = myslider_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;
clear all


% %%%%%% Initialzation of global Variables %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
global walking_mat;
walking_mat=load('walking_mat_slow');
global Turn_clockwise_mat;
Turn_clockwise_mat=load('Turn_clockwise_mat');
global climb_mat;
climb_mat=load('climb_mat');
global forward_grasp;
forward_grasp=load('forward_grasp_mat');

% Loading yoloV4 detector
global detector;
detector = yolov4ObjectDetector("csp-darknet53-coco");
disp('detector loaded');

% Setting up Bluetooth Connection
global ROBOT_controller;
ROBOT_controller=bluetooth("Stereo Headset");
ROBOT_controller.Timeout = 1;
 disp('Robot connected by bluetooth');


% %%%%%% Slider Controls %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% --- Executes on slider movement.
function slider1_Callback(hObject, eventdata, handles)
data= get(handles.slider1,'value');
set(handles.text1,'String',data)
b= get(hObject, 'value');
val=int2str(b)
global ROBOT_controller;
val="1"+" "+val
MessageVerification(ROBOT_controller,val);


% hObject    handle to slider1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider

% --- Executes during object creation, after setting all properties.
function slider1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called


% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end

% --- Executes on slider movement.
function slider2_Callback(hObject, eventdata, handles)
% hObject    handle to slider2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
data= get(handles.slider2,'value');
set(handles.text2,'String',data)
b= get(hObject, 'value');
val=int2str(b)
global ROBOT_controller;
val="2"+" "+val
MessageVerification(ROBOT_controller,val);


% --- Executes during object creation, after setting all properties.
function slider2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function slider3_Callback(hObject, eventdata, handles)
% hObject    handle to slider3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
data= get(handles.slider3,'value');
set(handles.text3,'String',data)
b= get(hObject, 'value');
val=int2str(b)
global ROBOT_controller;
val="3"+" "+val
MessageVerification(ROBOT_controller,val);


% --- Executes during object creation, after setting all properties.
function slider3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function slider4_Callback(hObject, eventdata, handles)
% hObject    handle to slider4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
data= get(handles.slider4,'value');
set(handles.text4,'String',data)
b= get(hObject, 'value');
val=int2str(b)
global ROBOT_controller;
val="4"+" "+val
MessageVerification(ROBOT_controller,val);


% --- Executes during object creation, after setting all properties.
function slider4_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function slider5_Callback(hObject, eventdata, handles)
% hObject    handle to slider5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
data= get(handles.slider5,'value');
set(handles.text5,'String',data)
b= get(hObject, 'value');
val=int2str(b)
global ROBOT_controller;
val="5"+" "+val
MessageVerification(ROBOT_controller,val);


% --- Executes during object creation, after setting all properties.
function slider5_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function slider6_Callback(hObject, eventdata, handles)
% hObject    handle to slider6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
data= get(handles.slider6,'value');
set(handles.text6,'String',data)
b= get(hObject, 'value');
val=int2str(b)
global ROBOT_controller;
val="6"+" "+val
MessageVerification(ROBOT_controller,val);



% --- Executes during object creation, after setting all properties.
function slider6_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function slider7_Callback(hObject, eventdata, handles)
% hObject    handle to slider7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
data= get(handles.slider7,'value');
set(handles.text7,'String',data)
b= get(hObject, 'value');
val=int2str(b)
global ROBOT_controller;
val="0"+" "+val
writeline(ROBOT_controller,val);
readline(ROBOT_controller)

% --- Executes during object creation, after setting all properties.
function slider7_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end

% --- Executes on slider movement.
function slider9_Callback(hObject, eventdata, handles)
% hObject    handle to slider9 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
data= get(handles.slider9,'value');
set(handles.text17,'String',data)
b= get(hObject, 'value');
val=int2str(b)
global ROBOT_controller;
val="70"+" "+val
writeline(ROBOT_controller,val);
readline(ROBOT_controller)

% %%%%%% Button Executions %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% --- Executes on button press in pushbutton1.
function pushbutton1_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global ROBOT_controller;
global climb_mat;
com_delay=0.08;
tic

for i=1:length(climb_mat.walk_motor_path_matrix)
   writeline(ROBOT_controller,"1"+" "+climb_mat.walk_motor_path_matrix(1,i))
      pause(com_delay)
   writeline(ROBOT_controller,"2"+" "+climb_mat.walk_motor_path_matrix(2,i))
      pause(com_delay)
   writeline(ROBOT_controller,"3"+" "+climb_mat.walk_motor_path_matrix(3,i))
      pause(com_delay)
   writeline(ROBOT_controller,"4"+" "+climb_mat.walk_motor_path_matrix(4,i))
      pause(com_delay)
   writeline(ROBOT_controller,"5"+" "+climb_mat.walk_motor_path_matrix(5,i))
      pause(com_delay)
   writeline(ROBOT_controller,"6"+" "+climb_mat.walk_motor_path_matrix(6,i))
   pause(com_delay)
   disp(i)

end   
toc

% %% TURN %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% --- Executes on button press in pushbutton21.
function pushbutton21_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton21 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
SingleTurn();


% --- Executes on button press in pushbutton38.
function pushbutton38_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton38 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global ROBOT_controller;
val="61";
writeline(ROBOT_controller,val);
pause(0.1);
data=readline(ROBOT_controller);
set(handles.text15,'String',data)

% --- Executes on button press in pushbutton39.
function pushbutton39_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton39 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global ROBOT_controller;
val="62";
writeline(ROBOT_controller,val);
pause(0.1);
data=readline(ROBOT_controller);
set(handles.text16,'String',data)

% %%%%%% Read Test Image %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% --- Executes on button press in pushbutton40.
function pushbutton40_Callback(hObject, eventdata, handles)

global detector;
I = imread('81.jpg');
 [bboxes,scores,~] = detect(detector,I);
I = insertObjectAnnotation(I,"rectangle",bboxes,scores);
set(handles.axes1,'Units','pixels');
resizePos = get(handles.axes1,'Position');
I= imresize(I, [resizePos(3) resizePos(3)]);
axes(handles.axes1);
imshow(I);
set(handles.axes1,'Units','normalized');

% %%%%%% Path Planning Algorithm %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% --- Executes on button press in pushbutton41.
function pushbutton41_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton41 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%path planning
disp("start planning")
PathPlanning();

% %%%%%% Object Grabbing Algorithm %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% --- Executes on button press in pushbutton44.
function pushbutton44_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton44 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Get Object from "pool" (small_angle_limit, large_angle_limit)
% Get Object from "pool" (small_angle_limit, large_angle_limit)
[small_angle_limit, large_angle_limit] = get_object_angle_limits("bottle");

% Scan and create 2xN vector
scanning_angle_range = 35;
step_size = 2;
range = [-1*scanning_angle_range:step_size:scanning_angle_range];
[scan_results]= Scanning(range);
destance_vec=scan_results(1,:);

% Determine angle + distance

[dist_to_obj, min_index] = min(destance_vec(~ismember(destance_vec,0)));
min_destance_index_vec=[];
for i=1:length(destance_vec)
    if dist_to_obj==destance_vec(i)
        min_destance_index_vec(end+1)=i;
    end
end
min_destance_index_vec
if length(min_destance_index_vec)>1
    average_index=round(min_destance_index_vec(1)+(min_destance_index_vec(end)-min_destance_index_vec(1))/2);
else
    average_index=min_index;
end
average_index
ang_to_obj = range(average_index)


rad_angles = deg2rad(scan_results(2,:));
figure
polarplot(rad_angles,scan_results(1,:),'*')
hold on
polarplot(deg2rad(ang_to_obj),dist_to_obj,'+',LineWidth=2)
hold on
polarplot(deg2rad(-1*scanning_angle_range*ones(2,1)),[0;10],'--',Color="green")
hold on
polarplot(deg2rad(scanning_angle_range*ones(2,1)),[0;10],'--',Color="green")
title("Ditance Scan Results")


% Transform to angle + distance in relation to rear joint
x = dist_to_obj*sind(ang_to_obj);
y = dist_to_obj*cosd(ang_to_obj);
D = 15; % To measure distance from rear joint to sonar
rear_ang_to_obj = atand(x/(D+y));
rear_dist_to_obj = (((D+y)^2)+(x^2))^0.5;

% If object distance is in legal range
if (ang_to_obj < small_angle_limit)
    run_small_grab(90+ang_to_obj,dist_to_obj) 
elseif (ang_to_obj < large_angle_limit)
    run_large_grab(rear_ang_to_obj,rear_dist_to_obj)
else
    disp("Object is not found in legal range")
end

% %%%%%% Functions %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function[small,large] = get_object_angle_limits(object)
%Get parameters for the chosen object
if (object == "Orange")
small = 13;%%%%%%
large = 20;%%%%%% 
end


function[]=move_to_angle(angle)
disp("I moved to angle:") %%%%%% 
% %(we should get some sort of log if was succesful? or return variable?)
disp(angle) %%%%%% 


function[distance]=get_distance(n,i)
distance = 8 + 2*sin((i)*2*pi/(n)); %%%%%%


function[]=run_small_grab(angle,distance)
    global ROBOT_controller;
    global forward_grasp;
    [forward_grasp]= grasp_live_motion_plan(angle)
    com_delay=0.05;
    tic
    
    for i=1:length(forward_grasp)
       writeline(ROBOT_controller,"1"+" "+forward_grasp(1,i))
          pause(com_delay)
       writeline(ROBOT_controller,"2"+" "+forward_grasp(2,i))
          pause(com_delay)
       writeline(ROBOT_controller,"3"+" "+forward_grasp(3,i))
          pause(com_delay)
       writeline(ROBOT_controller,"4"+" "+forward_grasp(4,i))
          pause(com_delay)
       writeline(ROBOT_controller,"5"+" "+forward_grasp(5,i))
          pause(com_delay)
       writeline(ROBOT_controller,"6"+" "+forward_grasp(6,i))
          pause(com_delay)
       writeline(ROBOT_controller,"70"+" "+forward_grasp(7,i))
          pause(com_delay) 
    end   
    toc

    
function [scan_results]= Scanning(range)
global ROBOT_controller;

writeline(ROBOT_controller,"5"+" "+(range(1)+90))
 pause(1);

samples_amount=3;
scan_results = zeros(2,length(range));
i = 1;
for angle = range
    distance=[];
    motor_angle=90+angle
    writeline(ROBOT_controller,"5"+" "+motor_angle)
    pause(0.05)
    for j=1:samples_amount
        val="61";
        writeline(ROBOT_controller,val);
        pause(0.01);
        distance(end+1)=readline(ROBOT_controller)
    end
    scan_results(:,i)=[mode(distance) ; angle];
    i = i + 1;
end


% --- Executes during object creation, after setting all properties.
function slider9_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider9 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end

function [forward_grasp]= grasp_live_motion_plan(angle)
x=angle
motors_limits= [42 160
                0   180
                45 135
                45 135
                0 180
                42 160
                45 98];

time_resulotion=0.25;
forward_grasp=[];

% the matrix of movement
position_matrix=[42   42   42    42   % 38  % 28  28   28
                 86   86   86    86    %86   %86  86   86
                 68   68   68    68   % 81  % 80  80   80
                 88   88   70    70   % 81  % 108  81  81
                 x    x    x     x    % 90  % 90  90   90
                 115  115  115   115  % 117 % 75  115  115
                 98   98   98    50   % 50  % 50  50   98
                 0    0.5  1.5   3    ];% 4  ]; % 4   5    6];
% create new time vector base on the resolotion of movement
t= position_matrix(end,:);
t_new=0:time_resulotion:position_matrix(end,end);
% figure
for i=1:7
    row_pos= position_matrix(i,:);
    forward_grasp(i,:) = pchip(t,row_pos, t_new); % choose pchip for fitting
end

for i=1:7   %change breaknig of mechnical limitations
    for j=1:length(forward_grasp)
        if forward_grasp(i,j)>motors_limits(i,2)
            forward_grasp(i,j)=motors_limits(i,2);
        elseif forward_grasp(i,j)<motors_limits(i,1)
            forward_grasp(i,j)=motors_limits(i,1);
        end
    end
end


function [home1]= home1()
motors_limits= [40 135
                0   180
                45 135
                45 135
                0 180
                42 160
                45 150];

time_resulotion=0.75;
home1=[];

% the matrix of movement
position_matrix=[42   42   47    70   70   70   70    70    70   70   70   42
                 86   86   86    86   86    100  72    86    86   86   86   86
                 68   68   68    128  128  128  128   128   112  112  112  68
                 88   88   117   112  112  112  112   112   67   67   100   88
                 90    90  90    90   105  120  60    90    90   90   90   90
                 115  115  90   115   115  115  115   115   136  120  115  115
                 98   98   98    98   98   98   98    98    98   98   98   98
                 0    3    6     9    12   15  20     22    27   28.5 31   35];
% create new time vector base on the resolotion of movement
t= position_matrix(end,:);
t_new=0:time_resulotion:position_matrix(end,end);
% figure
for i=1:7
    row_pos= position_matrix(i,:);
    home1(i,:) = pchip(t,row_pos, t_new); % choose pchip for fitting
end

for i=1:7   %change breaknig of mechnical limitations
    for j=1:length(home1)
        if home1(i,j)>motors_limits(i,2)
            home1(i,j)=motors_limits(i,2);
        elseif home1(i,j)<motors_limits(i,1)
            home1(i,j)=motors_limits(i,1);
        end
    end
end


% --- Executes on button press in pushbutton47.
function pushbutton47_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton47 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global ROBOT_controller;
writeline(ROBOT_controller,"52")

com_delay=0.05;   %home2
tic
matrix=home4();
for i=1:length(matrix)
   writeline(ROBOT_controller,"1"+" "+matrix(1,i))
      pause(com_delay)
   writeline(ROBOT_controller,"2"+" "+matrix(2,i))
      pause(com_delay)
   writeline(ROBOT_controller,"3"+" "+matrix(3,i))
      pause(com_delay)
   writeline(ROBOT_controller,"4"+" "+matrix(4,i))
      pause(com_delay)
   writeline(ROBOT_controller,"5"+" "+matrix(5,i))
      pause(com_delay)
   writeline(ROBOT_controller,"6"+" "+matrix(6,i))
      pause(com_delay)
   writeline(ROBOT_controller,"70"+" "+matrix(7,i))
      pause(0.2)
   i
   
end   
toc
function [] = MessageVerification(ROBOT_controller,message)
    writeline(ROBOT_controller,message);


function [home2]= home2()
motors_limits= [40 135
                0   180
                45 135
                45 135
                0 180
                42 160
                45 150];

time_resulotion=1.5;
home2=[];

% the matrix of movement
position_matrix=[42   42   42    70   70   70    70    70  70   70  42      %% 70   70    70    70   70   70   42
                 86   86   86    86   86   86    86    86  86   86  86      %%  100  72    86    86   86   86   86
                 68   68   88    135  135  135   135   135 135  135  68      %% 128  128   128   112  112  112  68
                 88   88   117   135  135  135   135   135  135 135  88      %% 112  112   112   67   67   100   88
                 90    90  90    90   90   90    90    90   90  90  90       %% 120  60    90    90   90   90   90
                 115  115  90   115   115  115   115   115  115 115  115      %% 115  115   115   136  120  115  115
                 98   98   98    98   98   98    60    98   60  98  98        %%98   98    98    98   98   98   98
                 0    3    6     9    12   13.5    15  16.5  18 27   33];   %%15  20     22    27   28.5 31   35];
% create new time vector base on the resolotion of movement
t= position_matrix(end,:);
t_new=0:time_resulotion:position_matrix(end,end);
% figure
for i=1:7
    row_pos= position_matrix(i,:);
    home2(i,:) = pchip(t,row_pos, t_new); % choose pchip for fitting
end

%change breaking of mechnical limitations
for i=1:7   
    for j=1:length(home2)
        if home2(i,j)>motors_limits(i,2)
            home2(i,j)=motors_limits(i,2);
        elseif home2(i,j)<motors_limits(i,1)
            home2(i,j)=motors_limits(i,1);
        end
    end
end


function [home3]= home3()
motors_limits= [40 135
                0   180
                45 135
                45 135
                0 180
                42 160
                45 150];

time_resulotion=1.5;
home3=[];

% the matrix of movement
position_matrix=[42   42   42    70   70   70          %% 70   70    70    70   70   70   42
                 86   86   86    86   86   86          %%  100  72    86    86   86   86   86
                 68   68   88    135  135  135        %% 128  128   128   112  112  112  68
                 88   88   117   135  135  135         %% 112  112   112   67   67   100   88
                 90    90  90    90   90   90           %% 120  60    90    90   90   90   90
                 115  115  90   115   115  115         %% 115  115   115   136  120  115  115
                 98   98   98    98   98   98            %%98   98    98    98   98   98   98
                 0    3    6     9    12   13.5    ];   %%15  20     22    27   28.5 31   35];
% create new time vector base on the resolotion of movement
t= position_matrix(end,:);
t_new=0:time_resulotion:position_matrix(end,end);
% figure
for i=1:7
    row_pos= position_matrix(i,:);
    home3(i,:) = pchip(t,row_pos, t_new); % choose pchip for fitting
end

for i=1:7   %change breaknig of mechnical limitations
    for j=1:length(home3)
        if home3(i,j)>motors_limits(i,2)
            home3(i,j)=motors_limits(i,2);
        elseif home3(i,j)<motors_limits(i,1)
            home3(i,j)=motors_limits(i,1);
        end
    end
end



function [home4]= home4()
motors_limits= [34 135
                0   180
                45 135
                45 135
                0 180
                42 160
                45 150];

time_resulotion=0.75;
home4=[];

% the matrix of movement
position_matrix=[38   38    70   70   70   70   42
                 86   86    86    86   86   86   86
                 81   81    128  112  112  112  68
                 81   81    112  75   75   100   88
                 90   90    90   90   90   90   90
                 117  117  115   120  120  115  115
                 50   50    50   50   98   98   98
                 0    3     9    13   16   19   22];
% create new time vector base on the resolotion of movement
t= position_matrix(end,:);
t_new=0:time_resulotion:position_matrix(end,end);
% figure
for i=1:7
    row_pos= position_matrix(i,:);
    home4(i,:) = pchip(t,row_pos, t_new); % choose pchip for fitting
end

for i=1:7   %change breaknig of mechnical limitations
    for j=1:length(home4)
        if home4(i,j)>motors_limits(i,2)
            home4(i,j)=motors_limits(i,2);
        elseif home4(i,j)<motors_limits(i,1)
            home4(i,j)=motors_limits(i,1);
        end
    end
end


function []=SingleStep()
global ROBOT_controller;
global walking_mat;

com_delay=0.01;  

tic
n_step=1;
oriantation=[];
MessageVerification(ROBOT_controller,"51")
for j=1:n_step
    for i=1:length(walking_mat.walk_motor_path_matrix)
        
       MessageVerification(ROBOT_controller,"1"+" "+round(walking_mat.walk_motor_path_matrix  (1,i)))
          pause(com_delay)
       MessageVerification(ROBOT_controller,"2"+" "+round(walking_mat.walk_motor_path_matrix  (2,i)))
          pause(com_delay)
       MessageVerification(ROBOT_controller,"3"+" "+round(walking_mat.walk_motor_path_matrix  (3,i)))
          pause(com_delay)
       MessageVerification(ROBOT_controller,"4"+" "+round(walking_mat.walk_motor_path_matrix  (4,i)))
          pause(com_delay)
       MessageVerification(ROBOT_controller,"5"+" "+round(walking_mat.walk_motor_path_matrix  (5,i)))
          pause(com_delay)
       MessageVerification(ROBOT_controller,"6"+" "+round(walking_mat.walk_motor_path_matrix  (6,i)))
          pause(0.05)
       %MessageVerification(ROBOT_controller,"62")
        %  pause(com_delay)
       %oriantation(end+1)=readline(ROBOT_controller);
       i
    end    
end
toc

function []= SingleTurn()
global ROBOT_controller;
global Turn_clockwise_mat;
com_delay=0.05;
loop_delay=0;
tic
writeline(ROBOT_controller,"52")
for i=1:length(Turn_clockwise_mat.Turn_Clockwise_motor_path_matrix)
   writeline(ROBOT_controller,"1"+" "+Turn_clockwise_mat.Turn_Clockwise_motor_path_matrix  (1,i))
      pause(com_delay)
   writeline(ROBOT_controller,"2"+" "+Turn_clockwise_mat.Turn_Clockwise_motor_path_matrix  (2,i))
      pause(com_delay)
   writeline(ROBOT_controller,"3"+" "+Turn_clockwise_mat.Turn_Clockwise_motor_path_matrix  (3,i))
      pause(com_delay)
   writeline(ROBOT_controller,"4"+" "+Turn_clockwise_mat.Turn_Clockwise_motor_path_matrix  (4,i))
      pause(com_delay)
   writeline(ROBOT_controller,"5"+" "+Turn_clockwise_mat.Turn_Clockwise_motor_path_matrix  (5,i))
      pause(com_delay)
   writeline(ROBOT_controller,"6"+" "+Turn_clockwise_mat.Turn_Clockwise_motor_path_matrix  (6,i))
      pause(com_delay)
   writeline(ROBOT_controller,"70"+" "+"98")
     pause(loop_delay)
   i
   
end   
toc

function latest_photo_path = get_latest_file(directory)
%This function returns the path of the latest filename from the directory
disp("start get latest file")
%Get the directory contents
dirc = dir(directory);

%Filter out all the folders.
dirc = dirc([dirc.isdir]==0);

%I contains the index to the biggest number which is the latest file
[~,I] = max([dirc(:).datenum]);

if ~isempty(I)
    latestfile = dirc(I).name;
    latest_photo_path = strcat(directory,'\',latestfile);
end
disp("finish get latest file")



function []= PathPlanning()
global detector
% Input: Detector, Object (name:,height,weight,angles,...)
obj_height = 3.5; %cm 

photo_folder_path = 'path';
disp(photo_folder_path);

% Get most recent picture & Run object detection
latest_photo_path = get_latest_file(photo_folder_path);
disp("Openeing photo:")
disp(latest_photo_path)
[img_bbox,scores,labels] = object_detection(latest_photo_path);
disp("Detection Results:")
disp(labels)
disp(scores)

% Make sure the object detection got the wanted object

object_name = 'bottle';

object_index = 0;
for i=1:length(labels)
    if (string(labels(i))==object_name)
        object_index=i;
    end
end


if object_index==0
    disp("Did not Detect the wanted object:")
    disp(object_name)
    disp("Check if it's int the frame")
else
    img_bbox = img_bbox(object_index,:);
    
    % Calculate distance & angle
    [distance, angle] = bbox_to_location(img_bbox,obj_height);
    disp("Distance: "+distance+ ", Angle:  "+angle+", Calculated from bbox")
    
    % Transform distance & angle to:
    % X - big turns
    % Y - small turns
    % Z - big steps
    % W - small steps
    % R - turn right (True/False)
    [X,Y,Z,W,R] = transform_obj_location_to_path(distance, angle);
    disp("big turns: "+X)
    disp("small turns: "+Y)
    disp("big steps: "+Z)
    disp("small steps: "+W)
    disp("turn right (true/false): "+R)
    for i=1:X
        SingleTurn();
    end
    for i=1:Z
        SingleStep();
    end
    disp("Path completed")
end

function [depth,angle] = bbox_to_location(bbox,obj_height)
% BBox to location
% Input params:
%   bbox of the shape [x,y,box width,box height]
%   height of the actual object
% Return:
%   angle and distance of object in the frame

% Hard Coded Variables
% Size of Image [px/px]
img_width = 800;
img_height = 600; 


% Assign variables
t = num2cell(bbox);
% Handle if no bbox is supplied
[x,y,w,h] = deal(t{:});
centroid = [(x+(w/2)) (y+(h/2))];

% Calculating Angle:

% Distance from cenral line
dist = -(img_width/2 - centroid(1));

% From curve fit:
% angle[deg] = dist[px]*0.03564[deg/px] -- changed to 0.0589
angle = dist*0.0589;

% Calculating Depth (Distance from camera)
% Frome curve fit
% central line depth[cm] = 1627[cm^2/px]/(normalized height[px/cm])
% depth = central line depth * abs(dist)*(0.0875/(i_width/2))

normalized_h = h/obj_height;
centr_depth = 1112/normalized_h;
depth = centr_depth + centr_depth*abs(dist)*(0.0875/(img_width)/2);


function [big_turns,small_turns,big_steps,small_steps,turn_right] = transform_obj_location_to_path(distance, angle)
% Transform Object Location (Distance and Angle) to
% a set of Turns and Steps that take you to the wanted location

%   Inputs: Distance, and Angle
%   Outputs: [
%       Num of big turns
%       Num of small turns
%       Num of big steps
%       Num of small steps
%       Direction of turn (Turn Right? Logical 1/0)
%       ]

% Hard Coded Variables:
% Turn and step Sizes
big_turn_angle = 10;
small_turn_angle = 10;
big_step_distance = 13.5;
small_step_distance = 13.5;

% Desired space from object to aim for
distance_wanted_from_object = -20;

%
turn_right = (angle>0);

angle = abs(angle);
distance = distance- distance_wanted_from_object;

big_turns = floor(angle/big_turn_angle);
small_turns = floor(mod(angle,big_turn_angle)/small_turn_angle);
angle_remainder = mod(mod(angle,big_turn_angle),small_turn_angle);
disp("Angle Error Remainder: "+ angle_remainder)

big_steps = floor(distance/big_step_distance);
small_steps = floor(mod(distance,big_step_distance)/small_step_distance);
distance_remainder = mod(mod(distance,big_step_distance),small_step_distance);
disp("Distance Error Remainder: "+ distance_remainder)


function [bboxes,scores,labels] = object_detection(photo_path)
global detector
% Input:
%  - path to the photo
%  - detector 
% Returns:
%  - image detection results

%I = imread(photo_path);
  I = imread(photo_path);
%   det = yolov4ObjectDetector("csp-darknet53-coco");
%  [bboxes,scores,labels] = detect(det,I);
 [bboxes,scores,labels] = detect(detector,I);
I = insertObjectAnnotation(I,"rectangle",bboxes,scores);
%figure
imshow(I)
new_photo_path = strcat(photo_path,'_detected.jpg');
imwrite(I,new_photo_path)

