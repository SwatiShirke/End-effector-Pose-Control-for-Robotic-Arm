function varargout = demo4(varargin)
% DEMO4 MATLAB code for demo4.fig
%      DEMO4, by itself, creates a new DEMO4 or raises the existing
%      singleton*.
%
%      H = DEMO4 returns the handle to a new DEMO4 or the handle to
%      the existing singleton*.
%
%      DEMO4('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in DEMO4.M with the given input arguments.
%
%      DEMO4('Property','Value',...) creates a new DEMO4 or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before demo4_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to demo4_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help demo4

% Author: Ao Jiang
% Last Modified by GUIDE v2.5 29-Apr-2024 22:39:29

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @demo4_OpeningFcn, ...
                   'gui_OutputFcn',  @demo4_OutputFcn, ...
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


% --- Executes just before demo4 is made visible.
function demo4_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to demo4 (see VARARGIN)

% Choose default command line output for demo4
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% Initialize global variables
global JointPos JointTau inMotion startTime;

inMotion = false;
JointPos = zeros(6,1);
JointTau = zeros(6,1);

% UIWAIT makes demo4 wait for user response (see UIRESUME)
% uiwait(handles.figure1);

% Draw robot
robot = make_robot();
handles.robot = robot;
axes(handles.axes1);
robot.plot(JointPos');

% Update handles structure
guidata(hObject, handles);

% Create timer for plotting
startTime = tic();
t = timer('StartDelay', 1, 'Period', 0.25, 'ExecutionMode', 'fixedRate', 'TimerFcn', {@myTimerFcn,handles});
start(t);

guidata(hObject, handles);

handles.timer = t;
%start(t);


function myTimerFcn(~,~,handles)
% Timer callback function for plotting
global JointPos JointTau inMotion startTime;
if not(inMotion)
    passingTime = toc(startTime);
    update_graph(handles,JointPos,zeros(6,1),zeros(6,1),JointTau,passingTime)
end
    

% --- Outputs from this function are returned to the command line.
function varargout = demo4_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;

% --- Executes on button press in pushbutton1.
function pushbutton1_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
addpath('../lib');
global JointPos JointTau inMotion startTime;
robot = make_robot();
[S,M] = make_kinematics_model(robot);

% Check if previous move is finished
if not(inMotion)
    inMotion = true;
    currentQ = JointPos;
    %disp(currentQ);

    targetX = str2double(handles.edit1.String);
    targetY = str2double(handles.edit2.String);
    targetZ = str2double(handles.edit3.String);
    targetRoll= str2double(handles.edit5.String);
    targetPitch = str2double(handles.edit6.String);
    targetYaw = str2double(handles.edit7.String);
    load = str2double(handles.edit4.String);

    % Check inputs for load position and rotation
    if isnan(load)
        load = 0;
        message = msgbox("Please enter valid load mass","Error","error");
        pause(1);
        delete(message);
    end

    if isnan(targetPitch) || isnan(targetYaw) || isnan(targetRoll)
        message = msgbox("Please enter valid rotation","Error","error");
        pause(1);
        delete(message);
        ik_method = 'Ja'; %set ik method
    else
        if targetPitch > pi || targetPitch < -pi ...
                || targetYaw > pi || targetYaw < -pi ...
                 || targetRoll > pi || targetRoll < -pi
            ik_method = 'Ja'; %set ik method
            message = msgbox("Please enter valid angles between -pi to pi","Error","error");
            pause(1);
            delete(message);
        else
            ik_method = 'Jo'; %set ik method
        end
    end

    if isnan(targetX) || isnan(targetY) || isnan(targetZ)
        targetX = M(1,4);
        targetY = M(2,4);
        targetZ = M(3,4);
        message = msgbox("Please enter valid positions","Error","error");
        pause(1);
        delete(message);
    end

    % position calculation for Ja method
    targetPosition = [targetX,targetY,targetZ]';
    T = fkine(S,M,currentQ,'space');
    startingPos = T(1:3,4); 

    % pose calculation for Jo method
    if strcmp(ik_method,'Jo')
        currentPose = MatrixLog6(T);
        startingPos = [currentPose(3,2) ...
            currentPose(1,3) ...
            currentPose(2,1) ...
            currentPose(1:3,4)']';

        R = eul2rotm([targetRoll, targetPitch, targetYaw],'XYZ');
        targetT = [ R targetPosition; 0 0 0 1];
        targetPose = MatrixLog6(targetT);
        targetPosition = [targetPose(3,2) ...
            targetPose(1,3) ...
            targetPose(2,1) ...
            targetPose(1:3,4)']';
    end

    axes(handles.axes1);

    % Calculate trajectory
    [TauM, PosM, VelM, AccM, TpassM] = point2point(startingPos, targetPosition ,currentQ' ,robot, load, ik_method);
    disp('point2point path calculated');

    fprintf('Robot Moving');

    % Ploting joint variables
    steps = 25;
    for ii = 1:steps
        percentage = 100*(ii/(steps));
        disp(percentage);
        passingtime = toc(startTime);
        JointPos = PosM(:,1 + ii*(500/steps));
        JointTau = TauM(:,1 + ii*(500/steps));
        currentPos = PosM(:,1 + ii*(500/steps));
        JointVel = VelM(:,1 + ii*(500/steps));
        JointAcc = AccM(:,1 + ii*(500/steps));
        currentTau = TauM(:,1 + ii*(500/steps));
        robot.plot(currentPos','trail',{'r', 'LineWidth', 2});
        update_graph(handles,currentPos,JointVel,JointAcc,currentTau,passingtime);
        %pause(0.05);
        %disp(ii);
    end
    inMotion = false;
else
    message = msgbox("Robot in movement","Error","error");
    pause(2);
    delete(message);
end

function update_graph(handles,JointPos,JointVel,JointAcc,JointTau,passingTime)
hold(handles.axes2, 'on');
grid(handles.axes2, 'on');
scatter(handles.axes2,passingTime,JointPos(1,:),5,'r', 'filled');
scatter(handles.axes2,passingTime,JointPos(2,:),5,'b', 'filled');
scatter(handles.axes2,passingTime,JointPos(3,:),5,'k', 'filled');
scatter(handles.axes2,passingTime,JointPos(4,:),5,'g', 'filled');
scatter(handles.axes2,passingTime,JointPos(5,:),5,'m', 'filled');
scatter(handles.axes2,passingTime,JointPos(6,:),5,'c', 'filled');
xlim(handles.axes2, [passingTime-20,passingTime+20]);
xlabel(handles.axes2,'Time(s)');
ylabel(handles.axes2,'Position(rad)');
title(handles.axes2,'Joint Position');

hold(handles.axes3, 'on');
grid(handles.axes3, 'on');
scatter(handles.axes3,passingTime,JointVel(1,:),5,'r', 'filled');
scatter(handles.axes3,passingTime,JointVel(2,:),5,'b', 'filled');
scatter(handles.axes3,passingTime,JointVel(3,:),5,'k', 'filled');
scatter(handles.axes3,passingTime,JointVel(4,:),5,'g', 'filled');
scatter(handles.axes3,passingTime,JointVel(5,:),5,'m', 'filled');
scatter(handles.axes3,passingTime,JointVel(6,:),5,'c', 'filled');
xlim(handles.axes3, [passingTime-20,passingTime+20]);
xlabel(handles.axes3,'Time(s)');
ylabel(handles.axes3,'Velocity(rad/s)');
title(handles.axes3,'Joint Velocity');

hold(handles.axes4, 'on');
grid(handles.axes4, 'on');
scatter(handles.axes4,passingTime,JointAcc(1,:),5,'r', 'filled');
scatter(handles.axes4,passingTime,JointAcc(2,:),5,'b', 'filled');
scatter(handles.axes4,passingTime,JointAcc(3,:),5,'k', 'filled');
scatter(handles.axes4,passingTime,JointAcc(4,:),5,'g', 'filled');
scatter(handles.axes4,passingTime,JointAcc(5,:),5,'m', 'filled');
scatter(handles.axes4,passingTime,JointAcc(6,:),5,'c', 'filled');
xlim(handles.axes4, [passingTime-20,passingTime+20]);
xlabel(handles.axes4,'Time(s)');
ylabel(handles.axes4,'Accleration(rad/s^s)');
title(handles.axes4,'Joint Accleration');

hold(handles.axes5, 'on');
grid(handles.axes5, 'on');
scatter(handles.axes5,passingTime,JointTau(1,:),5,'r', 'filled');
scatter(handles.axes5,passingTime,JointTau(2,:),5,'b', 'filled');
scatter(handles.axes5,passingTime,JointTau(3,:),5,'k', 'filled');
scatter(handles.axes5,passingTime,JointTau(4,:),5,'g', 'filled');
scatter(handles.axes5,passingTime,JointTau(5,:),5,'m', 'filled');
scatter(handles.axes5,passingTime,JointTau(6,:),5,'c', 'filled');
xlim(handles.axes5, [passingTime-20,passingTime+20]);
xlabel(handles.axes5,'Time(s)');
ylabel(handles.axes5,'Torque(Nm)');
title(handles.axes5,'Joint Torque');

hChildren = handles.axes2.Children;
if length(hChildren) > 200
    delete(handles.axes2.Children(150:end));
    delete(handles.axes3.Children(150:end));
    delete(handles.axes4.Children(150:end));
    delete(handles.axes5.Children(150:end));
end

% --- Executes on button press in pushbutton2.
function pushbutton2_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Clear axes that have plots
cla(handles.axes2);
cla(handles.axes3);
cla(handles.axes4);
cla(handles.axes5);



function edit1_Callback(hObject, eventdata, handles)
% hObject    handle to edit1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit1 as text
%        str2double(get(hObject,'String')) returns contents of edit1 as a double


% --- Executes during object creation, after setting all properties.
function edit1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit2_Callback(hObject, eventdata, handles)
% hObject    handle to edit2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit2 as text
%        str2double(get(hObject,'String')) returns contents of edit2 as a double


% --- Executes during object creation, after setting all properties.
function edit2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit3_Callback(hObject, eventdata, handles)
% hObject    handle to edit3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit3 as text
%        str2double(get(hObject,'String')) returns contents of edit3 as a double


% --- Executes during object creation, after setting all properties.
function edit3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit4_Callback(hObject, eventdata, handles)
% hObject    handle to edit4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit4 as text
%        str2double(get(hObject,'String')) returns contents of edit4 as a double


% --- Executes during object creation, after setting all properties.
function edit4_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit5_Callback(hObject, eventdata, handles)
% hObject    handle to edit5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit5 as text
%        str2double(get(hObject,'String')) returns contents of edit5 as a double


% --- Executes during object creation, after setting all properties.
function edit5_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit6_Callback(hObject, eventdata, handles)
% hObject    handle to edit6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit6 as text
%        str2double(get(hObject,'String')) returns contents of edit6 as a double


% --- Executes during object creation, after setting all properties.
function edit6_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit7_Callback(hObject, eventdata, handles)
% hObject    handle to edit7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit7 as text
%        str2double(get(hObject,'String')) returns contents of edit7 as a double


% --- Executes during object creation, after setting all properties.
function edit7_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



