%% This is a simple 3-link arm model created by designing an arm in
%% MapleSim4, exporting to C, and then importing into Matlab.
%% Created by Travis DeWolf, 2011

clear;
close all;

%% Constants
dt = .01; % timestep
dma = .01; % muscle activation change

boneWidth = 4;
muscleWidth = 2;

% arm constants
Constants.L = [.31 .27 .15]; % arm link lengths
Constants.m = [1.93 1.32 .35];  % mass of links
Constants.r = [.055 .055 .03 .03 .04 .04 .035 .05 .04]; % insertion point of each muscle
Constants.s = [.08 .08 .12 .12 .045 .045 .22 .25 .03]; % as a distance from the joint
Constants.com = [.165 .135 .075]; % centers of mass
Constants.p0 = [0 .045 .05];   % joint mount location offset
Constants.Izz = [0.0141 0.012 0.001];   % inertia of arm segment
Constants.Rmin = [-.1 -.1 -.1]; % minimum joint angles
Constants.Rmax = [pi-.1 pi-.1 pi-.1]; % maximum joint angles

L = Constants.L - Constants.p0; % length of arm segments for kinematics
p0 = Constants.p0;
m = Constants.m;
r = Constants.r;
s = Constants.s;
Izz = Constants.Izz;
Rmin = Constants.Rmin;
Rmax = Constants.Rmax;

%% Initial muscle activations
ma1 = 0;
ma2 = 0;
ma3 = 0;
ma4 = 0;
ma5 = 0;
ma6 = 0;
ma7 = 0;
ma8 = 0;
ma9 = 0;

ma = [ma1 ma2 ma3 ma4 ma5 ma6 ma7 ma8 ma9];

%% Set up initial state of the model

armState = [.125090126206834, .0565902847276667, .142797649926544, .101038760757725, .370120963157359, .249920932759681, .245996838252594, .217536963457561, .274588535098202, .125090126206834, 0., .0565902847276667, 0., .142797649926544, 0., .101038760757725, 0., .370120963157359, 0., .249920932759681, 0., .245996838252594, 0., .217536963457561, 0., .274588535098202, 0., .469255383384679, 0., 1.54307525393538, 0., .149106180027477, 0., -.211524990023434, 0., .794950682785412, 0., .77125108174557, 0., .100776408721091, 0., -.163249811549842, 0., .759643812216724, 0., .785398163397448, 0., .785398163397448, 0., .785398163397448, 0.]

% Load in the simulink model
mdl = load_system('MuscleArm_Subsystem');
set_param(mdl,'SaveFinalState','on', 'FinalStateName', ['armState'], 'LoadInitialState', 'on', ...
    'InitialState',['armState']);
[t,x,y] = sim('MuscleArm_Subsystem',[0 dt],[],[0 ma]);
x(1,:)

ch = '';
keydown=0;

% Set up figure and character input
figure(1); clf; hold on; grid;
set(gca, 'NextPlot', 'replacechildren');
set(gcf,'doublebuffer','on');
set(gcf,'KeyPressFcn','keydown=1;');

t = 0; % set start time

while 1==1
    t = t+dt;

    ma = [ma1 ma2 ma3 ma4 ma5 ma6 ma7 ma8 ma9];

    set_param(mdl, 'InitialState',['armState']);
    [t1,x,y] = sim('MuscleArm_Subsystem',[t t+dt],[],[t ma]);

    R = x(1,46:2:50); % get the joint angles from X
    MR4 = x(1,28:2:44); % get the orientation of the muscles
    MLength = x(1,10:2:26); % get the current muscle lengths


    %% Keyboard Control
    ch = get(gcf,'CurrentCharacter');
    if keydown==1
      switch(ch)

        case '1' % ma1 increase
            ma1=ma1+dma;
        case 'q' % set to 0!
            ma1 = 0;

        case '2' % ma2 increase
            ma2=ma2+dma;
        case 'w' % set to 0!
            ma2 = 0;

        case '3' % ma3 increase
            ma3=ma3+dma;
        case 'e' % set to 0!
            ma3 = 0;

        case '4' % ma4 increase
            ma4=ma4+dma;
        case 'r' % set to 0!
            ma4 = 0;

        case '5' % ma5 increase
            ma5=ma5+dma;
        case 't' % set to 0!
            ma5 = 0;

        case '6' % ma6 increase
            ma6=ma6+dma;
        case 'y' % set to 0!
            ma6 = 0;

        case '7' % ma7 increase
            ma7=ma7+dma;
        case 'u' % set to 0!
            ma7 = 0;

        case '8' % ma8 increase
            ma8=ma8+dma;
        case 'i' % set to 0!
            ma8 = 0;

        case '9' % ma9 increase
            ma9=ma9+dma;
        case 'o' % set to 0!
            ma9 = 0;

      end
      keydown=0;
    end

    %% Plot the arm and activations

    % y output is in form [elbow shoulder wrist] angles
    elbow = y(1,1);
    shoulder = y(1,2);
    wrist = y(1,3);
    S1 = sin(shoulder);
    C1 = cos(shoulder);
    C12 = cos(shoulder+elbow);
    S12 = sin(shoulder+elbow);
    C123 = cos(shoulder+elbow+wrist);
    S123 = sin(shoulder+elbow+wrist);

    x1 = L(1)*C1;
    y1 = L(1)*S1; %xy of elbow

    x21 = x1+L(2)*C12;
    y21 = y1 + L(2)*S12; %xy of wrist
    x22 = x1-p0(2)*C12;
    y22 = y1 - p0(2)*S12;

    x31 = x21 + L(3)*C123;
    y31 = y21 + L(3)*S123; %xy of finger
    x32 = x21 - p0(3)*C123;
    y32 = y21 - p0(3)*S123;

    clf; hold on; axis([-.7 .8 -.1 .8]); grid;
    plot([0 x1],[0 y1],'LineWidth',boneWidth);  %plot shoulder to elbow
    plot([x1 x21],[y1,y21],'LineWidth',boneWidth); %plot elbow to wrist
    plot([x1 x22],[y1,y22],'LineWidth',boneWidth);
    plot([x21 x31],[y21 y31],'LineWidth',boneWidth); % plot wrist to finger
    plot([x21 x32],[y21,y32],'LineWidth',boneWidth);
%         plot(x31,y31,'ks','LineWidth',boneWidth);  %plot finger
    plot(0,0,'ko','LineWidth',8); % plot shoulder ball
    plot(x1,y1,'ko','LineWidth',5); % plot elbow ball
    plot(x21,y21,'ko','LineWidth',5); % plot wrist ball

    %% print the torques
    text(.7,.15,sprintf('ma1=%2.2f',ma1));
    text(.7,.2,sprintf('ma2=%2.2f',ma2));
    text(.7,.25,sprintf('ma3=%2.2f',ma3));
    text(.7,.3,sprintf('ma4=%2.2f',ma4));
    text(.7,.35,sprintf('ma5=%2.2f',ma5));
    text(.7,.4,sprintf('ma6=%2.2f',ma6));
    text(.7,.45,sprintf('ma7=%2.2f',ma7));
    text(.7,.5,sprintf('ma8=%2.2f',ma8));
    text(.7,.55,sprintf('ma9=%2.2f',ma9));

    text(.7,.65,sprintf('Shoulder=%3.2f',y(1,2)*180/pi));
    text(.7,.7,sprintf('Elbow=%3.2f',y(1,1)*180/pi));
    text(.7,.75,sprintf('Wrist=%3.2f',y(1,3)*180/pi));

    text(-.65,.35,sprintf('x1=%2.4f',MLength(1)));
    text(-.65,.4,sprintf('x2=%2.4f',MLength(2)));
    text(-.65,.45,sprintf('x3=%2.4f',MLength(3)));
    text(-.65,.5,sprintf('x4=%2.4f',MLength(4)));
    text(-.65,.55,sprintf('x5=%2.4f',MLength(5)));
    text(-.65,.6,sprintf('x6=%2.4f',MLength(6)));
    text(-.65,.65,sprintf('x7=%2.4f',MLength(7)));
    text(-.65,.7,sprintf('x8=%2.4f',MLength(8)));
    text(-.65,.75,sprintf('x9=%2.4f',MLength(9)));

    %% Plot the muscles 1-9

    plot([-r(1) s(1)*C1],[0 s(1)*S1],'g','LineWidth',muscleWidth);
    plot([r(2) s(2)*C1],[0 s(2)*S1],'g','LineWidth',muscleWidth);
    plot([(L(1)-s(3))*C1 -r(3)*C12+x1],[(L(1)-s(3))*S1 -r(3)*S12+y1],'g','LineWidth',muscleWidth);
    plot([(L(1)-s(4))*C1 r(4)*C12+x1],[(L(1)-s(4))*S1 r(4)*S12+y1],'g','LineWidth',muscleWidth);
    plot([-r(5) s(5)*C12+x1],[0 s(5)*S12+y1],'g','LineWidth',muscleWidth);
    plot([r(6) -s(6)*C12+x1],[0 -s(6)*S12+y1],'g','LineWidth',muscleWidth);
    plot([(L(2)-(L(2)+p0(2)-s(7)))*C12+x1 r(7)*C123+x21],[(L(2)-(L(2)+p0(2)-s(7)))*S12+y1 r(7)*S123+y21],'g','LineWidth',muscleWidth);
    plot([(L(2)+p0(2)-s(8))*C12+x1 -r(8)*C123+x21],[(L(2)+p0(2)-s(8))*S12+y1 -r(8)*S123+y21],'g','LineWidth',muscleWidth);
    plot([(L(1)-r(9))*C1 s(9)*C123+x21],[(L(1)-r(9))*S1 s(9)*S123+y21],'g','LineWidth',muscleWidth);

    drawnow();
end
