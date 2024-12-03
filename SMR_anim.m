function SMR_anim(block)

%
% Level-2 M-file S-function for the animation of spring mass running. 
% The function has 4 input ports:
%
% Input port 1: CoM xy position (port width: 2)
% Input port 2: Foot Point xy position (port width: 2)
% Input port 3: ground level (port width: 1)
% Input port 4: state (+1: stance, 0: flight) (port width: 1) 
% Input port 5: linear DOF position (port width: 2)
%
% Additional parameters past into the function:
%
% Parameter 1: animation sampling time
%
% H.Geyer
% Sep 2024
%

% define persistent variables    
persistent hFig Spring Piston Skeleton

MakeAnimationFlag = 0;         

% horizontal view window
ViewRange = 5;   % [m]  
ReRange   = 0.0; % relative to viewrange      

setup(block);

% ###################################################################################
function setup(block)

  % specify the block simStateCompliance
  block.SimStateCompliance = 'DefaultSimState';

  % --- Register Ports -------------------------------------------------------------- 

  block.NumInputPorts  = 6;
  block.NumOutputPorts = 0;

  block.SetPreCompInpPortInfoToDynamic;
  block.SetPreCompOutPortInfoToDynamic;

  % set properties of input 1: CoM position
  block.InputPort(1).Dimensions  = 2;
  block.InputPort(1).DatatypeID  = 0;  
  block.InputPort(1).Complexity  = 'Real';
  block.InputPort(1).DirectFeedthrough = true;

  % set properties of input 2: Foot Point position
  block.InputPort(2).Dimensions  = 2;
  block.InputPort(2).DatatypeID  = 0;  
  block.InputPort(2).Complexity  = 'Real';
  block.InputPort(2).DirectFeedthrough = true;

  % set properties of input 3: ground level
  block.InputPort(3).Dimensions  = 1;
  block.InputPort(3).DatatypeID  = 0;  
  block.InputPort(3).Complexity  = 'Real';
  block.InputPort(3).DirectFeedthrough = true;

  % set properties of input 4: stance/flight state
  block.InputPort(4).Dimensions  = 1;
  block.InputPort(4).DatatypeID  = 8;  % boolean
  block.InputPort(4).DirectFeedthrough = true;

  % set properties of input 4: linear DOF position
  block.InputPort(5).Dimensions  = 1;
  block.InputPort(5).DatatypeID  = 0;  
  block.InputPort(5).Complexity  = 'Real';
  block.InputPort(5).DirectFeedthrough = true;

  % set properties of input 4: time
  block.InputPort(6).Dimensions  = 1;
  block.InputPort(6).DatatypeID  = 0;  
  block.InputPort(6).Complexity  = 'Real';
  block.InputPort(6).DirectFeedthrough = true;
  
  
  % --- Register Parameters ---------------------------------------------------------
  
  block.NumDialogPrms  = 1;
  block.SampleTimes    = [block.DialogPrm(1).Data 0];


  % --- Register Simulation Functions -----------------------------------------------

  block.RegBlockMethod('Start',         @Start);
  block.RegBlockMethod('Outputs',     @Outputs); % required
  block.RegBlockMethod('Update',       @Update);
  block.RegBlockMethod('Terminate', @Terminate); % required

end



% ###################################################################################
function Start(block)
  
  % --- Initialize Animation Figure -------------------------------------------------

  NameStr = 'Spring Mass Running Model';

  hFig = findobj(0, 'type', 'figure', 'name', NameStr);
  if isempty(hFig)
    hFig = figure('Name', NameStr, 'NumberTitle', 'off', 'MenuBar', 'none', ...
		              'Color',  [1 1 1], 'Position', [100   300   900   450]);
    hAx = axes('Parent', hFig, 'FontSize', 8, 'Units', 'normalized', ...
               'Position',  [0.01 0.01 0.98 0.98]);
  end

  cla reset
  hAx = gca;
  set(hAx, 'SortMethod', 'childorder', 'Visible', 'on', ...
           'Color', [1 1 1], 'XColor',  [0 0 0], 'YColor', [0 0 0]);
  axis on
  axis equal
  hold on

  axis([0 - ViewRange * ReRange, 0 + ViewRange * (1-ReRange), -0.1, 1.5]);

  
  % --- Create Animation Objects ----------------------------------------------------

  SpringX = [0    0  -1/6  1/6  -1/6  1/6 -1/6  1/6 -1/6     0 0] * 0.6;
  SpringY = [0 2/12  3/12 4/12  5/12 6/12 7/12 8/12 9/12 10/12 1];
  Spring  = [SpringX; SpringY]';

  PistonX = [ 0  0 -1 -1  1  1  1] * 0.05;
  PistonY = [ 1  0 -1  0  0 -1  0] * 0.05;
  Piston = [PistonX; PistonY]';

  % initialize plot handles (note zero multiplication to avoid graphic output)
    
  Skeleton = MoCapTools.Skeleton("49.asf");
  [G, xyz] = graphSkeleton(Skeleton, -1, 1);
  jointNames = table2array(convertvars(G.Nodes, 'Name', 'string'));

  plot(G, XData=xyz(:,3) + 10, YData=xyz(:,2), NodeLabel=repmat("", numel(G.Nodes), 1));
  plot(Piston(:,1) + 1, Piston(:,2)+1, 'Color', 'k', 'LineWidth', 3);
  plot(0, 1, 'o', 'MarkerSize', 15, 'MarkerEdgeColor', 'k', 'LineWidth', 8);
  plot(Spring(:,1), Spring(:,2), 'Color', [0.7 0 0],  'LineWidth', 3);
  plot( zeros(4000, 1), zeros(4000, 1), 'k', 'LineWidth', 1);
  

end
       


% ###################################################################################
function Outputs(block)
  % nothing to do here
end



% ###################################################################################
function Update(block)
 
  % assign inputs
  CM = block.InputPort(1).Data;
  FP = block.InputPort(2).Data;
  yGnd = block.InputPort(3).Data;
  mdl_state = block.InputPort(4).Data;
  piston_len = block.InputPort(5).Data;
  
  % set actual figure to FigureHndl
  set(0, 'currentfigure', hFig)
  
  % if COM leaves view range, adjust axes
  hAx = gca;
  if hAx.XLim(2) < (CM(1) + ViewRange * ReRange)
    set(gca, 'XLim', [CM(1) - ViewRange*ReRange, CM(1) + ViewRange * (1 - ReRange)])
  elseif hAx.XLim(1) > (CM(1) + ViewRange * ReRange)
    set(gca, 'XLim', [CM(1) - ViewRange*ReRange, CM(1) + ViewRange * (1 - ReRange)])
  end
  
  % update CoM
  hCoM = hAx.Children(3);
  set(hCoM,  'XData', CM(1),  'YData', CM(2))
  
  % update ground
  hGnd = hAx.Children(1);
  XData = get(hGnd, 'XData');
  YData = get(hGnd, 'YData');
  XData = [XData(2:end) FP(1)];
  YData = [YData(2:end) yGnd];

  set(hGnd, 'XData', XData, 'YData', YData)

  % update leg
  hLeg = hAx.Children(2);

  L       =  sqrt((CM(1) - FP(1))^2 + (CM(2) - FP(2))^2);
  Alpha   = atan2(FP(1) - CM(1), CM(2) - FP(2));
  SpringL = L - piston_len;
  
  SpringX = cos(Alpha) * Spring(:,1) - sin(Alpha) * Spring(:,2) * SpringL;
  SpringY = sin(Alpha) * Spring(:,1) + cos(Alpha) * Spring(:,2) * SpringL;
  SpringX = SpringX + FP(1);
  SpringY = SpringY + FP(2);
  
  Style =  char(58 - 13 * mdl_state); 
  
  set(hLeg,  'XData', SpringX,  'YData', SpringY, 'LineStyle', Style);
  
  hPiston = hAx.Children(4);
  PistonX = cos(Alpha) * Piston(:,1) - sin(Alpha) * Piston(:,2);
  PistonY = sin(Alpha) * Piston(:,1) + cos(Alpha) * Piston(:,2);
  PistonX = PistonX + sin(Alpha) * piston_len + CM(1);
  PistonY = PistonY - cos(Alpha) * piston_len + CM(2);
  PistonX(1) = CM(1);
  PistonY(1) = CM(2);
  set(hPiston, 'XData', PistonX, 'YData', PistonY, 'LineStyle', Style);

  % force plot
  drawnow expose

end



% ###################################################################################
function Terminate(block)
  % nothing to do here
end

end