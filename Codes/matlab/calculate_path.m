%%
% @file calculate_path.m
% @author <b>Group 20:</b>
% <ul>
%	<li>Pradyumna Kumar
%	<li>Jayanth Tadinada
%	<li>Sharjeel Imam
% </ul>
% @version 1.1
% @section LICENSE
% Copyright (c) 2010, ERTS Lab IIT Bombay erts@cse.iitb.ac.in 
% All rights reserved.
%
% Redistribution and use in source and binary forms, with or without
% modification, are permitted provided that the following conditions are met:
%
% Redistributions of source code must retain the above copyright
% notice, this list of conditions and the following disclaimer.
%
% Redistributions in binary form must reproduce the above copyright
% notice, this list of conditions and the following disclaimer in
% the documentation and/or other materials provided with the
% distribution.
%
% Neither the name of the copyright holders nor the names of
% contributors may be used to endorse or promote products derived
% from this software without specific prior written permission.
%
% Source code can be used for academic purpose. 
% For commercial use permission form the author needs to be taken.
%
% THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
% AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
% IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
% ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
% LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
% CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
% SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
% INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
% CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
% ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
% POSSIBILITY OF SUCH DAMAGE. 
%
% Software released under Creative Commence cc by-nc-sa licence.
% For legal information refer to: 
% http://creativecommons.org/licenses/by-nc-sa/3.0/legalcode

%% 
% Function to calculate the encoded path using the list of points and orientation angles
% @ param path The list of points in the detected path. Each entry [x,y,angle] represents the (x,y) coordinates of the center and the orientation angle
% @ param img_height Image height 
% @ param img_width Image width 
% @return final_path The final encoded task which is to be transferred to the learner robot

function [final_path] = calculate_path(path,img_height,img_width)
final_path = '';			% string to store the encoded path
temp = size(path);
points = temp(1);			% no. of points in the path list
start = 1;					% starting index of a new movement (linear or rotation)
mode = 1;					% 1 means calculating linear motion. 2 means calculating rotation
ang_threshold = 3.1416/36; 	% 5 degrees is the angle threshold
turn_angle = 0;				% variable to store the magnitude of the angle turned

for i=1:(points-1)
    if(mode==1) % linear movement
		% In linear motion, if angle gap between consecutive points is greater than threshold (5 degrees),
		% it could be a case of single error point or the starting of a turn
        if((abs(path(i,3)-path(i+1,3))>=ang_threshold && abs(path(i,3)-path(i+1,3))<=((2*3.1416)-ang_threshold)) ||...
                (i==(points-1)))
            if(check_turn(path,i,points,ang_threshold)==1)			% check if the point is the starting of a turn
                line = polyfit(path(start:i,1),path(start:i,2),1);	% finding the best fit line using the points
                
				% Distance will be found using the points at the extremes of the line segment
				x1 = min(path(start:i,1));				% minimum x coordinate
                x2 = max(path(start:i,1));				% maximum x coordinate
                
				% Calculate the corresponding y coordinates
				y1 = (line(1)*x1) + line(2);
                y2 = (line(1)*x2) + line(2);
                distance = sqrt((x1-x2)^2 + (y1-y2)^2);	% distance between (x1,y1) and (x2,y2)
                angle = atan(line(1));					% slope of the line
                
				% Find actual direction of motion, i.e the quadrant in which the angle of motion lies 
				% If angle is less than 70 degrees and greater than -70 degrees use x coordinates,
				% otherwise use y coordinates because the difference between x coordinates will be negligible

                if(angle<1.2217 && angle>-1.2217)	% (angle < 70 degrees) and (angle > -70 degrees)
                    % The angle changes if the x coordinate is decreasing
					if(path(start,1)>=path(i,1))	% x coordinate is decreasing
                        if(angle==0)
                            angle = 3.1416;			% 180 degrees
                        elseif(angle>0)
                            angle = angle - 3.1416; % 3rd quadrant (angle - pi)
                        else
                            angle = angle + 3.1416; % 2nd quadrant (angle + pi)
                        end
                    end
                else
					% Considering y coordinates
                    if(path(start,2)>=path(i,2) && angle>0)		% y coordinate is decreasing and slope is positive
                        angle = angle - 3.1416; 				% 3rd quadrant (angle - pi)
                    elseif(path(start,2)<path(i,2) && angle<0)	% y coordinate is increasing and slope is negative
                        angle = angle + 3.1416;					% 2nd quadrant (angle + pi)
                    end
                end
                
				% Finding the direction of bot motion
                direction = 'f';			% initially assume that it is forward
				% If the measured angle and the observed angle are of opposite signs, then the direction is backward
				% Additional checks should be made to see if the angle difference is not less than the angle threshold (5 degrees)
                if((angle*path(start,3)<0 && abs(angle-path(start,3))>ang_threshold && abs(angle-path(start,3))<((2*3.1416)-ang_threshold)) ||...
                        (path(start,3)==0 && abs(angle)>3.1416/2))
                    direction = 'b';
                end
                
				% Add this linear motion to the encoded path. NOTE: 1 px distance = 0.1563 cm distance
                final_path = [final_path, int2str(uint16(distance*0.1563)), direction]; % converting pixel distance to cm
                
                turn_angle = path(i,3)-path(i+1,3);	% turn has started. Initialize turn_angle
                mode = 2;							% start calculating rotation
                start = i;							% new start index
            else 			% this is a single error point, so replace it with the previous correct point
                path(i+1,:) = path(i,:);
            end
        end
    else		% calculating rotation
        turn_angle = turn_angle + (path(i,3)-path(i+1,3));	% update the turn angle
		
		% In rotation, if angle gap between consecutive points is less than threshold (5 degrees),
		% it could be a case of single error point or the start of linear motion
        if((abs(path(i,3)-path(i+1,3))<ang_threshold || abs(path(i,3)-path(i+1,3))>((2*3.1416)-ang_threshold)) ||...
                (i==(points-1)))
            if(check_linear(path,i,points,ang_threshold)==1)	% check if the point is the starting of linear motion
                
				% Finding the minimum and maximum values of the angles
				% One of them will be the starting angle and the other will be the ending angle of the turn
				ang_min = min(path(start:i,3));
                ang_max = max(path(start:i,3));
                angle1 = 0;						% angle value at the start of the turn
                angle2 = 0;						% angle value at the end of the turn
                if(path(start,3)>path(i,3))		% observed angle is decreasing
                    angle1 = ang_max;
                    angle2 = ang_min;
                else							% observed angle is increasing
                    angle1 = ang_min;
                    angle2 = ang_max;
                end
                
				turn_angle = abs(turn_angle);	% only get the magnitude of the angle
				
                % Calculating the direction of the turn
                dir = 'r';						% initially assume that it is right turn
                
				% If angle is greater than 180 degrees, reverse the turn direction
                % and change the angle to (360 - angle)
                if(turn_angle>3.1416)
                    turn_angle = (2*3.1416) - turn_angle;
                    dir = 'l';
                end
                
				% Calculating the direction for different cases
                if((angle1==0 && angle2<0) || (angle2==0 && angle1>0))	% one of the angles is zero
                    dir = 'l';
                elseif(angle1*angle2>0 && angle1>angle2)				% both angles have the same sign and angle1 is greater than angle2
                    dir = 'l';
                elseif(angle1>0 && angle2<0)							% angle1 is positive and angle2 is negative, interchange the directions
                    if(dir=='r')
                        dir = 'l';
                    else
                        dir = 'r';
                    end
                end
                
				% Add this rotation to the encoded path. Convert angle from radians to degrees
				final_path = [final_path, int2str(uint16(turn_angle*180/3.1416)), dir];
                
                mode = 1;		% start calculating linear motion
                start = i;		% new start index
                turn_angle = 0;	% reset turn angle
            else 				% this is a single error point, so replace it with the previous correct point
                path(i+1,:) = path(i,:);
            end
        end
    end
end