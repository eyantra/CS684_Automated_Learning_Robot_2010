%%
% @mainpage Automated Learning Robot
% @author <b>Group 20:</b>
% <ul>
%	<li>Pradyumna Kumar
%	<li>Jayanth Tadinada
%	<li>Sharjeel Imam
% </ul>
% @version 1.1
% @section DESCRIPTION
% Matlab captures the task performed by the robot using image processing. Then it encodes the task as a string and transmits it to the learner robot using serial communication. Each Matlab file contains one function.
%

%%
% @file bot_detect.m
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
% The main function to start image processing and task detection
% Image processing starts 5 secs after the preview is loaded so that the camera has fully adjusted to light conditions
% Zigbee communication starts 5 secs after the encoded path is obtained

function [] = bot_detect()
% Range of RGB values for orange and geen disks
orange1 = [130,70,30]; orange2 = [210,135,80];	% orange1 is lower limit of [R,G,B] values and orange2 is upper limit of [R,G,B] values
green1 = [60,80,65]; green2 = [110,190,130];	% same as orange

img_height = 480;		% image height
img_width = 640;		% image width
path(1,:) = [0,0,0];	% list of points in the detected path. [x coordinate, y coordinate, orientation angle] 
points = 1;				% no.of points collected
encoded_path = '';		% final encoded path

% Initialize camera and start preview
vid = videoinput('winvideo',2,'YUY2_640x480');	% change this accordingly
set(vid, 'ReturnedColorSpace','rgb');
preview(vid);
pause(5);	% Wait for 5 secs for camera image to stabilize

% Loop 160 times. Increase this value if image processing is to be done for a longer time
for count=1:160
    img = getsnapshot(vid);					% take snapshot
    img_bin = zeros(img_height,img_width);	% new B/W image to show only the orange and green circles
    [fcenter,img_bin] = get_center(img,img_height,img_width,orange1,orange2,img_bin);	% get center of orange disk (fcenter)
    [bcenter,img_bin] = get_center(img,img_height,img_width,green1,green2,img_bin);		% get center of green disk (bcenter)
    if((fcenter(1)==0&&fcenter(2)==0) || (bcenter(1)==0&&bcenter(2)==0))
        continue;
    end
    center = [(bcenter(1)+fcenter(1))/2,(bcenter(2)+fcenter(2))/2];		% center of the bot is midpoint of fcenter and bcenter
    
    % Calculating the angle of orientation of the bot 
    if(abs(fcenter(1)-bcenter(1))<=1)	% same x coordinates
        if(fcenter(2)>bcenter(2))		% y coordinate of front is greater than that of back
            angle = 1.5708;				% 90 degrees
        else
            angle = -1.5708;			% -90 degrees
        end
    else
        angle = atan((fcenter(2)-bcenter(2))/(fcenter(1)-bcenter(1))); % angle in 1st quadrant and 4th quadrant. slope = (y2 - y1)/(x2-x1)
        if(fcenter(1)<bcenter(1))
            if(angle==0)
                angle = 3.1416;			% 180 degrees
            elseif(angle>0)
                angle = angle - 3.1416; % 3rd quadrant (angle - pi)
            else
                angle = angle + 3.1416; % 2nd quadrant (angle + pi)
            end
        end
    end
    
    % Ignoring unnecessary points by comparing the position of the center and the angle with previous values
    if(points~=1)
        % if distance between centers is less than 5px and angle difference is less
        % than 15 degrees or greater than 345 degrees, then ignore those points
        if((sqrt((center(1)-pcenter(1))^2 + (center(2)-pcenter(2))^2)<=5) &&...
                (abs(angle-pangle)<3.1416/12 || abs(angle-pangle)>23*3.1416/12))
            continue;
        end
        
        % angle difference cannot be greater than 45 degrees and less than 315 degrees
        if((abs(angle-pangle)>3.1416/4 && abs(angle-pangle)<7*3.1416/4))
            continue;
        end
    end
    
    path(points,:) = [center,angle];	% store the center and the angle in path list
    points = points + 1;				% increment the no.of points
    
    % Drawing a ray to show the orientation of the bot
	% The ray starts from the center and is 50px long with a '+' to indicate the front side of the bot
    x = 0;
    y = 0;
    for k=0:50
        x = uint16(bcenter(1) + (k*cos(angle)));	% intermediate x coordinates
        y = uint16(bcenter(2) + (k*sin(angle)));	% intermediate y coordinates
        if(x~=0 && y~=0)
            img_bin(y,x) = 255;			% plot the point in the image
        end
        if(k==50 && x~=0 && y~=0)		% drawing the '+' symbol to denote the front side
            for temp=(x-5):(x+5)
                img_bin(y,temp) = 255;
            end
            for temp=(y-5):(y+5)
                img_bin(temp,x) = 255;
            end
        end
    end
	
    imshow(img_bin);	% show the B/W image
    pangle = angle;		% store previous angle value
    pcenter = center;	% store previous coordinates of back circle
    pause(0.15);
end

stop(vid);		% stop the video
delete(vid);
clear vid;

% Uncomment this part to print the points detected
%for i=1:(points-1)
%	fprintf('%d, %d, %d\n',uint16(path(i,1)),uint16(path(i,2)),int16(path(i,3)*180/3.1416));
%end

encoded_path = calculate_path(path,points-2,img_height,img_width);	% calculate the encoded path using the list of points detected
fprintf('ENCODED PATH: %s\n',encoded_path);								% print the encoded path

% Start Zigbee communication
pause(5);
fprintf('TRANSFERRING THROUGH ZIGBEE.....\n');
transfer_string(encoded_path); % function to transfer the path
fprintf('TRANSFER COMPLETE!\n');
