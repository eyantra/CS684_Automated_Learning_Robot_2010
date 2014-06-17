%%
% @file get_center.m
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
% Function to get the center of the coloured disk
% @param img Snapshot containing the
% @param img_height Image height
% @param img_width Image width
% @param low Lower limits of the [R,G,B] values of the disk
% @param high Higher limits of the [R,G,B] values of the disk
% @param old The B/W image on which the circle is to be shown
% @return center The [x,y] coordinates of the center of the disk
% @return new The new B/W image showing the disk

function [center,new] = get_center(img,img_height,img_width,low,high,old)
% These lists store the first and the last coordinates at which a pixel which matches the given colour range is found
xrange = [0,0];	% [xmin,xmax]
yrange = [0,0];	% [ymin,ymax]
new = old;		% start with the old B/W image

for i=1:img_height
	for j=1:img_width
		if((img(i,j,1)>=low(1) && img(i,j,1)<=high(1)) &&...
				(img(i,j,2)>=low(2) && img(i,j,2)<=high(2)) &&...
				(img(i,j,3)>=low(3) && img(i,j,3)<=high(3)))	% this pixel matches the given colour range
            new(i,j) = 255;						% plot it on the image
            if(xrange(1)==0 || j<xrange(1))
                xrange(1)=j;					% new xmin
            end
            if(xrange(2)==0 || j>xrange(2))
                xrange(2)=j;					% new xmax
            end
            if(yrange(1)==0 || i<yrange(1))
                yrange(1)=i;					% new ymin
            end
            if(yrange(2)==0 || i>yrange(2))
                yrange(2)=i;					% new ymax
            end
        end
	end
end

% We now take the average of x and y coordinates of all the matching pixels. This gives the center of the object 
xsum = 0;		% sum of x coordinates
ysum = 0;		% sum of y coordinates
no_points = 0;	% no. of matching pixels found

% We just need to loop in the range of [ymin,ymax] and [xmin,xmax]
for i=yrange(1):yrange(2)
    for j=xrange(1):xrange(2)
        if(i>0 && j>0 && new(i,j)==255)
            xsum = xsum + j;
            ysum = ysum + i;
            no_points = no_points + 1;
        end
    end
end

if(no_points>0)
    center = [(xsum/no_points), (ysum/no_points)];	% take the average of x and y coordinates
else
    center = [0,0];
end
