%%
% @file check_turn.m
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
% Function to check if rotation is starting from a specified point
% @param path The list of points in the detected path. Each entry [x,y,angle] represents the (x,y) coordinates of the center and the orientation angle
% @param first The index of starting point
% @param last The index of last point in the detected path
% @param ang The angle threshold
% @return res 1 if rotation is starting or if the path has come to an end; 0 otherwise

function res = check_turn(path,first,last,ang)
% Check for 3 consecutive points and if the angle gap is greater than threshold (5 degrees) then linear motion is starting
st = first+1;
ed = first+2;
res = 1;
if(st>=last || ed>=last) % reached the end of the list
    return;
end
for i=st:ed
    if(abs(path(i,3)-path(i+1,3))<ang ||...
            abs(path(i,3)-path(i+1,3))>((2*3.1416)-ang))
        res = 0;
        return;
    end
end