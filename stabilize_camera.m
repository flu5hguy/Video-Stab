% Digital Video Stabilization and Rolling Shutter Correction using Gyroscopes
% Copyright (C) 2011 Alexandre Karpenko
% 
% This program is free software: you can redistribute it and/or modify
% it under the terms of the GNU General Public License as published by
% the Free Software Foundation, either version 3 of the License, or
% any later version.
% 
% This program is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% GNU General Public License for more details.
% 
% You should have received a copy of the GNU General Public License
% along with this program.  If not, see <http://www.gnu.org/licenses/>.

%% extract data from captured video & gyro
clear all;

video_file = 'dolly2';

[gyro frame_time] = import_video_data(video_file);
run ../vlfeat-0.9.9/toolbox/vl_setup.m

warning('off', 'Images:initSize:adjustingMag');

%% meshwarp
clear meshwarpmex

% Loading Camera Parameters 
load(['mat/camera_param_shake2.mat'], 'cam_param');

fl = cam_param(1);
td = cam_param(2);
ts = cam_param(3);


g = gyro(:,1:3);

sigma2 = 4000;
% Note that the convlution in the time domain is translated to product in the frequency domain
gauss = exp(-(-120:120).^2 / sigma2);
gauss = gauss ./ sum(gauss);
% 'same' option of conv function means the output will be the same size as the input
g(:,1) = g(:,1) - conv(gyro(:,1), gauss, 'same');
g(:,2) = g(:,2) - conv(gyro(:,2), gauss, 'same');
g(:,3) = g(:,3) - conv(gyro(:,3), gauss, 'same');
dgt = diff(gyro(:,4));
theta = ((g(1:end-1,:) + g(2:end,:)) / 2) .* dgt(:,[1 1 1]);
theta = [0 0 0; cumsum(theta, 1)];
dth = diff(interp1(gyro(:,4) - td, theta, frame_time, 'linear', 'extrap'));

theta = ((gyro(1:end-1,1:3) + gyro(2:end,1:3)) / 2) .* dgt(:,[1 1 1]);
% first orientation is 0,0,0 degrees or radians
theta = [0 0 0; cumsum(theta, 1)];

% read corresponding movie
xyloObj = mmreader(['data/' video_file '.mov']);
display(xyloObj);
 
num_frames = xyloObj.NumberOfFrames;
vid_height = xyloObj.Height;
vid_width = xyloObj.Width;

crop_amount = 50;

frame = read(xyloObj, 1);
outvid = VideoWriter(['mat/' video_file '_meshwarp_large.avi']);
open(outvid);
writeVideo(outvid, cropim(frame, crop_amount, crop_amount));

% Read one frame at a time.
for f = 1:num_frames-1
    frame = read(xyloObj, f+1);
    stabilized = meshwarp(frame, dth(f,:), theta, gyro(:,4), frame_time(f+1), td, ts, fl);

    writeVideo(outvid, cropim(stabilized, crop_amount, crop_amount));
    display([num2str(f+1) ' of ' num2str(num_frames)]);
end

close(outvid);