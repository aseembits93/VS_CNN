function pose = deploy_matlab(net, im1, im2)  %matlab imread reads in rgb!!

%im1=imread('a.png');
%im2=imread('b.png');
%im1_data = im1(:, :, [3, 2, 1]); % permute channels from RGB to BGR
im1_data = im1;
im1_data = permute(im1_data, [2, 1, 3]); % flip width and height
im1_data = single(im1_data); % convert from uint8 to single
im1_data = imresize(im1_data, [512 384], 'bilinear'); % resize


%im2_data = im2(:, :, [3, 2, 1]); % permute channels from RGB to BGR
im2_data = im2;
im2_data = permute(im2_data, [2, 1, 3]); % flip width and height
im2_data = single(im2_data); % convert from uint8 to single
im2_data = imresize(im2_data, [512 384], 'bilinear'); % resize
ipblob={im1_data,im2_data};
output=net.forward(ipblob);
pose = [output{1}'  output{2}'];
%caffe.reset_all();
%posexyz = output['fc_pose_xyz'][0];
%poseq= output['fc_pose_wpqr'][0];
%pose=[0,0,0,0,0,0,0];
%pose(1:4)=posexyz;
%pose(4:7)=poseq;
%print pose	
end

