function net=initnet()
addpath(genpath('/home/aseem/harit_stuff/flownet-release'));
caffe.reset_all()
caffe.set_mode_gpu();
gpu_id = 1;
caffe.set_device(gpu_id);
model_root='/home/aseem/harit_stuff/Aseem/';
model_def = [model_root,'prototxt/flownet/deploy.prototxt'];
model_weights = [model_root,'snapshots/flownet/flownet_step_e-4ssize30k_iter_75000.caffemodel'];
net = caffe.Net(model_def, model_weights, 'test');
end