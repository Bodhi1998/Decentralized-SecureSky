clc;
tic;
[status, cmdout] = system('ipfs add endeckey.txt');
parts = strsplit(cmdout);
cid = parts{8};
[status, ss] = system(['ipfs cat ' cid]);
disp(ss);
time_operation1 = toc 