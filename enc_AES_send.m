clc;
%----------Encryption and Decryption Key-------------
fileID = fopen('endeckey.txt', 'r');
key = fscanf(fileID, '%s');
fclose(fileID);
fprintf('%s', 'Encode Decode Key - ');
disp(key);


%----------Remote ID-------------
data = fileread('data.txt');
fprintf('%s', 'Remote id - ');
disp(data);

%----------------Encrypt data----------
encryptedData = aes_encrypt(data, key);

% Save the encrypted data to a file
fid = fopen('data_encrypted.txt', 'w');
fwrite(fid, encryptedData);
fclose(fid);

% Add the encrypted file to IPFS and capture the output
[status, cmdout] = system('ipfs add data_encrypted.txt');

if status == 0
    disp('Encrypted file added to IPFS');
    %disp(cmdout);  % Display the output from the command
    % Extract the CID from the output
    parts = strsplit(cmdout);
    cid = parts{8};  % The CID is the second element
    disp(['CID: ', cid]);  % Display the CID
    fid = fopen('blockchainkey.txt', 'w');
    fwrite(fid, cid);
    fclose(fid);
    
else
    disp('Error adding file to IPFS');
end