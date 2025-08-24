clc;
%----------Encryption and Decryption Key-------------
key = fileread('endeckey.txt');

%----------Encryption and Decryption Key-------------
data = fileread('blockchainkey.txt');

%--------------------Retrieve the encrypted file using the CID

[status, ss] = system(['ipfs cat ' data]);
if status == 0
    %disp('Encrypted file retrieved from IPFS');
    %disp(ss);
    % Decrypt the retrieved file
    decryptedData = aes_decrypt(ss, key);  % Decrypt the data
    %disp('Decrypted content:');
    disp(['Decrypted text: ', decryptedData]);  % Display the original content
else
    disp('Error retrieving file from IPFS');
end