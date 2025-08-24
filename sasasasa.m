% Create a test file and try adding it to IPFS
testData = 'Hello IPFS from MATLAB!';
fileID = fopen('test_ipfs.txt', 'w');
fprintf(fileID, '%s', testData);
fclose(fileID);

% Try adding to IPFS
[status, cmdout] = system('ipfs add test_ipfs.txt');
if status == 0
    disp('Successfully added file to IPFS');
    disp(['Output: ', cmdout]);
    
    % Extract CID and try to retrieve
    parts = strsplit(cmdout);
    if length(parts) >= 2
        cid = parts{2};
        disp(['CID: ', cid]);
        
        % Try retrieving the file
        [status2, content] = system(['ipfs cat ', cid]);
        if status2 == 0
            disp('Successfully retrieved file from IPFS');
            disp(['Content: ', content]);
        else
            disp('Failed to retrieve file from IPFS');
            disp(['Error: ', content]);
        end
    end
else
    disp('Failed to add file to IPFS');
    disp(['Error: ', cmdout]);
end

% Clean up test file
delete('test_ipfs.txt');