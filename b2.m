% Define the words
word1 = 'first';
word2 = 'second';
word3 = 'third';
word4 = 'fourth';

% Open the file for writing
fileID = fopen('output.txt', 'w');

% Write the words into the file with space separation
fprintf(fileID, '%s %s %s %s\n', word1, word2, word3, word4);

% Close the file
fclose(fileID);
