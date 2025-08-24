% Open the file for reading
fileID = fopen('output.txt', 'r');

% Read the line from the file
line = fgetl(fileID);

% Split the line into individual words and store them in a cell array
wordsArray = strsplit(line, ' ');

% Close the file
fclose(fileID);

% Store each element in a separate variable
word1 = wordsArray{1};
word2 = wordsArray{2};
word3 = wordsArray{3};
word4 = wordsArray{4};

% Display the variables
disp(['Word 1: ', word1]);
disp(['Word 2: ', word2]);
disp(['Word 3: ', word3]);
disp(['Word 4: ', word4]);
