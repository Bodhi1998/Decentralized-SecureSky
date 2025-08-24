clc;
from_blockchain='ED0D12ED54029A6D85D21F0D8A92362C6BDBF928D173D73DE20C248E05D41F43';
key = fileread('endeckey.txt');
disp(decrypt(from_blockchain, key));