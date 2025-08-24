function decrypted_text = aes_decrypt(encrypted_text, key)
    % AES Decrypt the encrypted text using a key
    % encrypted_text: the string (in hex) to be decrypted
    % key: a string key for decryption (should be 128, 192, or 256 bits)
    
    % Convert the encrypted text (hex) back to byte array
    encrypted_bytes = hex2dec(reshape(encrypted_text, 2, [])')';
    key_bytes = uint8(key);
    
    % Create AES cipher object
    aes_cipher = javax.crypto.Cipher.getInstance('AES');
    secret_key = javax.crypto.spec.SecretKeySpec(key_bytes, 'AES');
    
    % Initialize cipher to decryption mode
    aes_cipher.init(javax.crypto.Cipher.DECRYPT_MODE, secret_key);
    
    % Perform AES decryption
    decrypted_bytes = aes_cipher.doFinal(encrypted_bytes);
    
    % Convert the decrypted byte array back to readable text
    decrypted_text = char(decrypted_bytes)';
end
