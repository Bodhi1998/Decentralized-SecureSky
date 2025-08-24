function encrypted_text = aes_encrypt(plain_text, key)
    % AES Encrypt the plain text using a key
    % plain_text: the string to be encrypted
    % key: a string key for encryption (should be 128, 192, or 256 bits)
    
    % Convert the plain text and key to uint8 (byte arrays)
    plain_text_bytes = uint8(plain_text);
    key_bytes = uint8(key);
    
    % Create AES cipher object (128-bit AES in this case)
    aes_cipher = javax.crypto.Cipher.getInstance('AES');
    secret_key = javax.crypto.spec.SecretKeySpec(key_bytes, 'AES');
    
    % Initialize cipher to encryption mode
    aes_cipher.init(javax.crypto.Cipher.ENCRYPT_MODE, secret_key);
    
    % Perform AES encryption
    encrypted_bytes = aes_cipher.doFinal(plain_text_bytes);
    
    % Convert the encrypted byte array to a readable hexadecimal string
    encrypted_text = dec2hex(encrypted_bytes)';
    encrypted_text = encrypted_text(:)';  % Convert to a row vector
end
