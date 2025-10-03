function out_dec = fun_hex2dec(dec1, dec2)

hex1 = dec2hex(dec1);
hex2 = dec2hex(dec2);
if hex1 == '0'
    hex1 = '00';
end
if length(hex1) == 1
    hex1 = ['0', hex1];
end
if hex2 == '0'
    hex2 = '00';
end
if length(hex2) == 1
    hex2 = ['0', hex2];
end
hex = [hex1, hex2];

% hex(complementary code) to dec
input_hex = hex;
input_bin = dec2bin(hex2dec(input_hex),16);%
my_code = input_bin;
nbit = length(input_bin); 
if my_code(1) == '0' % 是一个正整数
    out_dec = bin2dec(my_code);
else
    tmp = my_code(2:nbit);
    pos0 = find(tmp == '0');% find position 0
    pos1 = find(tmp == '1');
    my_code(pos0+1) = '1';
    my_code(pos1+1) = '0'; % 取反

 
    d = bin2dec( my_code(2:nbit) ) + 1; % +1
    
    d = dec2bin(d,nbit); % absolute value(dec)
    d = d(2:nbit); % base code(without mark )
    out_dec = -bin2dec(d);
end

end