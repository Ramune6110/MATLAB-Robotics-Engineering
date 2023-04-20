filename = 'test_co.slx';
L = strlength(filename);
filename(1)

extension_length = 4;
temp_str = '';
for i = 1:L - extension_length
    temp_str = append(temp_str,filename(i));
end

temp_str