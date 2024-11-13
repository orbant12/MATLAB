function [s]=note_gen(f,fs,Td)

t=0:1/fs:Td; 
s=sin(2*pi*f*t); 
sound(s,fs);
end