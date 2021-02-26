x = 1;

for x = 1:10
x = add_one(x);
disp(x);
end




function [x] = add_one(x)
x = x + 1;
end