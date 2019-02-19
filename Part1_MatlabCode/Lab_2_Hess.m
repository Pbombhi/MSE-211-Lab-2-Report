function Hess = Lab_2_Hess(x,y)
%f=(x.^2+y-11).^2+(x+y.^2-7).^2;
%ddx=12*x^2 + 4*y - 42;
%ddxy=4*x + 4*y;
%ddy=12*y^2 + 4*x - 26;

 Hess = [12*x^2 + 4*y - 42,     4*x + 4*y;...
        4*x + 4*y,          12*y^2 + 4*x - 26];

end