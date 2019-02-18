function gradX= Lab_2_Grad(x,y)
%f=(x.^2+y-11).^2+(x+y.^2-7).^2;
dx = 2*x + 4*x*(x^2 + y - 11) + 2*y^2 - 14;
dy = 2*y + 4*y*(y^2 + x - 7) + 2*x^2 - 22;

       gradX = [2*x + 4*x*(x^2 + y - 11) + 2*y^2 - 14;...
    2*y + 4*y*(y^2 + x - 7) + 2*x^2 - 22];
end