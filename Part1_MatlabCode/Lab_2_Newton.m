function [X,traj,f,k,Err] = Lab_2_Newton(x0,tol)
k = 0; ea = 1;
X = x0; traj=[];
f(1) = Lab_2_Fun(X(1),X(2));
Err=NaN;

while ea > tol
 grad=Lab_2_Grad(X(1),X(2));
 hess=Lab_2_Hess(X(1),X(2));

%  xnew(1)=X(1)-grad(1)/abs(hess);
%  xnew(2)=X(2)-grad(2)/abs(hess);
%  
 k = k+1;
 ea = norm(xnew-X); %Evaluate error
 Err = [Err ea];
 X = xnew;
 traj(k,:) = xnew; %Store solution in a vector
 f(k+1)=Lab_2_Fun(X(1),X(2));
 
 if xnew(1)>5 || xnew(1)<-5 || xnew(2)>5 || xnew(2)<-5
     text=['Gess x = ',num2str(x0(1)) ' and y = ' num2str(x0(2)),' causes function to go out of bounds'];
     disp(text)
     break;
     
 end
end