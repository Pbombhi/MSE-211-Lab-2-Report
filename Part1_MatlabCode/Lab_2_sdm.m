function [X,traj,f,k,Err,Path] = Lab2_sdm(x0,tol)
k = 0; ea = 1;
X = x0; traj=[];
f(1) = Lab_2_Fun(X(1),X(2));
Err=NaN;

while ea > tol
 grad=Lab_2_Grad(X(1),X(2));
 hess=Lab_2_Hess(X(1),X(2));
 
 stepsize=abs((transpose(grad)*grad)/(transpose(grad)*hess*grad));
 
 xnew(1)=X(1)-grad(1)*stepsize;
 xnew(2)=X(2)-grad(2)*stepsize;
 
 
 k = k+1;
 ea = norm(xnew-X); %Evaluate error
 Err = [Err ea];
 X = xnew;
 traj(k,:) = xnew; %Store solution in a vector
 f(k+1) =Lab_2_Fun(X(1),X(2));
 
end