clc
close all
clear all

%Define call function numerically
[x1,y1] = meshgrid(-5:0.1:5,-5:0.1:5);
f = Lab_2_Fun(x1,y1);

% Plot surface
% surface = figure; figure(surface);
% surf(x1,y1,f); shading interp;

%Plot contour 
contour_graph = figure; figure(contour_graph);
contour(x1,y1,f,100); hold on;
%--------------------------------------------------------------------------
%GUESSES
%--------------------------------------------------------------------------
guess_1=[0;0];
guess_2=[-1;0];
guess_3=[0;-1];
guess_4=[-1;-1];
%--------------------------------------------------------------------------
%STEEPEST DECENT
%--------------------------------------------------------------------------
[X,traj,Z,k,Err] = Lab_2_sdm(guess_1,10E-4); %SDM w/ guess 1
plot(X(1), X(2),'-go')
plot(traj(:,1),traj(:,2),'green+')

Estimated_Point_Gess1=traj(1:10:end,:);     %Outputting data to graph
Function_Eval_Guess1=transpose(Z(:,1:10:end));
Error_Guess1=transpose(Err(:,1:10:end));
for i=1:1:length(Estimated_Point_Gess1)
    Iteration_Guess1(i,:)=1+10*(i-1);
end
Guess1_Output=table(Iteration_Guess1,Estimated_Point_Gess1,...
    Function_Eval_Guess1,Error_Guess1)
Guess1_Output_Final=table(k,traj(end,:),Z(end),Err(end))

[X,traj,Z,k,Err] = Lab_2_sdm(guess_2,10E-4); %SDM w/ guess 2
plot(X(1), X(2),'-bo')
plot(traj(:,1),traj(:,2),'blue+')

Estimated_Point_Gess2=traj(1:10:end,:);     %Outputting data to graph
Function_Eval_Guess2=transpose(Z(:,1:10:end));
Error_Guess2=transpose(Err(:,1:10:end));
for i=1:1:length(Estimated_Point_Gess2)
    Iteration_Guess2(i,:)=1+10*(i-1);
end
Guess2_Output=table(Iteration_Guess2,Estimated_Point_Gess2,...
    Function_Eval_Guess2,Error_Guess2)
Guess2_Output_Final=table(k,traj(end,:),Z(end),Err(end))

[X,traj,Z,k,Err] = Lab_2_sdm(guess_3,10E-4); %SDM w/ guess 3
plot(X(1), X(2),'-ro')
plot(traj(:,1),traj(:,2),'red+')

Estimated_Point_Guess3=traj(1:10:end,:);     %Outputting data to graph
Function_Eval_Guess3=transpose(Z(:,1:10:end));
Error_Guess3=transpose(Err(:,1:10:end));
for i=1:1:length(Estimated_Point_Guess3)
    Iteration_Guess3(i,:)=1+10*(i-1);
end
Guess3_Output=table(Iteration_Guess3,Estimated_Point_Guess3,...
    Function_Eval_Guess3,Error_Guess3)
Guess3_Output_Final=table(k,traj(end,:),Z(end),Err(end))

[X,traj,Z,k,Err] = Lab_2_sdm(guess_4,10E-4); %SDM w/ guess 4
plot(X(1), X(2),'-mo')
plot(traj(:,1),traj(:,2),'magenta+')

Estimated_Point_Guess4=traj(1:10:end,:);     %Outputting data to graph
Function_Eval_Guess4=transpose(Z(:,1:10:end));
Error_Guess4=transpose(Err(:,1:10:end));
for i=1:1:length(Estimated_Point_Guess4)
    Iteration_Guess4(i,:)=1+10*(i-1);
end
Guess4_Output=table(Iteration_Guess4,Estimated_Point_Guess4,...
    Function_Eval_Guess4,Error_Guess4)
Guess4_Output_Final=table(k,traj(end,:),Z(end),Err(end))
%--------------------------------------------------------------------------
%Newtons Method
%--------------------------------------------------------------------------
% [X,traj,Z,k,Err] = Lab_2_Newton(guess_1,10E-4);
% plot(X(1), X(2),'-go')
% plot(traj(:,1),traj(:,2),'green+')
% 
% [X,traj,Z,k,Err] = Lab_2_Newton(guess_2,10E-4);
% plot(X(1), X(2),'-bo')
% plot(traj(:,1),traj(:,2),'blue+')
% 
% [X,traj,Z,k,Err] = Lab_2_Newton(guess_3,10E-4);
% plot(X(1), X(2),'-yo')
% plot(traj(:,1),traj(:,2),'yellow+')
% 
% [X,traj,Z,k,Err] = Lab_2_Newton(guess_4,10E-4);
% plot(X(1), X(2),'-mo')
% plot(traj(:,1),traj(:,2),'magenta+')

%--------------------------------------------------------------------------
%Matlab Funtion
%--------------------------------------------------------------------------
% fun=@(x,y)(x.^2+y-11).^2+(x+y.^2-7).^2;
% fv=@(v) fun(v(1),v(2));
% 
% x0=[0; 0]; %Initial Guess
% options = optimoptions('fminunc','GradObj','on','Algorithm','trust-region');
% [X,FVAL,EXITFLAG,OUTPUT] = fminunc(fv,x0)
% plot(X(1), X(2),'-go')
% 
% x0=[-1; 0]; %Initial Guess 
% options = optimoptions('fminunc','GradObj','on','Algorithm','trust-region');
% [X,FVAL,EXITFLAG,OUTPUT] = fminunc(fv,x0)
% plot(X(1), X(2),'-bo')
% 
% x0=[0; -1]; %Initial Guess 
% options = optimoptions('fminunc','GradObj','on','Algorithm','trust-region');
% [X,FVAL,EXITFLAG,OUTPUT] = fminunc(fv,x0)
% plot(X(1), X(2),'-ro')
% 
% x0=[-1; -1]; %Initial Guess
% options = optimoptions('fminunc','GradObj','on','Algorithm','trust-region');
% [X,FVAL,EXITFLAG,OUTPUT] = fminunc(fv,x0)
% plot(X(1), X(2),'-mo')