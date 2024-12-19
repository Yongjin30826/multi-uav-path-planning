function [xopt,fopt,niter,ConvergenceData,X] = BFGS(Gbest,SingleCostFunction,TModelInfor, LSMaximumFEs,AgentIndex)

% starting point
x0=Gbest;   % (Dimension,1)
xopt=x0;

xxx=[];


fopt=SingleCostFunction(Gbest, TModelInfor,AgentIndex);
Dimension=length(Gbest);
% termination tolerance
tol = -inf;

% maximum number of allowed iterations
maxiter = LSMaximumFEs;

% minimum allowed perturbation
dxmin =  -inf;

% initialize gradient norm, optimization vector, iteration counter, perturbation
gnorm = inf; x = x0; niter = 0; dx = inf;

% define the objective function:

%f2=@(x) feval(Problem, x', ProblemIndex);
% conjugate gradient descent algorithm:
i = 0;
while and(gnorm>=tol, and(niter <= maxiter, dx >= dxmin))
    
    % calculate gradient:
    g = gradFunction(SingleCostFunction, TModelInfor, x, AgentIndex);
    gnorm = norm(g);
    % inverse Hessian approximation Hk
    H0 = eye(Dimension); 
    % take first step: by grad
    if(i == 0)
        [alpha,~] = fminbnd(@(alpha) iterbyGrad(alpha,x,g,SingleCostFunction, TModelInfor,AgentIndex),0,5e-1);
        xnew = x - alpha  *  g;
        i = i + 1;
        pk = -gradFunction(SingleCostFunction, TModelInfor, x, AgentIndex);
        Hk = H0;
    elseif(i > 0)
        gnew = gradFunction(SingleCostFunction, TModelInfor, x, AgentIndex);
        gold = gradFunction(SingleCostFunction, TModelInfor,xold,AgentIndex);
        yk = gnew - gold;
        sk = x - xold;
        rho=1/(yk'*sk);
        %Hk = (H0 - rho*sk*yk') * Hk * (H0 - rho*yk*sk') + rho*(sk*sk');
        %HK1=Hk+(yk'*yk)/(sk*yk')-(Hk*sk'*sk*Hk)/(sk*Hk*sk');
        Hk = Hk + (((1+(yk'*Hk*yk)/(sk'*yk))*(sk*sk'))-(Hk*yk*sk')-(sk*yk'*Hk))/(sk'*yk); % BFGS
        pk1 = -Hk*gnew;
        
        if(pk1'*gnew <= 0)
            [tk1,~] = fminbnd(@(tk) iterbyQuasiNewton(tk,x,pk1,SingleCostFunction, TModelInfor,AgentIndex),0,0.11);
            xnew = x + tk1  *  pk1;
            pk = pk1;
        else
            i = 0;
            xnew = x;
        end
        
    end 
    
    % check step
    if ~isfinite(xnew)
        %display(['Number of iterations: ' num2str(niter)])
        %error('x is inf or NaN')
        break;
    end

    % plot current point
    %h = plot([x(1) xnew(1)],[x(2) xnew(2)],'k.-');
    %refreshdata(h,'caller');
    %drawnow;
    %hold on;
    % update termination metrics
    niter = niter + 1;
    ynew=SingleCostFunction(xnew, TModelInfor,AgentIndex);
    [xopt,fopt]=GreedySelection(xnew,ynew,xopt,fopt);
    
    xxx=[xxx,xopt];
   
    ConvergenceData(niter)=ynew;
    dx = norm(xnew-x);
    xold = x;
    x = xnew;
      
end
xopt = xopt';

yyy=SingleCostFunction(xxx, TModelInfor,AgentIndex);
[sortedyyy, indices] = sort(yyy);
selectedX = xxx(:, indices(1:40));
selectedY = sortedyyy(1:40);
X=selectedX;

%fopt = CostFunction(xopt', ModelInfor);
%niter = niter - 1;   原始版没有删掉
ConvergenceData(niter)=fopt;


end

function f = iterbyGrad(alpha,A,B,SingleCostFunction, TModelInfor,AgentIndex)
    xnew = A - B*alpha;
    f=SingleCostFunction(xnew, TModelInfor,AgentIndex);
end

function f = iterbyQuasiNewton(tk,A,B,SingleCostFunction, TModelInfor,AgentIndex)
    xnew = A + tk*B;
    f=SingleCostFunction(xnew, TModelInfor,AgentIndex);
    %f = Rosenbrock(xnew(1),xnew(2));
end
