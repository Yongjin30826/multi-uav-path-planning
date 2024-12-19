% code from Mahamed G. H. Omran
function [x,f,FEs] = hanDuplicate(x,f,LB, UB, ModelInfor,FEs, AgentIndex)
	[N,D]=size(x);

    % If duplicates, modifies one at random (Gaussian distribution)
    % Note that as x is sorted by increasing f, if two positions are similar
    % they have the same f, and therefore they are necessarily consecutive

        nbDup=0;
    for n=2:N
      if f(n)>f(n-1) 
          continue;            % 如果f(n)>f(n-1)，就一直执行这个条件
      end % Just to speed up the process
      %	As f is sorted, in that case we can not have x(n,:)==x(n-1,:)
      
      if x(n,:)==x(n-1,:)  % If same positions, modifies one       如果两个粒子的位置相同，那么对其中一个粒子进行扰动
        nbDup=nbDup+1;

        for d=1:D
          sigma=2*sqrt((UB(d)-LB(d))/N);
          g= sigma*randn;
          x (n,d) = min(UB(d),max(LB(d),x(n,d)+g));
        end                                        
      %	Evaluation
       %x(n,:)
      f(n)=SingleCostFunction(x(n,:)', ModelInfor, AgentIndex); %fnceval(x(n,:), I_fno);   % 重新计算扰动后的值
      %f(n) = fEval(x(n,:) ,fun, opt_f, FuncAddr);
      FEs = FEs + 1;
      end
    end %  for n=2:N

    % If some positions have been modified, it is needed to sort f again (and x)
    if nbDup>0
      [f, Ind]=sort(f);
      x=x(Ind,:);
    end
end

