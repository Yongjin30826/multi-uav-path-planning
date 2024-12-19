function X=BoundLimits(X,lower,upper)
   [D, S]=size(X);
   if length(lower)>1
      for i=1:S
           TX=X(:,i);
           TX(TX>upper)=upper(TX>upper);
           TX(TX<lower)=lower(TX<lower);
           X(:,i)=TX;
      end
   else
       X(X>upper)=upper;
       X(X<lower)=lower;
   end
end