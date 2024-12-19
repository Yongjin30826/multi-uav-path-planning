function [better, betterFit]=GreedySelection(a,aFit,b,bFit)

    if  aFit<bFit
        better=a;
        betterFit=aFit;
    else
        better=b;
        betterFit=bFit;
    end

end