function FunctionPlot3()

AlgorithmName={'PSO','CLPSO','CLPSOLSBFGS2', 'HTPC', 'HSES','ACO','AMCACO','AMSACO','ACODE300','MACON12','MFACODE4',...
               'MFACODE2', 'MFACODE3', 'HTPC', 'HSES', 'EBOwithCMAR','ACO','LMACON11', 'MACONGD'};
OutputName={'PSO','CLPSO','CLPSOLLS', 'HTPC', 'HSES','ACO_\mathbb{R}','AMCACO','AMSACO', 'ACOPA_\mathbb{R}'};

a=['-*', '-p' ,'-+', '-d' ,'-s', '-o' ,'-x' ,'->','-.' ,'-h', ':*', ':p' ,':o' ,':^', ':s', ':x', ':d'];
for Dimension=[40 ]
    FEs=(50000+(Dimension-20)*2500);
for TaskIndex=[ 1]
    Result=[];
    for AlgorithmIndex=1:9
        FileName=strcat(char(AlgorithmName(AlgorithmIndex)),'Prob',int2str(TaskIndex),'Dim',int2str(Dimension),'Data.txt'); 
        FindFile=fopen(FileName, 'r');
        OResult=fscanf(FindFile,'%50f',[1,inf]);
        if AlgorithmIndex==7|| 8
           OResult=OResult(1:300001);
        end
        Result(AlgorithmIndex,:)=log10(OResult);
        Length=length(Result);
        TaskNumber=3; %(Length-1)/FEs; 
    end
    step=ceil(FEs/10);
    for i=1:TaskNumber
        figure((TaskIndex-1)*3+i)
        for AlgorithmIndex=1:9
            plot(Result(AlgorithmIndex, (i-1)*FEs+10:i*FEs+1),a(AlgorithmIndex*2-1:AlgorithmIndex*2),'MarkerIndices',1:step:FEs);
            hold on; 
        end
        xlabel('FEs');
        ylabel('Cost (log) ');
        legend('PSO','CLPSO','CLPSOLLS', 'HTPC', 'HSES','ACO','AMCACO','AMSACO','ACOPA');
        TitleName=strcat(  int2str(Dimension), 'D','\_Case',int2str(TaskIndex),'\_UGV', int2str(i));
        title(TitleName);
        FigureName=strcat('Con','D', int2str(Dimension), 'P',int2str(TaskIndex),'U',int2str(i));
        print(FigureName,'-dpng');  
    end
     
    
    fclose all;
end
end           
end
