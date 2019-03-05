function [newSamples, newWeight] = resample(samples, weight)
n=length(weight);
wc=cumsum(weight);
newSamples=zeros(size(samples));
newWeight=zeros(size(weight));
r=rand/n;
j=1;
for i=1:n
    u=r+(i-1)/n;
    while u > wc(j)
        j=j+1;
    end
    newSamples(:,i)=samples(:,j);
    newWeight(i)=1/n;
end
% do some stuff here
