function [invjcon]= pseudoinv(jcon)
invjcon=[];
for i=1:size(jcon,2)
    if round(jcon(1,i),3)==0 && round(jcon(2,i),3)==0 && round(jcon(3,i),3)==0 && round(jcon(4,i),3)==0 && round(jcon(5,i),3)==0 && round(jcon(6,i),3)==0
      invjcon(i,1)= 0;invjcon(i,2)= 0;invjcon(i,3)= 0;invjcon(i,4)= 0;invjcon(i,5)= 0;invjcon(i,6)= 0;    
    else
    invjcon= [invjcon;(inv((jcon(:,i))' * jcon(:,i))*jcon(:,i)')];
    end
end
end
