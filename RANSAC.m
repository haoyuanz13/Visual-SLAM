function velocity=RANSAC(error,k,cnew,c,Zinv,dt)
% error - the maximum allowed error for a point to be considered an inlier 
% k - the number of iteration
% cnew - new coodinates of the points tracked - 3*n
% c - original coordinates of the points tracked - 3*n
% Zinv - inverse of z - 1*n
% dt - time difference between when cnew and c are tracked 

A=zeros(2*size(cnew,2),6);
v=(cnew-c)/dt;
index=[];
for i=1:size(cnew,2)
    A(2*i-1:2*i,:)=[...
        -Zinv(i) 0 c(1,i)*Zinv(i) c(1,i)*c(2,i) -1-(c(1,i))^2 c(2,i);
        0 -Zinv(i) c(2,i)*Zinv(i) 1+c(2,i)^2 -c(1,i)*c(2,i) -c(1,i)];
end

n=0;
v(3,:)=[];
for i=1:k
    [p,pindex]=datasample(v',3);
    A0=zeros(6,6);
    for j=1:3
        A0(2*j-1:2*j,:)=A(2*pindex(j)-1:2*pindex(j),:);
    end
    p=p';
    v_est=A0\p(:);
    v_est=A*v_est;
    e=v(:)-v_est;
    e=reshape(e,2,size(cnew,2));
    e=e(1,:).*e(1,:)+e(2,:).*e(2,:);
    index=find(e<error);
    nprime=size(index,2);
    if nprime>=n
        ind=index;
        n=nprime;
    end
end

inlier=v(:,ind);
A_inlier=zeros(2*size(ind,2),6);
for i=1:size(ind,2)
    A_inlier(2*i-1:2*i,:)=A(2*ind(i)-1:2*ind(i),:);
end

velocity=A_inlier\inlier(:);
end