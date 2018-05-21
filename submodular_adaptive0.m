function A1_opt = submodular_adaptive0(D, lambda, W, q, b, c, O, F, v, v_ref)
% input
% centralized voltage-control (submodular)
% W: weights (Jacobian inverse matrix)
% q: possible reactive power injections at each bus
% b: cost of switching from ON to OFF
% c: cost of switching from OFF to ON
% O: set of buses initially injecting reactive power, F=Omega\O

% output
% S: set of buses to inject reactive power
[N,m]=size(v);
epsilon=0;

A0=F;
A1=O;
v_star = [];
for ii = 1:m
    v_star(:,ii)=v_ref-v(:,ii);
end
v_hat_star=zeros(N,m);
Vi_hat = zeros(N,m);
Wq = [];
Pi = [];
Ri = [];
for ii=1:m
    for k=1:N
        buff=0;
        for i=O
            buff=buff+W(k,i,ii)*q(i);
        end
        v_hat_star(k,ii)=buff+v_star(k,ii);
    end

    %
    for i=1:N
        for j=1:N
            Wq(i,j,ii)=W(i,j,ii)*q(j);
        end
    end

    for i=1:N    
        a=1:N;
        Pi{ii}{i}=a(W(i,:,ii)>=0);
        Ri{ii}{i}=a(W(i,:,ii)<0);

        buff3=0;
        for j=Ri{ii}{i}
            buff3=buff3+Wq(i,j,ii);
        end
        Vi_hat(i,ii)=v_hat_star(i,ii)-buff3;
    end
end
%%
S=[];
flag=1;
while flag==1
    flag=0;
    f0=(1-epsilon)*f_hat_expected(D, lambda, b, c, O, F, A0, A1, Vi_hat, Wq, Pi, Ri);
    A0_max=A0;
    A1_max=A1;
    i_max=[];
    for i=A0
        a0=setdiff(A0,i);
        a1=union(A1,i);
        f1=f_hat_expected(D, lambda, b, c, O, F, a0, a1, Vi_hat, Wq, Pi, Ri);
        Df=f0-f1;
        if Df>0
            f0=f1;
            A0_max=a0;
            A1_max=a1;
            i_max=i;
            flag=1;
        end   
    end
%disp(['Found a device to switch on ' num2str(i)]);
    for i=A1
        a0=union(A0,i);
        a1=setdiff(A1,i);
        f1=f_hat_expected(D, lambda, b, c, O, F, a0, a1, Vi_hat, Wq, Pi, Ri);
        Df=f0-f1;
        if Df>0 
            f0=f1;
            A0_max=a0;
            A1_max=a1;
            i_max=i;
            flag=1;
        end   
    end
disp(['Found a device to switch ' num2str(i_max)]);
A0=A0_max;
A1=A1_max;
S=[S i_max];
end 
A1_opt=A1;
end

function f_hat_expected = f_hat_expected(D, lambda, b, c, O, F, A0, A1, Vi_hat, Wq, Pi, Ri)
    m = length(D);
    f_hat_expected = 0;
    for ii = 1:m
        f_hat_expected = f_hat_expected + D(ii) * f_hat(lambda, b, c, O, F, A0, A1, Vi_hat(:,ii), Wq(:,:,ii), Pi{ii}, Ri{ii});
    end
end

function f_hat=f_hat(lambda, b, c, O, F, A0, A1, Vi_hat, Wq, Pi, Ri)
    N=length(Vi_hat);
    sum1=0;
    for ii=1:N
        % if move the definitions of Pi, Ri, v_hat to the function above, we can save calculation but need more storage 
%         Pi=1:N;
%         Pi=Pi(W(ii,:)>=0);
%         Ri=1:N;
%         Ri=Ri(W(ii,:)<0);

        PinA1=intersect(A1,Pi{ii});
        buff1=0;
        if ~isempty(PinA1)
            for j=PinA1
                buff1=buff1+Wq(ii,j);
            end
        end
        
        RinA0=intersect(Ri{ii},A0);
        buff2=0;
        if ~isempty(RinA0)
            for j=RinA0
                buff2=buff2+abs(Wq(ii,j));
            end
        end
        
%         buff3=0;
%         for j=Ri
%             buff3=buff3+W(ii,j)*q(j);
%         end
%         Vi_hat=v_hat_star(ii)-buff3;
        
        sum1=sum1+P(buff1+buff2-Vi_hat(ii));

    end
    
    A0nO=intersect(A0,O);
    sum2=sum(b(A0nO));
    
    A1nF=intersect(A1,F);
    sum3=sum(c(A1nF));

    f_hat=lambda*sum1+sum2+sum3;
end