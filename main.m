%function [f_0,f_sub,t_sub,t_rand]=main_lambda(lambda)
clear all
lambda=1;
% [mpc,branch] = case300;
mpc=loadcase('case300');
mpc_mod=loadcase('case300mod');
opt  = mpoption('OUT_ALL', 0, 'VERBOSE', 1);
%initial operating point
pq=find(mpc.bus(:,2)==1);
%pq=pq(randi(231,50,1));
%pqload=intersect(find(mpc.bus(:,4)>0),pq);
%mpc.bus(pqload,4)=mpc.bus(pqload,4)+randi(15,length(pqload),1);
%mpc.bus([202,31,40],4)=mpc.bus([202,31,40],4)+1;
%mpc.bus(pq(randsample(length(pq),10)),4)=mpc.bus(pq(randsample(length(pq),10)),4)+5;
%mpc.bus(pq,4)=mpc.bus(pq,4)*1.2;
%mpc_mod.bus(pq,4)=mpc_mod.bus(pq,4)*1.2;

% Steady Operating Point
caseN=3;
if caseN==3
    %Lambda_star=0.2;
    mpc.bus(:,12)=1.05;
    mpc.bus(:,13)=0.95;
    mpc_mod.bus(:,12)=1.05;
    mpc_mod.bus(:,13)=0.95;
    
    mpc.bus(pq,3:4)=mpc.bus(pq,3:4).*0.89;
    mpc_mod.bus(pq,3:4)=mpc_mod.bus(pq,3:4).*0.89;
    
    result_opf=runopf(mpc,opt);
    result_opf_mod=runopf(mpc_mod,opt);
    
    v_steady = result_opf.bus(:,8);
    theta_steady = result_opf.bus(:,9)*pi/180;
    branch=mpc.branch;
    
    N=length(v_steady); %number of buses

    PQbus=find(result_opf.bus(:,2)==1); %indices of PQ buses
    % PVbus=find(result.bus(:,2)==2); %indices of PV buses
    % PQload=intersect(find(result.bus(:,4)>0),PQbus); %positive load bus
    PVSbus=setdiff(1:N,PQbus); %PV buses and slack bus
    
    n=length(PQbus); %number of PQ buses
    
    
    % desired voltages
    v_ref=ones(n,1);
    % set of buses whose capacitors are currently ON
    O=[];%1:n;
    %r=randperm(n);
    %O=r(1:50);
    % set of buses whose capacitors are currently OFF
    F=setdiff(1:n,O);

    % cost of switching from ON to OFF
    b=ones(n,1);%b=rand(n,1);
    % cost of switching from OFF to ON
    c=ones(n,1);%c=rand(n,1);
    
    v0_steady=v_steady(PQbus);
    theta0_steady=theta_steady(PQbus);
    
    
    % possible reactive power injection q at each PQ bus
    %q=0.2*v.^2;
    %q(266:300)=0.1*q(266:300);
    q=0.003*(v_steady.*mpc.bus(:,10)).^2/100;
    q(266:300)=10*q(266:300);

    q(PVSbus)=[];

    % slackbus=find(result.bus(:,2)==3); %slack bus
    % PQbus=setdiff(1:N,slackbus);
    
    %Print initial operating point information
    disp(' ')
    % disp(['v0_steady<0.94: ' num2str(length(find(v0_steady<0.94))) ' ; v0_steady>1.06: ' num2str(length(find(v0_steady>1.06)))])
    disp(['At steady state: ' num2str(length(find(v0_steady<0.95))) ' buses have voltage below 0.95; ' num2str(length(find(v0_steady>1.05))) ' buses have voltage above 1.05'])
    
end
%%

% Contingencies
v0 = [];
W = [];
results = [];
f_0 = [];
ii = 0;

disp(' ')
alpha_range = input('Increase overall load level by ... (Input a ratio between 1.04 - 1.08): ');
%alpha_range = 1.04;%[1.06:0.02:1.08]; % contingency space
m = length(alpha_range); % number of contingencies
for alpha = alpha_range
    % alpha=1.08;
    ii = ii + 1;
    
    result_alpha = result_opf;
    result_alpha_mod = result_opf_mod;
    
    result_alpha.bus(pq,3:4)=result_alpha.bus(pq,3:4).*alpha;
    result_alpha_mod.bus(pq,3:4)=result_alpha_mod.bus(pq,3:4).*alpha;

    result=runpf(result_alpha,opt);
    result_mod=runpf(result_alpha_mod,opt);
    % namebus=result.bus(:,1);

    results = [results, result];
    
    v=result.bus(:,8);
    % theta=result.bus(:,9)*pi/180;

    v0_alpha = v(PQbus);
    v0 = [v0, v0_alpha];
    
    % theta0=theta(PQbus);

    % Print contingency information
    % disp(['alpha: ' num2str(alpha)])
    % disp(['v0<0.94: ' num2str(length(find(v0_alpha<0.94))) ' ; v0>1.06: ' num2str(length(find(v0_alpha>1.06)))])
    %disp(['v0<0.95: ' num2str(length(find(v0_alpha<0.95))) ' ; v0>1.05: ' num2str(length(find(v0_alpha>1.05)))])
    disp(' ')
    disp(['Contingency occurs: ' num2str(length(find(v0_alpha<0.95))) ' buses have voltage below 0.95; ' num2str(length(find(v0_alpha>1.05))) ' buses have voltage above 1.05'])

    % Calculate Jacobian matrix
    J_full=makeJac(result_mod);
    W_full=inv(J_full);
    W(:,:,ii)=W_full(N:end,N:end);

    % %
    % bus_i=branch(:,1);
    % bus_j=branch(:,2);
    % r=branch(:,3);
    % x=branch(:,4);
    % 
    % 
    % % admittance g-jbx=1/(r+jx)
    % admittance=1./(r+1j.*x);
    % g=real(admittance);
    % bx=-imag(admittance);
    % %bx=1./x;
    % %
    % % conductance matrix
    % G=zeros(N,N);
    % % susceptance matrix
    % B=zeros(N,N);
    % %When bus name is not 1:N
    % for k=1:length(bus_i)
    %     bus_i_ind=find(namebus==bus_i(k));
    %     bus_j_ind=find(namebus==bus_j(k));
    %     G(bus_i_ind,bus_j_ind)=g(k);
    %     B(bus_i_ind,bus_j_ind)=bx(k);
    %     % symmetry
    %     G(bus_j_ind,bus_i_ind)=g(k);
    %     B(bus_j_ind,bus_i_ind)=bx(k);
    %     % Note that B(i,i)==0;
    % end




    % Jacobian (no resistance) J=Jqv-Jqt/(Jpt)*Jpv
    % J=zeros(N,N);
    % Jpt=zeros(N,N);
    % Jpv=zeros(N,N);
    % Jqt=zeros(N,N);
    % for i=1:N
    %     for j=1:N
    %         if i==j
    % %             buffj=0;
    % %             for k=1:N
    % %                 buffj=buffj+(2*v(i)*B(i,k)-v(k)*B(i,k)*cos((theta(i)-theta(k))));
    % %             end
    % %             J(i,i)=buffj;
    %             J(i,i)=sum(2*v(i)*B(i,:)-v(:)'.*B(i,:).*cos(theta(i)-theta'));
    %             Jpt(i,i)=v(i)*sum(v(:)'.*B(i,:).*cos(theta(i)-theta'));
    %             Jpv(i,i)=sum(v(:)'.*B(i,:).*sin(theta(i)-theta'));
    %             Jqt(i,i)=v(i)*sum(v(:)'.*B(i,:).*sin(theta(i)-theta'));
    %         else
    %             J(i,j)=-v(i)*B(i,j)*cos((theta(i)-theta(j)));
    %             Jpt(i,j)=-v(i)*v(j)*B(i,j)*cos((theta(i)-theta(j)));
    %             Jpv(i,j)=v(i)*B(i,j)*sin((theta(i)-theta(j)));
    %             Jqt(i,j)=-v(i)*v(j)*B(i,j)*sin((theta(i)-theta(j)));
    %         end
    %     end
    % end

    % Jacobian (with resistance) J=Jqv-Jqt/(Jpt)*Jpv
    % J=zeros(N,N);
    % Jpt=zeros(N,N);
    % Jpv=zeros(N,N);
    % Jqt=zeros(N,N);
    % for i=1:N
    %     for j=1:N
    %         if i==j
    % %             buffj=0;
    % %             for k=1:N
    % %                 buffj=buffj+(2*v(i)*B(i,k)-v(k)*B(i,k)*cos((theta(i)-theta(k))));
    % %             end
    % %             J(i,i)=buffj;
    %             J(i,i)=sum(2*v(i)*B(i,:)+v(:)'.*(G(i,:).*sin(theta(i)-theta')-B(i,:).*cos(theta(i)-theta')));
    %             Jpt(i,i)=v(i)*sum(v(:)'.*(-G(i,:).*sin(theta(i)-theta')+B(i,:).*cos(theta(i)-theta')));
    %             Jpv(i,i)=sum(-2*v(i)*G(i,:)+v(:)'.*(G(i,:).*cos(theta(i)-theta')+B(i,:).*sin(theta(i)-theta')));
    %             Jqt(i,i)=v(i)*sum(v(:)'.*(G(i,:).*cos(theta(i)-theta')+B(i,:).*sin(theta(i)-theta')));
    %         else
    %             J(i,j)=v(i)*(G(i,j)*sin((theta(i)-theta(j)))-B(i,j)*cos((theta(i)-theta(j))));
    %             Jpt(i,j)=v(i)*v(j)*(G(i,j)*sin((theta(i)-theta(j)))-B(i,j)*cos((theta(i)-theta(j))));
    %             Jpv(i,j)=v(i)*(G(i,j)*cos((theta(i)-theta(j)))+B(i,j)*sin((theta(i)-theta(j))));
    %             Jqt(i,j)=-v(i)*v(j)*(G(i,j)*cos((theta(i)-theta(j)))+B(i,j)*sin((theta(i)-theta(j))));
    %         end
    %     end
    % end
    % 
    % J(PVSbus,:)=[];
    % J(:,PVSbus)=[];
    % Jpt(slackbus,:)=[];
    % Jpt(:,slackbus)=[];
    % Jqt(PVSbus,:)=[];
    % Jqt(:,slackbus)=[];
    % Jpv(slackbus,:)=[];
    % Jpv(:,PVSbus)=[];
    % Jacobian inverse
    % J_full=[Jpt,Jpv;Jqt,J];






    % Contingency voltage deviation
        buff0=0;
        for i=1:n
            buff0=buff0+P(v0_alpha(i)-v_ref(i));
        end
    initial_voltage_deviation=buff0; % initial voltage deviation
    f_0_alpha=lambda*buff0; % initial total cost
    disp(['initial deviation: ' num2str(f_0_alpha)])
    disp(' ')
    f_0 = [f_0, f_0_alpha];

end

% Contingency Weights / Distribution
D = (1/m) * ones(1,m); % uniform distribution

f_0_expected = sum(f_0.*D);
%disp(['Expected initial deviation: ' num2str(f_0_expected)])


%% submodular approach
disp('Submodular searching:')
% measure running time
%tic;
S_adap0 = submodular_adaptive0(D, lambda, W, q, b, c, O, F, v0, v_ref);
%t_adap0=toc;
%
% Expected cost
f_sub = [];
for ii = 1:m
    [v_adap0, O_adap0, switch_cost_adap0] = Matpowerflow(results(ii), PQbus, S_adap0, q, O, v0(:,ii), W(:,:,ii), b, c);
    %[v_adap0, O_adap0, switch_cost_adap0] = powerflow(result, S_adap0, q, O, v0, W, b, c);
    buff11=0;
    for i=1:n
        buff11=buff11+P(v_adap0(i)-v_ref(i));
    end
    adap0_voltage_deviation=buff11; % voltage deviation after submodular switch   
    f_adap0=lambda*buff11+switch_cost_adap0; % total cost after submodular switch
    
    disp(['Contingency: ' num2str(ii)])
    disp(['Deviation after submodular control: ' num2str(adap0_voltage_deviation) ';  switching cost: ' num2str(switch_cost_adap0)])
    
    f_sub = [f_sub, f_adap0];  
end

f_sub_expected = sum(f_sub.*D);
%disp(['Expected submodular deviation: ' num2str(f_sub_expected)])
%% submodular approach
% disp('Submodular searching:')
% % measure running time
% tic;
% S_sub = submodular_sub(lambda, W, q, b, c, O, F, v0, v_ref);
% t_sub=toc;
% %%
% [v_sub, O_sub, switch_cost_sub] = Matpowerflow(result, PQbus, S_sub, q, O, v0, W, b, c);
% %[v_sub, O_sub, switch_cost_sub] = powerflow(result, S_sub, q, O, v0, W, b, c);
%     buff1=0;
%     for i=1:n
%         buff1=buff1+P(v_sub(i)-v_ref(i));
%     end
% submodular_voltage_deviation=buff1; % voltage deviation after submodular switch
% disp(['voltage deviation after submodular switch: ' num2str(submodular_voltage_deviation) ';  switching cost: ' num2str(switch_cost_sub)])
%     
%     f_sub=lambda*buff1+switch_cost_sub % total cost after submodular switch
%% sensitivity approach
% disp('Sensitivity searching:')
% Lambda=zeros(n,n); % sensitivity
% for k=1:n
%     for i=1:n
%         Lambda(k,i)=min(abs(W(k,i)./W(k,:)));
%     end
%     
%     Ak{k}=find(Lambda(k,:)>Lambda_star); %set of buses sensitive to injection k
% end
% ind=find(v0<v_ref-0.05);
% %ind=union(find(v0<v_ref-0.05),find(v0>v_ref+0.05));
%%
% measure running time
% tic;
% S_rand = sensitivity_algorithm(result, ind, Ak, lambda, W, q, b, c, O, F, v0, v_ref, f_0);
% t_rand=toc;
%%
% [v_rand, O_rand, switch_cost_rand] = Matpowerflow(result, PQbus, S_rand, q, O, v0, W, b, c);
% %[v_rand, O_rand, switch_cost_rand] = powerflow(result, S_rand, q, O, v0, W, b, c); 
% 
% buff2=0;
%     for i=1:n
%         buff2=buff2+P(v_rand(i)-v_ref(i));
%     end
%     random_voltage_deviation=buff2; % voltage deviation after random searching switch
% disp(['voltage deviation after sensitivity searching switch: ' num2str(random_voltage_deviation) ';  switching cost: ' num2str(switch_cost_rand)])
% 
%     f_rand=lambda*buff2+switch_cost_rand % total cost after random searching switch
%%
% display results
% disp(['initial total cost: ' num2str(f_0)])
% disp(['Adaptive submodular total cost: ' num2str(f_adap)])
% disp(['nonAdaptive0 submodular total cost: ' num2str(f_adap0)])
%disp(['submodular total cost: ' num2str(f_sub)])
% disp(['random searching total cost: ' num2str(f_rand)])

%end

