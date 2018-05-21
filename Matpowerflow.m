function [v_new,O_new, cost] = Matpowerflow(mpc, PQbus, S, q , O, v, W, b, c)
% input: set S of buses which are selected to be ON,
%        possible reactive power injection q at each bus,
%        currently On set O, 
%        initial voltages v,
%        Jacobian inverse matrix W
% output: resulting voltages v_new,
%         set O_new of buses whose capacitors are ON
mpc1=mpc;

[N,~]=size(W);
if ~exist('O')
    O=[];
end
if ~exist('S')
    S=O;
end
if ~exist('q')
    q=ones(N,1);
end

% switching ON set
A1=setdiff(S,O);
% switching OFF set
A0=setdiff(O,S);

% reactive power change at each bus
delta_q=zeros(N,1);
delta_q(A1)=q(A1);
delta_q(A0)=-q(A0);

% new set of buses which are ON
O_new=A1;

%calculate switching cost
c1=sum(b(A0));
c2=sum(c(A1));
cost=c1+c2;
%% calculate voltages change

mpc1.bus(PQbus,4)=mpc1.bus(PQbus,4)-mpc1.baseMVA*delta_q;
opt  = mpoption('OUT_ALL', 0, 'VERBOSE', 1);
result1=runpf(mpc1,opt);
v_new=result1.bus(PQbus,8);
%theta_new=result.bus(PQbus,9)*pi/180;

% delta_v=W*delta_q;
% v_new=v+delta_v;
end

