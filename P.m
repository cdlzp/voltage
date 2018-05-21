function p = P(dV)
% penalty function on voltage deviation at bus i (replace norms)
G0=1;

dVhigh=0.02;
dVmax=0.05;

dVlow=-0.02;
dVmin=-0.05;

if (dVlow<=dV)&&(dV<=dVhigh)
    p=0;
%elseif (dV>=dVmax)||(dV<=dVmin)
%    p=G0;
elseif (dVhigh<dV)%&&(dV<dVmax)
    p=G0*(dV-dVhigh)/(dVmax-dVhigh);
elseif (dV<dVlow)%&&(dVmin<dV)
    p=G0*(dV-dVlow)/(dVmin-dVlow);
end

p=p^4;
end

