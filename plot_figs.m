%clear all
close all
%clc
%case 1
%load('opf_heavycase(Vmax=1.05,Vmin=0.94).mat')
%case 2
%load('opf_heavycase_adap(Vmax=1.05,Vmin=0.95)Lambda=0.94.mat')
%load('data3_varbase_(all=0.003).mat')
%load('data3_varbase_(part=0.003).mat')
figure;
hold on;
stem(PQbus, v0_steady-v_ref)
set(gca,'FontSize',14,'Fontname','Times new roman')
legend('Voltage deviation', 'FontSize', 14,'Fontname','Times new roman')
title('Steady-state Voltage Deviation', 'FontSize', 20,'Fontname','Times new roman')
xlabel('Bus index', 'FontSize', 20,'Fontname','Times new roman')
ylabel('Voltage deviation, p.u.', 'FontSize', 20,'Fontname','Times new roman')
ylim([-0.15,0.1])
grid on
%plot([0,300],[1,1],'r:','LineWidth',1.5)
plot([0,300],[-0.05,-0.05],'r--','LineWidth',1.5)
plot([0,300],[0.05,0.05],'r--','LineWidth',1.5)
hold off;
%%
figure;
hold on;
stem(PQbus, v0-v_ref)
set(gca,'FontSize',14,'Fontname','Times new roman')
legend('Voltage deviation', 'FontSize', 14,'Fontname','Times new roman')
title('Initial Voltage Deviation', 'FontSize', 20,'Fontname','Times new roman')
xlabel('Bus index', 'FontSize', 20,'Fontname','Times new roman')
ylabel('Voltage deviation, p.u.', 'FontSize', 20,'Fontname','Times new roman')
ylim([-0.15,0.1])
grid on
%plot([0,300],[1,1],'r:','LineWidth',1.5)
plot([0,300],[-0.05,-0.05],'r--','LineWidth',1.5)
plot([0,300],[0.05,0.05],'r--','LineWidth',1.5)
hold off;
%%
figure;
hold on;
stem(PQbus, v_adap0-v_ref)
set(gca,'FontSize',14,'Fontname','Times new roman')
legend('Voltage deviation', 'FontSize', 14,'Fontname','Times new roman')
title('Voltage Deviation After Submodular Control', 'FontSize', 20,'Fontname','Times new roman')
xlabel('Bus index', 'FontSize', 20,'Fontname','Times new roman')
ylabel('Voltage deviation, p.u.', 'FontSize', 20,'Fontname','Times new roman')
ylim([-0.15,0.1])
grid on
%plot([0,300],[1,1],'r:','LineWidth',1.5)
plot([0,300],[-0.05,-0.05],'r--','LineWidth',1.5)
plot([0,300],[0.05,0.05],'r--','LineWidth',1.5)
hold off;

% figure;
% hold on;
% stem(PQbus, v_rand-v_ref)
% set(gca,'FontSize',14,'Fontname','Times new roman')
% legend('Voltage deviation', 'FontSize', 14,'Fontname','Times new roman')
% title('Voltage deviation after sensitivity control', 'FontSize', 20,'Fontname','Times new roman')
% xlabel('Bus index', 'FontSize', 20,'Fontname','Times new roman')
% ylabel('Voltage deviation, p.u.', 'FontSize', 20,'Fontname','Times new roman')
% ylim([-0.15,0.1])
% grid on
% %plot([0,300],[1,1],'r:','LineWidth',1.5)
% plot([0,300],[-0.05,-0.05],'r--','LineWidth',1.5)
% plot([0,300],[0.05,0.05],'r--','LineWidth',1.5)
% hold off;