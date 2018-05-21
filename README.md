# Submodular Voltage Control for Power Systems
The provided Matlab codes implement the voltage control method proposed by the following paper on IEEE 300-bus power system:

Z. Liu, A. Clark, P. Lee, L. Bushnell, D. Kirschen and R. Poovendran, "Submodular Optimization for Voltage Control," in IEEE Transactions on Power Systems, vol. 33, no. 1, pp. 502-513, Jan. 2018. doi: 10.1109/TPWRS.2017.2691320

To run the codes, please install MATPOWER, an open-source power system simulation tool for MATLAB, via link: http://www.pserc.cornell.edu/matpower/

Simulation instructions:
1. Run main.m to start simulation. The simulation consists of three stages: 1) At steady state, no low voltage problem exists, 2) Contingency occurs after increasing overall load level (user customized input), 3) Voltage problems are fixed after submodular voltage controller selects buses to inject reactive power/switch capacitor banks.
2. Run plot_figs.m to plot voltage deviations in each stage
