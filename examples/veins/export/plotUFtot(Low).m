type ScenarioLow.txt

A = zeros(12,89);
A = importdata('ScenarioLow.txt','-');
[r,c] = size(A);

title('Scenario Low Traffic - normalized UF total');
%plot([1:r], A, 'LineWidth',2);
hold;
plot([1:c], A, 'LineWidth',2);
xlabel('Time');
ylabel('Utility Function value');
legend('P_{CH} = 1/5 (c=4)', 'P_{CH} = 1/5 (c=5)', 'P_{CH} = 1/5 (c=6)', 'P_{CH} = 1/8 (c=7)', 'P_{CH} = 1/8 (c=8)', 'P_{CH} = 1/8 (c=9)', 'P_{CH} = 1/10 (c=9)', 'P_{CH} = 1/10 (c=10)', 'P_{CH} = 1/10 (c=11)', 'P_{CH} = 1/15 (c=14)', 'P_{CH} = 1/15 (c=15)', 'P_{CH} = 1/15 (c=15)','Location', 'Best');
grid on;
export_fig ScenarioLow_UF.pdf -transparent;

