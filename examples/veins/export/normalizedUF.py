import pandas as pd
from pylab import *


file = 1
scenario = ['ScenarioLow', 'ScenarioAverage']
measure = ['PCH=15,c=14', 'PCH=15,c=15', 'PCH=15,c=16', 'PCH=10,c=9', 'PCH=10,c=10', 'PCH=10,c=11', 'PCH=8,c=7', 'PCH=8,c=8', 'PCH=8,c=9', 'PCH=5,c=4', 'PCH=5,c=5', 'PCH=5,c=6']


for nScenario in range(2):
    out_file = open(scenario[nScenario] + ".txt", "w")
    for prove in range(12):
        ris = zeros(89)
        for repetition in range(10):
            UFs = pd.read_csv('UFs-'+str(file)+'.csv', index_col='time', names=['time', 'UF'], dtype=float)
            nodes = pd.read_csv('Nodes-'+str(file)+'.csv', index_col='time', names=['time', 'TotalNodes'], dtype=float)
            file += 1
            for riga in range(89):
                uf = UFs.iloc[riga]
                node = nodes.iloc[riga]
                if float(node) != 0:
                    ris[riga] += float(uf)/float(node)
                else:
                    ris[riga] = 0
        for riga in range(0, 89):
            ris[riga] /= 10
            g = float("{0:.2f}".format(ris[riga]))
            if riga < 88:
                out_file.write(str(g)+"-")
            else:
                out_file.write(str(g)+"\n")
    out_file.close()
