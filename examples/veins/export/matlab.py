import pandas as pd
import numpy as np
from pylab import *


file = 1
ris = zeros(89)
scenario = ['ScenarioLow', 'ScenarioMedium']
measure = ['PCH=15,c=14', 'PCH=15,c=15', 'PCH=15,c=16', 'PCH=10,c=9', 'PCH=10,c=10', 'PCH=10,c=11', 'PCH=8,c=7', 'PCH=8,c=8', 'PCH=8,c=9', 'PCH=5,c=4', 'PCH=5,c=5', 'PCH=5,c=6']
X = np.empty(shape=[0, 12])


for nScenario in range(2):
    #out_file = open(scenario[nScenario] + ".txt", "w")
    for prove in range(12):
        for repetition in range(10):
            UFs = pd.read_csv('UFs-'+str(file)+'.csv', index_col='time', names=['time', 'UF'], dtype=double)
            nodes = pd.read_csv('Nodes-'+str(file)+'.csv', index_col='time', names=['time', 'TotalNodes'], dtype=double)
            file += 1
            for riga in range(89):
                uf = UFs.iloc[riga]
                node = nodes.iloc[riga]
                if double(node) != 0:
                    ris[riga] += double(uf)/double(node)
                else:
                    ris[riga] = 0

        for riga in range(0, 89):
            ris[riga] /= 10
        X = np.append(X, ris)
    #out_file.write(str(riga)+"\t"+str(ris[riga])+"\n")
    #out_file.close()
    print X
