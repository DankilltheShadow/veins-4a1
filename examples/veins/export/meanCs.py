import pandas as pd
from pylab import *


file = 1
scenario = ['ScenarioLow-MeanCS', 'ScenarioAverage-MeanCS']
measure = ['PCH=15,c=14', 'PCH=15,c=15', 'PCH=15,c=16', 'PCH=10,c=9', 'PCH=10,c=10', 'PCH=10,c=11', 'PCH=8,c=7', 'PCH=8,c=8', 'PCH=8,c=9', 'PCH=5,c=4', 'PCH=5,c=5', 'PCH=5,c=6']


for nScenario in range(2):
    out_file = open(scenario[nScenario] + ".txt", "w")
    for prove in range(12):
        MeanCS = pd.read_csv('MeanCs-'+str(file)+'.csv', index_col='time', names=['time', 'mean'], dtype=float)
        MeanCS = MeanCS.fillna(0)
        file += 1
        for riga in range(89):
            m = float(MeanCS.iloc[riga])
            g = float("{0:.2f}".format(m))
            if riga < 88:
                out_file.write(str(g)+"-")
            else:
                out_file.write(str(g)+"\n")
    out_file.close()
