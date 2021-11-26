import pandas as pd

df = pd.read_excel("Sample_Data.xlsx", index_col=None, na_values=['NA'], usecols = "A:C")
#df = df.sort_values(['Induct Station', 'Shipment'], ascending=[False, True])
df1 = df[df['Induct Station']==1]
df2 = df[df['Induct Station']==2]
#df1 = df1.sort_values(by="")
for i in range(0, 5):
    print(df1.iloc[i][0], int(df1.iloc[i][1]), df1.iloc[i][2])