import pandas as pd

# Leggere il file CSV
data = pd.read_csv('calibration_data_withPW.csv')
movement = data['movement_current']
full_range = data['full_range_current'].dropna().mean()

# Visualizzare le prime righe del DataFrame
print(full_range)
