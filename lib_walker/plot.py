import matplotlib.pyplot as plt
import pandas as pd
from pathlib import Path
import os
if __name__ == "__main__":
    ax = plt.gca()
    file_path = os.getcwd()
    folder = os.listdir(Path(file_path).parent)
    for direc in folder:
        if direc == 'Data_Result':
            data_folder = Path(file_path).parent.joinpath(direc)
    data_list = os.listdir(data_folder)
    print(data_list)
    walker_file = pd.read_csv('C:/Users/kv4771/weichih/active_walker/Data_Result/walker_data_Sep_08_20_29_40.csv',
                              sep='\t')
    human_file = pd.read_csv('C:/Users/kv4771/weichih/active_walker/Data_Result'
                             '/human_data_walker_frame_Sep_08_20_29_40.csv',
                             sep='\t')
    """
    walker_df = pd.DataFrame(walker_file)
    human_df = pd.DataFrame(human_file)
    human_df.plot(kind='line', x='time', y='theta', color='red', ax=ax)
    walker_df.plot(kind='line', x='time', y='theta', color='blue', ax=ax)
    plt.show()
    """

