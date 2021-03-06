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
            total_data_folder = Path(file_path).parent.joinpath(direc).joinpath("test")
    total_data_list = os.listdir(total_data_folder)
    # print(total_data_list)
    for total_data in total_data_list:
        if Path(total_data_folder).joinpath(total_data).is_dir():
            data_folder = Path(total_data_folder).joinpath(total_data)
            # print(Path(total_data_folder).joinpath(total_data), "is a folder!!")
            data_list = os.listdir(data_folder)
            for data in data_list:
                if 'human' in data:
                    human_file = pd.read_csv(Path(data_folder).joinpath(data), sep=',')
                elif 'walker' in data:
                    walker_file = pd.read_csv(Path(data_folder).joinpath(data), sep=',')
                else:
                    print("NOT the human or walker file")
        else:
            if 'human' in total_data:
                old_human_file = pd.read_csv(Path(total_data_folder).joinpath(total_data), sep=',')
            elif 'walker' in total_data:
                old_walker_file = pd.read_csv(Path(total_data_folder).joinpath(total_data), sep=',')
            else:
                print("NOT the human or walker file")
            # print(Path(total_data_folder).joinpath(total_data), "is a file!!")
    walker_df = pd.DataFrame(walker_file)
    human_df = pd.DataFrame(human_file)
    print(human_df)
    #human_df.plot(kind='line', x='time', y='v', color='red', ax=ax)
    #human_df.plot(kind='line', x='time', y='v', color='black', ax=ax)
    walker_df.plot(kind='line', x='time', y='v', color='blue', ax=ax)
    walker_df.plot(kind='line', x='time', y='rel_dist', color='red', ax=ax)

    plt.show()
    """
    walker_file = pd.read_csv('C:/Users/kv4771/weichih/active_walker/Data_Result/walk_complex_path/walker_data.csv',
                              sep=',')
    human_file = pd.read_csv('C:/Users/kv4771/weichih/active_walker/Data_Result/walk_complex_path/human_data.csv',
                             sep=',')
    
    walker_df = pd.DataFrame(walker_file)
    human_df = pd.DataFrame(human_file)
    # human_df.plot(kind='line', x='time', y='rel_dist', color='red', ax=ax)
    walker_df.plot(kind='line', x='time', y='v', color='blue', ax=ax)
    walker_df.plot(kind='line', x='time', y='rel_dist', color='red', ax=ax)
    plt.show()
    """


