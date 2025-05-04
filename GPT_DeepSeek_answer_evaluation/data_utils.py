import pandas as pd

def read_excel(data_path):
    dataframe = pd.read_excel(data_path)

    return dataframe


def select_n_data(dataframe, sample_num):
    sample_df = dataframe.sample(n=sample_num).reset_index(drop=True)
    questions = sample_df.iloc[:, 0]
    gpt_answer = sample_df.iloc[:, 1]
    deepseek_answer = sample_df.iloc[:, 2]
    label = sample_df.iloc[:, 3]

    return questions, gpt_answer, deepseek_answer, label
