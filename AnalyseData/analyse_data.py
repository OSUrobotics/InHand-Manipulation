import pandas as pd
import numpy as np


def find_data_with_other_col(dataframe, col_to_find_with, equality, col_to_find_in):
    reqd_col = dataframe.loc[dataframe[col_to_find_with] == equality, col_to_find_in]
    return reqd_col


def add_column(data_dict, column_name, column_data):
    data_dict.update({column_name: column_data})
    return data_dict


def euc_dist_calc(pos_col):
    first_val = pos_col.iloc[0]
    if '[' in first_val:
        strip = ' []'
    elif '(' in first_val:
        strip = ' ()'
    else:
        raise SyntaxError
    pos_col = pos_col.apply(lambda s: [float(x.strip(strip)) for x in s.split(',')])
    prev_vals = np.asarray(pos_col.iloc[0])
    euc_dist = []
    for values in pos_col:
        new_vals = np.asarray(values)
        # print("PREV: {}, NEW: {}".format(prev_vals, new_vals))
        dist = np.linalg.norm(prev_vals - new_vals)
        prev_vals = new_vals
        euc_dist.append(dist)
    euc_dist = np.asarray(euc_dist)
    euc_dist_avg = euc_dist.mean()
    return euc_dist, euc_dist_avg, pos_col


if __name__ == '__main__':
    saved_data_file_name = 'AnalyseData/Data/filt_josh_2v2_g_none_1_save_data.csv'
    analysis_dict = {}
    df = pd.read_csv(saved_data_file_name)
    print(df.head())

    # Left Proximal Joint Angle and Velocties
    ja_l_prox = find_data_with_other_col(df, 'Phase', 'Move', "b'l_prox_pin'_joint_angle")
    jv_l_prox = find_data_with_other_col(df, 'Phase', 'Move', "b'l_prox_pin'_joint_vel")
    ja_diff_l_prox = ja_l_prox.diff()
    ja_diff_avg_l_prox = ja_diff_l_prox.mean()
    jv_avg_l_prox = jv_l_prox.mean()

    # Right Proximal Joint Angle and Velocties
    ja_r_prox = find_data_with_other_col(df, 'Phase', 'Move', "b'r_prox_pin'_joint_angle")
    jv_r_prox = find_data_with_other_col(df, 'Phase', 'Move', "b'r_prox_pin'_joint_vel")
    ja_diff_r_prox = ja_r_prox.diff()
    ja_diff_avg_r_prox = ja_diff_r_prox.mean()
    jv_avg_r_prox = jv_r_prox.mean()

    # Left Distal Joint Angle and Velocties
    ja_l_distal = find_data_with_other_col(df, 'Phase', 'Move', "b'l_distal_pin'_joint_angle")
    jv_l_distal = find_data_with_other_col(df, 'Phase', 'Move', "b'l_distal_pin'_joint_vel")
    ja_diff_l_distal = ja_l_distal.diff()
    ja_diff_avg_l_distal = ja_diff_l_distal.mean()
    jv_avg_l_distal = jv_l_distal.mean()

    # Right Distal Joint Angle and Velocties
    ja_r_distal = find_data_with_other_col(df, 'Phase', 'Move', "b'r_distal_pin'_joint_angle")
    jv_r_distal = find_data_with_other_col(df, 'Phase', 'Move', "b'r_distal_pin'_joint_vel")
    ja_diff_r_distal = ja_r_distal.diff()
    ja_diff_avg_r_distal = ja_diff_r_distal.mean()
    jv_avg_r_distal = jv_r_distal.mean()

    # Add to dictionary
    add_column(analysis_dict, column_name='JA_diff_l_prox_{}'.format(ja_diff_avg_l_prox), column_data=ja_diff_l_prox)
    add_column(analysis_dict, column_name='JV_l_prox_{}'.format(jv_avg_l_prox), column_data=jv_l_prox)

    add_column(analysis_dict, column_name='JA_diff_r_prox_{}'.format(ja_diff_avg_r_prox), column_data=ja_diff_r_prox)
    add_column(analysis_dict, column_name='JV_r_prox_{}'.format(jv_avg_r_prox), column_data=jv_r_prox)

    add_column(analysis_dict, column_name='JA_diff_l_distal_{}'.format(ja_diff_avg_l_distal), column_data=ja_diff_l_distal)
    add_column(analysis_dict, column_name='JV_l_distal_{}'.format(jv_avg_l_distal), column_data=jv_l_distal)

    add_column(analysis_dict, column_name='JA_diff_r_distal_{}'.format(ja_diff_avg_r_distal), column_data=ja_diff_r_distal)
    add_column(analysis_dict, column_name='JV_r_distal_{}'.format(jv_avg_r_distal), column_data=jv_r_distal)

    # Calculate euclidean distance between two consecutive positions

    # Left Proximal Link
    link_pos_l_prox = find_data_with_other_col(df, 'Phase', 'Move', "b'l_prox_pin'_link_pos")
    euc_dist_l_prox, euc_dist_avg_l_prox, link_pos_l_prox = euc_dist_calc(link_pos_l_prox)

    # Left Distal Link
    link_pos_l_distal = find_data_with_other_col(df, 'Phase', 'Move', "b'l_distal_pin'_link_pos")
    euc_dist_l_distal, euc_dist_avg_l_distal, link_pos_l_distal = euc_dist_calc(link_pos_l_distal)

    # Right Proximal Link
    link_pos_r_prox = find_data_with_other_col(df, 'Phase', 'Move', "b'r_prox_pin'_link_pos")
    euc_dist_r_prox, euc_dist_avg_r_prox, link_pos_r_prox = euc_dist_calc(link_pos_r_prox)

    # Right Distal Link
    link_pos_r_distal = find_data_with_other_col(df, 'Phase', 'Move', "b'r_distal_pin'_link_pos")
    euc_dist_r_distal, euc_dist_avg_r_distal, link_pos_r_distal = euc_dist_calc(link_pos_r_distal)

    add_column(analysis_dict, column_name='EUC_dist_l_prox_{}'.format(euc_dist_avg_l_prox), column_data=euc_dist_l_prox)
    add_column(analysis_dict, column_name='EUC_dist_l_distal_{}'.format(euc_dist_avg_l_distal), column_data=euc_dist_l_distal)
    add_column(analysis_dict, column_name='EUC_dist_r_prox_{}'.format(euc_dist_avg_r_prox), column_data=euc_dist_r_prox)
    add_column(analysis_dict, column_name='EUC_dist_r_distal_{}'.format(euc_dist_avg_r_distal), column_data=euc_dist_r_distal)

    # Cube Position of Controller
    cube_pos_controller = find_data_with_other_col(df, 'Phase', 'Move', 'Cube_Pos')
    euc_dist_cube_controller, euc_dist_avg_cube_controller, cube_pos_controller = euc_dist_calc(cube_pos_controller)

    # Cube Position of Human
    cube_pos_human = find_data_with_other_col(df, 'Phase', 'Move', 'human_cube_pos')
    cube_pos_human.iloc[0] = '(0.0, 0.0, 0.0)'
    euc_dist_cube_human, euc_dist_avg_cube_human, cube_pos_human = euc_dist_calc(cube_pos_human)

    add_column(analysis_dict, column_name='EUC_dist_cube_controller_{}'.format(euc_dist_avg_cube_controller), column_data=euc_dist_cube_controller)
    add_column(analysis_dict, column_name='EUC_dist_cube_human_{}'.format(euc_dist_avg_cube_human), column_data=euc_dist_cube_human)

    # print("DICT: {}".format(analysis_dict))

    # print("All Lenghts:", len(ja_diff_l_prox), len(jv_l_distal), len(link_pos_r_prox), len(link_pos_r_distal),
    #       len(cube_pos_controller), len(cube_pos_human), len(euc_dist_l_prox))

    print(analysis_dict.keys())

    df_new = pd.DataFrame.from_dict(analysis_dict)
    # print("DF {}".format(df_new.items))
    df_new.to_csv('AnalyseData/Data/'+ 'new_data.csv')
