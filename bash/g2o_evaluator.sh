#!/bin/bash

#path2_datasets="/home/slam-emix/Datasets/BACK_END/2D/"
path2_datasets="/home/slam-emix/Datasets/BACK_END/3D/"

#dataset_list="INTEL MIT CSAIL FR079 FRH"
dataset_list="TUM_FR1_DESK KITTI_05 KITTI_00"

outliers="10 20 30 40 50 60 70 80 90 100"
monte_runs="00 01 02 03 04 05 06 07 08 09"
#date="210823"  #2D
#date="311024"  #2D
#date="110125" #2D

#date="040724" #3D
#date="301124" #3D
date="110125" #3D

# G2O related solutiona
g2o_opt="G2O"
#g2o_list="MAXMIX IPC IPC_K2 IPC_K2_REC IPC_REC"
#g2o_list="IPC_K2 IPC_K3 IPC_K5 IPC_K10"
#g2o_list="SC_10 SC_100"
g2o_list="G2O_IPC_REC_1 G2O_IPC_REC_2 G2O_IPC_REC_5 G2O_IPC_REC_10 G2O_IPC_REC_20 G2O_IPC_REC_ALL"

# CLASSIC METRICS FOR EVALUATION
for algo in ${g2o_list}
do
    for dataset in ${dataset_list}
    do
        gt_traj=${path2_datasets}${dataset}"/GT.txt"
        for out in ${outliers}
        do
            for run in ${monte_runs}
            do
                input_traj=${path2_datasets}${dataset}"/EXP/"${date}"/"${g2o_opt}"_"${algo}"/"${out}"/"${run}".TRJ"
                output_res=${path2_datasets}${dataset}"/EXP/"${date}"/"${g2o_opt}"_"${algo}"/"${out}"/"${run}".TE" 
                ../build/evaluator/evaluator ${input_traj} ${gt_traj} ${output_res} &
            done
            wait
        done
        echo "Finished "${dataset}
    done
done


# NEW METRICS FOR EVALUATION
for algo in ${g2o_list}
do
    for dataset in ${dataset_list}
    do
        gt_traj=${path2_datasets}${dataset}"/GT.txt"
        for out in ${outliers}
        do
            for run in ${monte_runs}
            do
                graph_file=${path2_datasets}${dataset}"/SPOILED_DATA/"${out}"/"${run}".g2o"
                input_traj=${path2_datasets}${dataset}"/EXP/"${date}"/"${g2o_opt}"_"${algo}"/"${out}"/"${run}".TRJ"
                output_res=${path2_datasets}${dataset}"/EXP/"${date}"/"${g2o_opt}"_"${algo}"/"${out}"/"${run}".EV" 
                ../build/evaluator/test_newmetrics ${input_traj} ${gt_traj} ${output_res} ${graph_file} &
            done
            wait
        done
        echo "Finished "${dataset}
    done
done
