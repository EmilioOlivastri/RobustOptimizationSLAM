#!/bin/bash
path2_datasets="/home/slam-emix/Datasets/BACK_END/2D/"
#dataset_list="INTEL MIT M3500 CSAIL FR079 FRH"
dataset_list="CSAIL"
#outliers="10 20 30 40 50 60 70 80 90 100"
outliers="10"
monte_runs="00 01 02 03 04 05 06 07 08 09"
date="210823"

# G2O related solutiona
g2o_opt="G2O"
g2o_list="ADAPT MAXMIX IPC"

# GTSAM related solutions
gtsam_opt="GTSAM"
gtsam_list="GNC HUBER DCS PCM"

for algo in ${gtsam_list}
do
    for dataset in ${dataset_list}
    do
        gt_traj=${path2_datasets}${dataset}"/GT.txt"
        for out in ${outliers}
        do
            for run in ${monte_runs}
            do
                input_traj=${path2_datasets}${dataset}"/EXP/"${date}"/"${gtsam_opt}"_"${algo}"/"${out}"/"${run}".TRJ"
                output_res=${path2_datasets}${dataset}"/EXP/"${date}"/"${gtsam_opt}"_"${algo}"/"${out}"/"${run}".TE" 
                ../build/evaluator/evaluator ${input_traj} ${gt_traj} ${output_res} &
            done
            wait
        done
        echo "Finished "${dataset}
    done
done


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
