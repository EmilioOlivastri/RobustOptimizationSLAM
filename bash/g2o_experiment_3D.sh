#!/bin/bash
path2_datasets="/home/slam-emix/Datasets/BACK_END/3D/"
dataset_list="TUM_FR1_DESK KITTI_05 KITTI_00"
outliers="10 20 30 40 50 60 70 80 90 100"
monte_runs="00 01 02 03 04 05 06 07 08 09"
date="301124"

# G2O related solutiona
g2o_opt="G2O"
g2o_list="MAXMIX"

cd ../build/robust_g2o

for algo in ${g2o_list}
do
    for dataset in ${dataset_list}
    do
        cfg_file=${path2_datasets}${dataset}"/params.yaml"
        for out in ${outliers}
        do
            for run in ${monte_runs}
            do
                input_file=${path2_datasets}${dataset}"/SPOILED_DATA/"${out}"/"${run}".g2o"
                output_traj=${path2_datasets}${dataset}"/EXP/"${date}"/"${g2o_opt}"_"${algo}"/"${out}"/"${run}".TRJ"
                ./g2o_${algo}_3D -cfg ${cfg_file} -o ${output_traj} ${input_file} &
            done
            jobs
            wait
        done
        echo "Finished "${dataset}
    done
done

