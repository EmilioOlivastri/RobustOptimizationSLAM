#!/bin/bash

path2_datasets="/mnt/Dataset/"
dataset_list="AISKLINIK CITY100000 INTEL M3500 MIT"
outliers="10 20 30 40 50 60 70 80 90 100"
monte_runs="00 01 02 03 04 05 06 07 08 09"
date="210823"

# GTSAM related solutions
gtsam_opt="GTSAM"
gtsam_list="HUBER DCS GNC PCM"


path_exec="../build/robust_gtsam"
#path_exec="/home/benchmark_gtsam/build"

cd ${path_exec}

for algo in ${gtsam_list}
do
    for dataset in ${dataset_list}
    do
	cfg_file=${path2_datasets}${dataset}"/params.yaml"
        echo ${cfg_file}
        for out in ${outliers}
        do
            for run in ${monte_runs}
            do
                input_file=${path2_datasets}${dataset}"/SPOILED_DATA/"${out}"/"${run}".g2o"
                output_traj=${path2_datasets}${dataset}"/EXP/"${date}"/"${gtsam_opt}"_"${algo}"/"${out}"/"${run}".TRJ"
                ./gtsam_${algo} ${input_file} ${cfg_file} ${output_traj} &
            done
            jobs
            wait
        done
        echo "Finished "${dataset}
    done
done
