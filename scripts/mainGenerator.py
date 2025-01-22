import os


def main():
    n_loops = 60
    dataset = "/home/slam-emix/Datasets/BACK_END/3D/TUM_FR1_DESK/graph.g2o"
    output_dir = "/home/slam-emix/Datasets/BACK_END/3D/TUM_FR1_DESK/SPOILED_DATA/"
    res_out = "/0"
    
    dir_idx = 10
    perc = 0.1
    for _ in range(10):
        n = int(n_loops * perc)
        output = output_dir + str(dir_idx) + res_out

        for idx in range(10):
            command_gen = "python3 generateDataset.py -i " + dataset + " -n " + str(n)
            os.system(command_gen)
            command_ren = "mv new.g2o " + output + str(idx) + ".g2o"
            os.system(command_ren)

        dir_idx += 10
        perc += 0.1

    return 


if __name__ == '__main__' :
    main()
