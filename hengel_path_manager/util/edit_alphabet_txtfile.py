#!/usr/bin/env python

import os


path_str='/home/mjlee/catkin_ws/src/hengel_ros/hengel_path_manager/alphabet_path'
path_str2='/home/mjlee/catkin_ws/src/hengel_ros/hengel_path_manager/alphabet_path2'

# if os.path.isfile(path_str):
# for filename in os.listdir(path_str):
#     if filename[0]=='p':

for alph in range(65, 91):
    x_arr=[]
    y_arr=[]
    paths=[]
    for i in range(1,4):
        filepath= '/path_'+str(chr(alph))+'_'+str(i)+'.txt'
        if os.path.isfile(path_str+filepath):
            print(filepath+ " read")
            paths.append(filepath)
            with open(path_str+filepath, "r") as file_path:
                for idx, line in enumerate(file_path):
                    _str=line.split()
                    if(len(_str)==2):
                        x_arr.append(float(_str[0]))
                        y_arr.append(float(_str[1]))
        else:
            print("No file - "+filepath)
            break

    x_min=min(x_arr)
    print(x_min)
    x_max=max(x_arr)
    print(x_max)
    y_min=min(y_arr)
    y_max=max(y_arr)
    
    for path in paths:
        with open(path_str+path, "r") as alphabet_before:
            with open(path_str2+path, "w") as alphabet_after:
                for idx, line in enumerate(alphabet_before):
                    if len(line.split()) == 2:
                        x=float(line.split()[0])
                        y=float(line.split()[1])

                        x_aft= (x-x_min)/(x_max-x_min)*0.8
                        y_aft= (y-y_min)/(y_max-y_min)

                        alphabet_after.write(str(x_aft)+'\t'+str(y_aft)+'\n')
        print(path + " wrote")
        
    if (x_max==x_min):
        print(str(chr(alph))+"- x-dir size: %f, y-dir size: %f" %(x_max-x_min, y_max-y_min))
    else:
        print(str(chr(alph))+"- x-dir size: %f, y-dir size: %f + ratio: %f" %(x_max-x_min, y_max-y_min, (y_max-y_min)/float((x_max-x_min))))

