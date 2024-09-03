import argparse
import subprocess
import os

if __name__ == "__main__":
    parser = argparse.ArgumentParser("ik_solver_1")
    parser.add_argument("plugin", help="string containing a list of plugins", type=str)
    parser.add_argument("filename", help="string containing a list of config file", type=str)

    pack_names = parser.parse_args().plugin
    filenames = parser.parse_args().filename

    # Remove arg name
    pack_names = pack_names.split("=")[1]
    filenames = filenames.split("=")[1]
    print(pack_names)
    print(filenames)

    pack_splitted = pack_names.split(",")
    file_splitted = filenames.split(",")
    trans = str.maketrans("","","[' ]")
    trans2 = str.maketrans("","",'"')
    pack_list = [s.translate(trans).translate(trans2) for s in pack_splitted]
    print(pack_list)
    file_list = [s.translate(trans).translate(trans2) for s in file_splitted]
    print(file_list)
    if(len(pack_list) != len(file_list)):
        raise Exception(f"[ERROR]: Arguments: The number of packages is not equal to the number of filenames")
    
    load_plugin_param_proc = []
    ik_solver_nodes = []
    launch_node_after_load = []

    for pack, filen in zip(pack_list, file_list):
        command = ["rospack", "find", pack]
        package_path = subprocess.check_output(command, text=True).strip()
        cnr_param_load_proc = subprocess.Popen(["rosrun", "cnr_param", "cnr_param_server", "--path-to-file", os.path.join(package_path, "config", filen)])

        cnr_param_load_proc.wait()

        node_to_run = ["rosrun", "ik_solver", "ik_solver_node", "__ns:="+pack]
        subprocess.run(node_to_run)