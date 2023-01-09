

def postprocess(name, output, input = "cffirmware_osqp/src/generated/src/osqp/workspace.c"):
    """ Adds static keyword, so the workspace can be re-used """
        
    with open(input) as f:
        contents = f.read()

    datatypes = ["c_int", "c_float", "csc", "OSQPData", "OSQPSettings", "OSQPScaling", "QDLDL_float", "QDLDL_int", "QDLDL_bool", "qdldl_solver", "OSQPSolution", "OSQPInfo"]

    for datatype in datatypes:
        contents = contents.replace("{} ".format(datatype), "static {} ".format(datatype))

    contents = contents.replace("OSQPWorkspace workspace", "OSQPWorkspace workspace_{}".format(name))

    with open(output, 'w') as f:
        f.write(contents)


if __name__ == '__main__':
    postprocess("hyperplane", "../crazyflie-firmware/src/lib/osqp/src/osqp/workspace_hyperplane.c")