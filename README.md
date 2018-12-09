# 6.881 Final Project

## Pre-reqs
In the root directory of this repository, run the following command in a terminal to build a docker image that includes Drake (currently a prebuilt local version with added bindings) and denpendencies for PDDLStream:
```bash
$ docker build -t dirtran6881 -f ubuntu16_04.dockerfile .
``` 

## Use
In the root directory of this repository, run 
```bash
$ python ./docker_run.py --os [your operating system]
``` 
where `[your operating system]` should be replaced with `mac` or `linux`. This command will start a docker container (virtual machine) with the docker image you have created. The `PDDL_plan` folder on the host machine (your laptop/desktop) is mounted to `/6-881-final` in the docker container. 
