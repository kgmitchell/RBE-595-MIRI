# RBE-595-MIRI
Simulation for Medical Imaging and Robotic Instrumentation

Instructions:<br />
Note - Steps labeled (SETUP) are for first time only. Subsequent cycles do not need these steps again.<br />
1. (SETUP) Install Anaconda<br />
    https://www.anaconda.com/download
2. Launch Anaconda Prompt
3. (SETUP) Create environment<br />
    *conda create -n ENVIRONMENT_NAME python*
4. Activate environment<br />
    *conda activate ENVIRONMENT_NAME*
5. (SETUP) Install Pybullet<br />
    *conda install -c conda-forge pybullet*
6. (SETUP) Install scipy<br />
    *pip install scipy*
7. (SETUP) Install driver<br />
    *pip install driver*
8. (SETUP) Install transforms3d<br />
    *pip install transforms3d*
9. Navigate to file/folder/repository<br />
    *cd DIRECTORY*
10. Run script<br />
    *python simulation.py*