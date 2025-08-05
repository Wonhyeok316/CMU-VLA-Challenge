#!/bin/bash

sleep 3
source /opt/conda/etc/profile.d/conda.sh
conda activate leo
pip install pydantic-core
source ai_module/devel/setup.bash
cd ai_module/src/leo_vlm/embodied-generalist/model/pointnetpp
python setup.py install
cd ../../..
roslaunch interaction_manager interaction_manager.launch