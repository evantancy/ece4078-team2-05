#!/bin/bash
file_name="l_openvino_toolkit_p_2021.1.110_online"
wget https://registrationcenter-download.intel.com/akdlm/irc_nas/17062/$file_name.tgz
tar -xvf $file_name.tgz
cd $file_name/
sudo ./install_GUI.sh
cd /opt/intel/openvino_2021/install_dependencies/
sudo -E ./install_openvino_dependencies.sh
echo "source /opt/intel/openvino/bin/setupvars.sh" >> ~/.bashrc
echo "export ngraph_DIR=/opt/intel/openvino_2021/deployment_tools/ngraph/cmake/" >> ~/.bashrc