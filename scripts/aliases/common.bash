# ====================================================================================================================================
# @file     common.bash
# @author   Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @date     Tuesday, 1st March 2022 4:11:56 pm
# @modified   Thursday, 7th April 2022 5:39:58 pm
# @project  engineering-thesis
# @brief
#    
#    Set of handy function and aliases used when dealing with the project
#    
# @copyright Krzysztof Pierczyk © 2022
# ====================================================================================================================================

# ---------------------------------------------------------------------------------------
# @brief Resolves ros dependencies based on the packages in the ./src dircetory
# ---------------------------------------------------------------------------------------
alias rosdep_install_src="
rosdep install -i --from-path src --rosdistro $PROJECT_ROS_DISTRO -y
"
