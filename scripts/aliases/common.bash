# ====================================================================================================================================
# @file       common.bash
# @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @date       Tuesday, 1st March 2022 4:11:56 pm
# @modified   Monday, 11th July 2022 10:34:02 pm
# @project    engineering-thesis
# @brief      Set of handy function and aliases used when dealing with the project
#    
# @copyright Krzysztof Pierczyk © 2022
# ====================================================================================================================================

# ---------------------------------------------------------------------------------------
# @brief Resolves ros dependencies based on the packages in the ./src dircetory
# ---------------------------------------------------------------------------------------
alias rosdep_install_src="
rosdep install -i --from-path src --rosdistro $PROJECT_ROS_DISTRO -y
"

# ---------------------------------------------------------------------------------------
# @brief Prints git history in a pretty form
# @see https://stackoverflow.com/questions/1057564/pretty-git-branch-graphs
# ---------------------------------------------------------------------------------------
function git_adog() {
    git log --all --decorate --oneline --graph
}
