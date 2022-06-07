# ====================================================================================================================================
# @file       cyclictest.bash
# @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @date       Friday, 29th April 2022 1:30:25 pm
# @modified   Friday, 29th April 2022 6:38:15 pm
# @project    engineering-thesis
# @brief
#    
#    Set of handy command line tools working around `cyclictest`
#    
# @copyright Krzysztof Pierczyk © 2022
# ====================================================================================================================================

# ============================================================= Helpers ============================================================ #

# ---------------------------------------------------------------------------------------
# @brief Parses output of the `cyclictest ...` comand to abtain max latency
#
# @param out_file
#    name of the file containing raw output of the `cyclictest`
# @outputs 
#    parsed latency value
# ---------------------------------------------------------------------------------------
function parse_cyclictest_max_latency() {

    # Arguments
    local out_file="$1"

    # Parse histogram data lines
    grep "Max Latencies" $out_file | tr " " "\n" | sort -n | tail -1 | sed s/^0*//

}

# ---------------------------------------------------------------------------------------
# @brief Parses output of the `cyclictest -h<x> ...` comand to abtain lines containing 
#    actual histogram data
#
# @param out_file
#    name of the file containing raw output of the `cyclictest`
# @outputs 
#    parsed lines
# ---------------------------------------------------------------------------------------
function parse_cyclictest_hist_data() {

    # Arguments
    local out_file="$1"

    # Grep data lines, remove empty lines and create a common field separator
    grep -v -e "^#" -e "^$" "$out_file" | tr " " "\t"
}

# ---------------------------------------------------------------------------------------
# @brief Parses output of the `cyclictest -v ...` comand to abtain sorted latency data
#
# @param out_file
#    name of the file containing raw output of the `cyclictest`
# @outputs 
#    parsed lines
# ---------------------------------------------------------------------------------------
function parse_cyclictest_data() {
    
    # Arguments
    local out_file="$1"

    local raw_data_file=$(tempfile)

    # Grep data lines and trim whitespaces
    grep -e "^ " "$out_file" | awk '{$1=$1};1' > "$raw_data_file"
    
    # Create basename for per-thread output files
    local list_base_name=$(tempfile)
    # Remove same file
    rm $list_base_name

    # Parse raw data outputing indexed latencies of each thread to a separate file
    cat "$raw_data_file" | awk -F": " "{ outfile=sprintf(\"%s%s\", \"$list_base_name\", \$1); print \$3 >> outfile }"
    # Compile all files line by line separating filds with ','
    paste -d, $(ls -1 $(dirname ${list_base_name}) | grep $(basename ${list_base_name}) | awk "{ print \"$(dirname ${list_base_name})/\"\$0 }")
}

# ---------------------------------------------------------------------------------------
# @brief Parses output of the `cyclictest -v ...` comand to abtain sorted latency data.
#    Remove iterations for which not all threads has been measured.
#
# @param out_file
#    name of the file containing raw output of the `cyclictest`
# @outputs 
#    parsed lines
# ---------------------------------------------------------------------------------------
function parse_cyclictest_data_aligned() {
    
    # Arguments
    local out_file="$1"

    # Parse and cut non-full records
    parse_cyclictest_data "$out_file" | grep -v -e ",," -e ",\$"
}

# ========================================================= Histogram tools ======================================================== #

# ---------------------------------------------------------------------------------------
# @brief Runs `cyclctest` with given arguments and outputs histogram data
#
# @params ...
#     parameters passed to `cyclictest`
# 
# @options
#
#  -h US|--hist_max=US  maximal value of latency takent into account in histogram
#                       (default: 200 [us])
#            -s|--sudo  runs `cyclctest` with `sudo`, if given
#
# ---------------------------------------------------------------------------------------
function cyclictest_hist_data() {

    local LOG_CONTEXT="cyclictest_hist_data"

    # ---------------------------- Parse arguments ----------------------------

    # Function's options
    declare -a opt_definitions=(
        '-h|--hist_max',hist_max_us
        '-s|--sudo',with_sudo,f
    )
    
    # Parse arguments to a named array
    parse_options_s

    # Set list of packages as positional arguments
    set -- "${posargs[@]}"   

    # ------------------------------ Set defaults -----------------------------

    # Set default latency limit if not given
    is_var_set_non_empty options[hist_max_us] || options[hist_max_us]=200
    # Set sdo mode
    local sudo_mode=""
    is_var_set_non_empty options[with_sudo] && sudo_mode="sudo"

    # -------------------------------------------------------------------------
    
    # Create temporary files for histogram data
    local out_file=$(tempfile)
    local hist_file_processed=$(tempfile)

    # Run cyclictest
    $sudo_mode cyclictest "$@" -h"${options[hist_max_us]}" -q > "$out_file" || {
        log_error "'cyclictest' failed"
        return 1
    }

    # Grep data lines, remove empty lines and create a common field separator
    parse_cyclictest_hist_data "$out_file"
}

# ---------------------------------------------------------------------------------------
# @brief Runs `cyclctest` with given arguments and creates .png graph based on
#    histogram data. 
#
# @params ...
#     parameters passed to `cyclictest`
# 
# @options
#
#           --out=FILE  name of the output file (default: latency.png)
#  -h US|--hist_max=US  maximal value of latency takent into account in histogram
#                       (default: 200 [us])
#            -s|--sudo  runs `cyclctest` with `sudo`, if given
#         -t|--threads  number of threads on the plot (This must be set by the caller 
#                       to number equal to actual number of threads run by `cyclctest`
#                       for the given set of options)
#             --prefix  prefix of thread names on the graph (default: T)
#
# @source https://www.osadl.org/Create-a-latency-plot-from-cyclictest-hi.bash-script-for-latency-plot.0.html
# ---------------------------------------------------------------------------------------
function cyclictest_hist_plot_impl() {

    # ---------------------------- Parse arguments ----------------------------

    # Function's options
    declare -a opt_definitions=(

        # Own options
        '--out',output_file
        '-h|--hist_max',hist_max_us
        '-s|--sudo',with_sudo,f
        '-t|--threads',threads
        '--prefix',prefix
        
    )
    
    # Parse arguments to a named array
    parse_options_s

    # Set list of packages as positional arguments
    set -- "${posargs[@]}"   
    
    # ------------------------------ Set defaults -----------------------------

    # Set default output file, if not given
    ! is_var_set_non_empty options[output_file] && options[output_file]="latency.png"
    # Set default latency limit if not given
    ! is_var_set_non_empty options[hist_max_us] && options[hist_max_us]=200
    # Set sudo mode
    local sudo_mode=""
    is_var_set_non_empty options[with_sudo] && sudo_mode="sudo"
    # Set default number of threads
    ! is_var_set_non_empty options[threads] && options[threads]=$(nproc --all)
    # Set default prefix of threads names
    ! is_var_set_non_empty options[prefix] && options[prefix]="T"

    # -------------------------------------------------------------------------
    
    # Create temporary files for histogram data
    local out_file=$(tempfile)

    log_info "Running 'cyclictest'..."

    # Run cyclictest
    $sudo_mode cyclictest "$@" -h"${options[hist_max_us]}" -q > "$out_file" || {
        log_error "'cyclictest' failed"
        return 1
    }

    log_info "Latency data collected"

    # Get maximum latency
    local max=$(parse_cyclictest_max_latency $out_file)

    # Create temporary file for parsed histogram data
    local hist_file_processed=$(tempfile)
    
    # Grep data lines, remove empty lines and create a common field separator
    parse_cyclictest_hist_data "$out_file" > "$hist_file_processed" 
        
    # Create two-column data sets with latency bucket and frequency for each threa
    for i in $(seq 1 ${options[threads]}); do
        local column=$(expr $i + 1)
        cut -f1,$column "$hist_file_processed" > "${hist_file_processed}${i}"
    done

    # Create temporary file for plotting command
    local plotcmd_file=$(tempfile)

    # Create plot command header
    echo -n -e "set title \"Latency plot\"\n\
    set terminal png\n\
    set xlabel \"Latency (us), max $max us\"\n\
    set logscale y\n\
    set xrange [0:${options[hist_max_us]}]\n\
    set yrange [0.8:*]\n\
    set ylabel \"Number of latency samples\"\n\
    set output \"${options[output_file]}\"\n\
    plot " > "$plotcmd_file"

    # 7. Append plot command data references
    for i in $(seq 1 ${options[threads]}); do
        if test $i != 1; then
            echo -n ", " >> "$plotcmd_file"
        fi
        local thrno=`expr $i - 1`
        if test $thrno -lt 10; then
            local title=" ${options[prefix]}$thrno"
        else
            local title="${options[prefix]}$thrno"
        fi
        echo -n "\"${hist_file_processed}${i}\" using 1:2 title \"$title\" with histeps" >> "$plotcmd_file"
    done

    log_info "Plotting histogram..."

    # 8. Execute plot command
    gnuplot --persist < "$plotcmd_file"|| {
        log_error "'gnuplot' failed"
        return 1
    }

    log_info "Histogram has been saved to ${options[output_file]}"
}

# ---------------------------------------------------------------------------------------
# @brief Runs `cyclctest` with given arguments and creates .png graph based on
#    histogram data. The standard SMP test is run with thread per logical core
#
# @params ...
#     parameters passed to `cyclictest`
# 
# @options
#
#           --out=FILE  name of the output file (default: latency.png)
#  -h US|--hist_max=US  maximal value of latency takent into account in histogram
#                       (default: 200 [us])
#            -s|--sudo  runs `cyclctest` with `sudo`, if given
#
# @source https://www.osadl.org/Create-a-latency-plot-from-cyclictest-hi.bash-script-for-latency-plot.0.html
# ---------------------------------------------------------------------------------------
function cyclictest_smp_hist_plot() {

    local LOG_CONTEXT="cyclictest_smp_hist_plot"

    # ---------------------------- Parse arguments ----------------------------

    # Function's options
    declare -a opt_definitions=(

        # Own options
        '--out',output_file
        '-h|--hist_max',hist_max_us
        '-s|--sudo',with_sudo,f
        
    )
    
    # Parse arguments to a named array
    parse_options_s

    # Set list of packages as positional arguments
    set -- "${posargs[@]}"   
    
    # ------------------------------ Set defaults -----------------------------

    # Set default output file, if not given
    ! is_var_set_non_empty options[output_file] && options[output_file]="latency.png"
    # Set default latency limit if not given
    ! is_var_set_non_empty options[hist_max_us] && options[hist_max_us]=200

    # -------------------------------------------------------------------------
    
    local impl_options=()

    # Forward keyword options to actual implementation
    impl_options+=(
        "--out=${options[output_file]}"
        "--hist_max=${options[hist_max_us]}"
        "--threads=$(nproc --all)"
        "--prefix=CPU"
    )
    # Forward flag options to actual implementation
    is_var_set_non_empty options[with_sudo] && impl_options+=( "--sudo" )

    # Call implementation
    cyclictest_hist_plot_impl "${impl_options[@]}" -- "$@" --smp
}

# ---------------------------------------------------------------------------------------
# @brief Runs `cyclctest` with given arguments and creates .png graph based on
#    histogram data. The test is run with the given number of threads
#
# @params ...
#     parameters passed to `cyclictest`
# 
# @options
#
#           --out=FILE  name of the output file (default: latency.png)
#  -h US|--hist_max=US  maximal value of latency takent into account in histogram
#                       (default: 200 [us])
#            -s|--sudo  runs `cyclctest` with `sudo`, if given
#         -t|--threads  number of threads to be run (default: number of logical cores)
#
# @source https://www.osadl.org/Create-a-latency-plot-from-cyclictest-hi.bash-script-for-latency-plot.0.html
# ---------------------------------------------------------------------------------------
function cyclictest_thread_hist_plot() {

    local LOG_CONTEXT="cyclictest_thread_hist_plot"

    # ---------------------------- Parse arguments ----------------------------

    # Function's options
    declare -a opt_definitions=(

        # Own options
        '--out',output_file
        '-h|--hist_max',hist_max_us
        '-s|--sudo',with_sudo,f
        '-t|--threads',threads
        
    )
    
    # Parse arguments to a named array
    parse_options_s

    # Set list of packages as positional arguments
    set -- "${posargs[@]}"   
    
    # ------------------------------ Set defaults -----------------------------

    # Set default output file, if not given
    ! is_var_set_non_empty options[output_file] && options[output_file]="latency.png"
    # Set default latency limit if not given
    ! is_var_set_non_empty options[hist_max_us] && options[hist_max_us]=200
    # Set sudo mode
    local sudo_mode=""
    is_var_set_non_empty options[with_sudo] && sudo_mode="sudo"
    # Set default number of threads
    ! is_var_set_non_empty options[threads] && options[threads]=$(nproc --all)

    # -------------------------------------------------------------------------
    
    local impl_options=()

    # Forward keyword options to actual implementation
    impl_options+=(
        "--out=${options[output_file]}"
        "--hist_max=${options[hist_max_us]}"
        "--threads=${options[threads]}"
        "--prefix=Thread"
    )
    # Forward flag options to actual implementation
    is_var_set_non_empty options[with_sudo] && impl_options+=( "--sudo" )

    # Call implementation
    cyclictest_hist_plot_impl "${impl_options[@]}" -- "$@" --threads=${options[threads]}
}

# ========================================================== Latency tools ========================================================= #

# ---------------------------------------------------------------------------------------
# @brief Runs `cyclctest` with given arguments and outputs latency data data in a
#    sorted list
#
# @params ...
#     parameters passed to `cyclictest`
# 
# @options
#
#        -s|--sudo  runs `cyclctest` with `sudo`, if given
#     -a|--aligned  if set, only iterations for which all threads has been measured 
#                   will be printed
#
# ---------------------------------------------------------------------------------------
function cyclictest_data() {

    local LOG_CONTEXT="cyclictest_data"

    # ---------------------------- Parse arguments ----------------------------

    # Function's options
    declare -a opt_definitions=(
        '-s|--sudo',with_sudo,f
        '--aligned',aligned,f
    )
    
    # Parse arguments to a named array
    parse_options_s

    # Set list of packages as positional arguments
    set -- "${posargs[@]}"   

    # ------------------------------ Set defaults -----------------------------

    # Set sdo mode
    local sudo_mode=""
    is_var_set_non_empty options[with_sudo] && sudo_mode="sudo"

    # -------------------------------------------------------------------------
    
    # Create temporary files for histogram data
    local out_file=$(tempfile)

    # Run cyclictest
    $sudo_mode cyclictest "$@" -qv > "$out_file" || {
        log_error "'cyclictest' failed"
        return 1
    }

    # Parse output data
    if is_var_set_non_empty options[aligned]; then
        parse_cyclictest_data_aligned "$out_file"
    else
        parse_cyclictest_data "$out_file"
    fi
}

# ---------------------------------------------------------------------------------------
# @brief Runs `cyclctest` with given arguments and creates .png graph based on
#    latency data. 
#
# @params ...
#     parameters passed to `cyclictest`
# 
# @options
#
#           --out=FILE  name of the output file (default: latency.png)
#            -s|--sudo  runs `cyclctest` with `sudo`, if given
#         -t|--threads  number of threads on the plot (This must be set by the caller 
#                       to number equal to actual number of threads run by `cyclctest`
#                       for the given set of options)
#             --prefix  prefix of thread names on the graph (default: T)
#
# ---------------------------------------------------------------------------------------
function cyclictest_plot_impl() {

    # ---------------------------- Parse arguments ----------------------------

    # Function's options
    declare -a opt_definitions=(

        # Own options
        '--out',output_file
        '-s|--sudo',with_sudo,f
        '-t|--threads',threads
        '--prefix',prefix
        
        # Capture histogram options
        '-h|--histogram',histogram
        '-H|--histofall',histofall

    )
    
    # Parse arguments to a named array
    parse_options_s

    # Set list of packages as positional arguments
    set -- "${posargs[@]}"   
    
    # ------------------------------ Set defaults -----------------------------

    # Set default output file, if not given
    ! is_var_set_non_empty options[output_file] && options[output_file]="latency.png"
    # Set sudo mode
    local sudo_mode=""
    is_var_set_non_empty options[with_sudo] && sudo_mode="sudo"
    # Set default number of threads
    ! is_var_set_non_empty options[threads] && options[threads]=$(nproc --all)
    # Set default prefix of threads names
    ! is_var_set_non_empty options[prefix] && options[prefix]="T"

    # -------------------------------------------------------------------------
    
    # Create temporary files for output data
    local out_file=$(tempfile)

    log_info "Running 'cyclictest'..."

    # Run cyclictest
    $sudo_mode cyclictest "$@" -qv > "$out_file" || {
        log_error "'cyclictest' failed"
        return 1
    }

    log_info "Latency data collected"

    # Create temporary files for parsed data
    local data_processed_raw=$(tempfile)
    local data_processed=$(tempfile)
    local data_indexed=$(tempfile)

    # Parse output data
    parse_cyclictest_data_aligned "$out_file" > "$data_processed"
    # Preprocess list substituting ',' with ' ' and indexing samples
    cat "$data_processed" | tr "," " " | awk '{ line_id = NR - 1; print line_id " " $0 }' > "$data_indexed"
    # Create two-column data sets with latency index and value for each threa
    for i in $(seq 1 ${options[threads]}); do
        local column=$(expr $i + 1)
        cut -d' ' -f1,$column "$data_indexed" > "${data_indexed}${i}"
    done

    # Create temporary file for plotting command
    local plotcmd_file=$(tempfile)

    # Create plot command header
    echo -n -e "set title \"Latency plot\"\n\
    set terminal png\n\
    set xlabel \"Iterations\"\n\
    set xrange [0:$(cat $data_indexed | wc -l)]\n\
    set yrange [0.8:*]\n\
    set ylabel \"Latency [us]\"\n\
    set output \"${options[output_file]}\"\n\
    plot " > "$plotcmd_file"

    # Append plot command data references
    for i in $(seq 1 ${options[threads]}); do
        if test $i != 1; then
            echo -n ", " >> "$plotcmd_file"
        fi
        local thrno=`expr $i - 1`
        if test $thrno -lt 10; then
            local title=" ${options[prefix]}$thrno"
        else
            local title="${options[prefix]}$thrno"
        fi
        echo -n "\"${data_indexed}${i}\" using 1:2 title \"$title\" with impulses" >> "$plotcmd_file"
    done

    log_info "Plotting histogram..."
    
    # Execute plot command
    gnuplot --persist < "$plotcmd_file"|| {
        log_error "'gnuplot' failed"
        return 1
    }

    log_info "Histogram has been saved to ${options[output_file]}"
}


# ---------------------------------------------------------------------------------------
# @brief Runs `cyclctest` with given arguments and creates .png graph based on
#    latemcu data. The standard SMP test is run with thread per logical core
#
# @params ...
#     parameters passed to `cyclictest`
# 
# @options
#
#           --out=FILE  name of the output file (default: latency.png)
#            -s|--sudo  runs `cyclctest` with `sudo`, if given
#
# @source https://www.osadl.org/Create-a-latency-plot-from-cyclictest-hi.bash-script-for-latency-plot.0.html
# ---------------------------------------------------------------------------------------
function cyclictest_smp_plot() {

    local LOG_CONTEXT="cyclictest_smp_plot"

    # ---------------------------- Parse arguments ----------------------------

    # Function's options
    declare -a opt_definitions=(

        # Own options
        '--out',output_file
        '-s|--sudo',with_sudo,f
        
    )
    
    # Parse arguments to a named array
    parse_options_s

    # Set list of packages as positional arguments
    set -- "${posargs[@]}"   
    
    # ------------------------------ Set defaults -----------------------------

    # Set default output file, if not given
    ! is_var_set_non_empty options[output_file] && options[output_file]="latency.png"

    # -------------------------------------------------------------------------
    
    local impl_options=()

    # Forward keyword options to actual implementation
    impl_options+=(
        "--out=${options[output_file]}"
        "--threads=$(nproc --all)"
        "--prefix=CPU"
    )
    # Forward flag options to actual implementation
    is_var_set_non_empty options[with_sudo] && impl_options+=( "--sudo" )

    # Call implementation
    cyclictest_plot_impl "${impl_options[@]}" -- "$@" --smp
}

# ---------------------------------------------------------------------------------------
# @brief Runs `cyclctest` with given arguments and creates .png graph based on
#    latency data. The test is run with the given number of threads
#
# @params ...
#     parameters passed to `cyclictest`
# 
# @options
#
#           --out=FILE  name of the output file (default: latency.png)
#            -s|--sudo  runs `cyclctest` with `sudo`, if given
#         -t|--threads  number of threads to be run (default: number of logical cores)
#
# @source https://www.osadl.org/Create-a-latency-plot-from-cyclictest-hi.bash-script-for-latency-plot.0.html
# ---------------------------------------------------------------------------------------
function cyclictest_thread_plot() {

    local LOG_CONTEXT="cyclictest_thread_plot"

    # ---------------------------- Parse arguments ----------------------------

    # Function's options
    declare -a opt_definitions=(

        # Own options
        '--out',output_file
        '-s|--sudo',with_sudo,f
        '-t|--threads',threads
        
    )
    
    # Parse arguments to a named array
    parse_options_s

    # Set list of packages as positional arguments
    set -- "${posargs[@]}"   
    
    # ------------------------------ Set defaults -----------------------------

    # Set default output file, if not given
    ! is_var_set_non_empty options[output_file] && options[output_file]="latency.png"
    # Set sudo mode
    local sudo_mode=""
    is_var_set_non_empty options[with_sudo] && sudo_mode="sudo"
    # Set default number of threads
    ! is_var_set_non_empty options[threads] && options[threads]=$(nproc --all)

    # -------------------------------------------------------------------------
    
    local impl_options=()

    # Forward keyword options to actual implementation
    impl_options+=(
        "--out=${options[output_file]}"
        "--threads=${options[threads]}"
        "--prefix=Thread"
    )
    # Forward flag options to actual implementation
    is_var_set_non_empty options[with_sudo] && impl_options+=( "--sudo" )

    # Call implementation
    cyclictest_plot_impl "${impl_options[@]}" -- "$@" --threads=${options[threads]}
}

# ================================================================================================================================== #
