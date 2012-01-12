# Macros for automatically compiling LCM types into C, Java, and Python
# libraries.
#
# The primary macro is:
#     lcmtypes_build([C_AGGREGATE_HEADER header_fname] 
#                    [C_LIBNAME lib_name]
#                    [JAVA_DEST_DIR dir_name]
#                    [PY_DEST_DIR dir_name]
#                    )
# 
# It expects that the directory ${PROJECT_SOURCE_DIR}/lcmtypes contains all
# the LCM types used by the system.  The macro generates C, Java, and Python
# bindings.  See the C, Java, and Python sections below for information on
# language specific options and generated results.
#
# After invoking this macro, the following variables will be set:
#
#   LCMTYPES_LIBS
#   LCMTYPES_JAR
#
# 
# C
# ==
# 
# C bindings will be placed in ${PROJECT_SOURCE_DIR}/lcmtypes/c.  
#
# The autogenerated C bindings also get compiled to a static and shared
# library.  The library prefix will be stored in LCMTYPES_LIBS on output.
# This prefix can be manually set using the C_LIBNAME option.
# 
# Additionally, a header file will be generated that automatically includes
# all of the other automatically generated header files.  The name of this
# header file defaults to a cleaned-up version of "${PROJECT_NAME}.h" 
# (non-alphanumeric characters replaced with underscores), but can
# be manually set using the C_AGGREGATE_HEADER option.
#
# C++
# ==
# 
# C++ bindings will be placed in ${PROJECT_SOURCE_DIR}/lcmtypes/cpp.
#
# The autogenerated CPP bindings are header only, so no library is created.
# 
# A header file will be generated that automatically includes
# all of the other automatically generated header files.  The name of this
# header file defaults to a cleaned-up version of "${PROJECT_NAME}.hpp" 
# (non-alphanumeric characters replaced with underscores), but can
# be manually set using the CPP_AGGREGATE_HEADER option.
#
#
# Java
# ====
#
# If Java is available, then Java bindings are be generated and placed in 
#    ${PROJECT_SOURCE_DIR}/lcmtypes/java
#
# This directory can be changed using the JAVA_DEST_DIR option.
# 
# Additionally, targets are added to automatically compile the .java files to a
# .jar file. The location of this jar file is stored in LCMTYPES_JAR
#
# and the .jar file will be installed to 
#   ${CMAKE_INSTALL_PREFIX}/share/java
#
#
# Python
# ======
#
# If Python is enabled, then python bindings will be generated and placed in 
#    ${PROJECT_SOURCE_DIR}/lcmtypes/python
# 
# This directory can be changed by setting the PY_DEST_DIR option.
#
# Additionally, the .py files will be installed to 
#   ${CMAKE_INSTALL_PREFIX}/lib/python{X.Y}/dist-packages
#   
# where {X.Y} refers to the python version used to build the .py files.
#
# ----
# File: lcmtypes.cmake
# Distributed with pods version: 12.01.11

cmake_minimum_required(VERSION 2.6.0)

# Policy settings to prevent warnings on 2.6 but ensure proper operation on
# 2.4.
if(COMMAND cmake_policy)
    # Logical target names must be globally unique.
    cmake_policy(SET CMP0002 OLD)
    # Libraries linked via full path no longer produce linker search paths.
    cmake_policy(SET CMP0003 OLD)
    # Preprocessor definition values are now escaped automatically.
    cmake_policy(SET CMP0005 OLD)
    if(POLICY CMP0011)
        # Included scripts do automatic cmake_policy PUSH and POP.
        cmake_policy(SET CMP0011 OLD)
    endif(POLICY CMP0011)
endif()

macro(lcmtypes_get_types msgvar)
    # get a list of all LCM types
    file(GLOB __tmplcmtypes "${PROJECT_SOURCE_DIR}/lcmtypes/*.lcm")
    set(${msgvar} "")
    foreach(_msg ${__tmplcmtypes})
        # Try to filter out temporary and backup files
        if(${_msg} MATCHES "^[^\\.].*\\.lcm$")
            list(APPEND ${msgvar} ${_msg})
        endif(${_msg} MATCHES "^[^\\.].*\\.lcm$")
    endforeach(_msg)
endmacro()

function(lcmgen)
    execute_process(COMMAND ${LCM_GEN_EXECUTABLE} ${ARGV} RESULT_VARIABLE lcmgen_result)
    if(NOT lcmgen_result EQUAL 0)
        message(FATAL_ERROR "lcm-gen failed")
    endif()
endfunction()

function(lcmtypes_add_clean_dir clean_dir)
    get_directory_property(acfiles ADDITIONAL_MAKE_CLEAN_FILES)
    list(APPEND acfiles ${clean_dir})
    set_directory_properties(PROPERTIES ADDITIONAL_MAKE_CLEAN_FILES "${acfiles}")
endfunction()

function(lcmtypes_build_c)
    lcmtypes_get_types(_lcmtypes)
    list(LENGTH _lcmtypes _num_lcmtypes)
    if(_num_lcmtypes EQUAL 0)
        return()
    endif()

    string(REGEX REPLACE "[^a-zA-Z0-9]" "_" __sanitized_project_name "${PROJECT_NAME}")

    # set some defaults

    # library name
    set(libname "lcmtypes_${PROJECT_NAME}")

    # header file that includes all other generated header files
    set(agg_h_bname "${__sanitized_project_name}.h")

    # allow defaults to be overriden by function parameters
    set(modewords C_LIBNAME C_AGGREGATE_HEADER)
    set(curmode "")
    foreach(word ${ARGV})
        list(FIND modewords ${word} mode_index)
        if(${mode_index} GREATER -1)
            set(curmode ${word})
        elseif(curmode STREQUAL C_AGGREGATE_HEADER)
            set(agg_h_bname "${word}")
            set(curmode "")
        elseif(curmode STREQUAL C_LIBNAME)
            set(libname "${word}")
            set(curmode "")
        endif()
    endforeach()

    # generate C bindings for LCM types
    set(_lcmtypes_c_dir ${PROJECT_SOURCE_DIR}/lcmtypes/c/lcmtypes)

    # blow away any existing auto-generated files.
    file(REMOVE_RECURSE ${_lcmtypes_c_dir})

    # run lcm-gen now
    execute_process(COMMAND mkdir -p ${_lcmtypes_c_dir})
    lcmgen(--lazy -c --c-cpath ${_lcmtypes_c_dir} --c-hpath ${_lcmtypes_c_dir} --cinclude lcmtypes ${_lcmtypes})

    # run lcm-gen at compile time
    add_custom_target(lcmgen_c ALL 
        COMMAND sh -c '[ -d ${_lcmtypes_c_dir} ] || mkdir -p ${_lcmtypes_c_dir}'
        COMMAND sh -c '${LCM_GEN_EXECUTABLE} --lazy -c ${_lcmtypes} --c-cpath ${_lcmtypes_c_dir} --c-hpath ${_lcmtypes_c_dir}')

    # get a list of all generated .c and .h files
    file(GLOB _lcmtypes_c_files ${_lcmtypes_c_dir}/*.c)
    file(GLOB _lcmtypes_h_files ${_lcmtypes_c_dir}/*.h)

    include_directories(BEFORE ${PROJECT_SOURCE_DIR}/lcmtypes/c) #TODO: don't think this is necessary
    include_directories(${LCM_INCLUDE_DIRS})

    # aggregate into a static library
    add_library(${libname} STATIC ${_lcmtypes_c_files})
    set_source_files_properties(${_lcmtypes_c_files} PROPERTIES COMPILE_FLAGS "-fPIC")
    #    set_target_properties("${libname}-static" PROPERTIES OUTPUT_NAME "${libname}")
    set_target_properties(${libname} PROPERTIES PREFIX "lib")
    set_target_properties(${libname} PROPERTIES CLEAN_DIRECT_OUTPUT 1)
    add_dependencies(${libname} lcmgen_c)

    #    add_library("${libname}-static" STATIC ${_lcmtypes_c_files})
    #    set_source_files_properties(${_lcmtypes_c_files} PROPERTIES COMPILE_FLAGS "-I${PROJECT_SOURCE_DIR}/lcmtypes/c")
    #    set_target_properties("${libname}-static" PROPERTIES OUTPUT_NAME "${libname}")
    #    set_target_properties("${libname}-static" PROPERTIES PREFIX "lib")
    #    set_target_properties("${libname}-static" PROPERTIES CLEAN_DIRECT_OUTPUT 1)
    #    add_dependencies("${libname}-static" lcmgen_c)

    # XXX don't build a shared library, as it makes using 3rd-party/external
    # LCM types awkward (linker will try to link external symbols at library link time, 
    # rather than executable link time)

    #    # aggregate into a shared library
    #    add_library(${libname} SHARED ${_lcmtypes_c_files})
    #    set_target_properties("${libname}" PROPERTIES CLEAN_DIRECT_OUTPUT 1)
    #    add_dependencies("${libname}" lcmgen_c)
    #    target_link_libraries(${libname} ${LCM_LDFLAGS})

    # create a header file aggregating all of the autogenerated .h files
    set(__agg_h_fname "${_lcmtypes_c_dir}/${agg_h_bname}")
    file(WRITE ${__agg_h_fname}
        "#ifndef __lcmtypes_${__sanitized_project_name}_h__\n"
        "#define __lcmtypes_${__sanitized_project_name}_h__\n\n")
    foreach(h_file ${_lcmtypes_h_files})
        file(RELATIVE_PATH __tmp_path ${_lcmtypes_c_dir} ${h_file})
        file(APPEND ${__agg_h_fname} "#include \"${__tmp_path}\"\n")
    endforeach()
    file(APPEND ${__agg_h_fname} "\n#endif\n")
    list(APPEND _lcmtypes_h_files ${__agg_h_fname})
    unset(__sanitized_project_name)
    unset(__agg_h_fname)

    # make header files and libraries public
    pods_install_libraries(${libname})
    pods_install_headers(${_lcmtypes_h_files} DESTINATION lcmtypes)

    # set some compilation variables
    set(LCMTYPES_LIBS ${libname} PARENT_SCOPE)

    # create a pkg-config file
  	pods_install_pkg_config_file(${libname}
    	CFLAGS
    	DESCRIPTION "LCM types for ${PROJECT_NAME}"
        LIBS -l${libname}
    	REQUIRES lcm
    	VERSION 0.0.0)
  
    lcmtypes_add_clean_dir("${PROJECT_SOURCE_DIR}/lcmtypes/c")
endfunction()

function(lcmtypes_build_cpp)
    lcmtypes_get_types(_lcmtypes)
    list(LENGTH _lcmtypes _num_lcmtypes)
    if(_num_lcmtypes EQUAL 0)
        return()
    endif()

    string(REGEX REPLACE "[^a-zA-Z0-9]" "_" __sanitized_project_name "${PROJECT_NAME}")

    # set some defaults

    # header file that includes all other generated header files
    set(agg_hpp_bname "${__sanitized_project_name}.hpp")

    # allow defaults to be overriden by function parameters
    set(modewords CPP_AGGREGATE_HEADER)
    set(curmode "")
    foreach(word ${ARGV})
        list(FIND modewords ${word} mode_index)
        if(${mode_index} GREATER -1)
            set(curmode ${word})
        elseif(curmode STREQUAL CPP_AGGREGATE_HEADER)
            set(agg_hpp_bname "${word}")
            set(curmode "")
        endif()
    endforeach()

    # generate CPP bindings for LCM types
    set(_lcmtypes_cpp_dir ${PROJECT_SOURCE_DIR}/lcmtypes/cpp/lcmtypes)

    # blow away any existing auto-generated files.
    file(REMOVE_RECURSE ${_lcmtypes_cpp_dir})

    # run lcm-gen now
    execute_process(COMMAND mkdir -p ${_lcmtypes_cpp_dir})
    lcmgen(--lazy --cpp --cpp-cpath ${_lcmtypes_cpp_dir} --cpp-hpath ${_lcmtypes_cpp_dir} --cpp-include lcmtypes ${_lcmtypes})

    # run lcm-gen at compile time
    add_custom_target(lcmgen_cpp ALL 
        COMMAND sh -c '[ -d ${_lcmtypes_cpp_dir} ] || mkdir -p ${_lcmtypes_cpp_dir}'
        COMMAND sh -c '${LCM_GEN_EXECUTABLE} --lazy --cpp ${_lcmtypes} --cpp-cpath ${_lcmtypes_cpp_dir} --cpp-hpath ${_lcmtypes_cpp_dir}')

    # get a list of all generated .hpp files
    file(GLOB_RECURSE _lcmtypes_hpp_files  ${_lcmtypes_cpp_dir}/*.hpp)
    
    include_directories(BEFORE ${PROJECT_SOURCE_DIR}/lcmtypes/cpp) #TODO: don't think this is necessary
    include_directories(${LCM_INCLUDE_DIRS})

    # create a header file aggregating all of the autogenerated .hpp files
    set(__agg_hpp_fname "${_lcmtypes_cpp_dir}/${agg_hpp_bname}")
    file(WRITE ${__agg_hpp_fname}
        "#ifndef __lcmtypes_${__sanitized_project_name}_hpp__\n"
        "#define __lcmtypes_${__sanitized_project_name}_hpp__\n\n")
    foreach(hpp_file ${_lcmtypes_hpp_files})
        file(RELATIVE_PATH __tmp_path ${_lcmtypes_cpp_dir} ${hpp_file})
        file(APPEND ${__agg_hpp_fname} "#include \"${__tmp_path}\"\n")
        get_filename_component(__tmp_dir ${__tmp_path} PATH)
        pods_install_headers(${hpp_file} DESTINATION lcmtypes/${__tmp_dir})
    endforeach()
    file(APPEND ${__agg_hpp_fname} "\n#endif\n")
    pods_install_headers(${__agg_hpp_fname} DESTINATION lcmtypes)
    unset(__sanitized_project_name)
    unset(__agg_hpp_fname)

 

    lcmtypes_add_clean_dir("${PROJECT_SOURCE_DIR}/lcmtypes/cpp")
endfunction()

function(lcmtypes_build_java)
    lcmtypes_get_types(_lcmtypes)
    list(LENGTH _lcmtypes _num_lcmtypes)
    if(_num_lcmtypes EQUAL 0)
        return()
    endif()

    # do we have Java?
    find_package(Java)
    if(JAVA_COMPILE STREQUAL JAVA_COMPILE-NOTFOUND OR
       JAVA_ARCHIVE STREQUAL JAVA_ARCHIVE-NOTFOUND)
        message(STATUS "Not building Java LCM type bindings (Can't find Java)")
        return()
    endif()

    # do we have LCM java bindings?  where is lcm.jar?
    execute_process(COMMAND pkg-config --variable=classpath lcm-java OUTPUT_VARIABLE LCM_JAR_FILE)
    if(NOT LCM_JAR_FILE)
        message(STATUS "Not building Java LCM type bindings (Can't find lcm.jar)")
        return()
    endif()
    string(STRIP ${LCM_JAR_FILE} LCM_JAR_FILE)
    set(LCMTYPES_JAR ${CMAKE_CURRENT_BINARY_DIR}/lcmtypes_${PROJECT_NAME}.jar)

    # generate Java bindings for LCM types
    set(_lcmtypes_java_dir ${PROJECT_SOURCE_DIR}/lcmtypes/java)
    set(auto_manage_files YES)

    set(modewords JAVA_DEST_DIR)
    set(curmode "")
    foreach(word ${ARGV})
        list(FIND modewords ${word} mode_index)
        if(${mode_index} GREATER -1)
            set(curmode ${word})
        elseif(curmode STREQUAL JAVA_DEST_DIR)
            set(_lcmtypes_java_dir "${word}")
            set(auto_manage_files NO)
            set(curmode "")
        endif()
    endforeach()

    # blow away any existing auto-generated files?
    if(auto_manage_files)
        file(REMOVE_RECURSE ${_lcmtypes_java_dir})
    endif()

    # run lcm-gen now
    execute_process(COMMAND mkdir -p ${_lcmtypes_java_dir})
    lcmgen(--lazy -j ${_lcmtypes} --jpath ${_lcmtypes_java_dir})

    # run lcm-gen at compile time
    add_custom_target(lcmgen_java ALL
        COMMAND sh -c '[ -d ${_lcmtypes_java_dir} ] || mkdir -p ${_lcmtypes_java_dir}'
        COMMAND sh -c '${LCM_GEN_EXECUTABLE} --lazy -j ${_lcmtypes} --jpath ${_lcmtypes_java_dir}')

    if(NOT auto_manage_files)
        return()
    endif()

    # get a list of all generated .java files
    file(GLOB_RECURSE _lcmtypes_java_files ${_lcmtypes_java_dir}/*.java)

    set(java_classpath ${_lcmtypes_java_dir}:${LCM_JAR_FILE})

    # search for lcmtypes_*.jar files in well-known places and add them to the
    # classpath
    foreach(pfx /usr /usr/local ${CMAKE_INSTALL_PREFIX})
        file(GLOB_RECURSE jarfiles ${pfx}/share/java/lcmtypes_*.jar)
        foreach(jarfile ${jarfiles})
            set(java_classpath ${java_classpath}:${jarfile})
            #            message("found ${jarfile}")
        endforeach()
    endforeach()

    # convert the list of .java filenames to a list of .class filenames
    foreach(javafile ${_lcmtypes_java_files})
        string(REPLACE .java .class __tmp_class_fname ${javafile})
        #        add_custom_command(OUTPUT ${__tmp_class_fname} COMMAND
        #            ${JAVA_COMPILE} -source 6 -cp ${_lcmtypes_java_dir}:${lcm_jar} ${javafile} VERBATIM DEPENDS ${javafile})
        list(APPEND _lcmtypes_class_files ${__tmp_class_fname})
        unset(__tmp_class_fname)
    endforeach()

    # add a rule to build the .class files from from the .java files
    add_custom_command(OUTPUT ${_lcmtypes_class_files} COMMAND 
        ${JAVA_COMPILE} -source 6 -cp ${java_classpath} ${_lcmtypes_java_files} 
        DEPENDS ${_lcmtypes_java_files} VERBATIM)

    # add a rule to build a .jar file from the .class files
    add_custom_command(OUTPUT lcmtypes_${PROJECT_NAME}.jar COMMAND
        ${JAVA_ARCHIVE} cf ${LCMTYPES_JAR} -C ${_lcmtypes_java_dir} . DEPENDS ${_lcmtypes_class_files} VERBATIM)
    add_custom_target(lcmtypes_${PROJECT_NAME}_jar ALL DEPENDS ${LCMTYPES_JAR})

    add_dependencies(lcmtypes_${PROJECT_NAME}_jar lcmgen_java)

    install(FILES ${LCMTYPES_JAR} DESTINATION share/java)
    set(LCMTYPES_JAR ${LCMTYPES_JAR} PARENT_SCOPE)

    lcmtypes_add_clean_dir(${_lcmtypes_java_dir})
endfunction()

function(lcmtypes_build_python)
    lcmtypes_get_types(_lcmtypes)
    list(LENGTH _lcmtypes _num_lcmtypes)
    if(_num_lcmtypes EQUAL 0)
        return()
    endif()

    find_package(PythonInterp)
    if(NOT PYTHONINTERP_FOUND)
        message(STATUS "Not building Python LCM type bindings (Can't find Python)")
        return()
    endif()

    set(_lcmtypes_python_dir ${PROJECT_SOURCE_DIR}/lcmtypes/python)
    set(auto_manage_files YES)

    set(modewords PY_DEST_DIR)
    set(curmode "")
    foreach(word ${ARGV})
        list(FIND modewords ${word} mode_index)
        if(${mode_index} GREATER -1)
            set(curmode ${word})
        elseif(curmode STREQUAL PY_DEST_DIR)
            set(_lcmtypes_python_dir "${word}")
            set(auto_manage_files NO)
            set(curmode "")
        endif()
    endforeach()

    # purge existing files?
    if(auto_manage_files)
        file(REMOVE_RECURSE ${_lcmtypes_python_dir})
    endif()

    # generate Python bindings for LCM types
    execute_process(COMMAND mkdir -p ${_lcmtypes_python_dir})
    execute_process(COMMAND ${LCM_GEN_EXECUTABLE} --lazy -p ${_lcmtypes} --ppath ${_lcmtypes_python_dir})

    # run lcm-gen at compile time
    add_custom_target(lcmgen_python ALL
        COMMAND sh -c '${LCM_GEN_EXECUTABLE} --lazy -p ${_lcmtypes} --ppath ${_lcmtypes_python_dir}')

    if(NOT auto_manage_files)
        return()
    endif()
    
    pods_install_python_packages(${_lcmtypes_python_dir})

    lcmtypes_add_clean_dir(${_lcmtypes_python_dir})
endfunction()

function(lcmtypes_install_types)
    lcmtypes_get_types(_lcmtypes)
    list(LENGTH _lcmtypes _num_lcmtypes)
    if(_num_lcmtypes EQUAL 0)
        return()
    endif()

    install(FILES ${_lcmtypes} DESTINATION share/lcmtypes)
endfunction()

macro(lcmtypes_build)
    find_package(PkgConfig REQUIRED)
    pkg_check_modules(LCM REQUIRED lcm)
    
    #find lcm-gen (it may be in the install path)
    find_program(LCM_GEN_EXECUTABLE lcm-gen ${EXECUTABLE_OUTPUT_PATH} ${EXECUTABLE_INSTALL_PATH})
    if (NOT LCM_GEN_EXECUTABLE)
    	message(FATAL_ERROR "lcm-gen not found!\n")
    	return()
    endif()
    
    lcmtypes_build_c(${ARGV})
    lcmtypes_build_cpp(${ARGV})

    lcmtypes_build_java(${ARGV})
    lcmtypes_build_python(${ARGV})
    lcmtypes_install_types()
endmacro()
