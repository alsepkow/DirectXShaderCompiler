include_guard(GLOBAL)

# Downloads nuget.exe to the given path if it doesn't exist yet.
function(EnsureNugetExists target_path)
    # Download and install the credential provider if necessary.
    # This isn't need to download from public nuget feeds. But good to keep if we want to grab a package from a private ADO feed.
    if(NOT EXISTS $ENV{UserProfile}/.nuget/plugins/netfx/CredentialProvider.Microsoft)
        message("EnsureNugetExists:  Need Credential Provider")
        message(STATUS "Installing credential provider...")
        file(DOWNLOAD 
            https://github.com/microsoft/artifacts-credprovider/releases/download/v1.0.0/Microsoft.NuGet.CredentialProvider.zip
            $ENV{TEMP}/credProvider.zip
        )
        file(ARCHIVE_EXTRACT
            INPUT $ENV{TEMP}/credProvider.zip
            DESTINATION $ENV{TEMP}/credProvider
        )
        file(COPY 
            $ENV{TEMP}/credProvider/plugins
            DESTINATION $ENV{UserProfile}/.nuget
        )
    endif()

    # Download the latest nuget.exe to the given path.
    if(NOT EXISTS ${target_path})
        message(STATUS "Installing nuget.exe to ${target_path}...")
        file(DOWNLOAD 
            https://dist.nuget.org/win-x86-commandline/latest/nuget.exe
            ${target_path}
        )
    endif()
endfunction()

# Downloads nuget.exe to the given path if it doesn't exist yet.
function(GetNuGetPackageLatestVersion)
    set(params NAME ID SOURCE OUTPUT_DIR OUTPUT_VARIABLE)
    cmake_parse_arguments(PARSE_ARGV 0 ARG "" "${params}" "")

    if(NOT ARG_OUTPUT_DIR)
        set(ARG_OUTPUT_DIR )
    endif()

    set(nuget_exe_path "${ARG_OUTPUT_DIR}\\nuget.exe install")
    EnsureNugetExists(${nuget_exe_path})

    message("LATEST VERSION!")

    if (${ARG_ID}_LATEST_VERSION)
        message("Latest version TRUE")
        set(${ARG_OUTPUT_VARIABLE} ${${ARG_ID}_LATEST_VERSION} PARENT_SCOPE)
    else()
        message("Latest version FALSE")
        if(NOT ARG_SOURCE)
            set(ARG_SOURCE https://api.nuget.org/v3/index.json)
        endif()

        message("Begin execute_process ${ARG_ID}")
        message("NUGET EXE ${nuget_exe_path}")
        message("SOURCE ${ARG_SOURCE}")
        execute_process(
            COMMAND ${nuget_exe_path} 
            list "Microsoft.Direct3D.WARP"
            -Source ${ARG_SOURCE}
            RESULT_VARIABLE result
            OUTPUT_VARIABLE nuget_list_output
            OUTPUT_STRIP_TRAILING_WHITESPACE
        )
        message("End execute_process")

        if(NOT ${result} STREQUAL "0")
            message(FATAL_ERROR "NuGet failed to find latest version of package ${ARG_ID} with exit code ${result}.")
        endif()

        # Get last line of running nuget.exe list <ID>.
        string(REPLACE "\n" ";" nuget_list_output ${nuget_list_output})
        list(POP_BACK nuget_list_output nuget_list_last_line)
        if(nuget_list_last_line STREQUAL "No packages found.")
            message(FATAL_ERROR "NuGet failed to find latest version of package ${ARG_ID}.")
        endif()

        # The last line should have the format <ID> <VERSION>
        string(REPLACE " " ";" nuget_list_last_line ${nuget_list_last_line})
        list(POP_BACK nuget_list_last_line nuget_version)

        if(NOT nuget_version)
            message(FATAL_ERROR "NuGet failed to find latest version of package ${ARG_ID}.")
        endif()

        # Save output variable and cache the result so subsequent calls to the version-unspecified package 
        # are faster.
        set(${ARG_OUTPUT_VARIABLE} ${nuget_version} PARENT_SCOPE)
        set(${ARG_ID}_LATEST_VERSION ${nuget_version} CACHE INTERNAL "")
    endif()
endfunction()

# Installs a NuGet package under WAI_PACKAGE_CACHE. This is mainly useful for internal feeds that
# require authentication; prefer using direct downloads (e.g. FetchContent_Declare or CPMAddPackage) 
# when possible. Example:
#
# FetchNuGetPackage(
#     NAME dxc
#     ID DirectML.Dxc
#     VERSION 1.5.2010
#     SOURCE https://pkgs.dev.azure.com/microsoft/WindowsAI/_packaging/WindowsAI/nuget/v3/index.json
# )
#
# This functions sets a variable <name>_SOURCE_DIR (e.g. dxc_SOURCE_DIR in above example) to the 
# extract NuGet package contents.
function(FetchNuGetPackage)
    set(params NAME ID VERSION SOURCE OUTPUT_DIR)
    cmake_parse_arguments(PARSE_ARGV 0 ARG "" "${params}" "")

    # The NAME parameter is optional: if it's not set then the package ID is used as the name. The 
    # reason for having a separate NAME is to allow a consistent identifier for packages whose ID
    # changes with each release (e.g. GDK).
    if(NOT ARG_NAME)
        set(ARG_NAME ${ARG_ID})
    endif()

    if(NOT ARG_OUTPUT_DIR)
        set(ARG_OUTPUT_DIR ${CMAKE_BINARY_DIR}/temp)
    endif()
    
    set(nuget_exe_path ${ARG_OUTPUT_DIR}/nuget.exe)

    if(NOT ARG_SOURCE)
        set(ARG_SOURCE https://api.nuget.org/v3/index.json)
    endif()

    if(NOT ARG_VERSION)
        GetNuGetPackageLatestVersion(
            ID ${ARG_ID} 
            SOURCE ${ARG_SOURCE} 
            OUTPUT_DIR ${ARG_OUTPUT_DIR} 
            OUTPUT_VARIABLE ARG_VERSION
        )
    endif()

    set(nupkg_path ${ARG_OUTPUT_DIR}/${ARG_ID}.${ARG_VERSION}/${ARG_ID}.${ARG_VERSION}.nupkg)

    if(NOT EXISTS ${nupkg_path})
        message(STATUS "NuGet: adding package ${ARG_ID}.${ARG_VERSION}")

        EnsureNugetExists(${nuget_exe_path})

        set(retry_count 0)
        set(max_retries 10)
        set(retry_delay 10)
        set(result 1)

        # Run NuGet CLI to download the package.
        while(NOT ${result} STREQUAL "0" AND ${retry_count} LESS ${max_retries})
            message(STATUS "'${nuget_exe_path}' install '${ARG_ID}' -Version '${ARG_VERSION}' -Source '${ARG_SOURCE}' -OutputDirectory '${ARG_OUTPUT_DIR}' -DependencyVersion Ignore -Verbosity quiet")
            execute_process(
                COMMAND 
                ${nuget_exe_path} 
                install ${ARG_ID}
                -Version ${ARG_VERSION}
                -Source ${ARG_SOURCE}
                -OutputDirectory ${ARG_OUTPUT_DIR}
                -DependencyVersion Ignore
                -Verbosity quiet
                RESULT_VARIABLE result
            )
            if(NOT ${result} STREQUAL "0")
                math(EXPR retry_count "${retry_count} + 1")

                message(STATUS "Nuget failed: '${result}'. Retrying in ${retry_delay} seconds...")
                execute_process(
                    COMMAND 
                    ${CMAKE_COMMAND} -E sleep ${retry_delay}
                )
            endif()
        endwhile()

        if(NOT ${result} STREQUAL "0")
            message(FATAL_ERROR "NuGet failed: '${result}' Package  '${ARG_NAME}' (${ARG_ID}.${ARG_VERSION})")
        endif()
    endif()

    # Set output variable. The NAME parameter is optional: if it's not set then the package ID is used as the
    # name. The reason for having a separate NAME is for packages whose IDs change (e.g. GDK) so that callers
    # can use a fixed name in dependents. Example, targets can reference gdk_SOURCE_DIR with the snippet below
    # instead of having to reference Microsoft.GDK.Xbox.220300_SOURCE_DIR.
    #
    # FetchNuGetPackage(
    #     NAME gdk
    #     ID Microsoft.GDK.Xbox.220300
    #     VERSION 10.0.22000.1985-220112-2100.xb-gdk-2110co
    # )
    set(${ARG_NAME}_SOURCE_DIR ${ARG_OUTPUT_DIR}/${ARG_ID}.${ARG_VERSION} PARENT_SCOPE)
endfunction()