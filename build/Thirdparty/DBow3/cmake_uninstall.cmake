# -----------------------------------------------
# File that provides "make uninstall" target
#  We use the file 'install_manifest.txt'
# -----------------------------------------------
IF(NOT EXISTS "/home/geneta/Project/geneta_slam/build/Thirdparty/DBow3/install_manifest.txt")
  MESSAGE(FATAL_ERROR "Cannot find install manifest: \"/home/geneta/Project/geneta_slam/build/Thirdparty/DBow3/install_manifest.txt\"")
ENDIF(NOT EXISTS "/home/geneta/Project/geneta_slam/build/Thirdparty/DBow3/install_manifest.txt")

FILE(READ "/home/geneta/Project/geneta_slam/build/Thirdparty/DBow3/install_manifest.txt" files)
STRING(REGEX REPLACE "\n" ";" files "${files}")
FOREACH(file ${files})
  MESSAGE(STATUS "Uninstalling \"$ENV{DESTDIR}${file}\"")
#  IF(EXISTS "$ENV{DESTDIR}${file}")
#    EXEC_PROGRAM(
<<<<<<< HEAD
#      "/home/geneta/project/tools/cmake-3.27.0-linux-x86_64/bin/cmake" ARGS "-E remove \"$ENV{DESTDIR}${file}\""
=======
#      "/usr/bin/cmake" ARGS "-E remove \"$ENV{DESTDIR}${file}\""
>>>>>>> 723770ab0906c835a1ea9745461e7ef4041be37b
#      OUTPUT_VARIABLE rm_out
#      RETURN_VALUE rm_retval
#      )
	EXECUTE_PROCESS(COMMAND rm $ENV{DESTDIR}${file})
#    IF(NOT "${rm_retval}" STREQUAL 0)
#      MESSAGE(FATAL_ERROR "Problem when removing \"$ENV{DESTDIR}${file}\"")
#    ENDIF(NOT "${rm_retval}" STREQUAL 0)
#  ELSE(EXISTS "$ENV{DESTDIR}${file}")
#    MESSAGE(STATUS "File \"$ENV{DESTDIR}${file}\" does not exist.")
#  ENDIF(EXISTS "$ENV{DESTDIR}${file}")
ENDFOREACH(file)


