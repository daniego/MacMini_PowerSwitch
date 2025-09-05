cmake_minimum_required(VERSION 3.16)
set(SDKCONFIG_DEFAULTS
  "sdkconfig.defaults"
  "sdkconfig.secrets"   # <- local, git-ignored
)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(remote_servo)
