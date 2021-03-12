
#
# Components, types and plugins.
#
# The CMake 'target' names are identical to the first argument of the
# macros below, except for orocos_typegen_headers, where the target is fully
# controlled by generated code of 'typegen'.
#

# Creates a component library libwhoami-<target>.so
# and installs in the directory lib/orocos/whoami/
#
orocos_component(whoami whoami-component.hpp whoami-component.cpp) # ...you may add multiple source files
#
# You may add multiple orocos_component statements.

#
# Additional headers:
#
# Installs in the include/orocos/whoami/ directory
#
orocos_install_headers(whoami-component.hpp) # ...you may add multiple header files
#
# You may add multiple orocos_install_headers statements.

