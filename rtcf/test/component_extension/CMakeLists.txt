orocos_component(component_extension component.cpp)
target_compile_definitions(component_extension PUBLIC  ORO_MEMORY_POOL ORO_BUILD_RTALLOC OROSEM_LOG4CPP_LOGGING )

# ORO_MEMORY_POOL (by rtt/os/tlsf/tlsf.h)
# OS_RT_MALLOC (by oro_malloc.h)


# additonally in deployer
# ORO_BUILD_RTALLOC
# ORO_BUILD_LOGGING
# OROSEM_LOG4CPP_LOGGING
