build_exec(
  EXECNAME routing-protocol
  SOURCE_FILES routing-protocol.cc
              routing-protocol-simulator.cc
              routing-protocol-comm.cc
  LIBRARIES_TO_LINK ${ns3-libs} ${ns3-contrib-libs}
  EXECUTABLE_DIRECTORY_PATH ${CMAKE_OUTPUT_DIRECTORY}/scratch/routing
)
