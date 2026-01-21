set(SIMULATION_SUBDIRS 
        .
        Control
        Manager
        Model
        Wrap
        SimbodyEngine
        OpenSense)

set(SOURCES)
set(INCLUDES)
foreach(dir ${SIMULATION_SUBDIRS})
    file(GLOB SOURCES_ONEDIR ${dir}/*.cpp)
    file(GLOB INCLUDES_ONEDIR ${dir}/*.h)
    list(APPEND SOURCES ${SOURCES_ONEDIR})
    list(APPEND INCLUDES ${INCLUDES_ONEDIR})
endforeach()

set(SIMULATION_LINKLIBS ${Simbody_LIBRARIES} osimCommon)

if(OPENSIM_WITH_LEPTON)
    list(APPEND SIMULATION_LINKLIBS osimLepton)
endif()

OpenSimAddLibrary(
    KIT Simulation
    AUTHORS "Ajay_Seth-Michael_Sherman-Ayman_Habib"
    LINKLIBS ${SIMULATION_LINKLIBS}
    INCLUDES ${INCLUDES}
    SOURCES ${SOURCES}
    TESTDIRS SimbodyEngine Test
    INCLUDEDIRS ${SIMULATION_SUBDIRS}
)
