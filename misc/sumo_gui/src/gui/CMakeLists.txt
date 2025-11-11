add_subdirectory(dialogs)

set(gui_STAT_SRCS
   GUIApplicationWindow.cpp
   GUIApplicationWindow.h
   GUIEvent_SimulationEnded.h
   GUIEvent_SimulationLoaded.h
   GUIGlobals.cpp
   GUIGlobals.h
   GUIManipulator.h
   GUIManipulator.cpp
   GUILoadThread.cpp
   GUILoadThread.h
   GUIRunThread.cpp
   GUIRunThread.h
   GUISUMOViewParent.cpp
   GUISUMOViewParent.h
   GUITLLogicPhasesTrackerWindow.cpp
   GUITLLogicPhasesTrackerWindow.h
   GUIViewTraffic.cpp
   GUIViewTraffic.h
   TraCIServerAPI_GUI.cpp
   TraCIServerAPI_GUI.h
)

add_library(gui STATIC ${gui_STAT_SRCS})
target_link_libraries(gui /usr/lib/x86_64-linux-gnu/libGL.so /usr/lib/x86_64-linux-gnu/libGLU.so /usr/lib/x86_64-linux-gnu/libglut.so /usr/lib/x86_64-linux-gnu/libGLEW.so)

set(CMAKE_CXX_FLAGS "-lGL -lGLU -lGLEW -lglut -lpthread")
add_dependencies(gui generate-version-h)
set_property(TARGET gui PROPERTY PROJECT_LABEL "z_gui")
