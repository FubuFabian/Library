CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

PROJECT(SinglePointUltrasoundCalibration)

IF(COMMAND CMAKE_POLICY)
	   CMAKE_POLICY(SET CMP0003 NEW)
ENDIF(COMMAND CMAKE_POLICY)

FIND_PACKAGE(ITK)
IF(ITK_FOUND)
	INCLUDE(${ITK_USE_FILE})
ELSE(ITK_FOUND)
	MESSAGE(FATAL_ERROR "Cannot build SinglePointUltrasoundCalibration without ITK.  Please set ITK_DIR.")
ENDIF(ITK_FOUND)

FIND_PACKAGE (VTK)
IF (VTK_FOUND)
   INCLUDE(${USE_VTK_FILE})
ELSE(VTK_FOUND)
	MESSAGE(FATAL_ERROR "Cannot build SinglePointUltrasoundCalibration without VTK.  Please set ITK_DIR.")   
ENDIF(VTK_FOUND)

FIND_PACKAGE(LSQRRecipes)
IF(LSQRRecipes_FOUND)
    INCLUDE(${LSQRRecipes_USE_FILE})
ELSE(LSQRRecipes_FOUND)
	MESSAGE(FATAL_ERROR "Cannot build SinglePointUltrasoundCalibration without LSQRRecipes.")   
ENDIF(LSQRRecipes_FOUND)

# Find QT 
FIND_PACKAGE(Qt4 REQUIRED)
INCLUDE( ${QT_USE_FILE} )

# support for out-of-source build
INCLUDE_DIRECTORIES(
  ${CMAKE_CURRENT_BINARY_DIR} #this is where ui_ImageViewerUI.h is generated
  ${CMAKE_CURRENT_SOURCE_DIR}
)


SET(AppSrcs main.cpp mainwindow.cpp QVTKImageWidget.cpp QVTKImageWidgetCommand.cpp 
    ProbeCalibrationWidget.cpp SinglePointUltrasoundCalibration.cpp)
    
SET(AppHeaders mainwindow.h QVTKImageWidget.h QVTKImageWidgetCommand.h 
    ProbeCalibrationWidget.h SinglePointUltrasoundCalibration.h)

SET(AppUI mainwindow.ui ProbeCalibrationWidget.ui)
 
 
# for generate qt aditional files 
QT4_WRAP_UI(UISrcs ${AppUI})
QT4_WRAP_CPP(MOCSrcs ${AppHeaders} )
 
 
# Packaging source code in same files 
SOURCE_GROUP("Resources" FILES
  ${AppUI}
)
 
SOURCE_GROUP("Generated" FILES
  ${UISrcs}
  ${MOCSrcs}
)

ADD_EXECUTABLE(SinglePointUltrasoundCalibration ${AppSrcs} ${AppHeaders} ${UISrcs} ${MOCSrcs})

TARGET_LINK_LIBRARIES(SinglePointUltrasoundCalibration QVTK ${VTK_LIBRARIES} ${ITK_LIBRARIES} LSQRRecipes)



