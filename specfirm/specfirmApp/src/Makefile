TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

INC += ADSpecfirm.h
LIBRARY_IOC = specfirm
LIB_SRCS += ADSpecfirm.cpp
DBD += specfirmSupport.dbd

# ADBase, asyn and the rest of an areaDetector driver's libraries
include $(ADCORE)/ADApp/commonLibraryMakefile

#=============================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE
