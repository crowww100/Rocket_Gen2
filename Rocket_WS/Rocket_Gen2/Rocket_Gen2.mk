##
## Auto Generated makefile by CodeLite IDE
## any manual changes will be erased      
##
## Debug
ProjectName            :=Rocket_Gen2
ConfigurationName      :=Debug
WorkspacePath          :=/home/pi/Rocket_Gen2/Rocket_WS
ProjectPath            :=/home/pi/Rocket_Gen2/Rocket_WS/Rocket_Gen2
IntermediateDirectory  :=./Debug
OutDir                 := $(IntermediateDirectory)
CurrentFileName        :=
CurrentFilePath        :=
CurrentFileFullPath    :=
User                   :=
Date                   :=04/13/23
CodeLitePath           :=/home/pi/.codelite
LinkerName             :=/usr/bin/g++
SharedObjectLinkerName :=/usr/bin/g++ -shared -fPIC
ObjectSuffix           :=.o
DependSuffix           :=.o.d
PreprocessSuffix       :=.i
DebugSwitch            :=-g 
IncludeSwitch          :=-I
LibrarySwitch          :=-l
OutputSwitch           :=-o 
LibraryPathSwitch      :=-L
PreprocessorSwitch     :=-D
SourceSwitch           :=-c 
OutputFile             :=$(IntermediateDirectory)/$(ProjectName)
Preprocessors          :=
ObjectSwitch           :=-o 
ArchiveOutputSwitch    := 
PreprocessOnlySwitch   :=-E
ObjectsFileList        :="Rocket_Gen2.txt"
PCHCompileFlags        :=
MakeDirCommand         :=mkdir -p
LinkOptions            :=  -lwiringPi -lrt
IncludePath            :=  $(IncludeSwitch). $(IncludeSwitch)/home/pi/Rocket_Gen2/Rocket_WS/Rocket_Gen2/include 
IncludePCH             := 
RcIncludePath          := 
Libs                   := 
ArLibs                 :=  
LibPath                := $(LibraryPathSwitch). $(LibraryPathSwitch)/usr/lib $(LibraryPathSwitch)/usr/local/lib $(LibraryPathSwitch)/usr/include/c++/8 $(LibraryPathSwitch)/usr/include/ 

##
## Common variables
## AR, CXX, CC, AS, CXXFLAGS and CFLAGS can be overriden using an environment variables
##
AR       := /usr/bin/ar rcu
CXX      := /usr/bin/g++
CC       := /usr/bin/gcc
CXXFLAGS :=  -g -O0 -Wall $(Preprocessors)
CFLAGS   :=  -g -O0 -Wall $(Preprocessors)
ASFLAGS  := 
AS       := /usr/bin/as


##
## User defined environment variables
##
CodeLiteDir:=/usr/share/codelite
Objects0=$(IntermediateDirectory)/PCA9685.cpp$(ObjectSuffix) $(IntermediateDirectory)/HWT906.cpp$(ObjectSuffix) $(IntermediateDirectory)/main.cpp$(ObjectSuffix) 



Objects=$(Objects0) 

##
## Main Build Targets 
##
.PHONY: all clean PreBuild PrePreBuild PostBuild MakeIntermediateDirs
all: $(OutputFile)

$(OutputFile): $(IntermediateDirectory)/.d $(Objects) 
	@$(MakeDirCommand) $(@D)
	@echo "" > $(IntermediateDirectory)/.d
	@echo $(Objects0)  > $(ObjectsFileList)
	$(LinkerName) $(OutputSwitch)$(OutputFile) @$(ObjectsFileList) $(LibPath) $(Libs) $(LinkOptions)

MakeIntermediateDirs:
	@test -d ./Debug || $(MakeDirCommand) ./Debug


$(IntermediateDirectory)/.d:
	@test -d ./Debug || $(MakeDirCommand) ./Debug

PreBuild:


##
## Objects
##
$(IntermediateDirectory)/PCA9685.cpp$(ObjectSuffix): PCA9685.cpp $(IntermediateDirectory)/PCA9685.cpp$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "/home/pi/Rocket_Gen2/Rocket_WS/Rocket_Gen2/PCA9685.cpp" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/PCA9685.cpp$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/PCA9685.cpp$(DependSuffix): PCA9685.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/PCA9685.cpp$(ObjectSuffix) -MF$(IntermediateDirectory)/PCA9685.cpp$(DependSuffix) -MM PCA9685.cpp

$(IntermediateDirectory)/PCA9685.cpp$(PreprocessSuffix): PCA9685.cpp
	$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/PCA9685.cpp$(PreprocessSuffix) PCA9685.cpp

$(IntermediateDirectory)/HWT906.cpp$(ObjectSuffix): HWT906.cpp $(IntermediateDirectory)/HWT906.cpp$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "/home/pi/Rocket_Gen2/Rocket_WS/Rocket_Gen2/HWT906.cpp" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/HWT906.cpp$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/HWT906.cpp$(DependSuffix): HWT906.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/HWT906.cpp$(ObjectSuffix) -MF$(IntermediateDirectory)/HWT906.cpp$(DependSuffix) -MM HWT906.cpp

$(IntermediateDirectory)/HWT906.cpp$(PreprocessSuffix): HWT906.cpp
	$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/HWT906.cpp$(PreprocessSuffix) HWT906.cpp

$(IntermediateDirectory)/main.cpp$(ObjectSuffix): main.cpp $(IntermediateDirectory)/main.cpp$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "/home/pi/Rocket_Gen2/Rocket_WS/Rocket_Gen2/main.cpp" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/main.cpp$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/main.cpp$(DependSuffix): main.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/main.cpp$(ObjectSuffix) -MF$(IntermediateDirectory)/main.cpp$(DependSuffix) -MM main.cpp

$(IntermediateDirectory)/main.cpp$(PreprocessSuffix): main.cpp
	$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/main.cpp$(PreprocessSuffix) main.cpp


-include $(IntermediateDirectory)/*$(DependSuffix)
##
## Clean
##
clean:
	$(RM) -r ./Debug/


