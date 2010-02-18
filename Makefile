OCCDIR := /opt/OpenCASCADE6.3.0/ros
LIBPATH = lib
INCPATH = include

CXXFLAGS = -Wno-pmf-conversions -Dlinux32 -Wall -I$(INCPATH) -I/usr/include/ImageMagick -ggdb3
CPPFLAGS = -I$(OCCDIR)/inc -DHAVE_CONFIG_H -I$(OCCDIR)
LDFLAGS = -Wl,-rpath,. -Wl,-rpath,$(LIBPATH) -L$(LIBPATH) -L.

LDFLAGS +=			\
	   -lTKBRep		\
	   -lTKTopAlgo		\
	   -lTKSTL		\
	   -lTKPrim		\
	   -lTKOffset		\
	   -lTKernel		\
	   -lTKG2d		\
	   -lTKShHealing	\
	   -lTKGeomBase		\
	   -lTKBO		\
	   -lTKG3d		\
	   -lTKMath		\
	   -lTKGeomAlgo		\
	   -lTKMesh		\
	   -lTKBool

LDFLAGS += $(foreach lib,$(wildcard $(OCCDIR)/adm/make/*/.libs),-L${lib}) $(foreach lib,$(realpath $(wildcard $(OCCDIR)/adm/make/*/.libs)),-Wl,-rpath,${lib})

LIBS += -lrobotinocom -lnavi
VLIBS += -lhighgui -lcv
VLIBS += -lMagick++

all: videoclient client
	./$< || $(MAKE)

libnavi.so: $(addsuffix .o,$(basename $(wildcard navigation/*.cpp)))
	$(LINK.cpp) -shared $+ -o $@ -ldl

client: main.cpp $(wildcard *.cpp) libnavi.so
	$(LINK.cpp) $< -o $@ $(LIBS)

videoclient: main.cpp $(wildcard *.cpp) libnavi.so
	$(LINK.cpp) $< -o $@ $(LIBS) $(VLIBS) -DWITH_GUI

clean:
	$(RM) client videoclient main.o navigation/*.o libnavi.so *.log
