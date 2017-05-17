-include ../Makefile.config

PROJDIR = ../..
WORKDIR = $(PROJDIR)/toppers/oil

CPPFLAGS := $(CPPFLAGS) -I$(BOOST_DIR) -I$(PROJDIR)

CXXFILES = \
	cfg1_out.cpp \
	checker.cpp \
	configuration_manager.cpp \
	factory.cpp \
	oil_object.cpp \
	oil_parse.cpp \
	preprocess.cpp

OBJFILES := $(CXXFILES:%.cpp=%.o)

all: liboil.a

liboil.a: $(OBJFILES)
	$(AR) rcs liboil.a $(OBJFILES)

depend:
	$(CXX) $(CPPFLAGS) -M $(CXXFILES) > Makefile.depend

clean:
	-rm -f *.o *.a

-include Makefile.depend

