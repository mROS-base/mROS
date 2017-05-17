-include ../../Makefile.config

PROJDIR = ../..
WORKDIR = $(PROJDIR)/toppers/itronx

CPPFLAGS := $(CPPFLAGS) -I$(BOOST_DIR) -I$(PROJDIR)

CXXFILES = \
	cfg1_out.cpp \
	checker.cpp \
	component.cpp \
	factory.cpp \
	preprocess.cpp \
	static_api.cpp

OBJFILES := $(CXXFILES:%.cpp=%.o)

all: libitronx.a

libitronx.a: $(OBJFILES)
	$(AR) rcs libitronx.a $(OBJFILES)

depend:
	$(CXX) $(CPPFLAGS) -M $(CXXFILES) > Makefile.depend

clean:
	-rm -f *.o *.a

-include Makefile.depend

