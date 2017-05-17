-include ../Makefile.config

PROJDIR = ..
WORKDIR = $(PROJDIR)/cfg

ifdef HAS_CFG_XML
	CPPFLAGS := $(CPPFLAGS) -I$(XERCES_DIR)
endif

CPPFLAGS := $(CPPFLAGS) -I$(BOOST_DIR) -I$(PROJDIR)
LDFLAGS := $(CXXFLAGS)

ifeq ($(OSTYPE), msys)
	LDFLAGS := $(LDFLAGS) -static-libgcc -static-libstdc++
endif

CXXFILES = \
	cfg.cpp \
	cfg0.cpp \
	cfg1.cpp \
	cfg2.cpp \
	cfg3.cpp \
	cfg4.cpp \
	cfg_version.cpp

OBJFILES := $(CXXFILES:%.cpp=%.o)

BOOST_LIBFILES := $(BOOST_LIBFILES) -lboost_system$(LIBBOOST_SUFFIX) -lboost_filesystem$(LIBBOOST_SUFFIX) -lboost_program_options$(LIBBOOST_SUFFIX) -lboost_regex$(LIBBOOST_SUFFIX)
ifdef HAS_CFG_XML
	XERCES_LIBSFILE := $(XERCES_LIBSFILE) -lxerces-c -licui18n -licuuc -licudata
endif

LIBFILES = $(PROJDIR)/toppers/itronx/libitronx.a \
	$(PROJDIR)/toppers/oil/liboil.a

ifdef HAS_CFG_XML
	LIBFILES := $(LIBFILES) $(PROJDIR)/toppers/xml/libxml.a
endif

LIBFILES := $(LIBFILES) $(PROJDIR)/toppers/libtoppers.a \

all: cfg

cfg: $(OBJFILES) $(LIBFILES)
	$(CXX) $(LDFLAGS) -o cfg $(OBJFILES) -L$(LIBBOOST_DIR) -L$(LIBXERCES_DIR) $(LIBFILES) $(BOOST_LIBFILES) $(XERCES_LIBSFILE)

depend:
	$(CXX) $(CPPFLAGS) -M $(CXXFILES) > Makefile.depend

clean:
	-rm -f *.o cfg *.exe

-include Makefile.depend
