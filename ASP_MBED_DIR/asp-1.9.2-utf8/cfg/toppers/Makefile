-include ../Makefile.config

PROJDIR = ..
WORKDIR = $(PROJDIR)/toppers

CPPFLAGS := $(CPPFLAGS) -I$(BOOST_DIR) -I$(PROJDIR)

CXXFILES = \
	builtin_function.cpp \
	c_chlit_parser.cpp \
	c_ident_parser.cpp \
	c_int_parser.cpp \
	c_keywords.cpp \
	c_plus_plus_keywords.cpp \
	c_strlit_parser.cpp \
	cpp.cpp \
	diagnostics.cpp \
	gettext.cpp \
	global.cpp \
	io.cpp \
	macro_processor.cpp \
	mbchar_parser.cpp \
	nm_symbol.cpp \
	output_file.cpp \
	s_record.cpp \
	ucn_parser.cpp \

OBJFILES := $(CXXFILES:%.cpp=%.o)

all: libtoppers.a

libtoppers.a: $(OBJFILES)
	$(AR) rcs libtoppers.a $(OBJFILES)

depend:
	$(CXX) $(CPPFLAGS) -M $(CXXFILES) > Makefile.depend

clean:
	-rm -f *.o *.a

-include Makefile.depend

