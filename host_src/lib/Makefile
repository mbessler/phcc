#set this to 1 if you want to build under Windows:
WIN32=0
# set this to 1 if you are under unix but want windows binaries
CROSSCOMPILE=0
#


USE_CCACHE=0
USE_DEBUG=0

CXX=g++
CC=gcc

SUBDIRS=../lib
VPATH+=$(SUBDIRS)

INCLUDES=$(SUBDIRS:%=-I%)

CFLAGS=-I. -g -pedantic -Wall $(INCLUDES)
LDFLAGS=
EXEC_EXT=
LIB_EXT=.o
COMPILERPREFIX=

ifeq ($(CROSSCOMPILE),1)
  COMPILERPREFIX=/opt/xmingw/bin/i386-mingw32msvc-
endif

ifeq ($(USE_CCACHE),1)
  CXX:=ccache $(CXX)
  CC:=ccache $(CC)
endif


ifeq ($(WIN32),1)
  CFLAGS+=-D__WIN32__
  EXEC_EXT=.exe
  LIB_EXT=.lib
endif

ifeq ($(USE_DEBUG),1)
  CFLAGS+=-DDEBUG
endif


#ifeq ($(origin COMPILERPREFIX), undefined)
ifdef COMPILERPREFIX
  CXX:=$(COMPILERPREFIX)$(CXX)
  CC:=$(COMPILERPREFIX)$(CC)
endif

CXX_PROGS=testPHCC_Serial testMB_Opt
C_PROGS=
PROGS=$(C_PROGS) $(CXX_PROGS)

HEADERS=PHCC_Serial.h MB_Opt.h


testPHCC_Serial_OBJS=testPHCC_Serial PHCC_Serial
testPHCC_Serial_LIBS=
testPHCC_Serial_CFLAGS=

testMB_Opt_OBJS=testMB_Opt MB_Opt
testMB_Opt_LIBS=
testMB_Opt_CFLAGS=


ifeq ($(WIN32),1)
  $(CPPOBJS:%=%$(LIB_EXT))
endif

##########################################################################
all build: $(PROGS)

define CXX_PROGRAM_template
$(1): $$($(1)_OBJ) $$($(1)_OBJS:%=%$$(LIB_EXT))
	$$(CXX) $$(LDFLAGS) -o $$@$$(EXEC_EXT) $$^
ALL_OBJS += $$($(1)_OBJS:%=%$$(LIB_EXT))
endef

define C_PROGRAM_template
$(1): $$($(1)_OBJ) $$($(1)_OBJS:%=%$$(LIB_EXT))
	$$(CC) $$(LDFLAGS) -o $$@$$(EXEC_EXT) $$^
ALL_OBJS += $$($(1)_OBJS:%=%$$(LIB_EXT))
endef

$(foreach prog,$(CXX_PROGS),$(eval $(call CXX_PROGRAM_template,$(prog))))
$(foreach prog,$(C_PROGS),$(eval $(call C_PROGRAM_template,$(prog))))


.SUFFIXES:

%.lib : %.cpp $(HEADERS)
	$(CXX) $(CFLAGS) -c -o $@ $<

%.lib : %.c $(HEADERS)
	$(CC) $(CFLAGS) -c -o $@ $<

%.o : %.cpp $(HEADERS)
	$(CXX) $(CFLAGS) -c -o $@ $<

%.o : %.c $(HEADERS)
	$(CC) $(CFLAGS) -c -o $@ $<

%.h:
	@#echo "header file dependency: $<"

clean:
	rm -f core* $(PROGS) $(PROGS:%=%.exe) *.o *.lib $(ALL_OBJS)

doc-clean:
	rm -f *.pdf *.ps *.dvi *.aux *.log

dep depend install:
