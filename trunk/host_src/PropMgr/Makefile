USE_CCACHE=0
USE_LUA=1
USE_DEBUG=0

srcdir=.

ROOT=
prefix=/usr/local
exec_prefix=${prefix}
bindir=$(ROOT)${exec_prefix}/bin
mandir=$(ROOT)${prefix}/man


ifeq ($(USE_CCACHE),1)
    CXX=ccache g++
    CC=ccache gcc
else
    CXX=g++
    CC=gcc    
endif

ifeq ($(USE_LUA),1)
    CFLAGS=-I. -g -pedantic -Wall -DWITH_LUA
    LDFLAGS=-llua -llualib -lm -ldl

else
    CFLAGS=-I. -g -pedantic -Wall
    LDFLAGS=
endif

ifeq ($(USE_DEBUG),1)
    CFLAGS:=$(CFLAGS) -DDEBUG
endif


TEX=pslatex
DVIPS=dvips
PS2PDF=ps2pdf
TXT2PS=enscript
TXT2PSOPT=-Ecpp -T 4 -M letter
# --color

INSTALL=/usr/bin/install -c
INSTALL_PROGRAM=${INSTALL}
INSTALL_DATA=${INSTALL} -m 644
INSTALL_DIR=mkdir -p

PROGS=varmgrtest

HEADERS=OnChangeObj.h PropertyMgrCommon.h PropertyMgr.h PropertyBranch.h PropertyLeaf.h PropertyValue.h TimeStamp.h PropertyIO.h Timer.h FDstream.h
CPPOBJS=PropertyValue.o PropertyLeaf.o PropertyBranch.o PropertyMgr.o TimeStamp.o PropertyIO.o Timer.o 

ifeq ($(USE_LUA),1)
    HEADERS:=$(HEADERS) PropMgrLuaBinding.h
    CPPOBJS:=$(CPPOBJS) PropMgrLuaBinding.o
    PROGS=varmgrtestlua
endif

COBJS=
TEXS=*.tex
CPPSRCS=*.cpp
HSRCS=*.h
SRCDOCS=$(CPPSRCS) $(HSRCS)

##########################################################################

all build: $(PROGS)

$(PROGS): $(CPPOBJS) $(COBJS) $(HEADERS) $(PROGS).cpp
	$(CXX) $(CFLAGS) -o $@ $(CPPOBJS) $@.cpp $(LDFLAGS)

#for Georg:
valgrind:
	valgrind --leak-check=yes -v --show-reachable=yes  ./varmgrtest
  
%.o : %.cpp $(HEADERS)
	$(CXX) $(CFLAGS) -c -o $@ $<

%.o : %.c $(HEADERS)
	$(CC) $(CFLAGS) -c -o $@ $<

%.h:
	echo "header file dependency: $<"

parsertest: KeyConfigParser.cpp KeyConfigLexer.cpp parsertest.cpp KeyConfigParser.o KeyConfigLexer.o KeyConfig.h Switches.h
	$(CXX) -o $@ KeyConfigParser.o KeyConfigLexer.o -Wall $@.cpp -lfl
	
KeyConfigParser.cpp: KeyConfigParser.y
	bison++ -d KeyConfigParser.y -h KeyConfigParser.h -o KeyConfigParser.cpp

KeyConfigLexer.cpp: KeyConfigLexer.l
	flex++ -oKeyConfigLexer.cpp KeyConfigLexer.l
	

install: all
	$(INSTALL_DIR) $(bindir)
	$(INSTALL_PROGRAM) -s main $(bindir)

report: $(TEXS)
	@for file in $(TEXS) ; do make `basename $$file .tex`.dvi ; done
	@for file in $(TEXS) ; do make `basename $$file .tex`.ps ; done
	@for file in $(TEXS) ; do make `basename $$file .tex`.pdf ; done

doc: report srcdocs

srcdocs: $(SRCDOCS)
	@for file in $(SRCDOCS) ; do make $$file.ps ; done
	@for file in $(SRCDOCS) ; do make `basename $$file .ps`.pdf ; done

%.pdf : %.ps
	$(PS2PDF) $< $@

%.cpp.pdf : %.cpp.ps
	$(PS2PDF) $< $@

%.h.pdf : %.h.ps
	$(PS2PDF) $< $@

%.ps : %.dvi
	$(DVIPS) -o $@ $<

%.cpp.ps: %.cpp
	$(TXT2PS) $(TXT2PSOPT) -o $@ $< || true

%.h.ps: %.h
	$(TXT2PS) $(TXT2PSOPT) -o $@ $< || true

%.dvi : %.tex
	$(TEX) $<

clean:
	rm -f $(CPPOBJS) $(COBJS) core* main $(PROGS)

doc-clean:
	rm -f *.pdf *.ps *.dvi *.aux *.log

dep depend install:
