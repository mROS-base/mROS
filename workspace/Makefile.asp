#
#  TOPPERS/ASP Kernel
#      Toyohashi Open Platform for Embedded Real-Time Systems/
#      Advanced Standard Profile Kernel
# 
#  Copyright (C) 2000-2003 by Embedded and Real-Time Systems Laboratory
#                              Toyohashi Univ. of Technology, JAPAN
#  Copyright (C) 2006-2014 by Embedded and Real-Time Systems Laboratory
#              Graduate School of Information Science, Nagoya Univ., JAPAN
# 
#  上記著作権者は，以下の(1)～(4)の条件を満たす場合に限り，本ソフトウェ
#  ア（本ソフトウェアを改変したものを含む．以下同じ）を使用・複製・改
#  変・再配布（以下，利用と呼ぶ）することを無償で許諾する．
#  (1) 本ソフトウェアをソースコードの形で利用する場合には，上記の著作
#      権表示，この利用条件および下記の無保証規定が，そのままの形でソー
#      スコード中に含まれていること．
#  (2) 本ソフトウェアを，ライブラリ形式など，他のソフトウェア開発に使
#      用できる形で再配布する場合には，再配布に伴うドキュメント（利用
#      者マニュアルなど）に，上記の著作権表示，この利用条件および下記
#      の無保証規定を掲載すること．
#  (3) 本ソフトウェアを，機器に組み込むなど，他のソフトウェア開発に使
#      用できない形で再配布する場合には，次のいずれかの条件を満たすこ
#      と．
#    (a) 再配布に伴うドキュメント（利用者マニュアルなど）に，上記の著
#        作権表示，この利用条件および下記の無保証規定を掲載すること．
#    (b) 再配布の形態を，別に定める方法によって，TOPPERSプロジェクトに
#        報告すること．
#  (4) 本ソフトウェアの利用により直接的または間接的に生じるいかなる損
#      害からも，上記著作権者およびTOPPERSプロジェクトを免責すること．
#      また，本ソフトウェアのユーザまたはエンドユーザからのいかなる理
#      由に基づく請求からも，上記著作権者およびTOPPERSプロジェクトを
#      免責すること．
# 
#  本ソフトウェアは，無保証で提供されているものである．上記著作権者お
#  よびTOPPERSプロジェクトは，本ソフトウェアに関して，特定の使用目的
#  に対する適合性も含めて，いかなる保証も行わない．また，本ソフトウェ
#  アの利用により直接的または間接的に生じたいかなる損害に関しても，そ
#  の責任を負わない．
# 
#  $Id: Makefile 2728 2015-12-30 01:46:11Z ertl-honda $
# 

#
#  ターゲットの指定（Makefile.targetで上書きされるのを防ぐため）
#
all:

#
#  OSの判定
#
ifeq ($(OS),Windows_NT)
  UNAME := Windows
else
ifeq ($(shell uname),Linux)
  UNAME := Linux
endif
ifeq ($(shell uname),Darwin)
  UNAME := Darwin
endif
endif

#
#  ターゲット略称の定義
#
TARGET = gr_peach_gcc

#
#  プログラミング言語の定義
#
SRCLANG = c++
ifeq ($(SRCLANG),c)
  LIBS = -lc
endif
ifeq ($(SRCLANG),c++)
  USE_CXX = true
#  CXXLIBS = -lstdc++ -lm -lc
#  CXXRTS = cxxrt.o newlibrt.o
endif

#
#  ソースファイルのディレクトリの定義
#
# SRCDIR = ..

#
#  オブジェクトファイル名の拡張子の設定
#
OBJEXT = elf

#
#  実行環境の定義（ターゲット依存に上書きされる場合がある）
#
DBGENV := 

#
#  カーネルライブラリ（libkernel.a）のディレクトリ名
#  （カーネルライブラリもmake対象にする時は，空に定義する）
#
# KERNEL_LIB =

#
#  カーネルを関数単位でコンパイルするかどうかの定義
#
KERNEL_FUNCOBJS = 

#
#  トレースログを取得するかどうかの定義
#
ENABLE_TRACE = 

#EXECUTE_ON = RAM
#EXECUTE_ON = ROM

#
#  ユーティリティプログラムの名称
#
PERL = /usr/bin/perl
ifeq ($(UNAME),Windows)
  CFG = $(SRCDIR)/../../cfg_binary/cfg-mingw-static-1_9_6.exe
else
ifeq ($(UNAME),Linux)
  CFG = $(SRCDIR)/../../cfg_binary/cfg-linux-static-1_9_6
else
ifeq ($(UNAME),Darwin)
  CFG = $(SRCDIR)/../../cfg_binary/cfg-osx-static-1_9_5
else
endif
endif
endif

#
#  オブジェクトファイル名の定義
#
OBJNAME = asp
ifdef OBJEXT
  OBJFILE = $(OBJNAME).$(OBJEXT)
  CFG1_OUT = cfg1_out.$(OBJEXT)
else
  OBJFILE = $(OBJNAME)
  CFG1_OUT = cfg1_out
endif

#
#  ターゲット依存部のディレクトリの定義
#
TARGETDIR = $(SRCDIR)/target/$(TARGET)

#
#  ターゲット依存の定義のインクルード
#
include $(TARGETDIR)/Makefile.target

#
#  コンフィギュレータ関係の変数の定義
#
CFG_TABS := --api-table $(SRCDIR)/kernel/kernel_api.csv \
			--cfg1-def-table $(SRCDIR)/kernel/kernel_def.csv $(CFG_TABS)

CFG_ASMOBJS := $(CFG_ASMOBJS)
CFG_COBJS := kernel_cfg.o $(CFG_COBJS)
CFG_OBJS := $(CFG_ASMOBJS) $(CFG_COBJS)
CFG2_OUT_SRCS := kernel_cfg.h kernel_cfg.c $(CFG2_OUT_SRCS)

#
#  共通コンパイルオプションの定義
#
COPTS := $(COPTS) -g -O0 -ggdb
ifndef OMIT_WARNING_ALL
  COPTS := $(COPTS) -Wall
endif
ifndef OMIT_OPTIMIZATION
  COPTS := $(COPTS) -O2
endif
CDEFS := $(CDEFS) 
INCLUDES := -I. -I$(SRCDIR)/include -I$(SRCDIR)/arch -I$(SRCDIR) $(INCLUDES)
LDFLAGS := $(LDFLAGS) 
CFG1_OUT_LDFLAGS := $(CFG1_OUT_LDFLAGS) 
LIBS := $(LIBS) $(CXXLIBS)
CFLAGS = $(COPTS) $(CDEFS) $(INCLUDES)

#
#  アプリケーションプログラムに関する定義
#
# APPLNAME = sample1
# APPLDIR = 
# APPL_CFG = $(APPLNAME).cfg

APPL_DIR = $(APPLDIR) $(SRCDIR)/library
APPL_ASMOBJS := $(APPL_ASMOBJS)
ifdef USE_CXX
  APPL_CXXOBJS := $(APPLNAME).o $(APPL_CXXOBJS)
  APPL_COBJS := $(APPL_COBJS)
else
  APPL_COBJS = $(APPLNAME).o 
endif
APPL_COBJS := $(APPL_COBJS) log_output.o vasyslog.o t_perror.o strerror.o
# APPL_CFLAGS =
# APPL_LIBS =
ifdef APPLDIR
  INCLUDES := $(INCLUDES) $(foreach dir,$(APPLDIR),-I$(dir)) $(OPENCV_INCLUDE_PATHS)
  INCLUDE_PATHS :=  $(INCLUDE_PATHS) $(foreach dir,$(APPLDIR),-I$(dir))
endif

#
#  システムサービスに関する定義
#
SYSSVC_DIR := $(SYSSVC_DIR) $(SRCDIR)/syssvc $(SRCDIR)/library
SYSSVC_ASMOBJS := $(SYSSVC_ASMOBJS)
SYSSVC_COBJS := $(SYSSVC_COBJS) banner.o syslog.o serial.o logtask.o $(CXXRTS)
				 
SYSSVC_CFLAGS := $(SYSSVC_CFLAGS)
SYSSVC_LIBS := $(SYSSVC_LIBS)
INCLUDES := $(INCLUDES)

#
#  カーネルに関する定義
#
#  KERNEL_ASMOBJS: カーネルライブラリに含める，ソースがアセンブリ言語の
#				   オブジェクトファイル．
#  KERNEL_COBJS: カーネルのライブラリに含める，ソースがC言語で，ソース
#				 ファイルと1対1に対応するオブジェクトファイル．
#  KERNEL_LCSRCS: カーネルのライブラリに含めるC言語のソースファイルで，
#				  1つのソースファイルから複数のオブジェクトファイルを生
#				  成するもの．
#  KERNEL_LCOBJS: 上のソースファイルから生成されるオブジェクトファイル．
#  KERNEL_AUX_COBJS: ロードモジュールに含めないが，カーネルのソースファ
#					 イルと同じオプションを適用してコンパイルすべき，ソー
#					 スがC言語のオブジェクトファイル．
#
KERNEL_DIR := $(KERNEL_DIR) $(SRCDIR)/kernel
KERNEL_ASMOBJS := $(KERNEL_ASMOBJS)
KERNEL_COBJS := $(KERNEL_COBJS)
KERNEL_CFLAGS := $(KERNEL_CFLAGS) -I$(SRCDIR)/kernel
ifdef OMIT_MAKEOFFSET
  OFFSET_H =
else
  OFFSET_H = offset.h
ifndef OFFSET_TF
  KERNEL_AUX_COBJS := $(KERNEL_AUX_COBJS) makeoffset.o
endif
endif

#
#  ターゲットファイル（複数を同時に選択してはならない）
#
#all: $(OBJFILE)
all: $(OBJNAME).bin
#all: $(OBJNAME).srec

##### 以下は編集しないこと #####

#
#  環境に依存するコンパイルオプションの定義
#
ifdef DBGENV
  CDEFS := $(CDEFS) -D$(DBGENV)
endif

#
#  カーネルのファイル構成の定義
#
include $(SRCDIR)/kernel/Makefile.kernel
ifdef KERNEL_FUNCOBJS
  KERNEL_LCSRCS := $(KERNEL_FCSRCS)
  KERNEL_LCOBJS := $(foreach file,$(KERNEL_FCSRCS),$($(file:.c=)))
else
  KERNEL_CFLAGS := -DALLFUNC $(KERNEL_CFLAGS)
  KERNEL_COBJS := $(KERNEL_COBJS) \
					$(foreach file,$(KERNEL_FCSRCS),$(file:.c=.o))
endif

#
#  ソースファイルのあるディレクトリに関する定義
#
vpath %.c $(KERNEL_DIR) $(SYSSVC_DIR) $(APPL_DIR)
vpath %.cpp $(KERNEL_DIR) $(SYSSVC_DIR) $(APPL_DIR)
vpath %.S $(KERNEL_DIR) $(SYSSVC_DIR) $(APPL_DIR)
vpath %.cfg $(APPL_DIR)

#
#  コンパイルのための変数の定義
#
KERNEL_LIB_OBJS = $(KERNEL_ASMOBJS) $(KERNEL_COBJS) $(KERNEL_LCOBJS)
SYSSVC_OBJS = $(SYSSVC_ASMOBJS) $(SYSSVC_COBJS)
APPL_OBJS = $(APPL_ASMOBJS) $(APPL_COBJS) $(APPL_CXXOBJS)
ALL_OBJS = $(START_OBJS) $(APPL_OBJS) $(SYSSVC_OBJS) $(CFG_OBJS) \
											$(END_OBJS) $(HIDDEN_OBJS)
ifdef KERNEL_LIB
#libmros.a libmbed.aの順番で書く $(APPL_LIBS)で記述
  ALL_LIBS = libmros.a libmbed.a $(SYSSVC_LIBS) -lkernel $(LIBS) 
  LIBS_DEP = $(filter %.a,$(ALL_LIBS)) $(KERNEL_LIB)/libkernel.a
  LDFLAGS := $(LDFLAGS) -L$(KERNEL_LIB)
  REALCLEAN_FILES := libkernel.a $(REALCLEAN_FILES)
else
  ALL_LIBS = libmros.a libmbed.a $(SYSSVC_LIBS) libkernel.a $(LIBS)
  LIBS_DEP = $(filter %.a,$(ALL_LIBS))
endif

ifdef TEXT_START_ADDRESS
  LDFLAGS := $(LDFLAGS) -Wl,-Ttext,$(TEXT_START_ADDRESS)
  CFG1_OUT_LDFLAGS := $(CFG1_OUT_LDFLAGS) -Wl,-Ttext,$(TEXT_START_ADDRESS)
endif
ifdef DATA_START_ADDRESS
  LDFLAGS := $(LDFLAGS) -Wl,-Tdata,$(DATA_START_ADDRESS)
  CFG1_OUT_LDFLAGS := $(CFG1_OUT_LDFLAGS) -Wl,-Tdata,$(DATA_START_ADDRESS)
endif
ifdef LDSCRIPT
  LDFLAGS := $(LDFLAGS) -T $(LDSCRIPT)
  CFG1_OUT_LDFLAGS := $(CFG1_OUT_LDFLAGS) -T $(LDSCRIPT)
endif

#
#  オフセットファイル（offset.h）の生成規則
#
ifdef OFFSET_TF
offset.h: $(APPL_CFG) kernel_cfg.timestamp
	$(CFG) --pass 3 --kernel asp $(INCLUDES) \
				--rom-image cfg1_out.srec --symbol-table cfg1_out.syms \
				-T $(OFFSET_TF) $(CFG_TABS) $<
else
offset.h: makeoffset.s $(SRCDIR)/utils/genoffset
	$(PERL) $(SRCDIR)/utils/genoffset makeoffset.s > offset.h
endif

#
#  カーネルのコンフィギュレーションファイルの生成
#
cfg1_out.c: $(APPL_CFG)
	$(CFG) --pass 1 --kernel asp $(INCLUDES) $(CFG_TABS) $<

$(CFG2_OUT_SRCS): kernel_cfg.timestamp
kernel_cfg.timestamp: $(APPL_CFG) \
						$(START_OBJS) cfg1_out.o $(END_OBJS) $(HIDDEN_OBJS)
	$(LINK) $(CFLAGS) $(CFG1_OUT_LDFLAGS) -o $(CFG1_OUT) \
						$(START_OBJS) cfg1_out.o $(END_OBJS)
	$(NM) -n $(CFG1_OUT) > cfg1_out.syms
	$(OBJCOPY) -O srec -S $(CFG1_OUT) cfg1_out.srec
	$(CFG) --pass 2 --kernel asp $(INCLUDES) \
				-T $(TARGETDIR)/target.tf $(CFG_TABS) $<
ifneq ($(UNAME),Windows)
	touch -r kernel_cfg.c kernel_cfg.timestamp
else
	cmd /c copy /B kernel_cfg.c +,,
	cmd /c echo  -n > kernel_cfg.timestamp
#	touch -r kernel_cfg.c kernel_cfg.timestamp
endif

#
#  カーネルライブラリファイルの生成
#
libkernel.a: $(OFFSET_H) $(KERNEL_LIB_OBJS)
	rm -f libkernel.a
	$(AR) -rcs libkernel.a $(KERNEL_LIB_OBJS)
	$(RANLIB) libkernel.a

#
#  特別な依存関係の定義
#
banner.o: kernel_cfg.timestamp $(filter-out banner.o,$(ALL_OBJS)) $(LIBS_DEP)

#
#  全体のリンク
#	$(APPL_OBJS)の位置が変？デフォルト\のあと
#   OpenCVのリンクが必要
$(OBJFILE): $(APPL_CFG) kernel_cfg.timestamp $(ALL_OBJS) $(LIBS_DEP)
	$(LINK) $(CFLAGS) $(LDFLAGS) -o $(OBJFILE) $(START_OBJS) \
			$(APPL_OBJS) $(SYSSVC_OBJS) $(CFG_OBJS) $(ALL_LIBS) $(OPENCV_LIBS) $(END_OBJS) 
	$(NM) -n $(OBJFILE) > $(OBJNAME).syms
	$(OBJCOPY) -O srec -S $(OBJFILE) $(OBJNAME).srec
	$(CFG) --pass 3 --kernel asp $(INCLUDES) \
				--rom-image $(OBJNAME).srec --symbol-table $(OBJNAME).syms \
				-T $(TARGETDIR)/target_check.tf $(CFG_TABS) $<

#
#  バイナリファイルの生成
#
$(OBJNAME).bin: $(OBJFILE)
	$(OBJCOPY) -O binary -S $(OBJFILE) $(OBJNAME).bin
#	cp -X $(OBJNAME).bin /Volumes/MBED # for MacOSX
	
#
#  Sレコードファイルの生成
#
$(OBJNAME).srec: $(OBJFILE)
	$(OBJCOPY) -O srec -S $(OBJFILE) $(OBJNAME).srec

#
#  コンパイル結果の消去
#
.PHONY: clean
clean:
ifneq ($(USE_TRUESTUDIO),true)
	rm -f $(LIB).a $(ALL_OBJ) $(DEPS)
	rm -f \#* *~ *.o *.a $(CLEAN_FILES)
	rm -f $(OBJFILE) $(OBJNAME).syms $(OBJNAME).srec $(OBJNAME).bin
	rm -f kernel_cfg.timestamp $(CFG2_OUT_SRCS)
	rm -f cfg1_out.c $(CFG1_OUT) cfg1_out.syms cfg1_out.srec
ifndef KERNEL_LIB
	rm -f libkernel.a
endif
	rm -f makeoffset.s offset.h
else
	-rm -f $(LIB).a $(ALL_OBJ) $(DEPS)
	-rm -f *.o *.a *.d $(CLEAN_FILES)
	-rm -f $(OBJFILE) $(OBJNAME).syms $(OBJNAME).srec $(OBJNAME).bin
	-rm -f kernel_cfg.timestamp $(CFG2_OUT_SRCS)
	-rm -f cfg1_out.c $(CFG1_OUT) cfg1_out.syms cfg1_out.srec
ifndef KERNEL_LIB
	-rm -f libkernel.a
endif
	-rm -f makeoffset.s offset.h
endif


.PHONY: cleankernel
cleankernel:
ifneq ($(USE_TRUESTUDIO),true)
	rm -rf $(KERNEL_LIB_OBJS)
	rm -f makeoffset.s offset.h
else
	-rm -rf $(KERNEL_LIB_OBJS)
	-rm -f makeoffset.s offset.h
endif

.PHONY: cleandep
cleandep:
	if ! [ -f Makefile.depend ]; then \
		rm -f kernel_cfg.timestamp $(CFG2_OUT_SRCS); \
		rm -f cfg1_out.c cfg1_out.o $(CFG1_OUT) cfg1_out.syms cfg1_out.srec; \
		rm -f makeoffset.s offset.h; \
	fi
	rm -f Makefile.depend

.PHONY: realclean
ifneq ($(USE_TRUESTUDIO),true)
realclean: cleandep clean
	rm -f $(REALCLEAN_FILES)
else
realclean: cleandep clean
	-rm -f $(REALCLEAN_FILES)
endif

#
#  コンフィギュレータが生成したファイルのコンパイルルールと依存関係作成
#  ルールの定義
#
#  コンフィギュレータが生成したファイルは，アプリケーションプログラム用，
#  システムサービス用，カーネル用のすべてのオプションを付けてコンパイル
#  する．
#
ALL_CFG_COBJS = $(CFG_COBJS) cfg1_out.o
ALL_CFG_ASMOBJS = $(CFG_ASMOBJS)
CFG_CFLAGS = $(APPL_CFLAGS) $(SYSSVC_CFLAGS) $(KERNEL_CFLAGS)

$(ALL_CFG_COBJS): %.o: %.c
	$(CC) -c $(CFLAGS) $(CFG_CFLAGS) $<

ifneq ($(USE_TRUESTUDIO),true)
$(ALL_CFG_COBJS:.o=.s): %.s: %.c
	$(CC) -S $(CFLAGS) $(CFG_CFLAGS) $<
endif

$(ALL_CFG_COBJS:.o=.d): %.d: %.c
	@$(PERL) $(SRCDIR)/utils/makedep -C $(CC) $(MAKEDEP_OPTS) \
		-O "$(CFLAGS) $(CFG_CFLAGS)" $< >> Makefile.depend

$(ALL_CFG_ASMOBJS): %.o: %.S
	$(CC) -c $(CFLAGS) $(CFG_CFLAGS) $<

$(ALL_CFG_ASMOBJS:.o=.d): %.d: %.S
	@$(PERL) $(SRCDIR)/utils/makedep -C $(CC) $(MAKEDEP_OPTS) \
		-O "$(CFLAGS) $(CFG_CFLAGS)" $< >> Makefile.depend

#
#  特殊な依存関係作成ルールの定義
#
cfg1_out.depend: $(APPL_CFG)
	@$(CFG) -M cfg1_out.c $(INCLUDES) $< >> Makefile.depend

makeoffset.d: makeoffset.c
	@$(PERL) $(SRCDIR)/utils/makedep -C $(CC) $(MAKEDEP_OPTS) \
		-T "makeoffset.s" -O "$(CFLAGS) $(KERNEL_CFLAGS)" $< >> Makefile.depend

#
#  依存関係ファイルの生成
#
.PHONY: gendepend
gendepend:
	@echo "Generating Makefile.depend."

.PHONY: deparduino
deparduino:
	if [ -f arduino_app.h ]; then \
		touch arduino_app.h; \
	fi

.PHONY: depend
ifdef KERNEL_LIB
depend: cleandep kernel_cfg.timestamp gendepend deparduino\
		cfg1_out.depend cfg1_out.d \
		$(ALL_OBJS:.o=.d)
else
depend: cleandep $(OFFSET_H) kernel_cfg.timestamp gendepend deparduino\
		cfg1_out.depend cfg1_out.d \
		$(KERNEL_AUX_COBJS:.o=.d) $(KERNEL_ASMOBJS:.o=.d) \
		$(KERNEL_COBJS:.o=.d) $(KERNEL_LCSRCS:.c=.d) $(ALL_OBJS:.o=.d)
endif

#
#  依存関係ファイルをインクルード
#
-include Makefile.depend

#
#  開発ツールのコマンド名の定義
#
ifeq ($(TOOL),gcc)
  #
  #  GNU開発環境用
  #
  ifdef GCC_TARGET
    GCC_TARGET_PREFIX = $(GCC_TARGET)-
  else
    GCC_TARGET_PREFIX =
  endif
  CC = $(GCC_TARGET_PREFIX)gcc
  CXX = $(GCC_TARGET_PREFIX)g++
  AS = $(GCC_TARGET_PREFIX)as
  LD = $(GCC_TARGET_PREFIX)ld
  AR = $(GCC_TARGET_PREFIX)ar
  NM = $(GCC_TARGET_PREFIX)nm
  RANLIB = $(GCC_TARGET_PREFIX)ranlib
  OBJCOPY = $(GCC_TARGET_PREFIX)objcopy
  OBJDUMP = $(GCC_TARGET_PREFIX)objdump
endif

ifdef USE_CXX
  LINK = $(CXX)
else
  LINK = $(CC)
endif

#
#  コンパイルルールの定義
#
KERNEL_ALL_COBJS = $(KERNEL_COBJS) $(KERNEL_AUX_COBJS)

$(KERNEL_ALL_COBJS): %.o: %.c
	$(CC) -c $(CFLAGS) $(KERNEL_CFLAGS) $<

ifneq ($(USE_TRUESTUDIO),true)
$(KERNEL_ALL_COBJS:.o=.s): %.s: %.c
	$(CC) -S $(CFLAGS) $(KERNEL_CFLAGS) $<
endif

$(KERNEL_LCOBJS): %.o:
	$(CC) -DTOPPERS_$(*F) -o $@ -c $(CFLAGS) $(KERNEL_CFLAGS) $<

$(KERNEL_LCOBJS:.o=.s): %.s:
	$(CC) -DTOPPERS_$(*F) -o $@ -S $(CFLAGS) $(KERNEL_CFLAGS) $<

$(KERNEL_ASMOBJS): %.o: %.S
	$(CC) -c $(CFLAGS) $(KERNEL_CFLAGS) $<

$(SYSSVC_COBJS): %.o: %.c
	$(CC) -c $(CFLAGS) $(SYSSVC_CFLAGS) $<

ifneq ($(USE_TRUESTUDIO),true)
$(SYSSVC_COBJS:.o=.s): %.s: %.c
	$(CC) -S $(CFLAGS) $(SYSSVC_CFLAGS) $<
endif

$(SYSSVC_ASMOBJS): %.o: %.S
	$(CC) -c $(CFLAGS) $(SYSSVC_CFLAGS) $<

$(APPL_COBJS): %.o: %.c
	$(CC) -c $(CFLAGS) $(APPL_CFLAGS) $<

ifneq ($(USE_TRUESTUDIO),true)
$(APPL_COBJS:.o=.s): %.s: %.c
	$(CC) -S $(CFLAGS) $(APPL_CFLAGS) $<
endif

$(APPL_CXXOBJS): %.o: %.cpp
	$(CXX) -c $(CFLAGS) $(APPL_CXXFLAGS) $<

ifneq ($(USE_TRUESTUDIO),true)
$(APPL_CXXOBJS:.o=.s): %.s: %.cpp
	$(CXX) -S $(CFLAGS) $(APPL_CXXFLAGS) $<
endif

$(APPL_ASMOBJS): %.o: %.S
	$(CC) -c $(CFLAGS) $(APPL_CXXFLAGS) $<

#
#  依存関係作成ルールの定義
#
$(KERNEL_COBJS:.o=.d): %.d: %.c
	@$(PERL) $(SRCDIR)/utils/makedep -C $(CC) $(MAKEDEP_OPTS) \
		-O "$(CFLAGS) $(KERNEL_CFLAGS)" $< >> Makefile.depend

$(KERNEL_LCSRCS:.c=.d): %.d: %.c
	@$(PERL) $(SRCDIR)/utils/makedep -C $(CC) $(MAKEDEP_OPTS) -T "$($*)" \
		-O "-DALLFUNC $(CFLAGS) $(KERNEL_CFLAGS)" $< >> Makefile.depend

$(KERNEL_ASMOBJS:.o=.d): %.d: %.S
	@$(PERL) $(SRCDIR)/utils/makedep -C $(CC) $(MAKEDEP_OPTS) \
		-O "$(CFLAGS) $(KERNEL_CFLAGS)" $< >> Makefile.depend

$(SYSSVC_COBJS:.o=.d): %.d: %.c
	@$(PERL) $(SRCDIR)/utils/makedep -C $(CC) $(MAKEDEP_OPTS) \
		-O "$(CFLAGS) $(SYSSVC_CFLAGS)" $< >> Makefile.depend

$(SYSSVC_ASMOBJS:.o=.d): %.d: %.S
	@$(PERL) $(SRCDIR)/utils/makedep -C $(CC) $(MAKEDEP_OPTS) \
		-O "$(CFLAGS) $(SYSSVC_CFLAGS)" $< >> Makefile.depend

$(APPL_COBJS:.o=.d): %.d: %.c
	@$(PERL) $(SRCDIR)/utils/makedep -C $(CC) $(MAKEDEP_OPTS) \
		-O "$(CFLAGS) $(APPL_CFLAGS)" $< >> Makefile.depend

$(APPL_CXXOBJS:.o=.d): %.d: %.cpp
	@$(PERL) $(SRCDIR)/utils/makedep -C $(CXX) $(MAKEDEP_OPTS) \
		-O "$(CFLAGS) $(APPL_CFLAGS)" $< >> Makefile.depend

$(APPL_ASMOBJS:.o=.d): %.d: %.S
	@$(PERL) $(SRCDIR)/utils/makedep -C $(CC) $(MAKEDEP_OPTS) \
		-O "$(CFLAGS) $(APPL_CFLAGS)" $< >> Makefile.depend

#
#  デフォルトコンパイルルールを上書き
#
%.o: %.c
	@echo "*** Default compile rules should not be used."
	$(CC) -c $(CFLAGS) $<

%.s: %.c
	@echo "*** Default compile rules should not be used."
	$(CC) -S $(CFLAGS) $<

%.o: %.cpp
	@echo "*** Default compile rules should not be used."
	$(CXX) -c $(CFLAGS) $<

%.s: %.cpp
	@echo "*** Default compile rules should not be used."
	$(CXX) -S $(CFLAGS) $<

%.o: %.S
	@echo "*** Default compile rules should not be used."
	$(CC) -c $(CFLAGS) $<
