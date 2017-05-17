#
#  ターゲットの指定（Makefile.targetで上書きされるのを防ぐため）
#
all:

#
#  アプリケーションファイル
#
APPLNAME = blinky
USE_CXX = true
APPL_CFG = $(APPLNAME).cfg
USE_TRUESTUDIO = true

DEBUG = true
OMIT_OPTIMIZATION = true

#
#  MBEDライブラリのディレクトリの定義
#
MBED_LIB_DIR = ../../../mbed-lib

#
#  ASPソースファイルのディレクトリの定義
#
SRCDIR = ../../../asp-1.9.2-utf8

#
#  MBEDライブラリのビルド
#
include $(MBED_LIB_DIR)/common/Makefile.cmn
include $(MBED_LIB_DIR)/mbed-src/Makefile.src
include $(MBED_LIB_DIR)/SoftPWM/Makefile.pwm
include ../Makefile.mbd

#
#  ASPカーネルライブラリ
#
KERNEL_LIB = .
#include $(SRCDIR)/Makefile.asp
include ../Makefile.asp
