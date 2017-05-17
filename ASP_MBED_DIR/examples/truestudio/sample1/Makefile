#
#  ターゲットの指定（Makefile.targetで上書きされるのを防ぐため）
#
all:

#
#  アプリケーションファイル
#
APPLNAME = sample1
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
include $(MBED_LIB_DIR)/mbed-src/Makefile.src
include ../Makefile.mbd

#
#  ASPカーネルライブラリ
#
KERNEL_LIB = .
include ../Makefile.asp
