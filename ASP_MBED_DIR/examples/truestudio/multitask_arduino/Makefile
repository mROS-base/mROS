#
#  ターゲットの指定（Makefile.targetで上書きされるのを防ぐため）
#
all:

#
#  アプリケーションファイル
#
APPLNAME = multitask_arduino
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
include $(MBED_LIB_DIR)/arduino-lib/Makefile.ard # 同名のヘッダファイルがあるのでmbed-srcよりも前で読み込む
include $(MBED_LIB_DIR)/mbed-src/Makefile-Arduino.src # Makefile.src の代わりにこちらを使用
include $(MBED_LIB_DIR)/EthernetInterface/Makefile.eif
include ../Makefile.mbd

#
#  ASPカーネルライブラリ
#
KERNEL_LIB = .
include ../Makefile.asp
