#
#  ターゲットの指定（Makefile.targetで上書きされるのを防ぐため）
#

all: 

#
#  アプリケーションファイル
#
APPNAME = string_pubsub
APPLDIR = string_pubsub
APPLNAME = app
USE_CXX = true
APPL_CFG = $(APPLNAME).cfg
USE_TRUESTUDIO = true
GEN_MSG = false
MSG_SETTING_JSON = app.json
PYTHON = /usr/bin/python

DEBUG = true
OMIT_OPTIMIZATION = true

#
#  アプリケーションプログラムに関する定義
#  メインファイル以外に追加するものを記述する
#
APPL_ASMOBJS = 
APPL_CXXOBJS =  
APPL_COBJS = 


#
#  mros.cpp のコンパイル時にエラーとなる不具合を解消するため
#  オプション無しでコンパイルできるように変更すべき
#
APPL_CXXFLAGS := $(APPL_CXXFLAGS) -fpermissive

#
#  MBEDライブラリのディレクトリの定義
#
MBED_LIB_DIR = ../../asp_mbed/asp-gr_peach_gcc-mbed/mbed-lib
#
#  mROSライブラリのディレクトリの定義
#
MROS_DIR = ../../mros-lib
#
#  ASPソースファイルのディレクトリの定義
#
SRCDIR = ../../asp_mbed/asp-gr_peach_gcc-mbed/asp-1.9.2-utf8


#
#  MBEDライブラリのビルド
#
include $(MBED_LIB_DIR)/common/Makefile.cmn
include $(MBED_LIB_DIR)/mbed-src/Makefile.src
include $(MBED_LIB_DIR)/SoftPWM/Makefile.pwm
include $(MBED_LIB_DIR)/EthernetInterface/Makefile.eif
include ../Makefile.mbd
#
#  mROSライブラリのビルド
#
include $(MROS_DIR)/Makefile.m
include ../Makefile.mros

#
#  ASPカーネルライブラリ
#
KERNEL_LIB = .
include ../Makefile.asp

$(ALL_OBJ): kernel_cfg.h
