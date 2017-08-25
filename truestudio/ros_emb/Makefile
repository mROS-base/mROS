#
#  ターゲットの指定（Makefile.targetで上書きされるのを防ぐため）
#
all:

#
#  アプリケーションファイル
#
APPLDIR = ../../ros_emb
APPLNAME = ros_emb
USE_CXX = true
APPL_CFG = $(APPLNAME).cfg
USE_TRUESTUDIO = true

DEBUG = true
OMIT_OPTIMIZATION = true

#
#  アプリケーションプログラムに関する定義
#  メインファイル以外に追加するものを記述する
#
APPL_ASMOBJS = 
APPL_CXXOBJS = xmlparser.o xmlcall.o tcp_ros.o ros.o
APPL_COBJS = 

#
#  MBEDライブラリのディレクトリの定義
#
MBED_LIB_DIR = ../../../asp-gr_peach_gcc-mbed/mbed-lib

#
#  ASPソースファイルのディレクトリの定義
#
SRCDIR = ../../../asp-gr_peach_gcc-mbed/asp-1.9.2-utf8

#
#  MBEDライブラリのビルド
#
include $(MBED_LIB_DIR)/common/Makefile.cmn
include $(MBED_LIB_DIR)/mbed-src/Makefile.src
include $(MBED_LIB_DIR)/SoftPWM/Makefile.pwm
include $(MBED_LIB_DIR)/EthernetInterface/Makefile.eif
include ../Makefile.mbd

#
#  ASPカーネルライブラリ
#
KERNEL_LIB = .
include ../Makefile.asp

$(ALL_OBJ): kernel_cfg.h

#
#  ros_emb.cpp のコンパイル時にエラーとなる不具合を解消するため
#  オプション無しでコンパイルできるように変更すべき
#
APPL_CXXFLAGS := $(APPL_CXXFLAGS) -fpermissive
