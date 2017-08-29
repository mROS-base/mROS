# mROS

## 導入方法

作業ディレクトリを<git_clone_dir>とする

1. asp-gr_peach_gcc-mbed のclone
  * `$ cd <git_clone_dir>`
  * `$ git clone https://github.com/ncesnagoya/asp-gr_peach_gcc-mbed.git`
2. 本リポジトリのclone
  * `$ cd <git_clone_dir>`
  * `$ git clone https://github.com/tlk-emb/mROS.git`
3. TrueSTUDIOからプロジェクトを設定する
  * <git_clone_dir>/mROS/truestudio をworkspaceとして指定して開く
  * ~~ファイル > インポート > 一般 > 既存プロジェクトをワークスペースへ~~
  * ~~ルートディレクトリの選択: <git_clone_dir>/mROS/truestudio/ros_emb~~
    * .metadata, .settings も管理対象にしているため本操作は不要
4. TrueSTUDIO上でビルドする


## TODO

ros_emb.cpp のコンパイル時にエラーとなる不具合を解消するため，  
truestudio\ros_emb\Makefile 60行目にオプションを付与している．  
可能であればオプション無しでコンパイルできるように変更すべき  
 `APPL_CXXFLAGS := $(APPL_CXXFLGS) -fpermissive`



## 参考情報

TOPPERSコンフィギュレータは以下からRelease 1.9.6（Windows用バイナリ）を入手した  
https://www.toppers.jp/cfg-download.html

