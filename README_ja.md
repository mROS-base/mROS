# mROS

## Environment & Build

- IDE: TrueSTUDIO v.8.0.0
- Build
  - Open TrueSTUDIO and specify the workspace to `<git_clone_dir>/mROS/truestudio`
  - Select `camera_app` to target project and build it


## TODO

mros.cpp のコンパイル時にエラーとなる不具合を解消するため，  
truestudio\app\Makefile 60行目にオプションを付与している．  
可能であればオプション無しでコンパイルできるように変更すべき  
 `APPL_CXXFLAGS := $(APPL_CXXFLGS) -fpermissive`

ドキュメントの整備
チュートリアルと仕様書


## References

TOPPERSコンフィギュレータは以下からRelease 1.9.6を入手した  
https://www.toppers.jp/cfg-download.html


