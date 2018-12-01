mROSで実装した特徴点抽出パッケージ群です

使い方
    1.fast,mask,preprocessをcatkinワークススペースの/srcいかにコピーしてcatkin_make
    2.ホストデバイスのIPを192.168.11.4に設定する
    3.ROSのマスタURIを設定 
        $export ROS_MASTER_URI=http://192.168.11.4:11311
    4.ラウンチによりROSマスタとノードを起動 
        $roslaunch fast mros.launch width:=160 height:=120 mask:=1 rate:=10 ch:=2
    5.mROSのリセットボタンを押す

launchファイルとして
    fast.launch:特徴点抽出のみ表示
    fast_image.launch:カメラ画像のみ表示
    が可能
            