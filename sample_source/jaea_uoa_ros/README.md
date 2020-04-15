○choreonoidのビルドに必要なパッケージ
　・BUILD_AGX_BODYEXTENSION_PLUGIN
　・BUILD_AGX_DYNAMICS_PLUGIN
　・BUILD_WRS2018
　・BUILD_COMPETITION_PLUGIN
　・BUILD_WRS2019
　・BUILD_MULTICOPTER_PLUGIN

以下のコマンドを入力してchoreonoidをビルド

$ cd ~/catkin_ws/
$ catkin config --cmake-args -DBUILD_AGX_BODYEXTENSION_PLUGIN=ON - DBUILD_AGX_DYNAMICS_PLUGIN=ON -DBUILD_COMPETITION_PLUGIN=ON -DBUILD_MULTICOPTER_PLUGIN=ON -DBUILD_WRS2018=ON -DBUILD_WRS2019=ON 
$ catkin build


○agxクローラが前進しない場合はマテリアルの追記が必要。
    ~/catkin_ws/src/choreonoid/share/default/materials.yamlを
    ./material/materials.yamlに置き換える。

○HobbyDroneのビルドが失敗する場合
     ~/catkin_ws/src/choreonoid/src/MulticopterPlugin/CMakeLists.txtの記述を下記の様に変更

　　set(headers
  　　　RotorDevice.h
  　　　)
    ↓
　　set(headers
  　　　RotorDevice.h
 　　　 exportdecl.h # add
  　)








