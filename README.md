# waypoint_gui

移動ロボットのナビゲーションを行うときに、RViz上で経由点を指定し、計画された経路を表示するためのパッケージです。  
本パッケージではナビゲーションの手法として、[at-wat/neonavigation](https://github.com/at-wat/neonavigation)を採用しています。

![waypoints gui package gif](https://rt-net.github.io/images/waypoint_gui/waypoints.gif)

## 動作環境

- Ubuntu
    - Ubuntu 18.04
- ROS
    - ROS Melodic  

## インストールとビルド
### パッケージのインストール

 - 以下のコマンドで本リポジトリと必要なパッケージをダウンロードします

```bash
cd ~/catkin_ws/src
git clone https://github.com/rt-net/waypoint_gui.git
rosdep install -r -i -y --from-paths waypoint_gui
```

### パッケージのビルド

- 以下のコマンドでパッケージをビルドします

``` bash
cd ~/catkin_ws
catkin_make
source ~/catkin_ws/devel/setup.bash
```
## 使用方法（シミュレーション）

- 以下のコマンドで必要なノードを起動できます
    - `waypoint_gui.launch`はRVizと本ツールを起動します
        - 使用するナビゲーションによって、RViz上で表示するものが異なります。
        - 本パッケージでは、`neonavigation`の場合と`move_base`の場合の2種類のRVizファイルを用意しています。  
    - `simulate_path.launch`はナビゲーション時のパスをシミュレーションするためのノードを起動します
        - ナビゲーションに用いる手法を変更する場合はこのlaunchファイルを編集・置き換えます

```bash
roslaunch waypoint_gui waypoint_gui.launch localization:=$NAV_METHOD
roslaunch waypoint_neonavigation simulate_path.launch 
```
### 操作方法

- 経由点を設置し経路を指定  
    1. RVizの上画面に表示されたToolsから "2D Pose Estimate"を選択し、map上に現在のロボットの位置を指定します
        ![2d pose estimate](https://rt-net.github.io/images/waypoint_gui/2d_pose_estimate.png)
    2. 設置したい経由点の数だけ以下の①~③の操作を繰り返します  
① RVizの上画面に表示されたToolsから"Publish Point"をクリック  
② 地図上にカーソルを移動させ、行きたい経由点の場所で再度クリック  
③ 表示された矢印を選択しながらカーソルを動かし、経由点の細かい位置調整や向きを指定  
![2d pose estimate](https://rt-net.github.io/images/waypoint_gui/publish_point.png)　　 
 
    3. 経路を指定  
RVizの左下にあるwaypoint_guiパネルで、どの順路で経由点を通りたいか指定します 
![2d pose estimate](https://rt-net.github.io/images/waypoint_gui/route.png)　　 

    4. 移動開始   
    最後にwaypoint_guiパネルにあるStartボタンを選択し、表示されているロボットの移動を開始します  
    ![2d pose estimate](https://rt-net.github.io/images/waypoint_gui/start.png)　　

### 経由点を保存

- 上記の方法で経由点を指定後、RVizの左下にあるwaypoint_guiパネルの"Save"ボタンを押すと指定された経由点が保存されます

### 経由点を再指定

- 保存された経由点を読み込むことで経路を指定します  
以下のコマンド入力後、waypoint_guiパネルにある"Start"ボタンを押すと、前回保存された経由点を通り、移動を開始します

```bash
roslaunch waypoint_gui waypoint_gui.launch  
```  
## License

```
(C) 2021 RT Corporation <support@rt-net.jp>  
```

各ファイルはライセンスがファイル中に明記されている場合、そのライセンスに従います。特に明記されていない場合は、Apache License, Version 2.0に基づき公開されています。  
ライセンスの全文は[LICENSE](./LICENSE)または[https://www.apache.org/licenses/LICENSE-2.0](https://www.apache.org/licenses/LICENSE-2.0)から確認できます。
