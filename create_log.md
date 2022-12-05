# Docker コンテナ作成の記録

- ROS を使用するための Dockerfile は [Qiita](https://qiita.com/Yuya-Shimizu/items/3d9fc18f42aee40f23b3) を参考にしたが、画面接続でエラー発生
- Docker を使って ROS で GUI を操作できる環境を整える [Youtube](https://www.youtube.com/watch?v=qWuudNxFGOQ) があってそれにしたがってやってみたがだめだった
- 画面接続は [GitHub リポジトリ](https://github.com/m-tmatma/xeyes-docker) を元に Ubuntu に変更することで解決

## イメージの pull

```
# ros1 noetic
docker run -it --name=ros-noetic osrf/ros:noetic-desktop
# ros2 foxy
docker run -it --name=ros-foxy osrf/ros:foxy-desktop
```

## Noetic の実行

ターミナル①
```
docker run -it --name=ros-noetic osrf/ros:noetic-desktop
root@de692e2f1c85:/# roscore
```
ターミナル②
```
docker exec -it ros-noetic bash
root@de692e2f1c85:/# source /opt/ros/noetic/setup.bash 
root@de692e2f1c85:/# rosrun rospy_tutorials listener
```
ターミナル③
```
docker exec -it ros-noetic bash
root@de692e2f1c85:/# source /opt/ros/noetic/setup.bash 
root@de692e2f1c85:/# rosrun rospy_tutorials talker
```

## turtlesim のウィンドウのポップアップを表示できないことを確認

### Noetic

ターミナル①
```
docker exec -it ros-noetic bash
root@de692e2f1c85:/# source /opt/ros/noetic/setup.bash 
root@de692e2f1c85:/# roscore
```
ターミナル②
```
docker exec -it ros-noetic bash
root@de692e2f1c85:/# source /opt/ros/noetic/setup.bash 
root@de692e2f1c85:/# rosrun turtlesim turtlesim_node 
qt.qpa.xcb: could not connect to display 
```

### Foxy

```
docker run -it --name=ros-foxy osrf/ros:foxy-desktop
root@74cfb6a38d02:/# ros2 run turtlesim turtlesim_node
qt.qpa.xcb: could not connect to display
```

## 画面接続に係る問題

Youtube 内で、Windows では以下のコマンドでディスプレイが接続できると紹介されていた

```
docker run --name r1_noetic -e DISPLAY=host.internal:0.0 -it osrf/ros:noetic-desktop
```

しかし turtlesim を実行するとエラー
> docker exec -it r1_noetic bash
root@dab70c8872ba:/# source /opt/ros/noetic/setup.bash 
root@dab70c8872ba:/# rosrun turtlesim turtlesim_node 
qt.qpa.xcb: could not connect to display host.internal:0.0
qt.qpa.plugin: Could not load the Qt platform plugin "xcb" in "" even though it was found.
This application failed to start because no Qt platform plugin could be initialized. Reinstalling the application may fix this problem.
>
> Available platform plugins are: eglfs, linuxfb, minimal, minimalegl, offscreen, vnc, xcb.
>
> Aborted (core dumped)

- ディスプレイの環境変数が違うのだと考え、値を変えて試してみたが解決せず
  - $DISPLAY でディスプレイの環境変数を表示させられる（これに合わせても解決せず）
  - windowsだとxlaunchというアプリで画面番号を設定できる
- Ubuntu では、X Window System の設定を変更することができる nvidia-settings コマンドが提供されている
  - このコマンドにより、X Window System の設定を GUI 環境から変更することができる
    ```
    sudo apt install nvidia-settings
    nvidia-settings
    ```
- 上記を実行したところ `ERROR: Unable to load info from any available system`
- NVIDIA のグラフィックスドライバが正しくインストールされていないことが原因
  ```
  # 以下のコマンドで実行結果に、 nvidia-driver というパッケージが表示される場合、 NVIDIA のグラフィックスドライバは正しくインストールされています
  # → 表示されず、正しくドライバがインストールされていないことが発覚
  dpkg -l | grep -E 'nvidia|nouveau'
  # まず PCI デバイスを再読込する
  sudo update-pciids
  # 必要なドライバのバージョンを知るために GPU の詳細情報を確認する
  lspci | grep -i nvidia
  ```
- GeForce RTX 3060 Ti に必要な nvidia-driver のインストールコマンド
  ```
  sudo apt install nvidia-driver-450
  ```
- 結局、[ここ](https://qiita.com/m-tmatma/items/944237003fc2d6182eca) にリンクされている GitHub リポジトリを参考にやってみるとうまく行った

## turtlesim で動作確認

- ターミナル①：`roscore`
- ターミナル②：`rosrun turtlesim turtlesim_node`
- ターミナル③：`rosrun turtlesim turtle_teleop_key`

## GPU 関連

- run.sh 内の `--runtime=nvidia` でエラー
  > [WSL2]docker: Error response from daemon: Unknown runtime specified nvidia. 
  - `--runtime=nvidia` を `--gpus all` に変更することで解決
- Docker コンテナで GPU を使用できることを確認
  ```
  docker run --gpus all nvidia/cuda:11.1.1-cudnn8-runtime-ubuntu20.04 nvidia-smi
  ```
  - 以下のエラーが発生
    > docker: Error response from daemon: could not select device driver "" with capabilities: [[gpu]].
    > ERRO[0394] error waiting for container: context canceled
  - NVIDIA ドライバーのインストール忘れ
    ```
    sudo apt-get install nvidia-container-runtime
    ```

# GPU を使用できるよう修正

- 以下のコマンドで ROS Noetic の Docker イメージがどのように作成されたか見る
  ```
  docker history osrf/ros:noetic-desktop-full --no-trunc --format '{{ json .CreatedBy }}'
  ```
- nvidia/cuda よりも osrf/ros のほうが history が短かったので、nvidia/cuda を元に Dockerfile を再構成
