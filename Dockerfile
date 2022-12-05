# osrfが提供するrosイメージ（タグがnoetic-desktop-full）をベースとしてダウンロード
FROM osrf/ros:noetic-desktop-full

ENV DEBIAN_FRONTEND=noninteractive

# X Window Systemをインストールする
RUN apt-get update && apt-get install -y x11-xserver-utils xorg

# xeyesパッケージをインストールする
RUN apt-get update && apt-get install -y x11-apps

# Docker実行してシェルに入ったときの初期ディレクトリ（ワークディレクトリ）の設定
WORKDIR /root/

# nvidia-container-runtime（描画するための環境変数の設定）
ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

# ROSの環境整理
# ROSのセットアップシェルスクリプトを.bashrcファイルに追記
RUN echo "source /opt/ros/noetic/setup.sh" >> .bashrc
# 自分のワークスペース作成のためにフォルダを作成
RUN mkdir -p catkin_ws/src
# srcディレクトリまで移動して，catkin_init_workspaceを実行．
# ただし，Dockerfileでは，.bashrcに追記した分はRUNごとに反映されないため，
# source /opt/ros/noetic/setup.shを実行しておかないと，catkin_init_workspaceを実行できない
RUN cd catkin_ws/src && . /opt/ros/noetic/setup.sh && catkin_init_workspace
# ~/に移動してから，catkin_wsディレクトリに移動して，上と同様にしてcatkin_makeを実行．
RUN cd && cd catkin_ws && . /opt/ros/noetic/setup.sh && catkin_make
# 自分のワークスペースが反映されるように，.bashrcファイルに追記．
RUN echo "source ./catkin_ws/devel/setup.bash" >> .bashrc