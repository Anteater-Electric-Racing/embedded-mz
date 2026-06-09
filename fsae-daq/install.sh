#!/bin/bash
set -e

if [ "$(id -u)" -ne 0 ]; then
    echo "Please run as root (sudo ./install.sh)" >&2
    exit 1
fi

REAL_USER="${SUDO_USER:-$(logname)}"
REAL_HOME=$(eval echo "~$REAL_USER")

echo "==> Installing Rust for $REAL_USER..."
sudo -u "$REAL_USER" bash -c 'curl https://sh.rustup.rs -sSf | sh -s -- -y'

echo "==> Installing jdk"
curl https://download.java.net/java/GA/jdk26.0.1/458fda22e4c54d5ba572ab8d2b22eb83/8/GPL/openjdk-26.0.1_linux-aarch64_bin.tar.gz -o ~/Downloads/jdk.tar.gz
tar -xvf ~/Downloads/jdk.tar.gz
export JAVA_HOME=~/Downloads/jdk-26.0.1/

echo "==> Installing questdb"
curl https://github.com/questdb/questdb/releases/download/9.4.1/questdb-9.4.1-no-jre-bin.tar.gz -o ~/Downloads/qdb.tar.gz -L
tar -xvf ~/Downloads/qdb.tar.gz\
~/Downloads/questdb-9.4.1-no-jre-bin/questdb.sh start
~/Downloads/questdb-9.4.1-no-jre-bin/questdb.sh stop
sed -i 's/#telemetry.enabled=true/telemetry.enabled=false/g' ~/.questdb/conf/server.conf
~/Downloads/questdb-9.4.1-no-jre-bin/questdb.sh start
curl -G --data-urlencode "query=DROP TABLE telemetry" http://localhost:9000/exec

echo "==> Opening 1883 to allow tcp connections"
sudo nft add rule inet filter input tcp dport 1883 accept
sudo nft list ruleset

echo "==> Patching and installing systemd service files for user '$REAL_USER'..."
for SERVICE in fsae-daq.service; do
    sed \
        -e "s|__USER__|$REAL_USER|g" \
        -e "s|__HOME__|$REAL_HOME|g" \
        "$SERVICE" > "/etc/systemd/system/$SERVICE"
    systemctl daemon-reload
    systemctl enable --now "$SERVICE"
done

echo "==> Installing CAN network config..."
cp 80-can.network /etc/systemd/network/
