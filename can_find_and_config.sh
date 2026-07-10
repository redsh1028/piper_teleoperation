#!/bin/bash

# Usage: ./check_and_setup_can.sh <CAN_NAME> <BITRATE> <USB_ADDRESS> <TIMEOUT>

# 기본 인자
CAN_NAME="${1:-can0}"
BITRATE="${2:-1000000}"
USB_ADDRESS="${3}"
TIMEOUT="${4:-120}"  # 타임아웃 시간, 기본 120초

sudo -v

ROOT="$(dirname "$(readlink -f "$0")")"

if [ -z "$USB_ADDRESS" ]; then
    echo "오류: USB 주소가 필요합니다. "
    exit 1
fi

# 시작 시간
START_TIME=$(date +%s)

# 타임아웃 마크
TIMED_OUT=false

echo "USB 주소가 $USB_ADDRESS에 CAN 장치가 있는 지 확인합니다..."

while true; do
    # 현재 시간 가져오기
    CURRENT_TIME=$(date +%s)
    ELAPSED_TIME=$((CURRENT_TIME - START_TIME))

    # 시간 초과 여부 확인
    if [ "$ELAPSED_TIME" -ge "$TIMEOUT" ]; then
        echo "시간 초과: $TIMEOUT 초 동안 CAN 장치를 찾을 수 없습니다."
        TIMED_OUT=true
        break
    fi

    # 지정한 USB 주소에 연결된 장치가 있는지 찾기
    DEVICE_FOUND=false
    for iface in $(ip -br link show type can | awk '{print $1}'); do
        BUS_INFO=$(sudo ethtool -i "$iface" | grep "bus-info" | awk '{print $2}')
        if [ "$BUS_INFO" == "$USB_ADDRESS" ]; then
            DEVICE_FOUND=true
            break
        fi
    done

    if [ "$DEVICE_FOUND" == "true" ]; then
        echo "CAN 디바이스를 찾아 설정 스크립트를 호출합니다..."
        sudo bash $ROOT/can_activate.sh "$CAN_NAME" "$BITRATE" "$USB_ADDRESS"
        if [ $? -eq 0 ]; then
            echo "CAN 장비가 성공적으로 구성되었습니다."
            exit 0
        else
            echo "설정 스크립트가 실행되지 않았습니다."
            exit 1
        fi
    fi

    echo "CAN 디바이스를 찾을 수 없습니다. 대기하고 다시 시도하십시오..."

    # 5초마다 검사하기
    sleep 5
done

# 사이클이 종료되고 시간이 초과되면 시간 초과 정보를 출력합니다
if [ "$TIMED_OUT" == "true" ]; then
    echo "가 지정된 시간 내에 CAN 장치를 찾지 못하여 스크립트가 종료되었습니다."
    exit 1
fi
