# ethtool 패키지 설치 여부 확인하기
if ! dpkg -l | grep -q "ethtool"; then
    echo "\e[31m오류: 시스템에서 ethtool 패키지 감지되지 않습니다.\e[0m"
    echo "아래 명령을 이용하여 ethtool을 설치해주세요:"
    echo "sudo apt update && sudo apt install ethtool"
    exit 1
fi

# can-utils 패키지 설치 여부 확인하기
if ! dpkg -l | grep -q "can-utils"; then
    echo "\e[31m오류: 시스템에서 can-utils 패키지 감지되지 않습니다.\e[0m"
    echo "아래 명령을 이용하여 can-utils을 설치해주세요:"
    echo "sudo apt update && sudo apt install can-utils"
    exit 1
fi

echo "ethtool와 can-utils 모두 설치 완료되어 있습니다."

# 모든 CAN 인터페이스 사이를 옮겨다니기
for iface in $(ip -br link show type can | awk '{print $1}'); do
    # ethtool을 사용하여 bus- info 가져오기
    BUS_INFO=$(sudo ethtool -i "$iface" | grep "bus-info" | awk '{print $2}')
    
    if [ -z "$BUS_INFO" ];then
        echo "오류: $iface의 bus-info 정보를 가져올 수 없습니다."
        continue
    fi
    
    echo "USB 포트 $BUS_INFO에 인터페이스 $iface를 삽입할 것을 추천합니다."
done
