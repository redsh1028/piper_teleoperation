#!/bin/bash

# 1.전제조건
# 시스템에 IP 도구와 ethtool 도구를 설치해야 합니다.
# sudo apt install ethtool can-utils
# gs_usb 드라이브가 올바르게 설치되었는지 확인합니다.

# 2.배경
# 이 스크립트는 CAN(Controller Area Network) 인터페이스를 자동으로 관리하고 이름을 바꾸고 활성화하는 것을 목표로 합니다.
# 시스템의 현재 CAN 모듈 수를 확인하고 미리 정의된 USB 포트에 따라 CAN 인터페이스의 이름을 바꾸고 활성화합니다.
# 이는 여러 CAN 모듈이 있는 시스템, 특히 서로 다른 CAN 모듈이 특정 이름을 필요로 하는 경우에 유용합니다.

# 3.주요 기능
# CAN 모듈 수 확인: 시스템에서 감지된 CAN 모듈의 수가 미리 설정된 수와 일치하는지 확인합니다.
# USB 포트 정보 가져오기: ethool을 통해 각 CAN 모듈에 대한 USB 포트 정보를 가져옵니다.
# USB 포트 확인: 각 CAN 모듈의 USB 포트가 미리 정의된 포트 목록을 준수하는지 확인합니다.
# CAN 인터페이스 이름 변경: 미리 정의된 USB 포트에 따라 CAN 인터페이스의 이름을 대상 이름으로 변경합니다.

# 4.스크립트 설정 설명
# 스크립트의 주요 구성 항목에는 예상 CAN 모듈 수, 기본 CAN 인터페이스 이름 및 포트레이트 설정이 포함됩니다.
# 1. 예상 CAN 모듈 수:
# EXPECTED_CAN_COUNT=1
# 이 값은 시스템에서 감지해야 하는 CAN 모듈의 수를 결정합니다.
# 2. 단일 CAN 모듈의 경우 기본 CAN 인터페이스 이름:
# DEFAULT_CAN_NAME="${1:-can0}"
# 기본 CAN 인터페이스 이름은 명령줄 매개변수를 통해 지정할 수 있으며 매개변수가 제공되지 않는 경우 기본적으로 can0입니다.
# 3. 단일 CAN 모듈의 기본 비트레이트:
# DEFAULT_BITRATE="${2:-500000}"
# 개별 CAN 모듈의 비트 레이트는 매개변수가 제공되지 않는 경우 기본적으로 50,000으로 명령줄 매개변수를 통해 지정할 수 있습니다.
# 4. CAN 모듈이 여러 개일 때 설정:
# declare -A USB_PORTS
# USB_PORTS["1-2:1.0"]="can_device_1:500000"
# USB_PORTS["1-3:1.0"]="can_device_2:250000"
# 여기서 키는 USB 포트를 나타내고 값은 인터페이스 이름과 비트레이트이며 콜론으로 구분됩니다.

# 5.사용절차
# 1.스크립트 편집:
# 1. 미리 정의된 값 수정:
# - 미리 정의된 CAN 모듈 수 : EXPECTED_CAN_COUNT=2, 산업용 컨트롤러에 삽입되는 CAN 모듈 수량으로 수정 가능
# - can 모듈이 1개일 경우, 위의 파라미터를 설정한 후 바로 여기를 건너뛰고 뒤를 볼 수 있음
# - (복수의 CAN 모듈) 정의된 USB 포트와 대상 인터페이스 이름:
# 먼저 CAN 모듈을 예상 usb 포트에 삽입하고 초기 구성 시 매번 산업용 컨트롤러에 CAN 모듈을 삽입합니다.
# 그런 다음 sudo ethool-ican0 | grep bus를 실행하고 bus-info: 뒤에 인자를 기록합니다
# 이어서 다음 can모듈을 삽입하고, 지난번 can모듈에 삽입한 usb포트와 동일하지 않도록 주의한 후, 이전 단계를 반복하여 수행합니다.
# (usb주소에 따라 구분 모듈이 구분되기 때문에 can모듈 하나로 다른 usb를 꽂을 수 있습니다)
# 모든 모듈은 마땅히 있어야 할 USB 포트를 설계하고 기록을 완료한 후,
# 실제 상황에 따라 USB 포트(bus-info) 및 대상 인터페이스 이름을 수정합니다.
# can_device_1:500000, 전자는 설정된 can 이름, 후자는 설정된 비트레이트
# declare -A USB_PORTS
# USB_PORTS["1-2:1.0"]="can_device_1:500000"
# USB_PORTS["1-3:1.0"]="can_device_2:250000"
# 수정이 필요한 것은 USB_PORTS["1-3:1.0] 내 큰따옴표의 내용으로, 위에 기록된 bus-info: 뒷면의 파라미터로 수정
# 2. 스크립트 실행 권한 부여:
# 터미널을 열고 스크립트가 있는 디렉터리로 이동하여 다음 명령을 실행하여 스크립트 실행 권한을 부여합니다.
# chmod +x can_config. sh
# 3. 스크립트 실행:
# sudo를 사용하여 스크립트를 실행합니다. 스크립트는 네트워크 인터페이스를 수정하기 위해 관리자 권한이 필요하기 때문입니다:
# 1. 단일 CAN 모듈
# 1. 기본 CAN 인터페이스 이름과 비트레이트(기본값은 can0 및 500000)는 명령줄 매개변수를 통해 지정할 수 있습니다.
# sudo bash . /can_config. sh [CAN 인터페이스 이름] [비트율]
# 예를 들어 지정된 인터페이스 이름은 my_can_interface이고 비트레이트는 1000000:
# sudo bash . /can_config. sh my_can_interface 1000000
# 2. 지정된 USB 하드웨어 주소를 통해 CAN 이름을 지정할 수 있습니다.
# sudo bash . /can_config. sh [CAN 인터페이스 이름] [비트율] [usb 하드웨어 주소]
# 예를 들어 지정된 인터페이스 이름은 my_can_interface이고 비트레이트는 1000000이고 usb 하드웨어 주소는 1-3:1.0:
# sudo bash . /can_config. sh my_can_interface 1000000 1-3:1.0
# 즉, 1-3:1.0 usb 주소의 can 디바이스 지정명은 my_can_interface이고 비트레이트는 1000000
# 2. 다중 CAN 모듈
# 여러 CAN 모듈의 경우 스크립트에서 USB_PORTS 배열을 설정하여 각 CAN 모듈에 대한 인터페이스 이름과 비트레이트를 지정합니다.
# 추가 파라미터 없이 스크립트 실행:
# sudo . /can_config. sh

# 주의사항

# 권한 요청:
# 네트워크 인터페이스의 이름 변경 및 설정에 관리자 권한이 필요하기 때문에 스크립트는 sudo 권한을 사용해야 합니다.
# 스크립트를 실행할 수 있는 충분한 권한이 있는지 확인하십시오.

# 스크립트 환경:
# 이 스크립트는 bash 환경에서 실행된다고 가정합니다. 당신의 시스템이 다른 Shell(예: sh)이 아닌 bash를 사용하도록 하세요.
# 스크립트의 Shebang 행을 검사할 수 있습니다(#! /bin/bash) bash를 사용하도록 합니다.

# USB 포트 정보:
# 미리 정의된 USB 포트 정보(bus-info)가 실제 시스템에서 ethtool에서 출력되는 정보와 일치하는지 확인합니다.

# 미리 정의된 CAN 모듈 수
EXPECTED_CAN_COUNT=4

if [ "$EXPECTED_CAN_COUNT" -eq 1 ]; then
    # 기본 CAN 이름으로 명령줄 인자를 통해 설정할 수 있습니다
    DEFAULT_CAN_NAME="${1:-can0}"

    # 단일 CAN 모듈의 기본 비트레이트이며, 명령줄 인자를 통해 설정할 수 있습니다.
    DEFAULT_BITRATE="${2:-1000000}"

    # USB 하드웨어 주소(옵션)
    USB_ADDRESS="${3}"
fi

# 미리 정의된 USB 포트, 대상 인터페이스 이름 및 비트레이트 (여러 CAN 모듈에서 사용)
if [ "$EXPECTED_CAN_COUNT" -ne 1 ]; then
    declare -A USB_PORTS 
    USB_PORTS["3-1.1:1.0"]="can0:1000000"
    USB_PORTS["3-1.2:1.0"]="can1:1000000"
    USB_PORTS["3-1.3:1.0"]="can2:1000000"
    USB_PORTS["3-1.4.1:1.0"]="can3:1000000"    
fi

# 현재 시스템의 CAN 모듈 수 가져오기
CURRENT_CAN_COUNT=$(ip link show type can | grep -c "link/can")

# 현재 시스템에 있는 CAN 모듈의 수가 예상대로 증가하는지 확인합니다.
if [ "$CURRENT_CAN_COUNT" -ne "$EXPECTED_CAN_COUNT" ]; then
    echo "오류: 탐지된 CAN 모듈 수 ($CURRENT_CAN_COUNT)가 예상 수 ($EXPECTED_CAN_COUNT)와 일치하지 않습니다. "
    exit 1
fi

# gs_usb 모듈 불러오기
sudo modprobe gs_usb
if [ $? -ne 0 ]; then
    echo "오류: gs_usb 모듈을 불러올 수 없습니다."
    exit 1
fi

# 하나의 CAN 모듈만 처리할 것인지 여부 판단하기
if [ "$EXPECTED_CAN_COUNT" -eq 1 ]; then
    if [ -n "$USB_ADDRESS" ]; then
        echo "USB 주소 파라메터가 감지되었습니다: $USB_ADDRESS"
        
        # ethtool을 사용하여 USB 하드웨어 주소에 해당하는 CAN 인터페이스 찾기
        INTERFACE_NAME=""
        for iface in $(ip -br link show type can | awk '{print $1}'); do
            BUS_INFO=$(sudo ethtool -i "$iface" | grep "bus-info" | awk '{print $2}')
            if [ "$BUS_INFO" == "$USB_ADDRESS" ]; then
                INTERFACE_NAME="$iface"
                break
            fi
        done
        
        if [ -z "$INTERFACE_NAME" ]; then
            echo "오류: USB 주소 $USB_ADDRESS에 해당하는 CAN 인터페이스를 찾을 수 없습니다."
            exit 1
        else
            echo "USB 주소 $USB_ADDRESS에 해당하는 인터페이스 찾기: $INTERFACE_NAME"
        fi
    else
        # 고유한 CAN 인터페이스 가져오기
        INTERFACE_NAME=$(ip -br link show type can | awk '{print $1}')
        
        # 인터페이스 이름을 찾았는지 검사하기
        if [ -z "$INTERFACE_NAME" ]; then
            echo "오류: CAN 인터페이스를 감지할 수 없습니다."
            exit 1
        fi

        echo "CAN 모듈이 하나만 예상됨, 인터페이스 감지됨 $INTERFACE_NAME"
    fi

    # 현재 인터페이스가 활성화되었는지 확인합니다
    IS_LINK_UP=$(ip link show "$INTERFACE_NAME" | grep -q "UP" && echo "yes" || echo "no")

    # 현재 인터페이스의 비트레이트 가져오기
    CURRENT_BITRATE=$(ip -details link show "$INTERFACE_NAME" | grep -oP 'bitrate \K\d+')

    if [ "$IS_LINK_UP" == "yes" ] && [ "$CURRENT_BITRATE" -eq "$DEFAULT_BITRATE" ]; then
        echo "인터페이스 $INTERFACE_NAME이 활성화되었으며 비트레이트 $DEFAULT_BITRATE로 설정합니다."
        
        # 인터페이스 이름이 기본 이름과 일치하는지 확인합니다
        if [ "$INTERFACE_NAME" != "$DEFAULT_CAN_NAME" ]; then
            echo "인터페이스 $INTERFACE_NAME 이름을 $DEFAULT_CAN_NAME으로 변경"
            sudo ip link set "$INTERFACE_NAME" down
            sudo ip link set "$INTERFACE_NAME" name "$DEFAULT_CAN_NAME"
            sudo ip link set "$DEFAULT_CAN_NAME" up
            echo "인터페이스는 $DEFAULT_CAN_NAME으로 이름이 변경되고 다시 활성화되었습니다."
        else
            echo "인터페이스 이름은  이미 $DEFAULT_CAN_NAME로 설정되어 있습니다."
        fi
    else
        # 인터페이스가 활성화되지 않았거나 비트레이트가 다를 경우 설정
        if [ "$IS_LINK_UP" == "yes" ]; then
            echo "인터페이스 $INTERFACE_NAME이 활성화되었지만 비트레이트는 $CURRENT_BITRATE로 설정된 $DEFAULT_BITRATE와 일치하지 않습니다."
        else
            echo "인터페이스 $INTERFACE_NAME이 활성화되지 않았거나 비트레이트가 설정되지 않았습니다."
        fi
        
        # 인터페이스 비트레이트 설정 및 활성화
        sudo ip link set "$INTERFACE_NAME" down
        sudo ip link set "$INTERFACE_NAME" type can bitrate $DEFAULT_BITRATE
        sudo ip link set "$INTERFACE_NAME" up
        echo "인터페이스 $INTERFACE_NAME의 비트레이트가 $DEFAULT_BITRATE로 재설정되고 활성화되었습니다."
        
        # 인터페이스 이름을 기본 이름으로 바꿉니다
        if [ "$INTERFACE_NAME" != "$DEFAULT_CAN_NAME" ]; then
            echo "인터페이스 $INTERFACE_NAME 이름을 $DEFAULT_CAN_NAME으로 변경합니다."
            sudo ip link set "$INTERFACE_NAME" down
            sudo ip link set "$INTERFACE_NAME" name "$DEFAULT_CAN_NAME"
            sudo ip link set "$DEFAULT_CAN_NAME" up
            echo "인터페이스는 $DEFAULT_CAN_NAME으로 이름이 변경되고 다시 활성화되었습니다."
        fi
    fi
else
    # 여러 CAN 모듈 처리 중

    # USB 포트와 대상 인터페이스의 이름이 예상 CAN 모듈 수와 일치하는지 확인합니다
    PREDEFINED_COUNT=${#USB_PORTS[@]}
    if [ "$EXPECTED_CAN_COUNT" -ne "$PREDEFINED_COUNT" ]; then
        echo "오류: 미리 설정된 CAN 모듈 수 ($EXPECTED_CAN_COUNT)가 미리 정의된 USB 포트 수 ($PREDEFINED_COUNT)와 일치하지 않습니다."
        exit 1
    fi

    # 모든 CAN 인터페이스 사이를 옮겨다니기
    for iface in $(ip -br link show type can | awk '{print $1}'); do
        # ethtool을 사용하여 bus- info 가져오기
        BUS_INFO=$(sudo ethtool -i "$iface" | grep "bus-info" | awk '{print $2}')
        
        if [ -z "$BUS_INFO" ];then
            echo "오류: $iface의 bus-info 정보를 가져올 수 없습니다."
            continue
        fi
        
        echo "USB 포트 $BUS_INFO에 인터페이스 $iface를 삽입합니다."

        # bus- info가 미리 정의된 USB 포트 목록에 있는지 확인합니다
        if [ -n "${USB_PORTS[$BUS_INFO]}" ];then
            IFS=':' read -r TARGET_NAME TARGET_BITRATE <<< "${USB_PORTS[$BUS_INFO]}"
            
            # 현재 인터페이스가 활성화되었는지 확인합니다
            IS_LINK_UP=$(ip link show "$iface" | grep -q "UP" && echo "yes" || echo "no")

            # 현재 인터페이스의 비트레이트 가져오기
            CURRENT_BITRATE=$(ip -details link show "$iface" | grep -oP 'bitrate \K\d+')

            if [ "$IS_LINK_UP" == "yes" ] && [ "$CURRENT_BITRATE" -eq "$TARGET_BITRATE" ]; then
                echo "인터페이스 $iface가 활성화되었으며 비트레이트 $TARGET_BITRATE"
                
                # 인터페이스 이름이 대상 이름과 일치하는지 검사하기
                if [ "$iface" != "$TARGET_NAME" ]; then
                    echo "将接口 $iface 重命名为 $TARGET_NAME"
                    sudo ip link set "$iface" down
                    sudo ip link set "$iface" name "$TARGET_NAME"
                    sudo ip link set "$TARGET_NAME" up
                    echo "인터페이스는 $TARGET_NAME으로 이름이 변경되고 다시 활성화되었습니다."
                else
                    echo "인터페이스 이름이 이미 $TARGET_NAME로 설정되어 있습니다."
                fi
            else
                # 인터페이스가 활성화되지 않았거나 비트레이트가 다를 경우
                if [ "$IS_LINK_UP" == "yes" ]; then
                    echo "인터페이스 $iface가 활성화되었지만 비트레이트는 $CURRENT_BITRATE로 설정된 $TARGET_BITRATE와 일치하지 않습니다."
                else
                    echo "인터페이스 $iface가 활성화되지 않았거나 비트레이트가 설정되지 않았습니다."
                fi
                
                # 인터페이스 비트레이트 설정 및 활성화하기
                sudo ip link set "$iface" down
                sudo ip link set "$iface" type can bitrate $TARGET_BITRATE
                sudo ip link set "$iface" up
                echo "인터페이스 $iface가 비트레이트 $TARGET_BITRATE로 재설정되고 활성화되었습니다."
                
                # 인터페이스 이름 변경하기
                if [ "$iface" != "$TARGET_NAME" ]; then
                    echo "인터페이스 $iface의 이름을 $TARGET_NAME으로 변경합니다."
                    sudo ip link set "$iface" down
                    sudo ip link set "$iface" name "$TARGET_NAME"
                    sudo ip link set "$TARGET_NAME" up
                    echo "인터페이스는 $TARGET_NAME으로 이름이 변경되고 다시 활성화되었습니다."
                fi
            fi
        else
            echo "오류: 알 수 없는 USB 포트 $BUS_INFO에 대응하는 인터페이스는 $iface 입니다."
            exit 1
        fi
    done
fi

echo "모든 CAN 인터페이스가 성공적으로 이름을 바꾸고 활성화되었습니다."
