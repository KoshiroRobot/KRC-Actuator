#!/bin/bash
set -u
WANT=${1:?期待するノードIDを渡す}
ELF=${2:-/tmp/KRC_Actuator.elf}
OCD="sudo timeout 60 openocd -f interface/stlink.cfg -f target/stm32g4x.cfg"
mkdir -p ~/krc_fw_backup
BK=~/krc_fw_backup/node${WANT}_before_$(date +%Y%m%d_%H%M%S).bin
sudo rm -f /tmp/krc_dump.bin
echo "== 1. 吸い出す"
$OCD -c "init; halt; dump_image /tmp/krc_dump.bin 0x08000000 0x20000; reset run; exit" 2>&1 | grep -E "Error|Cortex" | head -3
[ -s /tmp/krc_dump.bin ] || { echo "!! 吸い出せない（SWD につながらない）"; exit 1; }
cp /tmp/krc_dump.bin "$BK"; md5sum "$BK"
echo "== 2. どの基板か"
NODE=$(python3 -c "
import struct
b=open('/tmp/krc_dump.bin','rb').read()
dw=struct.unpack_from('<Q',b,0x1F800)[0]
if (dw & 0xffffffff) != 0x4B524331: print('none'); raise SystemExit
print((dw>>54)&0xf)
print('offset %d pp %d dir %d' % ((dw>>32)&0x3fff,(dw>>46)&0x7f,-1 if (dw>>53)&1 else 1), file=__import__('sys').stderr)
")
echo "  つながっている基板: ノード $NODE"
if [ "$NODE" != "$WANT" ]; then echo "!! 期待はノード $WANT。焼かない（ケーブルを付け替えたか確かめる）"; sudo rm -f /tmp/krc_dump.bin; exit 2; fi
sudo rm -f /tmp/krc_dump.bin
echo "== 3. 焼く $(md5sum "$ELF" | cut -c1-8)"
$OCD -c "program $ELF verify reset exit" 2>&1 | grep -E "Error|Verified" | head -3
sleep 2
echo "== 4. 読み返し"
cd ~ && timeout 40 python3 krc.py -n "$WANT" param 2>&1 | grep -E "^ ?(1|2|3|4|5|6|7|16|17|21|33|34|35|36|37|38) "
