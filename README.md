# 校内赛上位机代码demo

## USB2TTL 设备芯片信息
/*
yimeng@yimeng-ASUS:~$ udevadm info -a -n /dev/ttyACM0 | grep -E "idVendor|idProduct|serial"
    ATTRS{idProduct}=="55e9"
    ATTRS{idVendor}=="1a86"
    ATTRS{serial}=="BC48272EABCD8F43"
    ATTRS{idProduct}=="0610"
    ATTRS{idVendor}=="05e3"
    ATTRS{idProduct}=="0002"
    ATTRS{idVendor}=="1d6b"
    ATTRS{serial}=="0000:68:00.3"
    ATTRS{dbc_idProduct}=="0010"
    ATTRS{dbc_idVendor}=="1d6b"
yimeng@yimeng-ASUS:~$ udevadm info -a -n /dev/ttyACM1 | grep -E "idVendor|idProduct|serial"
    ATTRS{idProduct}=="55e9"
    ATTRS{idVendor}=="1a86"
    ATTRS{serial}=="BC48271FABCD8F34"
    ATTRS{idProduct}=="0610"
    ATTRS{idVendor}=="05e3"
    ATTRS{idProduct}=="0002"
    ATTRS{idVendor}=="1d6b"
    ATTRS{serial}=="0000:68:00.3"
    ATTRS{dbc_idProduct}=="0010"
    ATTRS{dbc_idVendor}=="1d6b"
yimeng@yimeng-ASUS:~$ udevadm info -a -n /dev/ttyACM2 | grep -E "idVendor|idProduct|serial"(zywhilst)
    ATTRS{idProduct}=="55d3"
    ATTRS{idVendor}=="1a86"
    ATTRS{serial}=="5A7A106594"
    ATTRS{idProduct}=="0610"
    ATTRS{idVendor}=="05e3"
    ATTRS{idProduct}=="0002"
    ATTRS{idVendor}=="1d6b"
    ATTRS{serial}=="0000:68:00.3"
    ATTRS{dbc_idProduct}=="0010"
    ATTRS{dbc_idVendor}=="1d6b"

stm32h7
KERNEL=="ttyACM*", ATTRS{serial}=="BC48272EABCD8F43", MODE:="0777", SYMLINK+="tty_stm32h7"

蓝牙
KERNEL=="ttyACM*", ATTRS{serial}=="BC48271FABCD8F34", MODE:="0777", SYMLINK+="tty_bluetooth"
*/



