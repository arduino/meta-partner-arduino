`kernel-modules/x8h7`
=====================
#### Build/Deploy/Install
```bash
bitbake x8h7
adb push deploy/ipk/portenta_x8/x8h7_0.1-r1_portenta_x8.ipk /home/fio
```
```bash
adb shell
cd /home/fio
sudo opkg install --force-reinstall x8h7_0.1-r1_portenta_x8.ipk
```
#### Unload/Reload
```bash
lsmod | grep x8h7_
sudo rmmod x8h7*
sudo rmmod x8h7_drv.ko
cd /usr/arduino/extra
sudo ./load_modules.sh
```
