# 附录

- [附录](#附录)
  - [配置udev规则](#配置udev规则)

## 配置udev规则

因为Ubuntu会自动分配USB串口设备的串口号，所以有时候`/dev/ttyACM0`会变成`/dev/ttyACM1`，导致一些使用串口的程序在使用`/dev/ttyACM0`串口时会出现问题。通过配置udev规则可以实现将对应设备标识为正确的`/dev/ttyACM0`

1. 查看usb连接信息：
   - 输入`lsusb`可以显示出当前的串口设备的信息。如：

        ``` shell
        polarbear@polarbear-NUC:~$ lsusb
        Bus 004 Device 001: ID 1d6b:0003 Linux Foundation 3.0 root hub
        Bus 003 Device 003: ID 8087:0026 Intel Corp. AX201 Bluetooth
        Bus 003 Device 023: ID 0483:5740 STMicroelectronics Virtual COM Port
        Bus 003 Device 001: ID 1d6b:0002 Linux Foundation 2.0 root hub
        Bus 002 Device 001: ID 1d6b:0003 Linux Foundation 3.0 root hub
        Bus 001 Device 001: ID 1d6b:0002 Linux Foundation 2.0 root hub
        ```

   其中`STMicroelectronics Virtual COM Port`就是我们的C板设备，记住2个关键ID：`0483-idVendor`，`5740-idProduct`。接下来我们对该设备配置对应的udev规则。

2. 配置udev规则：
   1. 打开`.rules`文件来编辑udev规则

      ``` shell
      sudo nano /etc/udev/rules.d/99-C_Board.rules
      ```

   2. 写入udev规则，将我们的设备的别名配置为`ttyACM0`，这样如果我们的设备被检测到，系统就会在自动分配串口名称后通过该规则将设备配置一个别名`ttyACM0`。（将`idVendor`和`idProduct`替换为自己设备对应的id）

      ``` shell
      SUBSYSTEMS=="usb", KERNEL=="ttyACM*", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", SYMLINK+="ttyACM0"
      ```

   3. 按`ctrl+x`退出，按`y`保存文件

3. 重新加载udev规则：

   ``` shell
   sudo udevadm control --reload-rules
   ```

4. 重新插拔设备，再输入`ls /dev/tty*`
   - 即可看到`/dev/ttyACM1`和连接到的别名`/dev/ttyACM0`（蓝色的）
