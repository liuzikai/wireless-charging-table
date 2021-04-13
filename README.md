This is SP21 ECE445/ME470 code for team 24 at ZJUI

## Setup GPIO
Adapted from [JetsonGPIO README](https://github.com/pjueon/JetsonGPIO#setting-user-permissions)

In order to use the Jetson GPIO Library, the correct user permissions/groups must  
be set first. Or you have to run your program with root permission.

Create a new gpio user group. Then add your user to the newly created group.
```
sudo groupadd -f -r gpio
sudo usermod -a -G gpio $(whoami)
```
Install custom udev rules by copying the 99-gpio.rules file into the rules.d  
directory. The 99-gpio.rules file was copied from NVIDIA's official repository.

```
sudo cp lib/JetsonGPIO/99-gpio.rules /etc/udev/rules.d/
```

For the new rule to take place, you either need to reboot or reload the udev
rules by running:
```
sudo udevadm control --reload-rules && sudo udevadm trigger
```

## Build
```shell
git submodule update --init
mkdir build
cd build
cmake ..
make
```