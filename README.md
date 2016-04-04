sphero_ros
==========

checkout the [docs](http://mmwise.github.com/sphero_ros)

=======
## Installation
Installation is now based on catkin:

      cd <ws>/src
      git clone https://github.com/mmwise/sphero_ros
      cd <ws>
      catkin_make
      catkin_make install
      source <ws>/install/setup.bash
      ...

## BB8 driver
The connection is through BLUETOOTH LE

      hcitool dev
      hcitool (-i hci1) lescan
      hciconfig hci0 up/down

