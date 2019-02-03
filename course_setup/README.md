# Course computer setup

If you are dual-booting or using a virtual machine, this describes how to set up your computer identically to how the course computers are set up.

- First, make sure you install Ubuntu 14.04.
  Other versions of Ubuntu will not work.
- `source` the lab machine tools at the end of `~/.bashrc` with this line: `source ~/catkin_ws/src/cse481wi19/course_setup/labmachine.bash`
- If you are on a laptop, then most likely you will need to edit the course tools.
  Find the function `my_ip` and change `eth0` to `wlan0`.
  Typing `source ~/.bashrc; my_ip` into the terminal should show your IP address.
  If this doesn't work, check which network interface has been assigned an IP address using `ifconfig`.
- Run the install script:
  ```bash
  chmod +x install_cse481c.bash
  ./install_cse481c.bash
  ```

The process should take about 20 minutes.
If this doesn't work, let the course staff know.
