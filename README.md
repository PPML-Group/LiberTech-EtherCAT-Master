# Liber-EtherCAT-Master
The EtherCAT Master customized for robotic application.
***

## SOEM and Real-Time Setting 

This paper clarifies how to configure the EtherCAT Master for low-level motion control and sensor communication of a robot based on SOEM (Simple Open Source EtherCAT Master) which is opensource and still in maintance for now.   

We use the [EtherCAT SDK Master](https://github.com/leggedrobotics/ethercat_sdk_master) for High-level EtherCAT Functionalities

The packages uesd are listed below
|    Repo           |   url   |   Content   |
|   :---:           |  :---:  |   :---:     |
|ethercat_device_configurator| [device_configurator](https://github.com/leggedrobotics/ethercat_device_configurator.git) | Configures the EtherCAT communication with the device sdks |
| elmo_ethercat_SDK | [elmo](https://github.com/leggedrobotics/elmo_ethercat_sdk?tab=readme-ov-file)  | high-level interface for controlling Elmo motor controllers of the Gold line over EtherCAT (using the CANopen over EtherCAT CoE protocol). |
|ethercat_sdk_master|  [ethercat](https://github.com/leggedrobotics/ethercat_sdk_master) | High-level EtherCAT Functionalities |
|   soem_interface  | [soem_rsl](https://github.com/leggedrobotics/soem_interface) | Low-level EtherCAT Functionalities |
>**Note:** There are tow versions for the packages above, the old one (ethercat_old) is used now and the new one (ethercat_new) changed as below:
>* **EtherCAT device configurator:** Including the startup abort and cahnging the method of loading parameters based on [Anynode](https://github.com/ANYbotics/any_node.git);
>* **EtherCAT SDK Master:** Allowing abort of starup and renamed dependency to soem_interface_rsl;
>* **SOEM_Interface:** Adding soem_rsl and renamed to soem_interface_rsl. Preventing deadlock at shutdown in case slave does low-level soem calls (introduced because recursive mutex removed.)

### SOEM Configuration
#### Real-Time System
  The system need to boot a realtime patch to satisfy the low latency requirement for SOEM. We use the 'PREEMPT_RT' patch to rebuild the Linux kernel. 
  **There are three misconceptions need to distinguish before booting the RT kernel:**
* **A real-time system only needs a real-time kernel**
Before delving deeper into what constitutes real-time Linux, it is vital to understand that a real-time Linux kernel on its own will not necessarily make a system real-time. A kernel is only one component of a real-time system, and even the most efficient real-time operating system (RTOS) can be useless in the presence of other latency sinks. Even with a real-time capable kernel, each system usually requires specific tuning. Meeting the demands of real-time computing takes a careful understanding of the overall stack, from the underlying silicon to the operating system, the networking layer and applications.

* **Real-time = optimised performance**
Another common misconception is that real-time results in so-called optimised performance. The misunderstanding often arises from video applications described as real-time because of the lack of perceived latency. These are usually just best-effort systems that are performant enough to remove any human notice of deadline failures. A real-time Linux kernel does not result in optimised performance. We are looking at a deterministic response to an external event, aiming to minimise response latency rather than optimised throughput. Indeed, a real-time Linux kernel will almost certainly perform worse than CFS or other schedulers in anything but task schedule response.

* **Always necessary**
Also, a real-time OS is not always necessary. It sounds good for something to be ‘real time’, usually because of the performance connotation, but one must look at the actual consequences of a missed deadline and whether they warrant real-time requirements. For instance, if a deadline is on the order of seconds, a multi-GHz CPU with proper tuning will likely not miss it.

  > For more details about real-time system, please refer to [this](https://ubuntu.com/blog/what-is-real-time-linux-i).


#### Preempt-RT Kernel
The optimized real-time ubuntu is available in version of **ubuntu 22.04** or newer via [ubuntu pro](https://ubuntu.com/real-time). 
Ubuntu Pro is free for personal and small scale commercial use in up to 5 machines. With an Ubuntu Pro subscription, launching the real-time kernel is as easy as:
`pro attach <token>`
`pro enable realtime-kernel`

**For more, you can refer to [this](https://ubuntu.com/blog/real-time-kernel-technical).**

However, if you use ubuntu 20.04 or older and without consideration for ubuntu pro, building real-time linux kernel using preempt-RT patches is a good way. 
To get real-time support into a ubuntu system, the following steps have to be performed:
* Get the sources of a real-time kernel
* Compile the real-time kernel
* Setup user privileges to execute real-time tasks

This guide will help you setup your system with a real-time kernel.
##### Preparing
To build the kernel, you will need a couple of tools available on your system. You can install them using
``` bash
$ sudo apt-get install build-essential bc ca-certificates gnupg2 libssl-dev wget gawk flex bison
```
Before you download the sources of a real-time-enabled kernel, check the kernel version that is currently installed:

```bash
$ uname -r
5.15.0-107-generic 
```
You don't need to choose the same vesion exactly, there's no problem for a close version as long as the version of patches strictly corresponds to the kernel.
To continue with this tutorial, please create a temporary folder and navigate into it. You shouldhave sufficient space (around 25GB) there, as the extracted kernel sources take much space. After the new kernel is installed, you can delete this folder again.

In this example we will use a temporary folder inside our home folder:

```bash
$ mkdir -p ${HOME}/rt_kernel_build
$ cd ${HOME}/rt_kernel_build
```
All future commands are expected to be run inside this folder. If the folder is different, the `$` sign will be prefixed with a path relative to the above folder.

##### Getting the sources for a real-time kernel
To build a real-time kernel, we first need to get the kernel sources and the real-time patch.

First, we must decide on the kernel version that we want to use. Above, we determined that our system has a 5.15 kernel installed. However, real-time patches exist only for selected kernel versions. Those can be found on the [linuxfoundation wiki](https://wiki.linuxfoundation.org/realtime/preempt_rt_versions).

In this example, we will select a 5.15 kernel. Select a kernel version close to the
one installed on your system.

Go ahead and download the kernel sources, patch sources and their signature files:
```bash
$ wget https://cdn.kernel.org/pub/linux/kernel/projects/rt/5.15/patch-5.15.158-rt76.patch.xz
$ wget https://cdn.kernel.org/pub/linux/kernel/projects/rt/5.15/patch-5.15.158-rt76.patch.sign
$ wget https://mirrors.edge.kernel.org/pub/linux/kernel/v5.x/linux-5.15.158.tar.xz
$ wget https://mirrors.edge.kernel.org/pub/linux/kernel/v5.x/linux-5.15.158.tar.sign
```
To unzip the downloaded files do
```bash
$ xz -dk patch-5.15.158-rt76.patch.xz
$ xz -d linux-5.15.158.tar.xz
```

##### Verification
Technically, you can skip this section, it is however highly recommended to verify the file
integrity of such a core component of your system!

To verify file integrity, you must first import public keys by the kernel developers and the patch
author. For the kernel sources use (as suggested on
[kernel.org](https://www.kernel.org/signature.html))

```bash
$ gpg2 --locate-keys torvalds@kernel.org gregkh@kernel.org
```

and for the patch search for a key of the author listed on 
[linuxfoundation wiki](https://wiki.linuxfoundation.org/realtime/preempt_rt_versions).

```bash
$ gpg2 --keyserver hkp://keys.gnupg.net --search-keys zanussi
gpg: data source: http://51.38.91.189:11371
(1)     German Daniel Zanussi <german.zanussi@globant.com>
          4096 bit RSA key 0x537F98A9D92CEAC8, created: 2019-07-24, expires: 2023-07-24
(2)     Michael Zanussi <mzanussi@gmail.com>
          4096 bit RSA key 0x7C7F76A2C1E3D9EB, created: 2019-05-08
(3)     Tom Zanussi <tzanussi@gmail.com>
        Tom Zanussi <zanussi@kernel.org>
        Tom Zanussi <tom.zanussi@linux.intel.com>
          4096 bit RSA key 0xDE09826778A38521, created: 2017-12-15
(4)     Riccardo Zanussi <riccardo.zanussi@gmail.com>
          2048 bit RSA key 0xD299A06261D919C3, created: 2014-08-27, expires: 2018-08-27 (expired)
(5)     Zanussi Gianni <g.zanussi@virgilio.it>
          1024 bit DSA key 0x78B89CB020D1836C, created: 2004-04-06
(6)     Michael Zanussi <zanussi@unm.edu>
        Michael Zanussi <mzanussi@gmail.com>
        Michael Zanussi <michael_zanussi@yahoo.com>
        Michael Zanussi <michael@michaelzanussi.com>
          1024 bit DSA key 0xB3E952DCAC653064, created: 2000-09-05
(7)     Michael Zanussi <surfpnk@yahoo.com>
          1024 bit DSA key 0xEB10BBD9BA749318, created: 1999-05-31
(8)     Michael B. Zanussi <surfpnk@yahoo.com>
          1024 bit DSA key 0x39EE4EAD7BBB1E43, created: 1998-07-16
Keys 1-8 of 8 for "zanussi".  Enter number(s), N)ext, or Q)uit > 3
```

Now we can verify the downloaded sources:
```bash
$ gpg2 --verify linux-5.15.158.tar.sign
gpg: assuming signed data in 'linux-5.15.158.tar'
gpg: Signature made Fr 16 Aug 2019 10:15:17 CEST
gpg:                using RSA key 647F28654894E3BD457199BE38DBBDC86092693E
gpg: Good signature from "Greg Kroah-Hartman <gregkh@kernel.org>" [unknown]
gpg: WARNING: This key is not certified with a trusted signature!
gpg:          There is no indication that the signature belongs to the owner.
Primary key fingerprint: 647F 2865 4894 E3BD 4571  99BE 38DB BDC8 6092 693E

$ gpg2 --verify patch-5.15.158-rt76.patch.sign
gpg: assuming signed data in 'patch-5.15.158-rt76.patch'
gpg: Signature made Fr 23 Aug 2019 21:09:20 CEST
gpg:                using RSA key 0x0129F38552C38DF1
gpg: Good signature from "Tom Zanussi <tom.zanussi@linux.intel.com>" [unknown]
gpg:                 aka "Tom Zanussi <zanussi@kernel.org>" [unknown]
gpg:                 aka "Tom Zanussi <tzanussi@gmail.com>" [unknown]
gpg: WARNING: This key is not certified with a trusted signature!
gpg:          There is no indication that the signature belongs to the owner.
Primary key fingerprint: 5BDF C45C 2ECC 5387 D50C  E5EF DE09 8267 78A3 8521
     Subkey fingerprint: ACF8 5F98 16A8 D5F0 96AE  1FD2 0129 F385 52C3 8DF1
```

##### Compilation
Before we can compile the sources, we have to extract the tar archive and apply the patch

```bash
$ tar xf linux-5.15.158.tar
$ cd linux-5.15.158
linux-5.15.158$ xzcat ../patch-5.15.158-rt76.patch.xz | patch -p1 
```
Copy over your old config and use that to configure your new kernel. Now to configure your kernel, just type
```bash
linux-5.15.158$ cp /boot/config-5.15.0-107-generic .config
linux-5.15.158$ make oldconfig
```

```bash
Preemption Model
  1. No Forced Preemption (Server) (PREEMPT_NONE)
> 2. Voluntary Kernel Preemption (Desktop) (PREEMPT_VOLUNTARY)
  3. Preemptible Kernel (Low-Latency Desktop) (PREEMPT__LL) (NEW)
  4. Preemptible Kernel (Basic RT) (PREEMPT_RTB) (NEW)
  5. Fully Preemptible Kernel (RT) (PREEMPT_RT_FULL) (NEW)
choice[1-5]: 5
```

Revise the `.config` file，using `ctrl+F` find the key words，comment the five parameters

```bash
linux-5.15.158$ gedit .config
```

`CONFIG_MODULE_SIG_ALL`
`CONFIG_MODULE_SIG_KEY`
`CONFIG_SYSTEM_TRUSTED_KEYS`
`CONFIG_SYSTEM_REVOCATION_LIST`
`CONFIG_SYSTEM_REVOCATION_KEYS` 

and chang `CONFIG_DEBUG_INFO=n`, 不然新内核带debug信息超大。

This will ask for kernel options. For everything else then the `Preemption Model` use the default
value (just press Enter) or adapt to your preferences. For the preemption model select `Fully Preemptible Kernel`:

Now you can build the kernel. This will take some time...
```bash
linux-5.15.158$ make -j `getconf _NPROCESSORS_ONLN` deb-pkg
```

After building, install the `linux-headers` and `linux-image` packages in the parent folder (only
the ones without the `-dbg` in the name)

```bash
linux-5.15.158$ sudo dpkg -i ../linux-headers-5.15.158-rt76_5.15.158-rt76-1_amd64.deb ../linux-image-5.15.158-rt76_5.15.158-rt76-1_amd64.deb ../linux-libc-dev_5.15.158-rt76-1_amd64.deb
...
```

##### Setup user privileges to use real-time scheduling
To be able to schedule threads with user privileges (what the driver will do) you'll have to change
the user's limits by changing `/etc/security/limits.conf` (See [the manpage](https://manpages.ubuntu.com/manpages/bionic/man5/limits.conf.5.html) for details)

We recommend to setup a group for real-time users instead of writing a fixed username into the config
file:

```bash
$ sudo groupadd realtime
$ sudo usermod -aG realtime $(whoami)
```

Then, make sure `/etc/security/limits.conf` contains
```
@realtime soft rtprio 99
@realtime soft priority 99
@realtime soft memlock 102400
@realtime hard rtprio 99
@realtime hard priority 99
@realtime hard memlock 102400
```

Note: You will have to log out and log back in (Not only close your terminal window) for these
changes to take effect. No need to do this now, as we will reboot later on, anyway.

##### Setup GRUB to always boot the real-time kernel
To make the new kernel the default kernel that the system will boot into every time, you'll have to
change the grub config file inside `/etc/default/grub`.

Note: This works for ubuntu, but might not be working for other linux systems. It might be necessary
to use another menuentry name there.

But first, let's find out the name of the entry that we will want to make the default. You can list
all available kernels using

```bash
$ awk -F\' '/menuentry |submenu / {print $1 $2}' /boot/grub/grub.cfg

menuentry Ubuntu
submenu Advanced options for Ubuntu
    menuentry Ubuntu, with Linux 4.15.0-62-generic
    menuentry Ubuntu, with Linux 4.15.0-62-generic (recovery mode)
    menuentry Ubuntu, with Linux 4.15.0-60-generic
    menuentry Ubuntu, with Linux 4.15.0-60-generic (recovery mode)
    menuentry Ubuntu, with Linux 4.15.0-58-generic
    menuentry Ubuntu, with Linux 4.15.0-58-generic (recovery mode)
    menuentry Ubuntu, with Linux 5.15.158-rt66
    menuentry Ubuntu, with Linux 5.15.158-rt66 (recovery mode)
menuentry Memory test (memtest86+)
menuentry Memory test (memtest86+, serial console 115200)
menuentry Windows 7 (on /dev/sdc2)
menuentry Windows 7 (on /dev/sdc3)
```

From the output above, we'll need to generate a string with the pattern `"submenu_name>entry_name"`. In our case this would be

```
"Advanced options for Ubuntu>Ubuntu, with Linux 5.15.158-rt76"
```
**The double quotes and no spaces around the `>` are important!**

With this, we can setup the default grub entry and then update the grub menu entries. Don't forget this last step!

```bash
$ sudo sed -i 's/^GRUB_DEFAULT=.*/GRUB_DEFAULT="Advanced options for Ubuntu>Ubuntu, with Linux 5.15.158-rt76"/' /etc/default/grub
$ sudo update-grub
```

##### Reboot the PC
After having performed the above mentioned steps, reboot the PC. It should boot into the correct
kernel automatically.

##### Check for preemption capabilities
Make sure that the kernel does indeed support real-time scheduling:

```bash
$ uname -v | cut -d" " -f1-4 
#1 SMP PREEMPT RT
```

##### Optional: Disable CPU speed scaling
Many modern CPUs support changing their clock frequency dynamically depending on the currently
requested computation resources. In some cases this can lead to small interruptions in execution.
While the real-time scheduled controller thread should be unaffected by this, any external
components such as a visual servoing system might be interrupted for a short period on scaling
changes.

To check and modify the power saving mode, install cpufrequtils:
```bash
$ sudo apt install cpufrequtils
```

Run `cpufreq-info` to check available "governors" and the current CPU Frequency (`current CPU
frequency is XXX MHZ`). In the following we will set the governor to "performance".

```bash
$ sudo systemctl disable ondemand
$ sudo systemctl enable cpufrequtils
$ sudo sh -c 'echo "GOVERNOR=performance" > /etc/default/cpufrequtils'
$ sudo systemctl daemon-reload && sudo systemctl restart cpufrequtils
```

This disables the `ondemand` CPU scaling daemon, creates a `cpufrequtils` config file and restarts
the `cpufrequtils` service. Check with `cpufreq-info`.

For further information about governors, please see the [kernel
documentation](https://www.kernel.org/doc/Documentation/cpu-freq/governors.txt).
#### Other settings to reduce latency
It's also important to optimize the performance of NIC. Achieving a low latency in the real-time thread using 
`ethtool rx-usecs = 0`
Otherwise, NAPI intervenes in the network driver => if an NIC interrupt comes, all further NIC interrupts are blocked and the driver polls for a short time for further frames. The result are high latencies.

With rx-usecs 0 NAPI is disabled and you get the minimum latency 
` Copy Frame => NIC RAM => Reserve Kernel skb Buffer and Copy Frame => Copy Frame to Userspace RAM `

But at now, with rx-usecs 0, this process is fired on every receiving frame. Otherwise, the process is used for multiple frames and reduce processing time.
https://github.com/OpenEtherCATsociety/SOEM/issues/171

SOEM adds very little delay to the cycle. Most of it is used up in the Linux kernel network stack on receive. Optimizing packet receive to user space handover is a topic that is well described on the internet. Your friend is ethtool, see drvcomment.txt.

On the other hand it is not most optimal to send a packet and then wait for it to return (as you are doing).
Your situation :
- start cycle - send process data - receive process data - calculations - wait for next cycle start -
Optimal solution 1 :
- start cycle - receive process data - send process data - calculations - wait for next cycle start -
Optimal solution 2 :
- start cycle - receive process data - calculations - send process data - wait for next cycle start -

Solution 1 optimizes compute efficiency, solution 2 optimizes calculation to setpoint delay.

**There are known problems with NMI (for BIOS power management) that can generate latencies up to 2ms.**
NMI stands for Non-Maskable Interrupt, which is a type of hardware interrupt that cannot be ignored (or masked) by the processor. Unlike regular interrupts, which can be enabled or disabled by software, NMIs are used for critical events that require immediate attention, such as hardware failures.

##### Key Characteristics of NMI:
1. **Non-Maskable**: NMIs cannot be disabled or ignored by the CPU. This ensures that critical events are always handled promptly.
2. **High Priority**: NMIs have a higher priority than regular interrupts. When an NMI occurs, it immediately interrupts the current execution flow.
3. **Common Uses**:
   - **Hardware Failures**: NMIs are often used to signal severe hardware issues such as parity errors in memory or other hardware malfunctions.
   - **Watchdog Timers**: Used to detect system hangs or crashes, forcing a reset or other recovery action.
   - **Debugging**: In some systems, NMIs are used by debugging tools to interrupt the CPU for debugging purposes.

##### How NMIs Work in BIOS:
- When an NMI occurs, the CPU immediately suspends the current execution and jumps to a predefined interrupt handler defined by the BIOS or operating system.
- The BIOS typically includes an NMI handler routine to diagnose the cause of the NMI, log the error, and attempt to recover from it if possible.
- In some systems, the BIOS may also provide configuration options to enable or disable certain NMI sources.

##### Common Sources of NMI:
1. **Parity Errors**: Memory parity errors caused by faulty RAM modules.
2. **I/O Device Errors**: Errors from critical I/O devices that cannot be ignored.
3. **Watchdog Timer**: A hardware timer that triggers an NMI if the system becomes unresponsive.
4. **Debugging**: Certain debugging tools can trigger an NMI to gain control of the CPU.

##### Example of NMI Handling in BIOS:
When an NMI is triggered, the BIOS or operating system will execute an NMI handler function. This function typically involves:
1. **Logging the Error**: Recording details of the error in a log for diagnostics.
2. **Attempting Recovery**: Trying to reset or reinitialize the failing component.
3. **Alerting the User**: Providing a message to the user, often via a system beep or error message on the screen.
4. **System Halt or Restart**: If the error is severe and cannot be recovered, the system may halt or automatically restart.

##### Example Code Snippet:
Here's a simplified example of an NMI handler in a C-like pseudocode:

```c
void NMI_Handler() {
    // Log the NMI event
    logEvent("Non-Maskable Interrupt occurred");

    // Check the source of the NMI
    if (checkMemoryParityError()) {
        logEvent("Memory parity error detected");
        attemptMemoryRecovery();
    } else if (checkWatchdogTimeout()) {
        logEvent("Watchdog timer timeout");
        restartSystem();
    } else {
        logEvent("Unknown NMI source");
    }

    // Attempt to recover or halt the system
    if (!recoverableError()) {
        haltSystem();
    }
}
```

##### NMI Configuration in BIOS Setup:
Some BIOS setups provide options to configure NMI behavior:
- **Enable/Disable NMI**: Allow the user to enable or disable NMIs from certain sources.
- **NMI Logging**: Options to enable detailed logging of NMI events.
- **Watchdog Timer Settings**: Configure the behavior of the watchdog timer, including the timeout period and actions upon timeout.

##### Conclusion:
NMIs are crucial for handling critical system events that require immediate attention. The BIOS plays a key role in handling these interrupts, diagnosing the source of the problem, and attempting recovery. By understanding NMIs and their handling, you can better diagnose and troubleshoot severe hardware issues in computer systems.

### References:
> **Preempt-RT**
>https://chenna.me/blog/2020/02/23/how-to-setup-preempt-rt-on-ubuntu-18-04/
https://blog.csdn.net/qq_28882933/article/details/118293544
https://blog.csdn.net/Already8888/article/details/136230445
https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/edit/master/ur_robot_driver/doc/real_time.md
https://ubuntu.com/blog/real-time-kernel-technical
