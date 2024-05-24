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


#### Other settings to reduce latency


### References:
> **Preempt-RT**
>https://chenna.me/blog/2020/02/23/how-to-setup-preempt-rt-on-ubuntu-18-04/
https://blog.csdn.net/qq_28882933/article/details/118293544
https://blog.csdn.net/Already8888/article/details/136230445