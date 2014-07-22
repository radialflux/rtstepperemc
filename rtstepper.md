rtstepper

David Suffield has written a unique USB solution for CNC controllers with the rt-stepper USB-to-Parallel dongle. Users can use the rt-stepper dongle to convert a parallel port CNC controller into a USB CNC controller. Designers can integrate the rt-stepper dongle hardware into there own CNC controller.

Most CNC controllers are designed to be driven with a PC parallel port. Unfortunately parallel ports are thing of the past and many new PCs have no parallel port. Commercial USB-to-Parallel dongles do exist, but these dongles are designed for printers not CNC controllers. CNC controllers require XYZ step/direction signals. With the rt-stepper dongle you can now drive XYZ step/direction signals over the USB bus.

Driving a CNC controller with a PC parallel port requires a real time OS otherwise the stepper motors can loose synchronization. With the rt-stepper dongle this restriction is removed. The rt-stepper dongle provides the real time stepper motor control which means you can now use your favorite OS such as Linux, Mac or Windows. Each dongle provides a step resolution of 46,875 Hz.
Most commercial gcode interpreters do NOT support laptops with parallel port chipsets. This is not a problem with the rt-stepper dongle. You can use any Linux, Mac or Windows laptop or netbook with USB support. Laptops can provide dedicated CNC setups with minimal investment and work space.

The rt-stepper software (rtstepperemc) is open source. Starting with version 1.7 the software is based on the EMC2 open source project at www.linuxcnc.org. This provides a number of improvements over the original rt-stepper software - 4-axis support, complete gcode interpreter, trajectory planner, GUI front-end and Backploting. Since the rt-stepper dongle provides the real time step pulse generator this version of EMC2 can run on Linux, Mac and Windows with less runtime dependencies.
There is one process that runs on the PC - TkMini. TkMini is a GUI front-end program that communicates to the dongle. Both manual and auto CNC operations are supported. TkMini main screen is showed below, for more screen shots see the rt-stepper External Reference Specification (ERS).

The rt-stepper software supports the same gcode interpreter as EMC2. See the "Gcode Overview" section at www.linuxcnc.org for program syntax, parameters, expressions, binary operators, functions and looping constructs. See the ERS for rt-stepper specific features and instructions.

I started using EMC back in 2002. I developed this product after using EMC for several years with my Sherline CNC mills. EMC worked great, but requires a PC with a real-time kernel and a parallel port to control the mill. Now I have the best of both worlds EMC2 based software with no real-time kernel requirements.

I have two mills and two CNC controllers - Camtronics and Xylotex. I now use the rt-stepper software exclusively. I can use the latest notebooks, which have no parallel port, with my old CNC controllers.

I first introduced the rt-stepper dongle back in 2009 with my own small terminal program that ran a small subset of CNC gcodes. I have no plans for new development on the rt-stepper classic software, but rt-stepper classic software is still available on the download page.

Originally I built the dongle PCB on my mill. Now the dongle PCB is made at a local USA circut board manufacturer, but I still make the custom case with rt-stepper. You don't get a feel for a product unless you use it.

Find more information & purchase the USB dongle @ [http://www.ecklersoft.com][1]

[1]:	http://www.ecklersoft.com