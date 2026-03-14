#How to use Jetson Nano Origin 

There are two jetson nanos, but we(drone) are using the older Jetson(the newer/cleaner/better one is for Martin's capstone project so do not modify the microSD card in that Jetson or he will not be happy). However, we can swap the microSD cards, allowing us to use Martin's newer Jetson with our microSD card with our image/files loaded. (Remember to swap them back after done working). All the necessary power cables are going to be in the box with Martin's Jetson, so that box is heavier. 

You need peripherals to work on the Jetson(keyboard, mouse, monitor(with Displayport))

Troubleshooting Camera:
 * If running a script and you get an error stating no camera is detected,
 * Run sudo /opt/nvidia/jetson-io/jetson-io.py
 * Move cursor to Configure CSI connector and press enter
 * Use arrow keys to highlight + select Camer IMX477-C
 * Click save changes and reset

OR try:
 * power off the Jetson
 * wait ~20 seconds for the power to cool down
 * Unplug power connector + disconnect/reconnect camera from Jetson
 * Power on Jetson and try again

 * If the above doesn't work
 * Ask ChatGPT 
