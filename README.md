max_kinect
==========

Max/MSP/Jitter external for the Kinect, specialized for creating point clouds. 

It uses [libfreenect](https://github.com/OpenKinect/libfreenect) on OSX, and the [Kinect SDK](http://www.microsoft.com/en-us/kinectforwindowsdev/Start.aspx) on Windows. It can support multiple Kinects and different model revisions (tested 3 simultaneous devices with different revision numbers on a Windows machine). 

The help patch example demonstrates the use of calibration files (as produced by [RGBDemo](http://labs.manctl.com/rgbdemo/)) for more accurately calibrated depth / RGB data.

MIT Licensed.
