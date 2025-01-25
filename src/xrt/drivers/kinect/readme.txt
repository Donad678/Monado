Steps to make the Kinect driver work:
1. Obtain NiTE2 from somewhere.
2. Put libNiTE2.so in your LD_LIBRARY_PATH.
3. Install openni2. Put the openni2 directory containing headers somewhere in your include path.
4. Put the NiTE2 directory containing models somewhere.
5. At runtime, set the environment variable NITE2_PATH to the path to that directory.