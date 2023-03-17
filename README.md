# Lucid Drivers based on the Arena SDK

## Bandwidth Sharing

See https://docs.google.com/document/d/1IUDtQOMSp7KUJ4WhsTRIBgT0-2yy3JhPRoxAbJXgogc

To compute packet and frame delay, follow the guidelines there:
https://support.thinklucid.com/app-note-bandwidth-sharing-in-multi-camera-systems/


A basic setup is to compute frame transmission and packet delays (in
nanoseconds) as described in the Lucid document above, and configure the cameras
as follows:

~~~ yaml
ptp_config:
   enabled: true
image_config:
   acquisition_start_mode: ACQUISITION_START_MODE_PTPSYNC
transmission_config:
   frame_transmission_delay: <computed delay>
   packet_delay: <computed delay>
~~~

In the "fully interleaved" scheme, the packet delay is the same for all cameras
(transmission duration of one packet times number of cameras minus 1) and the
frame transmission delay is the packet delay times the "camera index".

The "full first frame and then interleaved" scheme would set:

- first camera: packet and frame transmission delay to zero
- cameras 0-N: packet delay is transmission of one packet times N-1 and frame
  transmission delay is the transmission of the first camera's full frame plus the
  transmission of one packet times i (the camera index)

To compute the transmission duration of a full frame, I use the rule of thumb of
a full Bayer frame being width * height * 1.1 (in bytes). Given that we already
have a 10% margin on the delays, it starts to add up and we can't use the full
link. But that works, and you can optimize afterwards.

