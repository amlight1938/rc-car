## Microcontroller

[Elegoo ESP32](https://www.amazon.com/dp/B0D8T53CQ5?lv=shuf&channelId=500&plpRedirect=mhFallback)

[ESP32 documentation](https://www.espboards.dev/esp32/esp32doit-devkit-v1/)

## Parts

[Motors](https://www.amazon.com/dp/B0DB28GC7B?smid=A2DSPVS66G15B3&ref_=chk_typ_imgToDp&th=1)
The motor purchased is the 12V 300RPM with a 7PPR "N20" Encoder Motor.  The gearbox seems to be 1:100 but its far from confirmed.
Using an encoder setup with Quadrature, we read the trigger count over 20 axle rotations (to reduce error).
20 Rot Counts were:  55400, 55420, 55470, 55460, 55430.  Average = 55436.
Dividing by 20 gives: 2772 triggers per axle rotation.
Dividing by 28 (7ppr*4 due to quadrature) gives: 99 motor rotations per axle rotation.
This falls in line with the assumed 1:100 gear ratio.
However, the nominal 1:100 gear ratio may not be mechanically accurate, and therefore, I will use the estimated 2772 triggers/axle rotation as the most accurate value.

[Battery](https://www.amazon.com/dp/B0FCFVWS1K?psc=1&smid=A3CTZBWJRYQEJA&ref_=chk_typ_imgToDp)

[Battery Plug Male](https://www.amazon.com/T-Plug-T-Connectors-Deans-Style-BDHI-28/dp/B07MDGT5C1/)

[Motor Driver](https://www.amazon.com/dp/B0GSGGZ39F?psc=1&smid=A2U5NW7LLKYQ0G&ref_=chk_typ_imgToDp)

[RC Car 3D Print Design](https://www.tinkercad.com/things/47Z9w4BictA-rc-car-template/edit?returnTo=https%3A%2F%2Fwww.tinkercad.com%2Fdashboard%2Fdesigns%2Fall&sharecode=sbzTbxtYG3wgvbyh8qcZ0lMcqWFFF9wqbeRjlP2IIGg)

We are primarily using 22awg solid wires for connections.  There are 20awg solid wires soldered to the battery adapter as a larger current step down.

## Part Dimensions

Battery: 67mm x 37mm x 19mm (leave 1 end cap open for wires)

2 bread boards edge to edge: 108mm x 82mm x 10mm