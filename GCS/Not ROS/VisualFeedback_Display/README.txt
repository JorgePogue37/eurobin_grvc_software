Tracking Module Display

Author: Alejandro Suárez Fernández-Miranda
Date: August 2014
Project: EC-SAFEMOBIL
Organization: University of Seville, GRVC


DESCRIPTION

The TMDisplay executable allows the visualization of JPEG encoded images sent by a tracking module through a WiFi network. Frames captured by the tracking module are encoded with a low quality so the resulting data is around 10 KBytes (for a JPEG quality of 25%, the size of the encoded image is around 9 KBytes). Before a encoded image is sent, the tracking module sent 20 bytes with an image header consisting in the following fields:

	- Eight characters with the "JPEG_IMG" sequence
	- The size of the encoded image (this is NOT the same as the image resolution)
	- The size of the packet
	- The 16-bit XOR checksum

Taking into account router limitations in packet size, it is possible that the encoded image is sent in one or more packets. The maximum size of the packets is specified in the tracking module side. For example, if the maximum size of the packet is 5000 bytes and the encoded image takes 9100 bytes, two packets will be sent: the first with 5000 bytes size, and the second with 4100 bytes size. Finally, the checksum received in the image header is compared with the checksum computed over the received encoded image. If both checksum's match, then the image is decoded.


EXECUTION

		$ ./TMDisplay ListeningPort


NOTES

The TMDisplay executable is designed for the visualizaton of the encoded images sent by a single tracking module. If there is more than one tracking module, for example TM1 and TM2, each tracking module will send their images through different ports, Port1 and Port2, Port1 != Port2.


