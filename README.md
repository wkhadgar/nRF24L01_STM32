# nRF24L01_STM32
based on https://github.com/LonelyWolf/stm32/tree/master/nrf24l01

This lib is an update an adaptation of the above lib, considering an updated aproach of the communication workflow.

Normal use is: 
1. TX starts a ```nRF24_Talk``` with a ```nrf24_data_t``` packet made with ```nRF24_PrepareData``` , and can, if needed, wait for an answer.
2. RX listens to the transmission and sets the ```RX_DR``` IRQ, after that you can ```nRF24_GetData``` the received packet.
4. If the packet is a ```REQUEST``` kind packet, RX should send an answer packet, with a ```RESPONSE``` kind. 
5. RX then retrieves the packet data, with ```nRF24_RetrieveData``` and do whatever it wants.

Some helpful notes:
1. If you pretend using this module, be aware that there is a huge mistake on the unoriginal ones, CRC can't be used normally and this leads to some problems with the full funcionality of the ESB feature.
2. Those modules are EXTREMELY sensitive to EMF, do your best to provide steady and clean current to them, also NEVER place it close to switching regulators/sources. I have placed it around 5cm close to a buck converter, and spent more then 2 weeks to finally realize the problem was the noise and coupling and not the module itself...
4. If possible, attach an 10uF capacitor to the VCC-GND of the module, this greatly improves it's stability.
5. If you have any  doubts about the use if this lib feel free to contact me!
