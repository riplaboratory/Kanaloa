# Wi-Fi Spectrum Regulations
When operating a RF system, there are requirements put in place by the FCC (in the US) that effectively limit how much output power you can transmit.  A Wi-Fi network falls under this umbrella.  This document briefly discusses the regulations in place for operating a Wi-Fi network on an open frequency.

## Frequency use
Wi-Fi is a type of radio frequency (RF) communication, which fundamentally described by electromagnetic radiation with frequency in the range from 3 Hz to 3 GHz.  In practice, RF also includes the microwave spectrum (3 GHz to 300 GHz) of electromagnetic radiation.  Thus, "RF" generally refers to electromagnetic radiation with frequency in the range from 3 Hz to 300 GHz.  

Because RF is very useful for wireless communication, in all countries around the world, the RF spectrum is controlled by the government.  Here in the US, it is controlled by the FCC (Federal Communications Commission), therefore, we should be following the laws they outline.  [A graphical depection of the RF frequency allocation in the US can be found here](https://www.ntia.doc.gov/files/ntia/publications/2003-allochrt.pdf).  

Wi-Fi commonly operates in the 900 MHz, 2.4 GHz, 3.3 GHz, and 5 GHz frequency band; this is largely because these frequency bands are regulated as "amateur" bands by the FCC, meaning that, with minor regulation, anyone can transmit and receive on these bands without a license.  In this primer document, we will particularly discuss the regulations imposed on the 2.4 and 5 GHz frequency bands.

## Radiator Power [dBm], Antenna Gain [dBi], and EIRP [dBm or watts]
A RF transmit system consists of two components: (1) an intential radiator that provides the signal, and (2) an antenna that broadcasts it.  The radiator can provide the signal at some power, typically measured in dBm or W, and the antenna can provide a gain (the larger the antenna the larger the gain) to this signal typically measured in dBi or W.  Adding these two values together results in your equivalent isotropically radiated power (EIRP) for that combination of radiator and antenna.  More radiator power, and a larger antenna gain results in more EIRP, which _generally_ results in more range (all other things equal).

These concepts are important to know because this is how the FCC limits your transmit power over RF.  Note that EIRP is not quite the same thing as ERP (effective radiated power), though they are similar terms.  [You may read about the difference here](https://en.wikipedia.org/wiki/Effective_radiated_power)

## 2.4 GHz
The screenshot taken from [air802.com](https://www.air802.com/fcc-rules-and-regulations.html) describes the relationship between radiator gain and antenna gain.
![screenshot](https://github.com/riplaboratory/Kanaloa/blob/master/PrimerDocuments/wifiSpectrumRegulations/FCC2.4GHzBANDRULES(POINT-TO-POINT).PNG)

## Regarding range
[Height above average terrain](https://en.wikipedia.org/wiki/Height_above_average_terrain)

