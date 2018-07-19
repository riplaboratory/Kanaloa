# Wi-Fi Spectrum Regulations
When operating a RF system, there are requirements put in place by the FCC (in the US) that effectively limit how much output power you can transmit.  A Wi-Fi network falls under this umbrella.  This document briefly discusses the regulations in place for operating a Wi-Fi network on an open frequency.

## Frequency use
Wi-Fi is a type of radio frequency (RF) communication, which fundamentally described by electromagnetic radiation with frequency in the range from 3 Hz to 3 GHz.  In practice, RF also includes the microwave spectrum (3 GHz to 300 GHz) of electromagnetic radiation.  Thus, "RF" generally refers to electromagnetic radiation with frequency in the range from 3 Hz to 300 GHz.  

Because RF is very useful for wireless communication, in all countries around the world, the RF spectrum is controlled by the government.  Here in the US, it is controlled by the FCC (Federal Communications Commission), therefore, we should be following the laws they outline.  [A graphical depection of the RF frequency allocation in the US can be found here](https://www.ntia.doc.gov/files/ntia/publications/2003-allochrt.pdf).  

Wi-Fi commonly operates in the 900 MHz, 2.4 GHz, 3.3 GHz, and 5 GHz frequency band; this is largely because these frequency bands are regulated as "amateur" bands by the FCC, meaning that, with minor regulation, anyone can transmit and receive on these bands without a license.  In this primer document, we will particularly discuss the regulations imposed on the 2.4 and 5 GHz frequency bands.

## Terms

 - *frequency*: the number of periodic waves per second in a signal (in this case, radio frequency signal).  The inverse of frequency is the period of the signal).
 - *band* or *bandwidth*: a term that loosely refers to a range of frequencies.
 - *channel*: a standardized band (range of frequencies) that a given signal is permitted to utilize to avoid interference between signals.  The larger the channel band, the less likely you are to interfere with another signal, but the less number of signals you can fit in a given frequency range.  In some standards the channels can overlap each other slightly, which increases the chance of interference, but allows more channels to fit in a given frequency range.
 - *intentional radiator power [dBm]*: the power of the source signal provided by the internal radiator; this must then be fed into an antenna to produce an RF wave.  More radiator power _generally_ results in more range. 
 - *antenna gain [dBi]*: the "power" (gain) of the antenna of an RF wave.  More antenna gain _generally_ results in more range.
 - *equivalent isotropically radiated power (EIRP) [dBm or watt]*: the sum of the intentional radiator power and the antenna gain.  A larger EIRP _generally_ results in more range.  Note that EIRP is not quite the same thing as ERP (effective radiated power), though they are similar terms.  [You may read about the difference here](https://en.wikipedia.org/wiki/Effective_radiated_power)

## 2.4 GHz
In north america, there are generally 13 channels, spaced at 5 MHz increments.  This screenshot taken from Wikipedia describes the center frequency of each of these 13 channels:
[Interference Concerns (Wikipedia)](https://github.com/riplaboratory/Kanaloa/blob/master/PrimerDocuments/wifiSpectrumRegulations/InterferenceConcerns.PNG)
The screenshot taken from [FCC 2.4 GHz BAND RULES (POINT-TO-POINT) (air802.com)](https://www.air802.com/fcc-rules-and-regulations.html) describes the relationship between radiator gain and antenna gain.
![screenshot](https://github.com/riplaboratory/Kanaloa/blob/master/PrimerDocuments/wifiSpectrumRegulations/FCC2.4GHzBANDRULES(POINT-TO-POINT).PNG)

## Regarding range
[Height above average terrain](https://en.wikipedia.org/wiki/Height_above_average_terrain)

