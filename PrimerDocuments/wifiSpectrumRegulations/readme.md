# Wi-Fi Spectrum Regulations
When operating a RF system, there are requirements put in place by the FCC (in the US) that effectively limit how much output power you can transmit.  A Wi-Fi network falls under this umbrella.  This document briefly discusses the regulations in place for operating a Wi-Fi network on an open frequency.

## Frequency use
Wi-Fi is a type of radio frequency (RF) communication, which fundamentally described by electromagnetic radiation with frequency in the range from 3 Hz to 3 GHz.  In practice, RF also includes the microwave spectrum (3 GHz to 300 GHz) of electromagnetic radiation.  Thus, "RF" generally refers to electromagnetic radiation with frequency in the range from 3 Hz to 300 GHz.  

Because RF is very useful for wireless communication, in all countries around the world, the RF spectrum is controlled by the government.  Here in the US, it is controlled by the FCC (Federal Communications Commission), therefore, we should be following the laws they outline.  [A graphical depection of the RF frequency allocation in the US can be found here](https://www.ntia.doc.gov/files/ntia/publications/2003-allochrt.pdf).  

Wi-Fi commonly operates in the 900 MHz, 2.4 GHz, and 5 GHz frequency band; this is largely because these frequency bands are regulated as "amateur" bands by the FCC, meaning that, with minor regulation, anyone can transmit and receive on these bands without a license.  In this primer document, we will particularly discuss the regulations imposed on the 2.4 and 5 GHz frequency bands.  

## Terms

 - **frequency**: the number of periodic waves per second in a signal (in this case, radio frequency signal).  The inverse of frequency is the period of the signal).
 - **band** or **bandwidth**: a term that loosely refers to a range of frequencies.
 - **channel**: a standardized band (range of frequencies) that a given signal is permitted to utilize to avoid interference between signals.  The larger the channel band, the less likely you are to interfere with another signal, but the less number of signals you can fit in a given frequency range.  In some standards the channels can overlap each other slightly, which increases the chance of interference, but allows more channels to fit in a given frequency range.
 - **intentional radiator power [dBm]**: the power of the source signal provided by the internal radiator; this must then be fed into an antenna to produce an RF wave.  More radiator power _generally_ results in more range. 
 - **antenna gain [dBi]**: the "power" (gain) of the antenna of an RF wave.  More antenna gain _generally_ results in more range.
 - **equivalent isotropically radiated power (EIRP) [dBm or watt]**: the sum of the intentional radiator power and the antenna gain.  A larger EIRP _generally_ results in more range.  Note that EIRP is not quite the same thing as ERP (effective radiated power), though they are similar terms.  [You may read about the difference here](https://en.wikipedia.org/wiki/Effective_radiated_power)

## 2.4 GHz
In north america, there are generally 13 channels, spaced at 5 MHz increments.  This screenshot taken from [this Wikipedia page](https://en.wikipedia.org/wiki/List_of_WLAN_channels) describes the center frequency of each of these 13 channels:
![screenshot](https://github.com/riplaboratory/Kanaloa/blob/master/PrimerDocuments/wifiSpectrumRegulations/InterferenceConcerns.PNG)
It is possible to have more than one device occupying a channel at the same time; however, you run a significant risk of interference when doing this.  Ideally, you want to have each device intelligently transmitting on different channels.

Regarding transmission power, the screenshot taken from [FCC 2.4 GHz BAND RULES (POINT-TO-POINT) (air802.com)](https://www.air802.com/fcc-rules-and-regulations.html) describes the relationship between radiator gain and antenna gain.
![screenshot](https://github.com/riplaboratory/Kanaloa/blob/master/PrimerDocuments/wifiSpectrumRegulations/FCC2.4GHzBANDRULES.PNG)

From this, you can see that the maximum allowable EIRP is 52 dBm, which can be achieved with a 22 dBm radiator power, and 30 dBi antenna.  A 30 dBi antenna designed for ~2.4 GHz is extremely large and cumbersome; however, so practical considerations need to be made regarding the relationship between radiator power and antenna gain.  

These regulations come from [FCC Title 47 CFR 15.247](https://www.law.cornell.edu/cfr/text/47/15.247).

## 5 GHz
In north america, there are genreally 196 channels, spaced at various increments (from 10 MHz all the way to 160 MHz), which means that certain channels will, by definition, overlap with others.  While this may sound like a lot more channels than 2.4 GHz, but not all of the channels are free to use.  Of those that are free to use, the channels are generally divided into four different "bands"

 - **U-NII-1 (low)**: 5.15 GHz to 5.25 GHz (channel 32 to channel 50)
 - **U-NII-2A (middle)**: 5.25 GHz to 5.35 GHz (channel 50 to channel 68)
 - **U-NII-2C (extended)**: 5.47 GHz to 5.730 GHz (channel 96 to channel 144)
 - **U-NII-3 (upper)**: 5.650 GHz to 5.835 GHz (channel 138 to channel 165)
 
While there can be some abiguity regarding these generalized bands (see this [Wikipedia link](https://en.wikipedia.org/wiki/List_of_WLAN_channels) if you're interested in a detailed breakdown), they are important to know, because the band (and subsequently, channel) you're transmitting on significantly affects your legal EIRP.  This screenshot taken from [scc-ares-races.org](https://www.scc-ares-races.org/mesh/doc/WiFi_Part_15_Power_Limits_v150424.pdf) describes the allowed antenna gain and maximum transmitter power for each band.  
![screenshot](https://github.com/riplaboratory/Kanaloa/blob/master/PrimerDocuments/wifiSpectrumRegulations/fccPart15PowerLimitsForWifi.PNG)
Note that "PTP" stands for point-to-point, and "PTMP" stands for point-to-multiple-point.  

These regulations come from [FCC Title 47 CFR 15.247](https://www.law.cornell.edu/cfr/text/47/15.247).

## Regarding range
 - [Height above average terrain](https://en.wikipedia.org/wiki/Height_above_average_terrain)
 - [Bandwith, frequency, and distance](https://physics.stackexchange.com/questions/303314/relation-between-data-rate-frequencyrf-and-distance)
 - [Understanding wireless range calculations](http://www.electronicdesign.com/communications/understanding-wireless-range-calculations)
 - [Eb/N0](https://en.wikipedia.org/wiki/Eb/N0)

