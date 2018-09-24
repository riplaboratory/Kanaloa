# Wi-Fi Spectrum Regulations
When operating a RF system, there are requirements put in place by the FCC (in the US) that effectively limit how much output power you can transmit.  A Wi-Fi network falls under this umbrella.  This document briefly discusses the regulations in place for operating a Wi-Fi network on an open frequency.

## Frequency use
Wi-Fi is a type of radio frequency (RF) communication, which communicates with frequencies in the range from 3 Hz to 300 GHz.  Note that, although this spectrum electromagnetic radiation is generally referred to as "RF" communication, it contains both radio frequencies (3 Hz to 3 GHz) and microwave frequencies (3 GHz to 300 GHz).

Because RF is very useful for wireless communication, in all countries around the world, the RF spectrum is controlled by the government.  Here in the US, it is controlled by the FCC (Federal Communications Commission), therefore, we should be following the laws they outline.  [A graphical depection of the RF frequency allocation in the US can be found here](https://www.ntia.doc.gov/files/ntia/publications/2003-allochrt.pdf).  Only those frequency bands 

The Wi-Fi standard most commonly-used frequency bands are:
 - 915 MHz (specifically 902 to 928 MHz) defined by 802.11ah
 - 2.4 GHz (specifically 2.390 to 2.450 GHz) defined by 802.11b/g/n/ax
 - 5 GHz (specifically 5.15 GHz to 5.35 GHz, and 5.47 GHz to 5.85 GHz (802.11a/h/j/n/ac/ax).

There are also Wi-Fi standndards in 3.65 GHz (802.11y), 4.9 GHz (802.11j), 5.9 GHz (802.11p), and 60 GHz (802.11ad/ay); however, the FCC requires special radio licenses to use these frequencies.  The 915 MHz, 2.4 GHz, and 5 GHz frequency bands are "amateur" bands that can be operated on without the need for a license.  In this primer document, we will particularly discuss the regulations imposed on these three frequencies.

## Terms

 - **frequency**: the number of periodic waves per second in a signal (in this case, radio frequency signal).  The inverse of frequency is the period of the signal).
 - **band** or **bandwidth**: a term that loosely refers to a range of frequencies (at least for the sake of this discussion).
 - **channel**: a standardized band (range of frequencies) that a given signal is permitted to utilize to avoid interference between other signals operating in a similar band of frequencies.  The larger the channel band, the less likely you are to interfere with another signal, but the less number of signals you can fit in a given frequency range.  In some standards the channels can overlap each other slightly, which increases the chance of interference, but allows more channels to fit in a given frequency range.
 - **intentional radiator power [dBm]**: the power of the source signal provided by the internal radiator; this must then be fed into an antenna to produce an RF wave.  More radiator power _generally_ results in more range. 
 - **antenna gain [dBi]**: the "power" (gain) of the antenna of an RF wave.  More antenna gain _generally_ results in more range.
 - **equivalent isotropically radiated power (EIRP) [dBm or watt]**: the sum of the intentional radiator power and the antenna gain.  A larger EIRP _generally_ results in more range.  Note that EIRP is not quite the same thing as ERP (effective radiated power), though they are similar terms.  [You may read about the difference here](https://en.wikipedia.org/wiki/Effective_radiated_power)
 
**When discussing RF operations, an operator must consider the frequency band their operations will occupy, as well as the EIRP they are sending down their frequency band**.  Different frequency bands will have different reguations, regarding EIRP, so careful selection depending on your range requirements is required.

## 915 MHz
The 915 MHz spectrum (902 to 928 MHz) has very little formal regulation.  The biggest in place by the FCC requires that operators utilize frequency hopping in this spectrum, and that your maximum EIRP is 1 watt (36 dBm).  

Sources:
 - [1] [Understanding the FCC regulations for low-power, non-licensed transmitters (FCC)](https://transition.fcc.gov/Bureaus/Engineering_Technology/Documents/bulletins/oet63/oet63rev.pdf)
 - [2] [FCC 47 CFR 15.247 (Cornell Law)](https://www.law.cornell.edu/cfr/text/47/15.247)

## 2.4 GHz
In north america, there are generally 14 channels, spaced at 5 MHz increments.  [This Wikipedia page](https://en.wikipedia.org/wiki/List_of_WLAN_channels) describes the center frequency of each of these 14 channels.  Becuause it is uncommon to use a channel width of 5 MHz (for bandwidth reasons), a single device will typically span multiple channels.  To avoid interference, you ideally want to have each device intelligently transmitting on different channels.  Some 2.4 GHz devices utilize frequency hopping, although this usually not the case.

Regarding transmission power, the general rule of thumb is a maximum EIRP of 1 watt (36 dBm); however, if utilizing a fixed point-to-point operation, operators may employ larger EIRPs if higher antenna gains [dBi] are utilized.  The relationship between antenna gain [dBi] and intentional radiator power [dBm] for EIRPs larger than 1 watt is given [at this air802.com link](https://www.air802.com/fcc-rules-and-regulations.html).  The graph is screenshotted below for conventience: 
![image](https://github.com/riplaboratory/Kanaloa/blob/master/PrimerDocuments/WifiSpectrumRegulations/Images/FCC2.4GHzBANDRULES.PNG)

From this, you can see that the maximum allowable EIRP is 52 dBm, which can be achieved with a 22 dBm radiator power, and 30 dBi antenna.  A 30 dBi antenna designed for ~2.4 GHz is extremely large and cumbersome; however, so practical considerations need to be made regarding the relationship between radiator power and antenna gain.  

Sources:
 - [1] [List of WLAN Channels (Wikipedia)](https://en.wikipedia.org/wiki/List_of_WLAN_channels)
 - [2] [Understanding the FCC regulations for low-power, non-licensed transmitters (FCC)](https://transition.fcc.gov/Bureaus/Engineering_Technology/Documents/bulletins/oet63/oet63rev.pdf)
 - [3] [FCC 47 CFR 15.247 (Cornell Law)](https://www.law.cornell.edu/cfr/text/47/15.247)
 - [4] [FCC Rules and Regulations (Air802)](https://www.air802.com/fcc-rules-and-regulations.html)

## 5 GHz
In north america, there are genreally 196 channels, spaced at various increments (from 10 MHz all the way to 160 MHz), which means that certain channels will, by definition, overlap with others.  While this may sound like a lot more channels than 2.4 GHz, but not all of the channels are free to use.  Of those that are free to use, the channels are generally divided into four different "bands"

 - **U-NII-1 (low)**: 5.15 GHz to 5.25 GHz (channel 32 to channel 50)
 - **U-NII-2A (middle)**: 5.25 GHz to 5.35 GHz (channel 50 to channel 68)
 - **U-NII-2C (extended)**: 5.47 GHz to 5.730 GHz (channel 96 to channel 144)
 - **U-NII-3 (upper)**: 5.650 GHz to 5.835 GHz (channel 138 to channel 165)
 
While there can be some abiguity regarding these generalized bands (see this [Wikipedia link](https://en.wikipedia.org/wiki/List_of_WLAN_channels) if you're interested in a detailed breakdown), they are important to know, because the band (and subsequently, channel) you're transmitting on significantly affects your legal EIRP.  This screenshot taken from [scc-ares-races.org](https://www.scc-ares-races.org/mesh/doc/WiFi_Part_15_Power_Limits_v150424.pdf) describes the allowed antenna gain and maximum transmitter power for each band.  
![image](https://github.com/riplaboratory/Kanaloa/blob/master/PrimerDocuments/WifiSpectrumRegulations/Images/fccPart15PowerLimitsForWifi.PNG)
Note that "PTP" stands for point-to-point, and "PTMP" stands for point-to-multiple-point.  

Sources:
 - [1] [List of WLAN Channels (Wikipedia)](https://en.wikipedia.org/wiki/List_of_WLAN_channels)
 - [2] [FCC Part 15 Power Limits for WiFi (24-Apr-2015) (Santa Clara County Amateur Radio Emergency Services/Radio Amateur Civil Emergency Services)](https://www.scc-ares-races.org/mesh/doc/WiFi_Part_15_Power_Limits_v150424.pdf)
 - [3] [Understanding the FCC regulations for low-power, non-licensed transmitters (FCC)](https://transition.fcc.gov/Bureaus/Engineering_Technology/Documents/bulletins/oet63/oet63rev.pdf)
 - [4] [FCC 47 CFR 15.247 (Cornell Law)](https://www.law.cornell.edu/cfr/text/47/15.247)

## Regarding range (for future reading)
 - [Height above average terrain](https://en.wikipedia.org/wiki/Height_above_average_terrain)
 - [Bandwith, frequency, and distance](https://physics.stackexchange.com/questions/303314/relation-between-data-rate-frequencyrf-and-distance)
 - [Understanding wireless range calculations](http://www.electronicdesign.com/communications/understanding-wireless-range-calculations)
 - [Eb/N0](https://en.wikipedia.org/wiki/Eb/N0)

