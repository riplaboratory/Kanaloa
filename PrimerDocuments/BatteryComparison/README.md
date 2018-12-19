# Battery Comparison

The purpose of this document is to analyze, at a very high level, the price-to-performance characteristics of nickel metal hydride, and lithium ion rechargeable batteries.

There are generally four different classes of rechargeable battery:

 1. Lead Acid
 2. Lithium Ion Li-Ion
 3. Nickel Cadmium (NiCd)
 4. Nickel Metal Hydride (NiMh)

It is generally well understood that NiMh and Li-Ion batteries provide the best performance in energy density, this document will explore the difference between these two technologies.

Note that significant linear approximations were made in these battery calculations.  

## Battery Calculations

 __Lead Acid: [WestMarine AGM 105](https://www.westmarine.com/buy/west-marine--group-31-dual-purpose-agm-battery-105-amp-hours--15020258)__
 
  - Battery Specifications
    - price: $339.99
    - capacity: 105 Ah or 1260 Wh per battery
    - max discharge: 800 A (cold), 1000 A ("marine")
    - mass: 31.3 kg per battery
    - volume: (0.32861m)(0.17145m)(0.23813m) = 0.013416 m^3 ≈ 13416 mL
  - Calculations:
    - $/Wh = (339.99$)/(1260Wh) = $0.26983 per Wh ≈ __$0.27 per Wh__ (lower is better)
    - mass energy density = (1260Wh)/(31.3kg) = 40.256 Wh per kg ≈ __40 Wh per kg__ (higher is better)
    - volumetric energy density = (1260Wh)/(0.013416m^3) = 93918 Wh per m^3 ≈ __93 kWh per m^3__ (higher is better)

__NiMh: [AmazonBasics AA High-Capacity Rechargable Batteries (8-Pack)](https://www.amazon.com/AmazonBasics-High-Capacity-Rechargeable-Batteries-Pre-charged/dp/B00HZV9WTM/)__

 - Battery Specifications:
   - price: $18.99 ($2.37 per battery)
   - capacity: 2.4 Ah or 2.88 Wh per battery
   - max discharge: 6 A per battery
   - mass: 31 g per battery
   - volume: (0.05m)*2*pi*(0.007m)^2 = 15.393E-6 m^3 ≈ 15.4 mL
 - Calculations:
   - $/Wh = (2.37$)/(2.88Wh) = $0.82291 per Wh ≈ __$0.82 per Wh__ (lower is better)
   - mass energy density = (2.88Wh)/(0.031kg) = 92.903 Wh per kg ≈ __93 Wh per kg__ (higher is better)
   - volumetric energy density = (2.88Wh)/(15.393E-6m^3) = 187100 Wh per m^3 ≈ __190 kWh per m^3__ (higher is better)
   
 __LiPo: [Multistar High Capacity 16000mAh 4S 12C Multi-Rotor Lipo Pack w/XT90](https://hobbyking.com/en_us/multistar-high-capacity-16000mah-4s-12c-multi-rotor-lipo-pack-w-xt90.html?___store=en_us)__

  - Battery Specifications
    - price: $142.74 (on sale for $74.94 on 2018.11.11)
    - capacity: 16 Ah or 236.8 Wh
    - max discharge: 12C or 192 A
    - mass: 1290 g
    - volume: (0.173m)(0.074m)(0.045m) = 576.09E-6 m^3 ≈ 576 mL
  - Calculations:
    - $/Wh = (142.74$)/(236.8Wh) = $0.60279 per Wh ≈ __$0.60 per Wh__ (lower is better)
    - mass energy density = (236.8Wh)/(1.290kg) = 183.57 Wh per kg ≈ __180 Wh per kg__ (higher is better)
    - volumetric energy density = (236.8Wh)/(576.09E-6m^3) = 411050 Wh per m^3 ≈ __410 kWh per m^3__ (higher is better)
    
 __Li-Ion: [Samsung High Drain INR18650-35E 4 Pcs](https://www.amazon.com/Samsung-INR18650-35E-Rechargeable-BD-Electronics/dp/B0762LDVF8/)__
 
   - Battery Specifications
     - price: $26.49 ($6.62 per battery)
     - capacity: 3.5 Ah or 12.95 Wh
     - max discharge: 8A
     - mass: 50 g
     - volume: (0.0651m)*pi*(0.01848m)^2 = 69.845E-6 m^3 ≈ 69.8 mL
   - Calculations:
     - $/Wh = (26.49$)/(12.95Wh) = $2.0455 per Wh ≈ __$2.05 per Wh__ (lower is better)
     - mass energy density = (12.95Wh)/(0.05kg) = 259 Wh per kg ≈ __260 Wh per kg__ (higher is better)
     - volumetric energy density = (12.95Wh)/(69.845E-6m^3) = 185410 Wh per m^3 ≈ __185 kWh per m^3__ (higher is better)
          
 ## Compiled Results
 
| Battery Type | Price/Capacity [$/Wh] | Mass Energy Density [Wh/kg] | Volumetric Energy Density [kWh/m^3] |
| :---: | :---: | :---: | :---: |
|  | (lower is better) | (higher is better) | (higher is better) |
| Lead Acid Example | 0.27 | 40 | 93 |
| NiMh Example | 0.82 | 93 | 190 |
| LiPo Example | 0.60 | 180 | 410 |
| Li-Ion Example | 2.05 | 260 | 185 |

.
.
.
.
.
.
.
.
.
.

## Realistic Alternatives

__NiMh: [AmazonBasics AA High-Capacity Rechargable Batteries (8-Pack)](https://www.amazon.com/AmazonBasics-High-Capacity-Rechargeable-Batteries-Pre-charged/dp/B00HZV9WTM/)__

Using a 10 slot holder [like this one](https://www.ebay.com/itm/2x-10-AA-Battery-Slot-Spring-Clip-Holder-Case-Plastic-Storage-Box-Flat-15V-DC/172767943184), we can hold 10 AA batteries in series, giving us effectively a single battery at (1.2V)(10) = 12V nominal.  Putting three of these packs in series will then give us a 36V nominal battery; we will refer to three of these 10x AA holders in series as a single "pack".  A single "pack" requires 30 AA batteries, and it can deliver around 6A continuous (because all of the batteries in a single pack are in series, it can only deliver the current of a single AA battery.  To achieve the 240A required by our WAMV, this then means that we need at least (240A)/(6A) = 40 "packs" total.  This requires (40 ["packs"])(30 [batteries/"pack"]) 1200 batteries in total.  At a price of $18.99 for 8 batteries, this will cost $2848.50 for all 1200 of the batteries that we will need for a single pack.  Factoring in the additional cost for 40 holders ($199), this brings us up to $3047.5
