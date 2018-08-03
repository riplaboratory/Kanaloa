# Wire Gauge Calculations (as a function of current)

This is a primer document detailing how to select the proper high-current wire gauge.  There are five important factors in this process:

 - Continuous DC (or AC) current that passes through the conductor.
 - Conductor length.
 - Gauge standard used; usually american wire gauge (AWG), but this is not always the case!
 - Conductor material; usually pure copper, or copper clad/coated aluminum (CCA).
 - Conductor stranding; solid core or stranded.

First, you decide on a wire gauge based on the first two factors (continuous current, and conductor length), then the remaining factors act as "modifiers" to your first wire gauge selection.  

Other modifiers exist that we do not account for in this primer document, as they are generally considered to be less significant, or are already considered by the manufacturer.  These include:

 - Ambient temperature
 - Insulation thickness
 
## 1. Wire sizing process

### Step 1: continuous current vs. conductor length

As increase in continuous current *and* conductor length will necessitate an increase in the wire gauge.  A chart detailing this information is available from [Canal Marketing](http://canalmarketing.info/copper-wire-load-chart-images#), and is shown below:

![image](https://github.com/riplaboratory/Kanaloa/blob/master/PrimerDocuments/WireGageCalculations/Images/WireLengthVsAmperage.jpg)

Based on your continuous current, and your conductor length, this chart will give you a conductor size in american wire gauge (AWG).  

### Step 2: convert AWG to cross sectional area

For the sake of standard sizing throughout this document, convert the conductor size from AWG to pure cross sectional area.  A chart describing AWG vs. cross sectional area is available from [Wikipedia](https://en.wikipedia.org/wiki/American_wire_gauge), and is shown below:

![image](https://github.com/riplaboratory/Kanaloa/blob/master/PrimerDocuments/WireGageCalculations/Images/AwgSpecifications.PNG)

The number you need is the _cross sectional area_ doe your conductor.  Knowing this number, you can move onto step 3.

### Step 3: conductor material modifier

Conductor material is typically assumed to be copper; however, copper-clad-aluminum (CCA) is often used for reasons of cost (copper is more conductive, but significantly more expensive, than aluminum), and pure aluminum wire is also used in rarer cases.  There is a significant amount of circumstantial information on this topic online.  To be certain, it is best to look at actual emperical evidence of conductivity of these wire types, particularly in the larger gauge varieties.  There is an excellent article detailing this debate at [Budgetphile](http://www.budgetphile.com/2013/11/budget-wiring-reality-of-copper-clad.html).  In this work, it was found that 100 ft of 16 AWG CCA has a resistance of around 0.6 ohms.  This results in a resistivity of 2.5591E-8 [ohm•m], or a conductivity of 3.9077E7 [1/(ohm•m)] ≈ 3.91E7 [1/(ohm•m)].

- [Conductivity of copper](https://en.wikipedia.org/wiki/Electrical_resistivity_and_conductivity) = 5.95E7 [1/(ohm•m)]
- [Conductivity of aluminum](https://en.wikipedia.org/wiki/Electrical_resistivity_and_conductivity) = 3.77E7 [1/(ohm•m)]
- Conductivity of CCA = 3.91E7 [1/(ohm•m)]

Given that CCA slots somewhere in between copper and pure aluminum, this result is quite believable.  Thusly, this leaves us with the following CCA:copper and aluminum:copper conductivity ratios: 

- **CCA:copper conductivity ratio: (3.91E7)/(5.95E7) = 0.657143 ≈ 0.66**
- **aluminum:copper conductivity ratio: (3.77E7)/(5.95E7) = 0.63361 ≈ 0.63**

Thusly, your first modifier is: **If you are using CCA, then take your cross sectional area from the previous step and multiply it by (1/0.66); if you are using aluminum, then take your cross sectional area from the previous step, and multiple it by (1/0.63).**  This will result in an increased required cross sectional area when using CCA or aluminum wire.

### Step 4: stranding modifier

Next you should also consider the relationship between stranded and single core wire.  Because wire gauge standard is based on the diameter of the conductor, solid core wire has the best conductivity, since stranded wire is not 100% dense.  However, solid core wire is rarely used for large gauge applications, because way too stiff to be useful for anything.  For an exact calculation of the losses assocaited with stranding, you need to know the exact method of stranding used in your particular wire.  However, as a general rule, according to [Wikipedia](https://en.wikipedia.org/wiki/American_wire_gauge) the gaps in a stranded wire configuration occupy approximately 10% of the wire area.  

Therefore, your second modifier is: **If you are using stranded wire, take your cross sectional area from the previous step, and multiply it by (1.1) from the chart above**

This will result in increased required cross sectional area when using stranded wire. 

## 2. Considerations when selecting wire

**Be careful when reading the product description of wire listings**.

In particular, pay close attention to how the listing specifies the wire gauge (size), and the type of material used in the wire.  This will not always be clear!  Sizing wire is very important for safety, so we do not want to make any crucial screw ups!

Regarding wire size, resellers will say something like "10 gauge" which may lead you to believe that the wire is 10 AWG (american wire gauge), but it is actually something smaller.  I have found that many white label resellers use a "gauge" system that falls one size _smaller_ than true AWG.  For example, "10 gauge" ≈ 11 AWG; "6 gauge" ≈ 7 AWG.  In my experience, it is totally fine to use this type of wire, as long as you take this into account when sizing your system.  

Regarding wire material, it is much more difficult for resellers to "lie" about the wire material (you can't really make up a new sizing system when it comes to material), so often times, companies will simply fail to mention that information in their listing.  If the listing explicitly says pure copper, CCA, or aluminum as the wire material, you can generally trust that information as correct; however, if it is not mentioned, you should not assume it is pure copper (because it probably isn't, and you should probably avoid that product altogether).

## 3. Examples

### 3.1. Need to carry 150 A over a range of 2 ft

Design problem:
 - 150 A
 - 2 ft distance
 - ["10 gauge" (more like 11 AWG), CCA, stranded](https://www.amazon.com/gp/product/B00J357DGW/) (approximately 4.3 [mm^2] = 4.3E-6 [m^2] cross sectional area).  OR [6 AWG, pure copper, stranded](https://www.amazon.com/Welding-Battery-Copper-Flexible-Inverter/dp/B01MTALKID/) (13.3 [mm^2] = 13.3E-6 [m^2] cross sectional area)

["10 gauge" (more like 11 AWG), CCA, stranded](https://www.amazon.com/gp/product/B00J357DGW/) (approximately 4.3 [mm^2] = 4.3E-6 [m^2] cross sectional area) wire version
 - Step 1: 8 AWG
 - Step 2: 8 AWG = 8.37 [mm^2] = 8.37E-6 [m^2]
 - Step 3: CCA modifier = (8.37E-6)•(1/0.66) = 12.7E-6 [m^2]
 - Step 4: Stranded modifier = (12.7E-6)•(1.1) = 14.0E-6 [m^2]
 
[6 AWG, pure copper, stranded](https://www.amazon.com/Welding-Battery-Copper-Flexible-Inverter/dp/B01MTALKID/) (13.3 [mm^2] = 13.3E-6 [m^2] cross sectional area wire version 
 - Step 1: 8 AWG
 - Step 2: 8 AWG = 8.37 [mm^2] = 8.37E-6 [m^2]
 - Step 3: No CCA modifier (pure copper)
 - Step 4: Stranded modifier = (8.37E-6)•(1.1) = 9.21E-6 [m^2]
 
If using the ["10 gauge" CCA](https://www.amazon.com/gp/product/B00J357DGW/), you will need at least four runs to meet the necessary cross sectional area.  If using [6 AWG, pure copper, stranded](https://www.amazon.com/Welding-Battery-Copper-Flexible-Inverter/dp/B01MTALKID/), a single run will suffice. 
 
## 4. Large gauge wire we generally buy

 - [2 AWG, pure copper, stranded](https://www.amazon.com/gp/product/B01MUC9VT3/)
 - [6 AWG, pure copper, stranded](https://www.amazon.com/Welding-Battery-Copper-Flexible-Inverter/dp/B01MTALKID/)
 - ["10 gauge" (more like 11 AWG), CCA, stranded](https://www.amazon.com/gp/product/B00J357DGW/)
