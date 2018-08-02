# Wire Gauge Calculations (as a function of current)

This is a primer document detailing how to select the proper high-current wire gauge.  There are three important factors in this process:

 - Continuous DC (or AC) current that passes through the conductor.
 - Conductor length.
 - Gauge standard used (usually american wire gauge (AWG), but this is not always the case!)
 - Conductor stranding (solid core or stranded)
 - Conductor material (usually pure copper, or copper coated aluminum (CCA)).

First, you decide on a wire gauge based on the first two factors (continuous current, and conductor length), then the remaining factors act as "modifiers" to your first wire gauge selection.  

Other modifiers that exist that we do not account for in this primer document, as they are generally considered to be less significant, or are already considered by the manufacturer.  These include:

 - Ambient temperature
 - Insulation thickness

## Step 1: continuous current vs. conductor length

As current draw increases, the wire gauge, and wire length must also increase.  A handy chart taken from (Canal Marketing)[http://canalmarketing.info/copper-wire-load-chart-images#] demonstrates this relationship:

(!image)[https://github.com/riplaboratory/Kanaloa/blob/master/PrimerDocuments/WireGageCalculations/Images/WireLengthVsAmperage.jpg]

There are also two significantly impactful variables that are ignored in this table: conductor material, and stranded vs. solid wire.

Conductor material is typically assumed to be copper; however, for large gauge wire, copper-clad-aluminum (CCA) is generally used for reasons of cost (copper is more conductive, but more expensive, than aluminum).  There is a lot of conflicting, circumstantial information on this topic online.  Generally speaking, for small conductors, pure copper and CCA resistivity is similar because the copper makes up a more significant fraction of the conductor, and due to the skin effect (for AC applications).  However, for large conductors, the conservative assumption is to assume that the conductivity of the CCA is closer to the conductivity of aluminum.  This is a simple linear multiplier (by the diameter).

Conductivity of copper = 5.95E7 [1/ohm*m]
Conductivity of aluminum = 3.77E7 [1/ohm*m]
Aluminum/Copper conductivity ratio: (3.77E7)/(5.95E7) = 0.63361 ≈ 0.63

Secondly, you should also consider the relationship between stranded and single core wire.  Because wire gauge standard is based on the diameter of the conductor, solid core wire has the best conductivity, since stranded wire is not 100% dense.  However, solid core wire is rarely used for large gauge applications, because it is far too stiff.  Thusly, you need to know how many “strands” (cores) are in your wire stranded wire.

Solid core conductivity = c
3 core conductivity = c*0.67221 ≈ c*0.67
4-6 core conductivity = c*0.53777 ≈ c*0.54
7-24 core conductivity = c*0.46708 ≈ c*0.47
25-42 core conductivity = c*0.40333 ≈ c*0.40
43+ core conductivity = c*0.33611 ≈ c*0.34

This information was “back calculated” from this site.  

Process
The workflow for calculating wire gauge is as follows:
Identify current you need to handle, and over what wire length.  
Use wire length table to figure out what gauge wire you need for the task.
Determine the diameter of that gauge wire.  Then determine the cross sectional area of that conductor (A=pi*r^2).  
Identify whether you’re using pure copper, or CCA.  If you are using CCA, then divide your wire area by 0.63.  This modifier should result in a larger required area.
Identify the number of cores (strands) in your wire.  Divide your revised required area by the core conductivity modifiers (0.67 for 3 core, 0.54 for 4-6 core, 0.47 for 7-24 core, etc.).  This modifier should result in a larger required area.
Convert this revised required cross sectional area to a diameter.  Back translate this revised required diameter to an AWG size.
If this AWG size is too large, or too expensive, then you can consider using multiple parallel runs of a smaller wire diameter. 

Meeting all of these requirements will give you an ultra-conservative idea of the wiring system you need.

4’ Primary Power Transmission (1100A, 4’, Stranded)

1100A
4’ length
Pure Copper
Stranded (43+ core)

1100 A is not on the table, so let’s estimate that we need 4*250+1*100.  This means we would need four runs of 4 gauge, and one run of 8 gauge based on the table.  4 AWG is 0.2043” diameter (0.032781 in^2), and 8 AWG is 0.1285” diameter (0.012968 in^2).  That’s a total cross sectional area of 0.045750 in^2.  No CCA multiplier, but there is a stranded multiplier of 0.34.  This gives (0.045750 [in^2])*(1/0.34) = 0.13456 [in^2].  This translates back to a diameter of 0.41392 in (diameter = 2*sqrt(area/pi)).  

WindyNation 4 Gauge: 0.23” dia, 0.041548 in^2 area
2 AWG: 0.2576” dia, 0.052117 in^2 area
WindyNation 2 Gauge: 0.30” dia, 0.070686 in^2 area
1/0: 0.3249” dia, 0.082906 in^2 area
WindyNation 1/0 Gauge: 0.37” dia, 0.10752 in^2 area
2/0: 0.3648” dia, 0.10452 in^2 area
WindyNation 2/0 Gauge: 0.43” dia, 0.14522 in^2 area

For this application three runs of 2 AWG (0.2576” diameter, 0.052117 in^2 area) would be sufficient.  WindyNation 2 gauge wire is 0.30” diameter, or 0.070686 in^2 area), which would only need two runs if true.

10’ Rear Thruster Power Transmission (200A, 10’, Stranded)

200A
10’ length
Pure Copper
Stranded (43+ core)

200A with a 10’ wire length calls for 1/0 AWG according to the table.  1/0 AWG is 0.3249” diameter, which is 0.082907 in^2.  No CCA multiplier, but there is a stranded multiplier of 0.34.  This gives (0.082907 [in^2])*(1/0.34) = 0.24384 [in^2].  This translates back to a diameter of 0.55720” (diameter = 2*sqrt(area/pi)). 

2 AWG: 0.2576” dia, 0.052117 in^2 area
WindyNation 2 Gauge: 0.30” dia, 0.070686 in^2 area
1/0: 0.3249” dia, 0.082906 in^2 area
WindyNation 1/0 Gauge: 0.37” dia, 0.10752 in^2 area
2/0: 0.3648” dia, 0.10452 in^2 area
WindyNation 2/0 Gauge: 0.43” dia, 0.14522 in^2 area

Also note that because it’s a three wire motor, you’re dividing the current between three conductors instead of two.  Therefore you should meet the total cross sectional area requirement over three wires instead of two.  Realistically, I’m OK with

Old stuff:

Using the “10 gauge” Amazon Wire (from China)
The “10 gauge” wire we have from Amazon is very cheap, though you should consider that (1) it’s not actually 10 AWG, it’s 10 “gauge”, which is ultimately somewhere between 10 and 12 AWG, (2) it’s CCA, and (3) it’s stranded.

Let’s say we want to flow 200A with this wire for the thrusters, which is approximately an 8ft wire length.  From the table we can see that we need around 4 AWG.  4 AWG is 0.2043” diameter, which is 0.032781 in^2.  Dividing by the CCA modifier (0.63) and the 7-24 core conductivity modifier (0.47), we end up with an area of 0.032781[in^2]*(1/0.63)*(1/0.47)=0.11071 [in^2].  This translates back to a diameter of 0.37545 in (diameter = 2*sqrt(area/pi).

This tells us that, if we use stranded CCA, we need a wire diameter of 0.37545 in (or cross sectional area of 0.11071 in^2.  The “10 gauge” amazon wire has a diameter of ≈ 0.09 in, or 0.0063617 in^2.  To reach a desired cross sectional area of 0.11071 in^2, we would need (0.11071)/(0.0063617)=17.403 strands of “10 gauge” wire.

Which is sort of ridiculous.  But these are very conservative numbers.  It probably makes more sense to buy very large gauge wire for these long transfers of power.  Need to go through this calculation for each run of high current wire.  

Using “4 gauge” Amazon Wire (from China)
Link here

Similar to before, let’s make the assumption that this wire gauge is somewhere between 4 and 3 gauge, CCA and heavily stranded.  

If we want to flow 200A over 8 ft, we need 4 AWG.  4 AWG is 0.2043” diameter, which is 0.032781 in^2.  Dividing by the CCA modifier (0.63) and the 25-42 core conductivity modifier (0.40), we end up with an area of 0.032781[in^2]*(1/0.63)*(1/0.40)=0.13008 [in^2].  This translates back to a diameter of 0.40697 in (diameter = 2*sqrt(area/pi).  

The “6 gauge” amazon wire (assuming true 5 AWG) has a diameter of ≈ 0.1819 in, or 0.025987 in^2.  To reach a desired cross sectional area of 0.13008 in^2, we would need (0.13008)/(0.025987) = 5.0056 strands of “4 gauge” wire.

This is more feasible. 


