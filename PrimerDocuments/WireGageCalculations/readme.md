# Wire Gauge Calculations (as a function of current)

This is a primer document detailing how to select the proper high-current wire gauge.  There are three important factors in this process:

 - Continuous DC (or AC) current that passes through the conductor.
 - Conductor length.
 - Gauge standard used (usually american wire gauge (AWG), but this is not always the case!)
 - Conductor material (usually pure copper, or copper coated aluminum (CCA)).
 - Conductor stranding (solid core or stranded)

First, you decide on a wire gauge based on the first two factors (continuous current, and conductor length), then the remaining factors act as "modifiers" to your first wire gauge selection.  

Other modifiers that exist that we do not account for in this primer document, as they are generally considered to be less significant, or are already considered by the manufacturer.  These include:

 - Ambient temperature
 - Insulation thickness

## Step 1: continuous current vs. conductor length

As increase in continuous current *and* conductor length will necessitate an increase in the wire gauge.  A chart detailing this information is available from [Canal Marketing](http://canalmarketing.info/copper-wire-load-chart-images#), and is shown below:

![image](https://github.com/riplaboratory/Kanaloa/blob/master/PrimerDocuments/WireGageCalculations/Images/WireLengthVsAmperage.jpg)

Based on your continuous current, and your conductor length, this chart will give you a conductor size in american wire gauge (AWG).  

## Step 2: convert AWG to cross sectional area

For the sake of standard sizing throughout this document, convert the conductor size from AWG to pure cross sectional area.  A chart describing AWG vs. cross sectional area is available from [Engineering Toolbox](https://www.engineeringtoolbox.com/wire-gauges-d_419.html), and is shown below:

![image](https://github.com/riplaboratory/Kanaloa/blob/master/PrimerDocuments/WireGageCalculations/Images/AwgChart.PNG)

The number you need is the _cross sectional area_ doe your conductor.  Knowing this number, you can move onto step 3.

## Step 3: conductor material modifier

Conductor material is typically assumed to be copper; however, copper-clad-aluminum (CCA) is often used for reasons of cost (copper is more conductive, but significantly more expensive, than aluminum).  There is a lot of conflicting, circumstantial information on this topic online.  For clear reasons, companies that manufacture pure copper wire will defend that it is cost-superior to CCA, whereas companise that manufacture CCA will defend that it is cost-superior to pure copper.  There are some circumstantial claims regarding the skin effect for conductors in support of CCA; however, to be certain, it is best to look at actual emperical evidence of conductivity.  There is an excellent article detailing this debate at [Budgetphile](http://www.budgetphile.com/2013/11/budget-wiring-reality-of-copper-clad.html).  In this work, it was found that 100 ft of 16 AWG CCA has a resistance of around 0.6 ohms.  This results in a resistivity of 2.5591E-8 [ohm•m], or a conductivity of 3.9077E7 [1/(ohm•m)].

Conductivity of copper = 5.95E7 [1/(ohm•m)]
Conductivity of aluminum = 3.77E7 [1/(ohm•m)]
Conductivity of CCA = 3.9077E7 [1/(ohm•m)]
CCA/Copper conductivity ratio: (3.91E7)/(5.95E7) = 0.657143 ≈ 0.66

Given that CCA slots somewhere in between copper and pure aluminum, this result is quite believable.  Thusly, your first modifier is: **If you are using CCA, then take your cross sectional area from the previous step and multiply it by 1/0.66**

This will result in an increased required cross sectional area when using CCA.

## Step 4: stranding modifier

Next you should also consider the relationship between stranded and single core wire.  Because wire gauge standard is based on the diameter of the conductor, solid core wire has the best conductivity, since stranded wire is not 100% dense.  However, solid core wire is rarely used for large gauge applications, because way too stiff to be useful for anything.  Thusly, you need to know how many “strands” (cores) are in your wire stranded wire.  The chart previously linked from [Engineering Toolbox](https://www.engineeringtoolbox.com/wire-gauges-d_419.html) also discusses wire stranding.  Based on this analysis, the following multiplers can be used:

 - solid core conductivity, c = 1
 - 3 core conductivity ratio, c = 0.67221 ≈ 0.67
 - 4-6 core conductivity ratio, c = 0.53777 ≈ 0.54
 - 7-24 core conductivity ratio, c = 0.46708 ≈ 0.47
 - 25-42 core conductivity ratio, c = 0.40333 ≈ 0.40
 - 43+ core conductivity ratio, c = 0.33611 ≈ 0.34

Therefore, your second multipler is: **If you are using stranded wire, take your cross sectional area from the previous step, and multiply it by 1/c from the chart above**

This will result in increased required cross sectional area when using stranded wire. 

https://docs.google.com/document/d/1gT1hnzMKZPhWRTM7eS9sFVf8cWUCZ6KWmbnEr8JUOXc/edit#
