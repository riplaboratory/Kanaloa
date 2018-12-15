Paul Baessler

ME 396

Team Kanaloa: Technical Documentation

Mechanical Subsystem: Mock “Scan the Code Obstacle”

Within the last month or so, I have been in charge of designing and putting
together the mock “Scan the Code” obstacle to help simulate one of the two tasks
given at the Maritime Robot X competition this December 2018. Up to this point,
we had already constructed a “Scan the code” obstacle, but was only capable of
holding up the light pole which could only help us out with one of the two
tasks.

The new design allows us to help prepare for not only the “Scan the Code”
challenge but for the “Identifying Shapes” portion as well by being able to not
only emit patterns of light, but to also hold up multiple shapes of different
colors for the WAM-V to scan.

Coming up with this design, we had to start everything from scratch. After the
idea came into place, we needed to perform calculations on how much the system
could tilt without it turning over.

After seeing that everything should’ve been safe, we began construction over the
next few days. The final design was a cruciform of two 1 x 8 ft\^2 plywood
boards with a hollow PVC cube bungee tied on top (Figure 1). The light pole
resides in the middle of the system with bungee cords attached to all four ends
of the buoy. The light pole’s circuit box is fixed in the center of the
cruciform. What keeps the system afloat are four 1 gallon buckets at the ends of
the four buoy. Here is an image:

![](media/8116d34ee7e973a3a09d427531bfb62c.jpg)

Figure 1: Image of Mock “Scan the Code” Obstacle

Figure 1: Image of Mock “Scan the Code” Obstacle

As it is visible in the image, there is a little bit of sinking occurring in the
middle: The middle is submerged and had it not been for the waterproof casing,
the circuit box would have been filled with water.

The next task was to come up with a solution for this problem. From the
description of the problem itself, it was clear that the board wasn’t stiff
enough to support the centric loading of the light pole. The answer was to then
increase the stiffness since there was bending in the boards. After reviewing
some material mechanics, the best option was to increase the second moment of
area as the amount of deflection was inversely proportional to it.

This was accomplished by increasing the thickness of the boards by attaching
smaller 0.3 x 8 ft\^2 boards to the long ends of both pieces of 1 x 8 ft\^2
plywood (Figure 2). Although this has yet to be tested, we are confident in this
solution as the mathematics show that system was now 64 times stiffer. The rest
of the design stayed the same, except the three dimensional cubic frame now
rests atop the added pieces of plywood.

![](media/63cdce0fbd691f1a873114ce238858b7.jpg)

Figure 2: Image of updated board (could be better)

To set up the mock “Scan the Code Buoy”, follow these steps

1.  Place the “Top” wood piece on top of “Bottom” wood piece with the 4
    quadrants lined up. Use 3/8’’ fasteners and bolts to hold them down.

2.  Assemble the PVC cube frame and place on top of the wooden cruciform with
    the corners on each piece of wood. Two corners will end up on the protruding
    area of the bottom piece.

3.  Tie down the cubic at its corners using bungee cords. Use 2 red cords or 1
    blue cord for each respective corner.

4.  Insert the black PVC rod into the center of the cross and use a 3/8’’
    fastener (with bolt) to hold it in place. This will also act as what keeps
    up the light pole.

5.  Insert four Home-Depot one-gallon buckets into the holes of the cruciform
    and tie them down using orange ratchet straps. There should be a hole for
    the ratchet straps on each end of the bucket.

6.  Hook up 2 yellow bungee cords on 2 opposing corners of the cruciform at the
    ratchet straps, and attach the other ends to the light pole. Use two blue
    cords each on the other two opposing sides and repeat.
