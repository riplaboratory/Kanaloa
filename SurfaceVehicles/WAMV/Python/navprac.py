# File:  navprac.py
# By:   Haley DeTure for Kanaloa

# This script should take in information about current position, desired
# position, and current heading of the vehicle and find all necessary
# information to navigate from current to desired position.

        #r is radius of Earth
        #lat1 is current latitude (signed decimal)
        #lon1 is current longitude (signed decimal)
        #heading is current heading (degrees CCW from E)
        #lat2 is desired latitude
        #lon2 is desired longitude
        #ADD THE REST

from math  import *

r = 6371 #radius of Earth, km

#input
lon1 = float(input('Enter current longitude (E+ W-):\t'))
lat1 = float(input('Enter current latitude (N+ S-):\t\t'))

lon2 = float(input('Enter desired longitude (E+ W-):\t'))
lat2 = float(input('Enter desired latitude (N+ S-):\t\t'))

heading = float(input('Enter current heading (Degrees CCW from East): '))

#convert to coodinates to radians
lat1,lon1,lat2,lon2 = map(radians, [lat1,lon1,lat2,lon2])

# Calculate distance between GPS coordinates (Haversine Formula)
deltalon = lon2 - lon1
deltalat = lat2 - lat1
alpha = 2*asin( sqrt( (sin(abs(deltalat)/2))**2 + cos(lat1)*cos(lat2)*((sin(abs(deltalon)/2))**2) ) )
distance = alpha * r

#print('\nDistance = ', distance, 'km')

#OKAY NOW turn angle calc
x = cos(lat2)*sin(deltalon)
y = cos(lat1)*sin(lat2) - sin(lat1)*cos(lat2)*cos(deltalon)
bearing = degrees(atan2(y,x)) #Measured from East (-180,180) ccw+ cw-

#print('\nDEBUG initbearing = ',bearing)

#Want bearing to be measured ccw from East (0,360)
if bearing<0 and bearing>-180:
        bearing += 360
        #print('DEBUG neg init bearing being updated',bearing)
  
print('\n...In degrees CCW from East...')
print('original heading: ', heading, '\nbearing: {:.2f}'.format(bearing))
print('\n..............................')

theta = heading - bearing #theta is going to be turn angle

if theta>0 and theta>180:
        print('Turn CCW by {:.2f} degrees'.format(360-theta))
if theta<0 and -theta<180:
        print('Turn CCW by {:.2f} degrees'.format(-theta))
if theta>0 and theta<180:
        print('Turn CW by {:.2f} degrees'.format(theta))
if theta<0 and -theta>180:
        print('Turn CW by {:.2f} degrees'.format(360 - abs(theta)))

print('Travel {:.2f} km forward'.format(distance))

#OKAY NOW try implement PID

close_rad = 5 #radius of waypoint in which don't want vehicle to try to turn all the way around if we overshoot [km]
max_range = 50  #not sure what true 'max range' on wamv or mruh is, but this is that [km]

if distance >= close_rad:
        #run two ongoing control loops, one checking for angle error, one checking for distance error
        var = 'just need something indented for time being'

if distance < close_rad:
        #run only one control loop for distance, to avoid combersome turns
        #implement this one first for practice
        var = 'blah'
