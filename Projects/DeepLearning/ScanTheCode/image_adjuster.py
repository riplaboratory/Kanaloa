#!/usr/bin/env python
from Tkinter import *

import rospy
from std_msgs.msg import String
from std_msgs.msg import Int16


class ros_class:

  def __init__(self):
	pub1 = rospy.Publisher('image_adjuster/x_min', Int16, queue_size=10)
	pub2 = rospy.Publisher('image_adjuster/x_max', Int16, queue_size=10)
	pub3 = rospy.Publisher('image_adjuster/y_min', Int16, queue_size=10)
	pub4 = rospy.Publisher('image_adjuster/y_max', Int16, queue_size=10)
	pub5 = rospy.Publisher('image_adjuster/blur', Int16, queue_size=10)
	pub6 = rospy.Publisher('image_adjuster/toss_images', Int16, queue_size=10)

	rospy.init_node('talker', anonymous=True)
	rate = rospy.Rate(10) # 10hz

	while not rospy.is_shutdown():
		# hello_str = "hello world %s" % rospy.get_time()
		# rospy.loginfo(hello_str)
		# pub.publish(hello_str)
		def show_values1():
			rospy.loginfo("Published Red Values")
			pub1.publish(w_x_min.get())
			pub2.publish(w_x_max.get())
			pub3.publish(w_y_min.get())
			pub4.publish(w_y_max.get())
			pub5.publish(w_blur.get())
			pub6.publish(w_image_tosser.get())
			rate.sleep()


		master = Tk()

		w = 180
		h = 400
		x = 580
		y = 550

		master.geometry('%dx%d+%d+%d' % (w, h, x, y))

		w = Label(master, text="Image Adjuster", bg="black", fg="white")
		w.pack(fill=X)
		
		w = Label(master, text="X Min", bg="white", fg="black")
		w.pack(fill=X)

		w_x_min = Scale(master, from_=0, to=100, length= 160,orient=HORIZONTAL)
		w_x_min.set(0)
		w_x_min.pack()

		w = Label(master, text="X Max", bg="white", fg="black")
		w.pack(fill=X)

		w_x_max = Scale(master, from_=0, to=100, length= 160,orient=HORIZONTAL)
		w_x_max.set(100)
		w_x_max.pack()

		w = Label(master, text="Y Min", bg="white", fg="black")
		w.pack(fill=X)

		w_y_min = Scale(master, from_=0, to=100, length= 160,orient=HORIZONTAL)
		w_y_min.set(0)
		w_y_min.pack()

		w = Label(master, text="Y Max", bg="white", fg="black")
		w.pack(fill=X)

		w_y_max = Scale(master, from_=0, to=100, length= 160,orient=HORIZONTAL)
		w_y_max.set(100)
		w_y_max.pack()

		w = Label(master, text="Image Blur", bg="white", fg="black")
		w.pack(fill=X)

		w_blur = Scale(master, from_=3, to=20, length= 160,orient=HORIZONTAL)
		w_blur.set(7)
		w_blur.pack()

		w = Label(master, text="Images to Toss", bg="white", fg="black")
		w.pack(fill=X)

		w_image_tosser = Scale(master, from_=1, to=6, length= 160,orient=HORIZONTAL)
		w_image_tosser.set(1)
		w_image_tosser.pack()

		Button(master, text='Show', command=show_values1).pack()

		mainloop()



if __name__ == '__main__':
    try:
        a = ros_class()

    except rospy.ROSInterruptException:
        pass
