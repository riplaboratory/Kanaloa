#!/usr/bin/env python
from Tkinter import *

import rospy
from std_msgs.msg import String
from std_msgs.msg import Int16


class ros_class:

  def __init__(self):
	pub1 = rospy.Publisher('lower_threshold_r/hue', Int16, queue_size=10)
	pub2 = rospy.Publisher('lower_threshold_r/saturation', Int16, queue_size=10)
	pub3 = rospy.Publisher('lower_threshold_r/value', Int16, queue_size=10)
	pub4 = rospy.Publisher('upper_threshold_r/hue', Int16, queue_size=10)
	pub5 = rospy.Publisher('upper_threshold_r/saturation', Int16, queue_size=10)
	pub6 = rospy.Publisher('upper_threshold_r/value', Int16, queue_size=10)
	pub7 = rospy.Publisher('lower_threshold_b/hue', Int16, queue_size=10)
	pub8 = rospy.Publisher('lower_threshold_b/saturation', Int16, queue_size=10)
	pub9 = rospy.Publisher('lower_threshold_b/value', Int16, queue_size=10)
	pub10 = rospy.Publisher('upper_threshold_b/hue', Int16, queue_size=10)
	pub11 = rospy.Publisher('upper_threshold_b/saturation', Int16, queue_size=10)
	pub12 = rospy.Publisher('upper_threshold_b/value', Int16, queue_size=10)
	pub13 = rospy.Publisher('lower_threshold_g/hue', Int16, queue_size=10)
	pub14= rospy.Publisher('lower_threshold_g/saturation', Int16, queue_size=10)
	pub15= rospy.Publisher('lower_threshold_g/value', Int16, queue_size=10)
	pub16= rospy.Publisher('upper_threshold_g/hue', Int16, queue_size=10)
	pub17= rospy.Publisher('upper_threshold_g/saturation', Int16, queue_size=10)
	pub18= rospy.Publisher('upper_threshold_g/value', Int16, queue_size=10)

	rospy.init_node('talker', anonymous=True)
	rate = rospy.Rate(10) # 10hz

	while not rospy.is_shutdown():
		# hello_str = "hello world %s" % rospy.get_time()
		# rospy.loginfo(hello_str)
		# pub.publish(hello_str)
		def show_values1():
			rospy.loginfo("Published Red Values")
			pub1.publish(w_r_l_h.get())
			pub2.publish(w_r_l_s.get())
			pub3.publish(w_r_l_v.get())
			pub4.publish(w_r_u_h.get())
			pub5.publish(w_r_u_s.get())
			pub6.publish(w_r_u_v.get())
			rate.sleep()

		def show_values2():
			rospy.loginfo("Published Blue Values")
			pub7.publish(w_b_l_h.get())
			pub8.publish(w_b_l_s.get())
			pub9.publish(w_b_l_v.get())
			pub10.publish(w_b_u_h.get())
			pub11.publish(w_b_u_s.get())
			pub12.publish(w_b_u_v.get())
			rate.sleep()

		def show_values3():
			rospy.loginfo("Published Green Values")
			pub13.publish(w_g_l_h.get())
			pub14.publish(w_g_l_s.get())
			pub15.publish(w_g_l_v.get())
			pub16.publish(w_g_u_h.get())
			pub17.publish(w_g_u_s.get())
			pub18.publish(w_g_u_v.get())
			rate.sleep()

		master = Tk()
		master2 = Tk()
		master3 = Tk()
		w = Label(master, text="Red", bg="red", fg="white")
		w.pack(fill=X)
		
		w = Label(master, text="Lower Hue", bg="white", fg="black")
		w.pack(fill=X)

		w_r_l_h = Scale(master, from_=0, to=180, orient=HORIZONTAL)
		w_r_l_h.set(140)
		w_r_l_h.pack()

		w = Label(master, text="Lower Saturation", bg="white", fg="black")
		w.pack(fill=X)

		w_r_l_s = Scale(master, from_=0, to=255, orient=HORIZONTAL)
		w_r_l_s.set(120)
		w_r_l_s.pack()

		w = Label(master, text="Lower Value", bg="white", fg="black")
		w.pack(fill=X)

		w_r_l_v = Scale(master, from_=0, to=255, orient=HORIZONTAL)
		w_r_l_v.set(60)
		w_r_l_v.pack()

		w = Label(master, text="Upper Hue", bg="white", fg="black")
		w.pack(fill=X)

		w_r_u_h = Scale(master, from_=0, to=180, orient=HORIZONTAL)
		w_r_u_h.set(176)
		w_r_u_h.pack()

		w = Label(master, text="Upper Saturation", bg="white", fg="black")
		w.pack(fill=X)

		w_r_u_s = Scale(master, from_=0, to=255, orient=HORIZONTAL)
		w_r_u_s.set(255)
		w_r_u_s.pack()

		w = Label(master, text="Upper Value", bg="white", fg="black")
		w.pack(fill=X)

		w_r_u_v = Scale(master, from_=0, to=255, orient=HORIZONTAL)
		w_r_u_v.set(200)
		w_r_u_v.pack()

		# lower_blue = np.array([107,170,60])
		# upper_blue = np.array([115,220,225])

		w = Label(master2, text="Blue", bg="blue", fg="white")
		w.pack(fill=X)

		w = Label(master2, text="Lower Hue", bg="white", fg="black")
		w.pack(fill=X)

		w_b_l_h = Scale(master2, from_=0, to=180, orient=HORIZONTAL)
		w_b_l_h.set(107)
		w_b_l_h.pack()

		w = Label(master2, text="Lower Saturation", bg="white", fg="black")
		w.pack(fill=X)

		w_b_l_s = Scale(master2, from_=0, to=255, orient=HORIZONTAL)
		w_b_l_s.set(170)
		w_b_l_s.pack()

		w = Label(master2, text="Lower Value", bg="white", fg="black")
		w.pack(fill=X)

		w_b_l_v = Scale(master2, from_=0, to=255, orient=HORIZONTAL)
		w_b_l_v.set(60)
		w_b_l_v.pack()

		w = Label(master2, text="Upper Hue", bg="white", fg="black")
		w.pack(fill=X)

		w_b_u_h = Scale(master2, from_=0, to=180, orient=HORIZONTAL)
		w_b_u_h.set(115)
		w_b_u_h.pack()

		w = Label(master2, text="Upper Saturation", bg="white", fg="black")
		w.pack(fill=X)

		w_b_u_s = Scale(master2, from_=0, to=255, orient=HORIZONTAL)
		w_b_u_s.set(220)
		w_b_u_s.pack()

		w = Label(master2, text="Upper Value", bg="white", fg="black")
		w.pack(fill=X)

		w_b_u_v = Scale(master2, from_=0, to=255, orient=HORIZONTAL)
		w_b_u_v.set(225)
		w_b_u_v.pack()

		# lower_green = np.array([74,30,30])
		# upper_green = np.array([85,255,255])


		w = Label(master3, text="Green", bg="green", fg="black")
		w.pack(fill=X)

		w = Label(master3, text="Lower Hue", bg="white", fg="black")
		w.pack(fill=X)

		w_g_l_h = Scale(master3, from_=0, to=180, orient=HORIZONTAL)
		w_g_l_h.set(74)
		w_g_l_h.pack()

		w = Label(master3, text="Lower Saturation", bg="white", fg="black")
		w.pack(fill=X)

		w_g_l_s = Scale(master3, from_=0, to=255, orient=HORIZONTAL)
		w_g_l_s.set(30)
		w_g_l_s.pack()

		w = Label(master3, text="Lower Value", bg="white", fg="black")
		w.pack(fill=X)

		w_g_l_v = Scale(master3, from_=0, to=255, orient=HORIZONTAL)
		w_g_l_v.set(30)
		w_g_l_v.pack()

		w = Label(master3, text="Upper Hue", bg="white", fg="black")
		w.pack(fill=X)

		w_g_u_h = Scale(master3, from_=0, to=180, orient=HORIZONTAL)
		w_g_u_h.set(85)
		w_g_u_h.pack()

		w = Label(master3, text="Upper Saturation", bg="white", fg="black")
		w.pack(fill=X)

		w_g_u_s = Scale(master3, from_=0, to=255, orient=HORIZONTAL)
		w_g_u_s.set(240)
		w_g_u_s.pack()

		w = Label(master3, text="Upper Value", bg="white", fg="black")
		w.pack(fill=X)

		w_g_u_v = Scale(master3, from_=0, to=255, orient=HORIZONTAL)
		w_g_u_v.set(220)
		w_g_u_v.pack()

		Button(master, text='Show', command=show_values1).pack()
		Button(master2, text='Show', command=show_values2).pack()
		Button(master3, text='Show', command=show_values3).pack()

		mainloop()


 #  def tkinter_bar(self):

	# def show_values():
	# 	print (w1.get(), w2.get())

	# master = Tk()
	# w1 = Scale(master, from_=0, to=42)
	# w1.set(19)
	# w1.pack()
	# w2 = Scale(master, from_=0, to=200, orient=HORIZONTAL)
	# w2.set(23)
	# w2.pack()
	# Button(master, text='Show', command=show_values).pack()

	# mainloop()

if __name__ == '__main__':
    try:
        a = ros_class()
        # a.tkinter_bar()

    except rospy.ROSInterruptException:
        pass


# def show_values():
#     print (w1.get(), w2.get())

# master = Tk()
# w1 = Scale(master, from_=0, to=42)
# w1.set(19)
# w1.pack()
# w2 = Scale(master, from_=0, to=200, orient=HORIZONTAL)
# w2.set(23)
# w2.pack()
# Button(master, text='Show', command=show_values).pack()

# mainloop()