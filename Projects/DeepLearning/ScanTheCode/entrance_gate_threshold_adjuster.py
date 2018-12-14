#!/usr/bin/env python
from Tkinter import *

import rospy
from std_msgs.msg import String
from std_msgs.msg import Int16


class ros_class:

  def __init__(self):
	pub1 = rospy.Publisher('entrance_gate/lower_threshold_r/hue', Int16, queue_size=10)
	pub2 = rospy.Publisher('entrance_gate/lower_threshold_r/saturation', Int16, queue_size=10)
	pub3 = rospy.Publisher('entrance_gate/lower_threshold_r/value', Int16, queue_size=10)
	pub4 = rospy.Publisher('entrance_gate/upper_threshold_r/hue', Int16, queue_size=10)
	pub5 = rospy.Publisher('entrance_gate/upper_threshold_r/saturation', Int16, queue_size=10)
	pub6 = rospy.Publisher('entrance_gate/upper_threshold_r/value', Int16, queue_size=10)
	
	pub13= rospy.Publisher('entrance_gate/lower_threshold_g/hue', Int16, queue_size=10)
	pub14= rospy.Publisher('entrance_gate/lower_threshold_g/saturation', Int16, queue_size=10)
	pub15= rospy.Publisher('entrance_gate/lower_threshold_g/value', Int16, queue_size=10)
	pub16= rospy.Publisher('entrance_gate/upper_threshold_g/hue', Int16, queue_size=10)
	pub17= rospy.Publisher('entrance_gate/upper_threshold_g/saturation', Int16, queue_size=10)
	pub18= rospy.Publisher('entrance_gate/upper_threshold_g/value', Int16, queue_size=10)

	rospy.init_node('talker', anonymous=True)
	rate = rospy.Rate(10) # 10hz

	while not rospy.is_shutdown():
		def show_values1():
			rospy.loginfo("Published Red Values")
			pub1.publish(w_r_l_h.get())
			pub2.publish(w_r_l_s.get())
			pub3.publish(w_r_l_v.get())
			pub4.publish(w_r_u_h.get())
			pub5.publish(w_r_u_s.get())
			pub6.publish(w_r_u_v.get())
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
		master3 = Tk()

		w = 180
		h = 425
		x1 = 1400
		y = 550
		# y1 = 500
		x2 = x1 + w
		# y2 = 500
		x3 = x2 + w
		# y3 = 500

		master.geometry('%dx%d+%d+%d' % (w, h, x1, y))
		master3.geometry('%dx%d+%d+%d' % (w, h, x2, y))


		w = Label(master, text="Red", bg="red", fg="white")
		w.pack(fill=X)
		
		w = Label(master, text="Lower Hue", bg="white", fg="black")
		w.pack(fill=X)

		w_r_l_h = Scale(master, from_=0, to=180, length= 175, orient=HORIZONTAL)
		w_r_l_h.set(140)
		w_r_l_h.pack()

		w = Label(master, text="Lower Saturation", bg="white", fg="black")
		w.pack(fill=X)

		w_r_l_s = Scale(master, from_=0, to=255, length= 175, orient=HORIZONTAL)
		w_r_l_s.set(120)
		w_r_l_s.pack()

		w = Label(master, text="Lower Value", bg="white", fg="black")
		w.pack(fill=X)

		w_r_l_v = Scale(master, from_=0, to=255, length= 175, orient=HORIZONTAL)
		w_r_l_v.set(60)
		w_r_l_v.pack()

		w = Label(master, text="Upper Hue", bg="white", fg="black")
		w.pack(fill=X)

		w_r_u_h = Scale(master, from_=0, to=180, length= 175, orient=HORIZONTAL)
		w_r_u_h.set(176)
		w_r_u_h.pack()

		w = Label(master, text="Upper Saturation", bg="white", fg="black")
		w.pack(fill=X)

		w_r_u_s = Scale(master, from_=0, to=255, length= 175, orient=HORIZONTAL)
		w_r_u_s.set(255)
		w_r_u_s.pack()

		w = Label(master, text="Upper Value", bg="white", fg="black")
		w.pack(fill=X)

		w_r_u_v = Scale(master, from_=0, to=255, length= 175, orient=HORIZONTAL)
		w_r_u_v.set(200)
		w_r_u_v.pack()

		# lower_green = np.array([74,30,30])
		# upper_green = np.array([85,255,255])


		w = Label(master3, text="Green", bg="green", fg="black")
		w.pack(fill=X)

		w = Label(master3, text="Lower Hue", bg="white", fg="black")
		w.pack(fill=X)

		w_g_l_h = Scale(master3, from_=0, to=180, length= 175, orient=HORIZONTAL)
		w_g_l_h.set(74)
		w_g_l_h.pack()

		w = Label(master3, text="Lower Saturation", bg="white", fg="black")
		w.pack(fill=X)

		w_g_l_s = Scale(master3, from_=0, to=255, length= 175, orient=HORIZONTAL)
		w_g_l_s.set(30)
		w_g_l_s.pack()

		w = Label(master3, text="Lower Value", bg="white", fg="black")
		w.pack(fill=X)

		w_g_l_v = Scale(master3, from_=0, to=255, length= 175, orient=HORIZONTAL)
		w_g_l_v.set(30)
		w_g_l_v.pack()

		w = Label(master3, text="Upper Hue", bg="white", fg="black")
		w.pack(fill=X)

		w_g_u_h = Scale(master3, from_=0, to=180, length= 175, orient=HORIZONTAL)
		w_g_u_h.set(85)
		w_g_u_h.pack()

		w = Label(master3, text="Upper Saturation", bg="white", fg="black")
		w.pack(fill=X)

		w_g_u_s = Scale(master3, from_=0, to=255, length= 175, orient=HORIZONTAL)
		w_g_u_s.set(240)
		w_g_u_s.pack()

		w = Label(master3, text="Upper Value", bg="white", fg="black")
		w.pack(fill=X)

		w_g_u_v = Scale(master3, from_=0, to=255, length= 175, orient=HORIZONTAL)
		w_g_u_v.set(220)
		w_g_u_v.pack()

		Button(master, text='Show', command=show_values1).pack()
		Button(master3, text='Show', command=show_values3).pack()

		mainloop()


if __name__ == '__main__':
    try:
        a = ros_class()
        # a.tkinter_bar()

    except rospy.ROSInterruptException:
        pass
