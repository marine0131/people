#!/usr/bin/env python
import rospy
import threading
import math
from multiprocessing import  Queue
import time
import RPi.GPIO as GPIO

from people_msgs.msg import PositionMeasurementArray
from people_msgs.msg import People

led_gpio_pin_list = [36, 38, 40]
infrade_gpio_pin_list = [35, 37]
queue = Queue(1)
last_queue_val = False
leg_dict_old = {}
skip_cnt = 0
fix_point_array = []
fix_point_prob = []
program_started = True
delay_time = 3
x_min = 0
x_max = 1.5
y_min = 0
y_max = 6
leg_id_dict = {}
class smart_road():
    global fix_point_array
    global fix_point_prob
    def __init__(self):
        global x_min, y_min, x_max, y_max, use_vel
        global delay_time
        global leg_id_dict
        delay_time = rospy.get_param("~delay_time")
        x_min = rospy.get_param("~x_min")
        x_max = rospy.get_param("~x_max")
        y_min = rospy.get_param("~y_min")
        y_max = rospy.get_param("~y_max")
        rospy.loginfo("delay_time:%d\n", delay_time)
        rospy.loginfo("\nx:%.2f ~ %.2f\ny:%.2f ~ %.2f\n",x_min,x_max,y_min,y_max)
        self.found_people = False
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(led_gpio_pin_list,GPIO.OUT)
        GPIO.setup(infrade_gpio_pin_list,GPIO.IN,pull_up_down=GPIO.PUD_UP)
        led_thread = threading.Thread(target=self.led_water_thread,args=(queue,0.15))
        led_thread.setDaemon(True)
        led_thread.start()
        rospy.Subscriber("leg_tracker_measurements",PositionMeasurementArray,self.leg_found_cb)

    def queue_put_value(self,val):
        try:
            queue.put(val,False)
        except:
            pass

    def fix_point_add_multi(self,fix_point):
        for point in fix_point:
            try:
                fix_point_array.index(point)
            except:
                #rospy.loginfo("add new point:%.2f,%.2f",point[0] / 100.0,point[1] / 100.0)
                fix_point_array.append(point)
                fix_point_prob.append(0.5)

    def check_point(self, suspect_point, step):
        new_leg = False
        suspect_new_point = suspect_point
        fix_point_idx_tmp = []
        if len(fix_point_array) == 0:
            if len(suspect_point) > 0:
                new_leg = True
                self.fix_point_add_multi(suspect_point)
        else:
            for fix_point in fix_point_array:
                point_loss = True
                idx = fix_point_array.index(fix_point)
                if len(suspect_point) > 0:
                    for point in suspect_point:
                        dist = self.position_dist(fix_point,point) / 100.0
                        #rospy.loginfo(dist)
                        if dist < 0.10:
                            point_loss = False
                            fix_point_prob[idx] += step
                            if fix_point_prob[idx] > 1:
                                fix_point_prob[idx] = 1
                            # clean suspect_new_point
                            try:
                                suspect_new_point.remove(point)
                            except:
                                pass
                if point_loss:
                    fix_point_prob[idx] -= step
                    if fix_point_prob[idx] < 0:
                        fix_point_prob[idx] = 0

                if fix_point_prob[idx] < 0.3:
                    #rospy.loginfo("del fix point:")
                    rospy.loginfo(fix_point_array[idx])
                    del fix_point_prob[idx]
                    del fix_point_array[idx]

            if len(suspect_new_point) > 0:
                new_leg = True
                self.fix_point_add_multi(suspect_new_point)

        return new_leg

    def position_dist(self,p1,p2):
        return math.sqrt((p1[0]- p2[0])**2 + (p1[1] - p2[1])**2)

    def timer_cb(self,total,freq):
        self.led_flash(freq)

    def start_led_flash(self,total,freq):
        global led_timer
        led_timer = threading.Timer(0.1,self.timer_cb,args=(total,freq))
        try:
            led_timer.start()
        except:
            pass

    def stop_led_flash(self):
        global led_timer
        try:
            led_timer.cancel()
        except:
            pass

        self.led_off()

    def extra_point_delete(self,pos):
        #if pos[0] <= -7 and pos[0] >= -9 and pos[1] < -6 and pos[1] > -9:
        #    return False
        #else:
            return True

    def judge_is_man(self,id_cnt,pos):
        x_dist = math.fabs(pos.x)
        is_man = False
        if x_dist > 7 and id_cnt > 10:
            is_man = True
        elif x_dist > 5 and id_cnt > 12:
            is_man = True
        elif x_dist <= 5 and id_cnt >= 13:
            is_man = True
        return (is_man)

    def leg_found_cb(self,msg):
        global program_started
        new_leg = False
        leg_list = []
        for leg in msg.people:
            delete_point_flag = self.extra_point_delete([leg.pos.x,leg.pos.y])
            if delete_point_flag and leg.pos.x < x_max and leg.pos.y < y_max and leg.pos.x > x_min and leg.pos.y > y_min:
                if leg_id_dict.has_key(leg.object_id) == False:
                    leg_id_dict[leg.object_id] = 1
                else:
                    leg_id_dict[leg.object_id] = leg_id_dict[leg.object_id] + 1
                rospy.loginfo("%.2f,%.2f,%d",leg.pos.x,leg.pos.y,leg_id_dict[leg.object_id])
                if leg_id_dict.has_key(leg.object_id) and self.judge_is_man(leg_id_dict[leg.object_id],leg.pos) :
                    leg_list.append([int(leg.pos.x * 100), int(leg.pos.y * 100)])

        if program_started:
            program_started = False
            self.fix_point_add_multi(leg_list)
        else:
            new_leg = self.check_point(leg_list,0.01)

        if new_leg:
            rospy.loginfo("Found New Leg")
            self.queue_put_value(True)
            #self.start_led_flash(3,1)
        else:
            self.queue_put_value(False)
            #self.stop_led_flash()

    def led_flash(self,freq):
        GPIO.output(led_gpio_pin_list[0],GPIO.HIGH)
        GPIO.output(led_gpio_pin_list[1],GPIO.LOW)
        GPIO.output(led_gpio_pin_list[2],GPIO.LOW)
        time.sleep(freq)
        GPIO.output(led_gpio_pin_list[0],GPIO.LOW)
        GPIO.output(led_gpio_pin_list[1],GPIO.HIGH)
        GPIO.output(led_gpio_pin_list[2],GPIO.LOW)
        time.sleep(freq)
        GPIO.output(led_gpio_pin_list[0],GPIO.LOW)
        GPIO.output(led_gpio_pin_list[1],GPIO.LOW)
        GPIO.output(led_gpio_pin_list[2],GPIO.HIGH)
        time.sleep(freq)

    def led_off(self):
        GPIO.output(led_gpio_pin_list[0],GPIO.LOW)
        GPIO.output(led_gpio_pin_list[1],GPIO.LOW)
        GPIO.output(led_gpio_pin_list[2],GPIO.LOW)

    def led_water_thread(self,queue,freq):
        global found_people
        delay_off = True
        max_delay_time = 0
        last_time = 0
        people_cnt = 0
        extra_time = 0
        while True:
            try:
                self.found_people = queue.get(False)
            except:
                pass

            io_state_1 = GPIO.input(infrade_gpio_pin_list[0])
            io_state_2 = GPIO.input(infrade_gpio_pin_list[1])
            if io_state_1 == GPIO.LOW:
                rospy.loginfo("infrad trig 1")
                self.found_people = True
                extra_time = 2

            if io_state_2 == GPIO.LOW:
                rospy.loginfo("infrad trig 2")
                self.found_people = True
                extra_time = 1

            if self.found_people:
                delay_off = True
                people_cnt = people_cnt + 1
                self.led_flash(freq)
            else:
                if delay_off:
                    max_delay_time = people_cnt

                    if max_delay_time > delay_time:
                        people_cnt = delay_time
                        max_delay_time = delay_time
                    elif max_delay_time <= 1:
                        max_delay_time = 2

                    people_cnt = people_cnt - 1

                    if people_cnt < 0:
                        people_cnt = 0

                    last_time = time.time()
                    while time.time() - last_time < max_delay_time + extra_time:
                        self.led_flash(freq)
                    try:
                        people_cnt = 0
                        extra_time = 0
                        queue.get(False)
                    except:
                        pass
                    delay_off = False
                   #idel_time = time.time()

                #if time.time() > idel_time + 10:
                #    leg_id_dict.clear()

                self.led_off()


if __name__ == '__main__':
    rospy.init_node('smart_road')
    smart_road()
    rospy.spin()
