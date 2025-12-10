///*
// * finalproject.c
// *
// *  Created on: Nov 12, 2025
// *      Author: jz23
// */
//
//
//#include "open_interface.h"
//#include "Timer.h"
//#include "lcd.h"
//#include "uart_extra_help.h"
//#include "adc.h"
//#include <stdint.h>
//#include <math.h>
//#include "movement.h"
//#include "servo.h"
//#include "ping.h"
////
//void uart_sendStr(const char* str){
//    int i = 0;
//    while(str[i] != '\0'){
//        uart_sendChar(str[i]);
//        i++;
//    }
//
//}
//
//typedef struct obstacle_object {
//    int angle;
//    double distance;
//    int width;
//} obstacle_object;
//
//
//
//float scan(int degrees){
//    float distance = 0;
//    servo_move(degrees);
//    int cycles = ping_read();
//    distance = (float) cycles /16000000;
//    distance = distance / 2.0;
//    distance = distance * 34000;
//    timer_waitMillis(100);
//    return distance;
//}
//
//
//
//int main(void)
//{
//    timer_init();
//    lcd_init();
//    uart_init();
//    adc_init();
//    servo_init();
//    ping_init();
//
//    oi_t *sensor_data = oi_alloc();
//    oi_init(sensor_data);
//
//    char my_data;
//    char command[100];
//    int index = 0;
//
//
//
//    lcd_printf("Running");
//
//    while (1)
//    {
//        // Receive command
//        index = 0;
//        my_data = uart_receive();
//        while (my_data != '\n')
//        {
//            command[index++] = my_data;
//            my_data = uart_receive();
//        }
//        command[index] = '\n';
//        command[index + 1] = 0;
//
//        lcd_printf("Got: %s", command);
//
//        // ---- MAIN COMMAND SWITCH ----
//        switch (command[0])
//        {
//        case 't':  // Manual drive mode
//        {
//
//        int flag = 0;
//            while (flag == 0)
//            {
//                my_data = uart_receive(); // Wait for next command
//                switch (my_data)
//                {
//                case 'w':
//                    oi_setWheels(100, 100);
//                    timer_waitMillis(300);
//                    oi_setWheels(0, 0);
//                    uart_sendStr("Moved forward\n");
//                    break;
//
//                case 's':
//                    oi_setWheels(-100, -100);
//                    timer_waitMillis(300);
//                    oi_setWheels(0, 0);
//                    uart_sendStr("Moved backward\n");
//                    break;
//
//                case 'a':
//                    oi_setWheels(-100, 100);
//                    timer_waitMillis(300);
//                    oi_setWheels(0, 0);
//                    uart_sendStr("Turned left\n");
//                    break;
//
//                case 'd':
//                    oi_setWheels(100, -100);
//                    timer_waitMillis(300);
//                    oi_setWheels(0, 0);
//                    uart_sendStr("Turned right\n");
//                    break;
//
//                case 'b': // exit manual mode
//                    oi_setWheels(0, 0);
//                    uart_sendStr("Exiting manual mode\n");
//                    flag = 1;
//                    break;
//
//
//                default:
//                    oi_setWheels(0, 0);
//                    break;
//                }
//            }
//
//        }
//
//        case 'h':
//        case 'm':
//        {
//            // --------------- Object Scanning ---------------
//            uint16_t IR_1;
//            float IR_dist[91];
//            int angle = 0, i = 0, start_degree = 0;
//            int max_object_distance = 75;
//            int object_detected = 0, objCount = 0;
//            char buffer[50];
//            obstacle_object object_data[10];
//
//
//            for (angle = 0; angle <= 180; angle += 2)
//            {
//                servo_move(angle);
//                IR_1 = adc_read();
//                IR_dist[i] = (float)(336233 * pow(IR_1, -1.342));
//
//
//                // Object detection
//                if (IR_dist[i] <= max_object_distance && !object_detected)
//                {
//                    start_degree = angle;
//                    object_detected = 1;
//                }
//                if ((IR_dist[i] > max_object_distance || angle == 180) && object_detected)
//                {
//                    int width = angle - start_degree;
//                    object_detected = 0;
//                    if (width > 3 && objCount < 10)
//                    {
//                        obstacle_object new_object;
//                        int center_angle = start_degree + (width / 2);
//                        new_object.width = width;
//                        new_object.angle = center_angle;
//                        new_object.distance = scan(angle);
//
//                        object_data[objCount++] = new_object;
//                    }
//                }
//                // Send scan data to client for plotting
//                char buffer[10];
//                sprintf(buffer,"%d %.2f\n",angle,IR_dist[i]/100);
//                uart_sendStr(buffer);
//
//            }
//
//            // ---- Find smallest object ----
//            if (objCount > 0)
//            {
//                int smallest_width = object_data[0].width;
//                int smallest_angle = object_data[0].angle;
//                double object_distance = object_data[0].distance;
//                int k = 1;
//                for (k = 1; k < objCount; k++)
//                {
//                    if (object_data[k].width < smallest_width)
//                    {
//                        smallest_width = object_data[k].width;
//                        smallest_angle = object_data[k].angle;
//                        object_distance = object_data[k].distance;
//
//                    }
//                }
//
//
//
//                // ---------- WAIT FOR NEXT COMMAND ----------
//                uart_sendStr("END\n");
//                uart_sendStr("Scan complete. Awaiting next command (p=approach, b=cancel)\n");
//
//                while (1)
//                {
//                    my_data = uart_receive(); // Get next user command
//
//                    if (my_data == 'p')
//                    {
//                        // Move toward smallest object
//                        if (smallest_angle >= 90)
//                            turn_clockwise(sensor_data, smallest_angle - 92.5);
//                        else
//                            turn_counter_clockwise(sensor_data, 92.5 - smallest_angle);
//
//                        double sum = 0;
//                        if (object_distance > 0)
//                        {
//                            oi_setWheels(150, 150);
//                            while (sum < (object_distance * 10) - 125)
//                            {
//                                oi_update(sensor_data);
//                                sum += sensor_data->distance;
//
//                                if (sensor_data->bumpLeft || sensor_data->bumpRight)
//                                {
//                                    int left_bump = sensor_data->bumpLeft;
//                                    int right_bump = sensor_data->bumpRight;
//
//                                    move_forward(sensor_data, -150);
//
//                                    if (left_bump && !right_bump)
//                                        turn_counter_clockwise(sensor_data, 60);
//                                    else if (right_bump && !left_bump)
//                                        turn_clockwise(sensor_data, 60);
//                                    else
//                                        turn_clockwise(sensor_data, 60);
//
//                                    move_forward(sensor_data, 250);
//
//                                    if (left_bump && !right_bump)
//                                        turn_clockwise(sensor_data, 90);
//                                    else
//                                        turn_counter_clockwise(sensor_data, 90);
//
//                                    break;
//                                }
//                            }
//                        }
//                        oi_setWheels(0, 0);
//                        break; // Exit after driving
//                    }
//                    else if (my_data == 'b')
//                    {
//                        uart_sendStr("Autonomous canceled\n");
//                        break;
//                    }
//                }
//            }
//            break;
//        }
//
//        case 'b': // General stop
//            oi_setWheels(0, 0);
//            break;
//
//        default:
//            uart_sendStr("Unknown command\n");
//            break;
//        }
//
//        if (command[0] != '\n')
//        {
//            uart_sendChar('\n');
//        }
//    }
//
//    oi_free(sensor_data);
//    return 0;
//}
