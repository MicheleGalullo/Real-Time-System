#define variable

CFLAGS = `allegro-config --libs` -lpthread -lrt -lm


obj1 = main.o global.o utility_functions.o graphical_functions.o layout_design.o motors_parameters.o PID_parameters.o control_position_functions.o 
obj2 = control_velocity_functions.o control_torque_functions.o trends.o thread_graphics_tasks.o threads_control_task.o thread_mouse_task.o

obj = $(obj1) $(obj2)
DEPS = global.h functions.h

%.o: %.c $(DEPS)
	gcc -c -o $@ $< $(CFLAGS)

all: DCmotors

DCmotors: $(obj)
	gcc $(obj) -o DCmotors $(CFLAGS)

main.o: main.c
	gcc -c main.c $(CFLAGS)

global.o: global.c
	gcc -c global.c $(CFLAGS)

utility_functions.o: utility_functions.c
	gcc -c utility_functions.c $(CFLAGS)

graphical_functions.o: graphical_functions.c
	gcc -c graphical_functions.c $(CFLAGS)

layout_design.o: layout_design.c
	gcc -c layout_design.c $(CFLAGS)

motors_parameters.o: motors_parameters.c
	gcc -c motors_parameters.c $(CFLAGS)

PID_parameters.o: PID_parameters.c
	gcc -c PID_parameters.c $(CFLAGS)

control_position_functions.o: control_position_functions.c
	gcc -c control_position_functions.c $(CFLAGS)

control_velocity_functions.o: control_velocity_functions.c
	gcc -c control_velocity_functions.c $(CFLAGS)

control_torque_functions.o: control_torque_functions.c
	gcc -c control_torque_functions.c $(CFLAGS)

trends.o: trends.c
	gcc -c trends.c $(CFLAGS)

thread_graphics_tasks.o: thread_graphics_tasks.c
	gcc -c thread_graphics_tasks.c $(CFLAGS)

threads_control_task.o: threads_control_task.c
	gcc -c threads_control_task.c $(CFLAGS)

thread_mouse_task.o: thread_mouse_task.c
	gcc -c thread_mouse_task.c $(CFLAGS)

start:
	sudo ./DCmotors

clean:
	rm -rf *o DCmotors
