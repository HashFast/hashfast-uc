# ------------------------------------------------------------------------------
# crude simulation make file
#
VFLAGS=+acc +define+SIMULATION +define+VERSION=1 +define+TEST
SRCS=   top.v		\
	sync-gen.v	\
	spi-slave.v	\
	tacho-timer.v	\
	adc-poller.v

# 	i2c-master.v


all: work compile

.IGNORE: compile
compile: ${SRCS}
	vlog ${VFLAGS} ${SRCS}

work:
	vlib work

clean:
	rm -rf work transcript vsim.wlf

