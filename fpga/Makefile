# Crude make file to generate fpga_program.h
#
# Assumes the that design has already been
# synthesized.
#
# Could be extended to run the synthesis too
#
# ------------------------------------------------------------------------------
#

all: fpga_program.h

fpga_program.h: fpga_Implmnt/sbt/outputs/bitmap/top_bitmap.bin
	version=`grep 'define DEFAULT_VERSION' src/defaults.h | awk '{print $$3;}'`;	\
	./make-c --version=$$version $< > $@

fpga_Implmnt/sbt/outputs/bitmap/top_bitmap.bin: src/defaults.h
	@echo ERROR: $@ must be rebuilt by hand; exit 1

install: all
	if [ -f ../src/fpga_program.h ]; then				\
		cp ../src/fpga_program.h ../src/fpga_program.h.save;	\
	fi
	cp fpga_program.h ../src

clean:
	rm -f stdout.log

veryclean: clean fpga_implement_clean

#
# toss everything not in the repository in the fpga_Implmnt
# directory.
#
fpga_implement_clean:
	rm -f `hg status | grep '^? fpga/fpga_Implmnt' | sed 's/^\? fpga\///g'`
