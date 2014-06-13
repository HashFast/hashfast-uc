#
# Make a pdf out of ../hf_protocol.h for inclusion in the user guide
#

all:
	vim -c 'set printoptions=paper:letter,portrait:n' -c 'hardcopy > hf_protocol.ps' -c 'quit' ../hf_protocol.h
	ps2pdf hf_protocol.ps

