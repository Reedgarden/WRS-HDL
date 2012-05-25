make -f Makefile

vsim -L secureip -L unisim -t 10fs work.main -voptargs="+acc" +nowarn8684 +nowarn8683

set StdArithNoWarnings 1
set NumericStdNoWarnings 1
do wave.do
#do wave_allports.do
radix -hexadecimal
run 4000us
wave zoomfull
radix -hexadecimal