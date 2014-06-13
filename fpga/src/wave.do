onerror {resume}
quietly WaveActivateNextPane {} 0

add wave -noupdate -format Logic /test/SCL
add wave -noupdate -format Logic /test/SDA
add wave -noupdate -format Logic /test/assert_sda
add wave -noupdate -divider {Dut Interface}
add wave -noupdate -format Logic /test/dut/clk
add wave -noupdate -format Logic /test/dut/reset
add wave -noupdate -format Logic /test/dut/clk16
add wave -noupdate -format Logic /test/dut/slow_mode
add wave -noupdate -format Logic /test/dut/SDA_in
add wave -noupdate -format Logic /test/dut/SCL_in
add wave -noupdate -format Logic /test/dut/SDA_out
add wave -noupdate -format Logic /test/dut/SCL_out
add wave -noupdate -format Logic /test/dut/SDA_en
add wave -noupdate -format Logic /test/dut/SCL_en
add wave -noupdate -format Logic /test/dut/read
add wave -noupdate -format Logic /test/dut/write
add wave -noupdate -format Logic /test/dut/yield
add wave -noupdate -radix binary /test/dut/data_length
add wave -noupdate -radix hexadecimal /test/dut/slave_addr
add wave -noupdate -radix hexadecimal /test/dut/write_data
add wave -noupdate -format Logic /test/dut/started
add wave -noupdate -format Logic /test/dut/complete
add wave -noupdate -format Logic /test/dut/no_slave
add wave -noupdate -format Logic /test/dut/no_write
add wave -noupdate -format Logic /test/dut/timo
add wave -noupdate -radix hexadecimal /test/dut/read_data

add wave -noupdate -divider {Clocks}
add wave -noupdate -format Literal /test/dut/clk_div
add wave -noupdate -format Literal /test/dut/clk_cnt
add wave -noupdate -format Logic /test/dut/ph1
add wave -noupdate -format Logic /test/dut/p1
add wave -noupdate -format Logic /test/dut/ph2
add wave -noupdate -format Logic /test/dut/p2
add wave -noupdate -format Logic /test/dut/ph3
add wave -noupdate -format Logic /test/dut/p3
add wave -noupdate -format Logic /test/dut/ph4
add wave -noupdate -format Logic /test/dut/p4
add wave -noupdate -format Logic /test/dut/ph5
add wave -noupdate -format Logic /test/dut/p5
add wave -noupdate -format Logic /test/dut/ph_end

add wave -noupdate -divider {Serial Data}
add wave -noupdate -format Logic /test/dut/sda
add wave -noupdate -format Logic /test/dut/sda_meta
add wave -noupdate -format Logic /test/dut/scl
add wave -noupdate -format Logic /test/dut/scl_meta

add wave -noupdate -divider {State}

add wave -noupdate -format Literal /test/dut/state
add wave -noupdate -radix unsigned /test/dut/wait_cnt
add wave -noupdate -format Logic /test/dut/slave_wait
add wave -noupdate -radix unsigned /test/dut/bcnt
add wave -noupdate -radix hexadecimal /test/dut/txd
add wave -noupdate -radix hexadecimal /test/dut/rxd
add wave -noupdate -format Logic /test/dut/last
add wave -noupdate -format Literal /test/dut/dix
add wave -noupdate -format Logic /test/dut/reading
add wave -noupdate -radix hexadecimal /test/dut/wd

add wave -noupdate -divider {Slave}
add wave -noupdate -format Logic /test/running

TreeUpdate [SetDefaultTree]
WaveRestoreCursors {{Cursor 1} {0 ps} 0}
configure wave -namecolwidth 271
configure wave -valuecolwidth 100
configure wave -justifyvalue left
configure wave -signalnamewidth 0
configure wave -snapdistance 10
configure wave -datasetprefix 0
configure wave -rowmargin 4
configure wave -childrowmargin 2
configure wave -gridoffset 0
configure wave -gridperiod 1
configure wave -griddelta 40
configure wave -timeline 0
configure wave -timelineunits ns
update
WaveRestoreZoom {0 ps} {8200 ps}
