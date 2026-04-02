alias r := run
alias w := wave

run file:
    make run IMG={{file}} && make trace && ./obj_dir/Vdrowsiness_detection_top {{file}}

wave:
    gtkwave drowsiness_sim.vcd
