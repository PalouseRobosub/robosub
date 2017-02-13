
ret=""

generate_message() {
    count=$1
    speed=$2
    ret="data: ["
    for i in $(seq 0 $(($count - 1)))
    do
        ret="$ret 0,"
    done
    ret="$ret $speed,"

    for i in $(seq $(($count + 1)) 8)
    do
        ret="$ret 0,"
    done

    ret="${ret%?}]"
}

for thruster in {0..8..1}
do
    rosbag record --duration=55 -o thruster_${thruster}_ramp /thruster /orientation /orientation/pretty /acceleration/linear &
    pid=$1

    for speed in $(seq 0 .1 0.7)
    do
        generate_message $thruster $speed
        rostopic pub -1 /thruster robosub/thruster "$ret"
    done

    for speed in $(seq 0 -0.1 -0.7)
    do
        generate_message $thruster $speed
        rostopic pub -1 /thruster robosub/thruster "$ret"
    done

    wait $pid
done
